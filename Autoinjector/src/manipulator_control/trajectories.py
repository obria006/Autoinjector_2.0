""" Scripts/classes for coordinating manipulator trajectories """
import time
import numpy as np
from PyQt6.QtCore import QObject, pyqtSignal, QMutex, QTimer, pyqtSlot
from PyQt6.QtTest import QTest
from src.miscellaneous.standard_logger import StandardLogger
from src.miscellaneous.thread_manager import aQThreader, aQWorker
from src.manipulator_control.calibration import Calibrator
from src.manipulator_control.error_utils import TrajectoryError
from src.manipulator_control.sensapex_utils import SensapexDevice

class ManipulatorModel(QObject):
    """
    Simplified SensapexDevice wrapper that also tracks if the device is
    moving (when moves are called from this class) to prevent numerous
    trajectories trying to coordinate manipulator movements siultaneously
    """
    move_started = pyqtSignal()
    move_completed = pyqtSignal()
    _internal_move_completed = pyqtSignal()
    errors = pyqtSignal(Exception)

    def __init__(self, dev:SensapexDevice):
        """
        Arguments:
            dev (SensapexDevice): Object that manipulates the sensapex manipulator
        """
        super().__init__()
        self._logr = StandardLogger(__name__)
        self.dev = dev
        self._moving = False
        self._internal_move_completed.connect(self._on_move_completed)

    def is_moving(self) -> bool:
        """ Returns true if manipulator is moving, else false """
        return self._moving

    def get_axis_angle(self) -> float:
        """ Returns manipulator injection axis angle in degrees """
        return self.dev.get_axis_angle()
    
    def get_pos(self) -> list[float, float, float, float]:
        """ Returns list of manipulator axis positions [x,y,z,d] """
        return self.dev.get_pos()

    def goto_pos(self, pos:list, speed:int):
        """
        Go to absolute manipulator positoin

        Args:
            pos (list): list of absolute axis positions [x,y,z,d]
            speed (int): speed of movement in um/s
        """
        try:
            if self.is_moving() is True:
                raise TrajectoryError("CRITICAL ERROR: Manipulator move commanded when already moving.")
            self.move_started.emit()
            self._moving = True
            self.move_request = self.dev.goto_pos(pos, speed)
            # Thread the blocking `_wait_for_move` function to maintain GUI responsiveness
            self._worker = aQWorker(self._wait_for_move, self.move_request)
            self._threader = aQThreader(self._worker)
            self._threader.start()
        except Exception as e:
            self.errors.emit(e)
    
    def _wait_for_move(self, move_request):
        """
        Wait for manipulator movement to finish, and emit signal to set `_moving`
        attribute to false (so `is_moving()` will return correct status).

        !! WARNING !! This function is blocking, so it is recommended to call it
        inside a worker thread.

        Args:
            move_request: move_request from SensapexDevice `goto_pos` return
        """
        move_request.finished_event.wait()
        self._internal_move_completed.emit()

    @pyqtSlot()
    def _on_move_completed(self):
        """
        Emits public signal of `move_completed` and sets `_moving` to false, so
        `is_moving()` returns correct status.
        """
        # Wait to give time for thread to be deleted before another move can be called
        QTimer.singleShot(50,self._emit_move_completed)

    def _emit_move_completed(self):
        self._moving = False
        self.move_completed.emit()
        

class XYCalibrationTrajectory(QObject):
    """
    Performs XY calibration trajectory by doing intial 3-point calibration,
    then moving to the FOV 4 corners and the center.
    
    Signals:
        started: emitted when the manipulator is commanded to first position
        finished: emitted when trajectory has moved through all positions
        move_started: emitted when the manipulator starts moving to any position
        move_completed: emiited when manipulator arrives at new position
        errors: emitted on exceptions
    """
    started = pyqtSignal()
    finished = pyqtSignal()
    move_completed = pyqtSignal()
    move_started = pyqtSignal()
    errors = pyqtSignal(Exception)

    def __init__(self, mdl:ManipulatorModel, cal:Calibrator, img_w:int, img_h:int, z_polarity:int, z_scaling:float, pip_angle:float, obj_mag:float, opto_mag:float, delta_nm:float=50000, speed_ums:int=1000):
        """
        Arguments:
            mdl (ManipulatorModel): Object that manipulates the sensapex manipulator
            cal (Calibrator): Object of micromanipulator <-> external calibraiton
            img_w (int): Pixel width of screen/image
            img_h (int): Pixel height of screen/image
            z_polarity (int): Polarity between focus and man. z. +1 if same direction, else -1
            z_scaling (float): Ratio of microscope z to manipulator z (microscope z/ manipulator z)
            pip_angle (float): manipulator angle in radians
            obj_mag (float): Current objective magnification
            opto_mag (float): Current optovar magnification
        """
        super().__init__()
        self._logr = StandardLogger(__name__)
        self.mdl = mdl
        self.mdl.move_started.connect(self._on_move_started)
        self.mdl.move_completed.connect(self._on_move_completed)
        self.started.connect(self._on_started)
        self.finished.connect(self._on_finished)
        self.finished.connect(self.deleteLater)
        self.cal = cal
        self.speed_ums = speed_ums
        self.z_polarity = z_polarity
        self.z_scaling = z_scaling
        self.pip_angle = pip_angle
        self.obj_mag = obj_mag
        self.opto_mag = opto_mag
        self._move_index = 0
        self._active = False
        self._available = True
        self._auto = False
        # Sequential manipulator displacements for prelim calibraiton
        # aka the manipulator will be dispalced from the current position by these amoutns
        self._man_disps = [[-delta_nm/2, -delta_nm, 0, 0],
                           [delta_nm, 0, 0, 0]]
        # Pixel positions for secondary calibration
        w = img_w
        dw = img_w*0.15
        h=img_h
        dh=img_h*0.15
        self._ex_positions = [[0 + dw, 0 + dh],
                              [w - dw, 0 + dh],
                              [w - dw, h - dh],
                              [0 + dw, h - dh],
                              [w/2, h/2]]
        self._man_positions = []
        # Wipe calibration data to start with clean slate
        self.cal.data.rm_all()

    @pyqtSlot()
    def _on_move_completed(self):
        """
        When manipulator finish move, emit signal that move is finished
        and set `_avialable` to true
        """
        # wait before emitting move_complete to give time for video latency
        # Qtest.qWait would cause crash if trajectory was deleted during the
        # qWait period and time.sleep is blocking. QTImer solved the problem
        # and allows proper use of deleteLater()
        QTimer.singleShot(250, self._emit_move_completed)

    def _emit_move_completed(self):
        # _available must come before _move_completed, so that something acting
        # on the _move_completed signal and checks is_available gets right status
        self._available = True
        self.move_completed.emit()

    @pyqtSlot()
    def _on_move_started(self):
        """
        When manipulator start move, emit signal that move is started
        and set `_avialable` to false
        """
        # _available must come before _move_started, so that something acting
        # on the _move_started signal and checks is_available gets right status
        self._available = False
        self.move_started.emit()

    @pyqtSlot()
    def _on_started(self):
        """ When trajectory is started, set `_active` to True """
        self._active = True

    @pyqtSlot()
    def _on_finished(self):
        """ When trajectory is started, set `_active` to False """
        self._active = False
        self._move_index = 0
        self._man_positions = []
    
    def is_active(self):
        """ Return whether the calibration trajectory is active (it has been started) """
        return self._active

    def is_available(self):
        """
        Return whether the calibration trajectory is availble (it is stopped at a position
        and can accept a move command)
        """
        return self._available

    def is_auto(self):
        """ True if `_auto` is true (the calibration is controlled by auto calibratoin) """
        return self._auto

    def make_auto(self):
        """ Makes `is_auto()` return True """
        self._auto = True

    def next_cal_position(self):
        """
        Move to the next calibration postition.

        Starts by registering 3 points for preliminary calibration (the first clicked point
        and the 2 `_man_disps`). Then computes preliminary calibration, uses calibration to
        compute secondary calibraiton positoins of manipulator near FOV corners and center.
        Waits for user to click those positiosn as it automatically moves to those points.
        """
        try:
            # Compute calibration if have reached first 3 calibraiton points
            if self._move_index == len(self._man_disps):
                # Dont save temp calibration model
                self.cal.compute(z_polarity=self.z_polarity, z_scaling=self.z_scaling, pip_angle=self.pip_angle, obj_mag=self.obj_mag, opto_mag=self.opto_mag, save=False)
                # COmpute secondary calibration positiosn
                ex_z = np.mean(self.cal.data.data_df['ex_z'].to_numpy())
                self.compute_man_positions(ex_z=ex_z)
                # reset calibration after positoins are computed
                self.cal.data.rm_all()
                self.cal.model.reset_calibration()
            # Emit finished when all points have been reached
            if self._move_index == (len(self._man_disps) + len(self._ex_positions)):
                self.finished.emit()
                return None
            # Prelimnary calibration with small displacements around start positoin
            if self._move_index < len(self._man_disps):
                if self._move_index == 0:
                    self.started.emit()
                cur_pos = self.mdl.get_pos()
                disp = self._man_disps[self._move_index]
                new_pos = [axis_pos + disp[axis] for axis, axis_pos in enumerate(cur_pos)]
                self.mdl.goto_pos(new_pos, self.speed_ums)
            # Secondary calibration with moving to the corners and center
            else:
                tmp_index = self._move_index - len(self._man_disps)
                new_man_pos = self._man_positions[tmp_index]
                self.mdl.goto_pos(new_man_pos, self.speed_ums)
            # Increment move index
            self._move_index += 1
        except Exception as e:
            self.errors.emit(e)

    def compute_man_positions(self, ex_z:float)-> list:
        """
        Using the preliminary calibration, and the focus position `ex_z`, compute
        the secondary calibration positoins in manipulator coordinates

        Args:
            ex_z (float): Focal plane position of secondary calibration

        Returns:
            list of manipulator coordinates as [[x, y, z, d], ...]
        """
        # Generate calibration positions
        for pos_ind, pix_pos in enumerate(self._ex_positions):
            ex_pos = pix_pos.append(ex_z)
            new_man_pos = self.cal.model.inverse(ex=pix_pos, man_axis_const='d')
            self._man_positions.append(new_man_pos)

class AutofocusTrajectory(QObject):
    """
    Performs autofocussing of pipette tip by soliciting focus detection from main GUI
    (via a signal) and moving the manipulator towards "in-focus" based on focus detection.
    Reduces movement increment by 50% if over shoot "in-focus"
    
    Signals:
        finished: emitted when no longer doing autofocus (regardless if it fails or succeeds)
        failed: exceeded when it could not autofocus the tip
        succeeded: emitted when the focus detection was "in-focus"
        ask_for_focus: emitted when needs new focus detection to move manipulator
        errors: emitted on exceptions
    """
    finished = pyqtSignal()
    failed = pyqtSignal()
    succeeded = pyqtSignal()
    ask_for_focus = pyqtSignal()
    errors = pyqtSignal()

    def __init__(self, mdl:SensapexDevice, z_polarity:int):
        super().__init__()
        self._logr = StandardLogger(__name__)
        self.mdl = mdl
        self.z_polarity = z_polarity
        self.UM2NM = 1000
        self.speed_ums = 1000
        self.disp_limit = 250*self.UM2NM
        self.move_limit = 25
        self.time_limit = 10000
        self._initialize()
        self.timer = QTimer()
        self.timer.timeout.connect(self._autofocus)
        self.timeout = 200

    def _initialize(self):
        """ INitialize autofocus parameters """
        self.t0 = time.time()
        self.inc = 15*self.UM2NM
        self._move_index = 0
        self.history = {'focus':[], 'disp':[], 'pos':[]}
        self.disp = 0
        self.waiting_for_focus = True
        self.focus = None
    
    def start(self):
        """ 
        Runs the autofocus procedure.

        Starts a timer to periodically run `_autofocus` function and requests
        first focus detection.
        """
        self._initialize()
        # Start autofocus loop by calling timer to periodically run `_autofocus`
        if self.timer.isActive() is False:
            self.timer.start(self.timeout)
        # Query the initial focus detection
        self._request_focus()

    def stop(self):
        """
        Stops the autofocus procedure.

        Stops the timer running the `_autofocus` function and emit `finished`
        """
        if self.timer.isActive():
            self.timer.stop()
        self.finished.emit()
        self.failed.emit()
    
    def _finish_as_fail(self):
        """ Stop looping autofocus procedure and emit `failed` and `finished`"""
        self.timer.stop()
        self.failed.emit()
        self.finished.emit()
    
    def _finish_as_success(self):
        """ Stop looping autofocus procedure and emit `succeeded` and `finished`"""
        self.timer.stop()
        self.succeeded.emit()
        self.finished.emit()

    def _are_limits_reached(self) -> bool:
        """
        Evaluate whether the number of autofocus moves, total movement displacement,
        or the time duration is exceeded
        """
        if self._move_index > self.move_limit:
            return True
        elif abs(self.disp) > self.disp_limit:
            return True
        elif (time.time() - self.t0) > self.time_limit:
            return True
        else:
            return False

    def _autofocus(self):
        """ Autofocus procedure that is continuously looped with a timer """

        try:
            # Stop looping if # moves, total distance, or total time excceeded
            if self._are_limits_reached():
                self._finish_as_fail()
                return

            # Evaluate focus after manipulator done moving and recieve focus detection
            if not self.waiting_for_focus and not self.mdl.is_moving():
                focus = self.focus

                # Stop autofocus procedure if in focus
                if focus == 'in':
                    self._finish_as_success()

                # Move manipulator towards "in-focus" based on current focus detection
                else:
                    # Dont make another move if it will exceed the movement limits because
                    # the focus level of the movement won't be detected (since we stop the
                    # loop at the beginning of this function if limits exceeded)
                    if self._move_index + 1 > self.move_limit:
                        self._finish_as_fail()
                    else:
                        self.adjust_inc(focus)
                        self.move_manip(focus)
                        # Wait to give time for video latency before query another detection
                        # The wait time must be shorter than the looping duration, so it can
                        # recieve the focus detection before the next loop
                        QTimer.singleShot(100, self._request_focus)

        except Exception as e:
            self.errors.emit(e)

    def _request_focus(self):
        '''
        Emit signal to request focus detection and set attribute indicating
        that we are waiting for a response.
        '''
        self.waiting_for_focus = True
        self.ask_for_focus.emit()  
                    
    def recieve_focus(self, focus:str):
        """
        Set the focus value and indicator that it is recieved

        Args:
            focus (str): focus level as 'below', 'in', or 'above'
        """
        self.focus = focus
        self.waiting_for_focus = False
    
    def adjust_inc(self, focus):
        """
        Adjust the manipulator movement increment.

        Reduces increment by 50% if current focus detection does not match
        the previous detection

        Args:
            focus (str): focus level as 'below', 'in', or 'above'
        """
        if len(self.history['focus'])>0:
            if focus != self.history['focus'][-1]:
                self.inc  = self.inc * 0.5

    def move_manip(self, focus):
        """
        Move manipulator to new postion and add info to history

        Args:
            focus (str): focus level as 'below', 'in', or 'above'
        """
        if focus == 'above':
            direction = -1
        elif focus == 'below':
            direction = 1
        disp = direction * self.inc * self.z_polarity
        pos = self.mdl.get_pos()
        new_pos = pos[:]
        new_pos[2] += disp
        move_req = self.mdl.goto_pos(new_pos, self.speed_ums)
        self.history['focus'].append(focus)
        self.history['disp'].append(disp)
        self.history['pos'].append(pos[2])
        self.disp += disp
        self._move_index += 1
""" Scripts/classes for coordinating manipulator trajectories """
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
        finished: emitted when trajectory has moved through all positions
        move_completed: emiited when manipulator arrives at new position
    """
    started = pyqtSignal()
    finished = pyqtSignal()
    move_completed = pyqtSignal()
    move_started = pyqtSignal()
    errors = pyqtSignal(Exception)

    def __init__(self, mdl:ManipulatorModel, cal:Calibrator, img_w:int, img_h:int, z_polarity:int, pip_angle:float, obj_mag:float, opto_mag:float, delta_nm:float=50000, speed_ums:int=1000):
        """
        Arguments:
            mdl (ManipulatorModel): Object that manipulates the sensapex manipulator
            cal (Calibrator): Object of micromanipulator <-> external calibraiton
            img_w (int): Pixel width of screen/image
            img_h (int): Pixel height of screen/image
            z_polarity (int): Polarity between focus and man. z. +1 if same direction, else -1
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
                self.cal.compute(z_polarity=self.z_polarity, pip_angle=self.pip_angle, obj_mag=self.obj_mag, opto_mag=self.opto_mag, save=False)
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

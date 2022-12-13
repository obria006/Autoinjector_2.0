"""
WORK IN PROGRESS trajectories to enable z-scalibration for `trajectories.py`. 
This z-calibration trajectroy with the additional WIP calibration functions
in `WIP_calibraiton.py` would enable 3D calibration.
"""
from PyQt6.QtCore import QObject, pyqtSignal, QTimer, pyqtSlot
from src.trajectories import ManipulatorModel
from src.manipulator_control.calibration import Calibrator
from src.miscellaneous.standard_logger import StandardLogger

class ZCalibrationTrajectory(QObject):
    """
    Performs Z calibration trajectory.
    
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
        self._man_disps = [[0, 0, -delta_nm, 0],
                           [0, 0, -delta_nm*2, 0],
                           [0, 0, -delta_nm*3, 0],
                           [0, 0, -delta_nm*4, 0]]
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
            # Emit finished when all points have been reached
            if self._move_index == len(self._man_disps):
                # Save calibration
                self.cal.compute_z(z_polarity=self.z_polarity, pip_angle=self.pip_angle, obj_mag=self.obj_mag, opto_mag=self.opto_mag, save=False)
                # reset calibration after positoins are computed
                self.cal.data.rm_all()
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
            # Increment move index
            self._move_index += 1
        except Exception as e:
            self.errors.emit(e)
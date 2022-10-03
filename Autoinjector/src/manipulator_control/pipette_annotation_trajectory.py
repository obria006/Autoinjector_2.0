""" Handles control of the micromanipulator to perform automatic annotation of
the pipette tip for generating data to train n tip detection neural network """
from PyQt6.QtCore import QObject,  pyqtSignal, QTimer
import numpy as np
from src.manipulator_control.calibration import Calibrator
from src.manipulator_control.sensapex_utils import SensapexDevice, UMP
from src.miscellaneous.standard_logger import StandardLogger

class PipetteAnnotationTrajectory(QObject):
    finished = pyqtSignal()
    move_finished = pyqtSignal()
    next_fov_pos = pyqtSignal([int, int, float])
    progress = pyqtSignal(float)

    def __init__(self, dev:SensapexDevice, cal:Calibrator, img_w:int, img_h:int, in_focus_foc_z:float, in_focus_man_z:float, num_poses:int):
        """
        Arguments:
            dev (SensapexDevice): Object that manipulates the sensapex manipulator
            cal (Calibrator): Object of micromanipulator <-> external calibraiton
            img_w (int): Pixel width of screen/image
            img_h (int): Pixel height of screen/image
            in_focus_foc_z (float): Height of focus controller where pipette tip is infocus
            in_focus_man_z (float): Z height of the manip when in focus
            num_poses (float): Number of annotations to perform
        """
        super().__init__()
        self._logr = StandardLogger(__name__)
        self.dev = dev
        self.cal = cal
        self.img_w = img_w
        self.img_h = img_h
        self.dz_std = 15
        self.speed_ums = 1000
        self.in_focus_foc_z = in_focus_foc_z
        self.in_focus_man_z = in_focus_man_z
        if num_poses < 1:
            raise ValueError(f"Invalid number of annotation positions: {num_poses}. Must be greater than 1")
        self.num_poses = num_poses
        self._move_index = 0
        self._timer = QTimer()
        self._timer.timeout.connect(self.wait_for_move_to_finish)


    def random_fov_xyz(self, dz_std:int)->list:
        """
        Generate random x,y,z coordinates. X and Y are uniform randomly sampled
        from anywhere within the the center 85%  of th eFOV. Z is sampled from a
        gaussian distribution from the "in-focus" z of the pipette with standard
        deviation `dz_std`.

        Args:
            dz_std (float): Standard deviation of delta z from infocus
        """
        assert dz_std>0
        x_min = self.img_w * 0.075
        x_max = self.img_w - x_min
        y_min = self.img_h * 0.075
        y_max = self.img_h - y_min
        x = np.random.randint(int(x_min), int(x_max))
        y = np.random.randint(int(y_min), int(y_max))
        z = np.random.normal(self.in_focus_foc_z, dz_std)
        return [x, y, z]

    def next_position(self):
        """
        Annotation position
        """
        self.progress.emit((self._move_index/self.num_poses)*100)
        if self._move_index < self.num_poses:

            fov_xyz = self.random_fov_xyz(dz_std=self.dz_std)
            self.next_fov_pos.emit(fov_xyz[0],fov_xyz[1],round(fov_xyz[2]-self.in_focus_foc_z,2))
            man_xyz_d = self.cal.model.inverse(ex=fov_xyz,man_axis_const='d')
            # print(f"Focus vs man: {round(fov_xyz[2]-self.in_focus_foc_z,2)} vs. {round(man_xyz_d[2] - self.in_focus_man_z,2)}")
            self.move_req = self.dev.goto_pos(man_xyz_d, self.speed_ums)
            if self._timer.isActive():
                self._logr.warning('Timer already active')
            else:
                self._timer.start()
            self._move_index += 1
        else:
            self.finished.emit()

    def wait_for_move_to_finish(self):
        if self.move_req.finished is True:
            self._timer.stop()
            self.move_finished.emit()

    def stop(self):
        self._timer.stop()
        self.finished.emit()
        
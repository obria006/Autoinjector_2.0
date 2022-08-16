''' Handles manipulator control and calibration for semi-auto and automatic calibration '''
from PyQt6.QtCore import QObject, QThread, pyqtSignal
import numpy as np
import time
from src.manipulator_control.calibration import Calibrator
from src.manipulator_control.sensapex_utils import SensapexDevice, UMP
from src.miscellaneous.standard_logger import StandardLogger 

class SemiAutoCalibrationTrajectoryFORTHREAD(QObject):
    finished = pyqtSignal()

    def __init__(self, dev:SensapexDevice, cal:Calibrator, img_w:int, img_h:int, delta_nm:float=50000, speed_ums:int=1000):
        """
        Arguments:
            dev (SensapexDevice): Object that manipulates the sensapex manipulator
            cal (Calibrator): Object of micromanipulator <-> external calibraiton
            img_w (int): Pixel width of screen/image
            img_h (int): Pixel height of screen/image
            delta_nm (float): Initial change in position for calibration
        """
        super().__init__()
        self._logr = StandardLogger(__name__)
        self.dev = dev
        self.cal = cal
        self._width=img_w
        self._height=img_h
        self.delta_nm = delta_nm
        self.speed_ums = speed_ums
        self._move_complete = False

    def pre_cal_trajectory(self):
        man_disp = [[0, self.delta_nm/2, 0, 0],
                    [-self.delta_nm/2, -self.delta_nm, 0, 0],
                    [self.delta_nm, 0, 0, 0]]
        for ind, disp in enumerate(man_disp):
            self._move_complete = False
            # Relative move to new position
            cur_pos = self.dev.get_pos()
            new_pos = [axis_pos + disp[axis] for axis, axis_pos in enumerate(cur_pos)]
            self.dev.goto_pos(new_pos, self.speed_ums)
            # Wait to move to next position
            while self._move_complete is False:
                time.sleep(0.1)

    def cal_trajectory(self, ex_z:float):
        # Generate calibration positions
        w = self._width
        dw = 0.1*self._width
        h = self._height
        dh = 0.1*self._height
        #TODO insert z axis position into list
        abs_pix_positions = [[0 + dw, 0 + dh, ex_z],
                             [w - dw, 0 + dh, ex_z],
                             [w - dw, h - dh, ex_z],
                             [0 + dw, h - dh, ex_z],
                             [w/2, h/2, ex_z]]
        for pos_ind, pix_pos in enumerate(abs_pix_positions):
            self._move_complete = False
            new_man_pos = self.cal.model.inverse(ex=pix_pos, man_axis_const='d')
            self.dev.goto_pos(new_man_pos, self.speed_ums)
            # Wait to move to next position
            while self._move_complete is False:
                time.sleep(0.1)


    def entire_trajectory(self, z_polarity=-1, pip_angle:float=np.deg2rad(45), obj_mag:float=None, opto_mag:float=None):

        # Conduct intial trajectory for rought estimate of calirbation
        self.pre_cal_trajectory()
        # Get z position
        ex_z = np.mean(self.cal.data.data_df['ex_z'].to_numpy())
        # Compute initial calibration
        self.cal.compute(z_polarity=z_polarity, pip_angle=pip_angle, obj_mag=obj_mag, opto_mag=opto_mag)
        # Reset calibration data
        self.cal.data.rm_all()
        # Conduct larger trajectory for better aclibartion
        self.cal_trajectory(ex_z=ex_z)
        self.finished.emit()

    def move_to_next(self):
        self._move_complete = True

class SemiAutoCalibrationTrajectory(QObject):
    finished = pyqtSignal()

    def __init__(self, dev:SensapexDevice, cal:Calibrator, img_w:int, img_h:int, ex_z:float, z_polarity:int, pip_angle:float, obj_mag:float, opto_mag:float, delta_nm:float=50000, speed_ums:int=1000):
        """
        Arguments:
            dev (SensapexDevice): Object that manipulates the sensapex manipulator
            cal (Calibrator): Object of micromanipulator <-> external calibraiton
            img_w (int): Pixel width of screen/image
            img_h (int): Pixel height of screen/image
            delta_nm (float): Initial change in position for calibration
        """
        super().__init__()
        self._logr = StandardLogger(__name__)
        self.dev = dev
        self.cal = cal
        self.speed_ums = speed_ums
        self.z_polarity = z_polarity
        self.pip_angle = pip_angle
        self.obj_mag = obj_mag
        self.opto_mag = opto_mag
        self._move_index = 0
        self._man_disps = [[0, delta_nm/2, 0, 0],
                           [-delta_nm/2, -delta_nm, 0, 0],
                           [delta_nm, 0, 0, 0]]
        w = img_w
        dw = img_w*0.1
        h=img_h
        dh=img_h*0.1
        self._ex_positions = [[0 + dw, 0 + dh, ex_z],
                              [w - dw, 0 + dh, ex_z],
                              [w - dw, h - dh, ex_z],
                              [0 + dw, h - dh, ex_z],
                              [w/2, h/2, ex_z]]
        self.next_cal_position()

    def next_cal_position(self):
        # Compute calibration if have reached first 3 calibraiton points
        if self._move_index == len(self._man_disps):
            self.cal.compute(z_polarity=self.z_polarity, pip_angle=self.pip_angle, obj_mag=self.obj_mag, opto_mag=self.opto_mag)
            self.cal.data.rm_all()
        # Initial calibration with small displacements
        if self._move_index < len(self._man_disps):
            cur_pos = self.dev.get_pos()
            disp = self._man_disps[self._move_index]
            new_pos = [axis_pos + disp[axis] for axis, axis_pos in enumerate(cur_pos)]
            self.dev.goto_pos(new_pos, self.speed_ums)
        # Calibration with moving to the corners and center
        else:
            tmp_index = self._move_index - len(self._man_disps)
            pix_pos = self._ex_positions[tmp_index]
            new_man_pos = self.cal.model.inverse(ex=pix_pos, man_axis_const='d')
            self.dev.goto_pos(new_man_pos, self.speed_ums)
        # Emit finished when all points have been reached
        if self._move_index == (len(self._man_disps) + len(self._ex_positions) -1):
            self.finished.emit()
        # Increment move index
        self._move_index += 1

                


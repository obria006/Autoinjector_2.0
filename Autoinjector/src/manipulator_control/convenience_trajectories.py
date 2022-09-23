""" Handles auxilliary trajectories of pipette """
from PyQt6.QtCore import QObject, pyqtSignal, QMutex
from src.miscellaneous.standard_logger import StandardLogger
from src.miscellaneous.thread_manager import aQThreader, aQWorker
from src.manipulator_control.error_utils import TrajectoryError
import time

class TrajectoryModel(QObject):
    """
    Handles the micromanipulator control/movements for the convenience
    trajectories.
    """
    moving = pyqtSignal(bool)

    def __init__(self, dev, speed_ums:int=5000):
        """
        Arguments:
            dev (SensapexDevice): Object that manipulates the sensapex manipulator
            speed_ums (int): Speed of pipette movements in um/s
        """
        super().__init__()
        self._mutex = QMutex()
        self.dev = dev
        self.speed_ums = speed_ums
        self._position_history = []
        self.UM_TO_NM = 1000

    def start_unload(self):
        """ Start thread to move micromanipulator to 'unload' position """
        # Generate positions (retract z to 1000um then d to 1000um)
        pos1 = self.dev.get_pos()
        pos1[2] = 1000*self.UM_TO_NM
        pos2 = list(pos1)
        pos2[3] = 1000*self.UM_TO_NM
        poses = [pos1, pos2]
        # Run the trajectory
        self._worker = aQWorker(self._run_trajectory, poses)
        self._threader = aQThreader(self._worker)
        self._threader.start()

    def start_displace(self):
        """ Start thread to move micromanipulator to 'displace' position """
        # Generate positions (retract z by 1000um then x by 8000)
        pos1 = self.dev.get_pos()
        pos1[2] = max(1000*self.UM_TO_NM, pos1[2] - 1000*self.UM_TO_NM)
        pos2 = list(pos1)
        pos2[0] = max(1000*self.UM_TO_NM, pos2[0] - 8000*self.UM_TO_NM)
        poses = [pos1, pos2]
        # Run the trajectory
        self._worker = aQWorker(self._run_trajectory, poses)
        self._threader = aQThreader(self._worker)
        self._threader.start()

    def start_center(self, cal, center_pos:list):
        """
        Start thread to move micromanipulator to 'center' position
        
        Args:
            cal: Pipette calibration model to compute manipulator positoins
            center_pos (list): Center of FOV as [x_pixel, y_pixel, focus_height]
        """
        # Generate positions to move to center
        starting_pos = self.dev.get_pos()
        # Desired manipulator position
        man_center_pos = cal.model.inverse(ex=center_pos, man_axis_const='d')
        # Move z 1000um above desired z
        pos1 = list(starting_pos)
        pos1[2] = max(100*self.UM_TO_NM, man_center_pos[2] - 1000*self.UM_TO_NM)
        # Move to desired positoin with the 1000um z offset
        pos2 = list(man_center_pos)
        pos2[2] = pos1[2]
        # Finally move to desired position
        pos3 = list(man_center_pos)
        poses = [pos1, pos2, pos3]
        # Run the trajectory
        self._worker = aQWorker(self._run_trajectory, poses)
        self._threader = aQThreader(self._worker)
        self._threader.start()

    def start_undo(self):
        """ Start thread to undo a micromanipulator move """
        if len(self._position_history) == 0:
            raise TrajectoryError("Cannot undo micromanipulator move because most recent moves have already been undone.")
        # Generate trajecotry that goes oppoiste the most recent trajectory
        reverse_poses = list(self._position_history[::-1])
        # Run the trajectory
        self._worker = aQWorker(self._run_trajectory, reverse_poses)
        self._threader = aQThreader(self._worker)
        self._threader.start()

    def _run_trajectory(self, positions:list):
        """
        Guide pipette along trajectory of positions

        Args:
            positions (list): List of manipulator positions as [[x1,y1,z1,d1],...]
            type_ (str): String identifier of trajectory type (like 'unload')
        """
        # Emit signal of trajectory started
        self.moving.emit(True)
        # Guide pipette along trajectory
        self._mutex.lock()
        prev_positions = []
        for pos in positions:
            move_req = self.dev.goto_pos(pos, speed = self.speed_ums)
            prev_positions.append([int(round(axis*self.UM_TO_NM)) for axis in move_req.start_pos])
            while move_req.finished is False:
                time.sleep(0.1)
        self._mutex.unlock()
        # Add trajectory to movement history
        self._update_position_history(prev_positions)
        # Emit signal of trajectory complete
        self.moving.emit(False)

    def _update_position_history(self, prev_positions:list):
        """
        Set the trajectory history to `prev_positions`

        Args:
            prev_positions (list): List of manipulator previous positions from oldest to newest
        """
        self._position_history = list(prev_positions)


class ConvenienceTrajectories():
    """
    Class to manage trajectories of convienent control of micromanipulator
    like moving the micromanipulator to the "unload" position, or centering
    the micropipette in the field of view.
    """

    def __init__(self, dev, speed_ums:int=5000):
        """
        Arguments:
            dev (SensapexDevice): Object that manipulates the sensapex manipulator
            speed_ums (int): Speed of pipette movements in um/s
        """
        self.is_moving = False
        self._model = TrajectoryModel(dev, speed_ums)
        self._model.moving.connect(self._update_is_moving)

    def _update_is_moving(self, status:bool):
        """ Sets `is_moving` to `status`.
        
        Args:
            status (bool): Value for `is_moving`
        """
        self.is_moving = status

    def goto_unloaded(self):
        """ Move micromanipulator to the 'unload' position """
        if self.is_moving is True:
            raise TrajectoryError("Cannot move micromanipulator to unload position. Micromanipulator is currently moving.")
        else:
            self._model.start_unload()

    def goto_displaced(self):
        """ Move micromanipulator to the 'displaced' position """
        if self.is_moving is True:
            raise TrajectoryError("Cannot move micromanipulator to displaced position. Micromanipulator is currently moving.")
        else:
            self._model.start_displace()

    def goto_centered(self, cal, center_pos:list):
        """ Move micromanipulator (micropipette) to center of FOV 
        
        Args:
            cal: Pipette calibration model to compute manipulator positoins
            center_pos (list): Center of FOV as [x_pixel, y_pixel, focus_height]
        """
        if self.is_moving is True:
            raise TrajectoryError("Cannot move micromanipulator to centered position. Micromanipulator is currently moving.")
        else:
            self._model.start_center(cal, center_pos)

    def goto_undo(self):
        """ Undo the previous move """
        if self.is_moving is True:
            raise TrajectoryError("Cannot undo micromanipulator move. Micromanipulator is currently moving.")
        else:
            self._model.start_undo()
    
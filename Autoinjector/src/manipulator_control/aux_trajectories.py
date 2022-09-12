""" Handles auxilliary trajectories of pipette """
from PyQt6.QtCore import QObject, pyqtSignal, QMutex
from src.miscellaneous.standard_logger import StandardLogger
from src.thread_manager.thread_manager import aQThreader, aQWorker
from src.manipulator_control.error_utils import TrajectoryError
import time

class SwapTrajectoryModel(QObject):
    unloaded = pyqtSignal()
    reloaded = pyqtSignal()

    def __init__(self, dev, speed_ums:int=1000):
        """
        Arguments:
            dev (SensapexDevice): Object that manipulates the sensapex manipulator
            speed_ums (int): Speed of pipette movements in um/s
        """
        super().__init__()
        self._mutex = QMutex()
        self.dev = dev
        self.speed_ums = speed_ums
        self.UM_TO_NM = 1000
        self.RELOAD_ZOFFSET = 300*self.UM_TO_NM

    def generate_positions(self):
        """
        Generate the manipulator positions for the trajectory.
        
        Trajectory starts at the initial position of the pipette, retracts
        the z to 1000, and retracts the d to 1000 to finish at the unload
        position. User would remove the pipette and request the reload
        position. The pipette would then advance to the original d, and to 
        the original z (with an offset to not collide with stuff in FOV).
        """
        # Initial position
        _starting_pos = self.dev.get_pos()
        # rectract z
        self._unload_1 = list(_starting_pos)
        self._unload_1[2] = 1000 * self.UM_TO_NM
        # retract d
        self._unload_2 = list(self._unload_1)
        self._unload_2[3] = 1000 * self.UM_TO_NM
        # Advance d
        self._reload_1 = list(self._unload_1) 
        # Advance z
        self._reload_2 = list(_starting_pos)
        self._reload_2[2] = max(_starting_pos[2]-self.RELOAD_ZOFFSET, 1000 * self.UM_TO_NM)

    def unload_pipette(self):
        self._worker = aQWorker(self._goto_unload_position)
        self._threader = aQThreader(self._worker)
        self._threader.start()

    def reload_pipette(self):
        self._worker = aQWorker(self._goto_reload_position)
        self._threader = aQThreader(self._worker)
        self._threader.start()

    def _goto_unload_position(self):
        """ directs sensapex movements to unload position """
        self._mutex.lock()
        # Move to first unload position
        move_req = self.dev.goto_pos(self._unload_1, speed = self.speed_ums)
        while move_req.finished is False:
            time.sleep(0.1)
        # move to second unload positoin
        move_req = self.dev.goto_pos(self._unload_2, speed = self.speed_ums)
        while move_req.finished is False:
            time.sleep(0.1)
        self._mutex.unlock()
        # Emit unload signal
        self.unloaded.emit()

    def _goto_reload_position(self):
        """ directs sensapex movements to reload position """
        self._mutex.lock()
        # Move to first reload positoin
        move_req = self.dev.goto_pos(self._reload_1, speed = self.speed_ums)
        while move_req.finished is False:
            time.sleep(0.1)
        # move to second reload positoin
        move_req = self.dev.goto_pos(self._reload_2, speed = self.speed_ums)
        while move_req.finished is False:
            time.sleep(0.1)
        self._mutex.unlock()
        # emti relaod posiiton
        self.reloaded.emit()

class SwapTrajectory():
    """
    Class to manage pipette movements for swapping the pipette. Moves pipette
    automatically between a "unload" and "reload" position where the user can
    easily remove and replace the pipette.

    Attributes:
        is_unloaded (bool): indicator whether pipette has finished move to
            unload position
        is_reloaded (bool): indicator whether pipette has finished move to 
            reload position

    Methods:
        unload_pipette: Moves pipette to unload position
        reload_pipette: Moves pipette to reload positoin
    """

    def __init__(self, dev, speed_ums:int=5000):
        """
        Arguments:
            dev (SensapexDevice): Object that manipulates the sensapex manipulator
            speed_ums (int): Speed of pipette movements in um/s
        """
        self._logger = StandardLogger(__name__)
        self.is_unloaded = False
        self.is_reloaded = False
        self.is_moving = False
        self._model = SwapTrajectoryModel(dev, speed_ums)
        self._model.unloaded.connect(self._trueify_unloaded)
        self._model.reloaded.connect(self._trueify_reloaded)

    def _trueify_unloaded(self):
        """ Sets `is_unloaded` attribute to true and `is_moving` to false """
        self.is_unloaded = True
        self.is_moving = False

    def _trueify_reloaded(self):
        """ Sets `is_reloaded` attribute to true and `is_moving` to false """
        self.is_reloaded = True
        self.is_moving = False

    def unload_pipette(self):
        """ Move the pipette to the 'unload' position """
        if self.is_moving is True:
            raise TrajectoryError("Cannot unload pipette. Pipette currently moving.")
        elif self.is_unloaded is True:
            raise TrajectoryError("Cannot unload pipette. Pipette already unloaded.")
        else:
            self.is_reloaded = False
            self.is_moving = True
            self._model.generate_positions()
            self._model.unload_pipette()

    def reload_pipette(self):
        """ Move the pipette to the 'reload' position """
        if self.is_moving is True:
            raise TrajectoryError("Cannot reload pipette. Pipette currently moving.")
        elif self.is_unloaded is False:
            raise TrajectoryError("Cannot reload pipette. Pipette has not been unloaded.")
        elif self.is_reloaded is True:
            raise TrajectoryError("Cannot relaod pipette. Pipette has already been reloaded")
        else:
            self.is_unloaded = False
            self.is_moving = True
            self._model.reload_pipette()



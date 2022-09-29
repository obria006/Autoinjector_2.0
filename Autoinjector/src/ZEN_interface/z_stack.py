""" Classes/functions for performing a z-stack """
from dataclasses import dataclass, field
from typing import List, Tuple, Optional
import numpy as np
from PyQt6.QtCore import pyqtSignal, pyqtBoundSignal, pyqtSlot, QObject, QTimer
from PyQt6.QtTest import QTest
from src.ZEN_interface.ZEN_mvc import ControllerZEN
from src.miscellaneous.standard_logger import StandardLogger
from src.miscellaneous import validify as val

class ZStackManager(QObject):
    """
    Class for conducting z-stack and constructing image stack object. Uses a
    QTimer to periodically run `_conduct_zstack` for conducting the z-stack.
    This function emits a `ask_for_data` signal to request an image that is
    to be added into the stack via the `set_stack_data`.

    !! IMPORTANT: When instantiating this class, you must make a connection to
    this class's `set_stack_data` (with a signal that emits the current video
    frame, so the image can be added into the stack) !!

    # ---- Example usage to run a z-stack ----
    # Instantiate the zstack manager somewhere in code
    zen = zen_controller # This is an instance of the ControllerZEN class
    z_stack = ZStackManager(zen)
    z_stack.ask_for_data.connect(lambda: z_stack.set_stack_data(send_frame_foo(), send_annotation_foo()))
    z_stack.finished.connect(do_something_after_stack_foo)
    # Somewhere later run the z stack
    start_height = 1000
    stop_height = 1500
    num_slices = 6
    z_stack.run(start_height, stop_height, num_slices)

    Signals:
        ask_for_data: emitted when the z-stack needs a new image for the stack
        finished: emitted when the z-stack is complete Emits ZStackData class.
        progress: emits the progress of the z-stack on scale 0 to 100
        errors: emits exception when exception occurs
    """
    ask_for_data = pyqtSignal()
    finished = pyqtSignal(object)
    progress = pyqtSignal(float)
    errors = pyqtSignal(Exception)

    def __init__(self, zen:ControllerZEN):
        """
        Args:
            zen (ControllerZEN): Zeiss ZEN controller for conducting focus changes
        """
        super().__init__()
        self._logger = StandardLogger(__name__)
        self.is_running = False
        self.zen = zen

    def _init_parameters(self, start:float, stop:float, slices:int):
        """ 
        Initialize the operational parameters for the z-stack 
        
        Args:
            start (float): The focus height to start the z-stack
            stop (float): The focus height to end the z-stack
            slices (int): Number of slices (# of images) for the z-stack
        """
        self.start = start
        self.stop = stop
        self.slices = slices
        self._dz = round((self.stop - self.start) / (self.slices - 1),2)
        self.data = ZStackDataWithAnnotations()
        self._stop_zstack = False
        self._focus_to_next_slice = True
        self._recieved_image = False
        self._slice_ind = 0

    def _validate_args(self, start:float, stop:float, slices:int):
        """
        Validate proper arguments, and return as useable types.

        Args:
            start (float): The focus height to start the z-stack
            stop (float): The focus height to end the z-stack
            slices (int): Number of slices (# of images) for the z-stack
        """
        if not val.is_valid_number(start):
            raise ValueError(f"Invalid z-stack start height: {start}. Must be a valid number.")
        if not val.is_valid_number(stop):
            raise ValueError(f"Invalid z-stack stop height: {stop}. Must be a valid number.")
        if not val.is_valid_number(slices):
            raise ValueError(f"Invalid z-stack number of slices: {slices}. Must be a valid number.")
        if float(slices) != int(slices):
            raise ValueError(f"Invalid z-stack number of slices: {slices}. Must be an integer.")
        if slices <= 1:
            raise ValueError(f"Invalid z-stack number of slices: {slices}. Must be a positive integer greater than 1.")

        return float(start), float(stop), int(slices)

    def run(self, start:float, stop:float, slices:int):
        """
        Public method to run the z-stack procedure

        Args:
            start (float): The focus height to start the z-stack
            stop (float): The focus height to end the z-stack
            slices (int): Number of slices (# of images) for the z-stack
        """
        if self.is_running is True:
            raise NotImplementedError("Cannot start a new z-stack when a z-stack is already running.\n\nIf you want to start a new z-stack, please stop the current z-stack then start a new one.")
        start, stop, slices = self._validate_args(start, stop, slices)
        self._init_parameters(start, stop, slices)
        self._timer = QTimer()
        self._timer.timeout.connect(self._conduct_zstack)
        TIMEOUT_MS = 250
        self._timer.start(TIMEOUT_MS)

    def stop(self):
        """ Public method to stop the z-stack procedure """
        self._stop_zstack = True

    def _conduct_zstack(self):
        """
        Private method called internally to run the z-stack procedure.

        Z-stack procedure consists of:
            1. Evaluate if all slices have been imaged
                i. If yes, then finish the z-stack
            2. Make a move to the next position
            3. Ask for image frame at the 'new' position by emit signal
            4. Wait for image to be sent from main GUI
            5. Recieve the image
            6. Add the image to the stack
            7. Increment the slice counter
            8. Repeat
        """
        try:
            self.is_running=True
            self.progress.emit((self._slice_ind/self.slices)*100)
            if self._is_zstack_complete() is True or self._stop_zstack is True:
                self._finish_zstack()
                return
            if self._focus_to_next_slice is True:
                self._focus_at_next()
                self._focus_to_next_slice = False
                self._recieved_image = False
                # Add non-blocking delay before asking for image, so camera has time to update 
                # to image the new focal plane
                QTest.qWait(500)
                self.ask_for_data.emit()
                return
            if self._recieved_image is True:
                self._add_to_stack(self._tmp_image, self._tmp_annotation)
                self._focus_to_next_slice = True
                self._slice_ind = self._slice_ind + 1
                return
        except Exception as e:
            # Need to emit error here because this isn't called by main GUI,
            # so we need to send the error to main GUI so it can be handled
            msg = f"Error during z-stack: {e}. See logs for more info."
            self._logger.exception(msg)
            self.errors.emit(msg)
            self._silent_finish()
        except:
            # Need to emit error here because this isn't called by main GUI,
            # so we need to send the error to main GUI so it can be handled
            err = RuntimeError('Critical error during z-stack. See logs for more info.')
            self.errors.emit(err)
            self._logger.exception(err)
            self._silent_finish()

    def _is_zstack_complete(self)->bool:
        """
        Returns True if the z-stack is done. (All slices have been imaged as
        indicated by the slice index matching the total # of slices)
        """
        return self._slice_ind >= self.slices

    def _focus_at_next(self):
        """
        Command the Zeiss ZEN focus controller to go to the next position.
        If imaging the first, slice, then send the focus to the starting
        position, otherwise make relative move to the new slice position
        """
        cur_pos = round(self.zen.get_focus_um(),2)
        if self._slice_ind == 0:
            self.zen.goto_focus_absolute(self.start)
        else:
            self.zen.goto_focus_relative(self._dz)
    
    @pyqtSlot(np.ndarray)
    def set_stack_data(self, image:np.ndarray, annotation:Optional[list[float,float]] = None):
        """
        Sets image and optionally the taret annotation for z-stack

        Args:
            image (np.ndarray): Image to add to stack
            annotation (list): List of annotated target coordinates as [[x, y],...]
        """
        self._tmp_image = image
        self._tmp_annotation = annotation
        self._recieved_image = True

    def _add_to_stack(self, image:np.ndarray, annotation:Optional[list[float,float]] = None):
        """
        Add image and target annotation to z-stack. (Appends to images list)

        Args:
            image (np.ndarray): Image to add to stack
            annotation (list): List of annotated target coordinates as [[x, y],...]
        """
        z  = self.zen.get_focus_um()
        self.data.append(image, z, annotation)

    def _silent_finish(self):
        """ Finish the z-stack without emitting the finished signal """
        self._timer.stop()
        self.is_running=False

    def _finish_zstack(self):
        """ Stops the z-stack procedure, makes the z-stack, and emits the finished signal """
        self._timer.stop()
        self.is_running=False
        self.finished.emit(self.data)

@dataclass
class ZStackData:
    images: List[np.ndarray] = field(default_factory=list)
    z_heights: List[float] = field(default_factory=list)

    def append(self, image:np.ndarray, z:float):
        """
        Append image and its associated focus-level/z-height to data attributes.

        Args:
            image (np.ndarray): Image to append
            z (float): Focus level of image to append
        """
        if not isinstance(image, np.ndarray):
            raise TypeError(f"Invalid image type: {type(image)}. Must be np.ndarray.")
        if not val.is_valid_number(z):
            raise ValueError(f"Invalid z-height: {z}. Must be a valid number.")
        self.images.append(np.copy(image))
        self.z_heights.append(z)

    def is_empty(self)->bool:
        """
        Returns true if there is no data in the z-stack (no images exists)
        """
        return len(self.images) == 0

    def get_data_from_index(self, ind:int) -> Tuple[np.ndarray, float]:
        """
        Returns image and its associated z-height for the slice index

        Args:
            ind (int): Slice index of image/z-height to return

        Returns:
            image (np.ndarray): Image at index
            z (float): Z-height of image
        """
        if ind > len(self.images) - 1 or ind > len(self.z_heights) - 1 :
            raise KeyError(f"Invalid index for z-stack data: {ind}. Must be in 0 - {len(self.images) - 1}")
        return np.copy(self.images[ind]), self.z_heights[ind]

    def get_image_from_exact_z(self, z:float) -> Tuple[np.ndarray, int]:
        """
        Returns image and its associated index from the specific z-height

        Args:
            z (float): z-height to query for the associated image

        Returns:
            image (np.ndarray): Image assoicated with z-height
            ind (int): Index of image and focus
        """
        if z not in self.z_heights:
            raise KeyError(f"Invalid z-height: {z}. Z-height not found in existing z-heights.")
        ind = self.z_heights.index(z)
        image = np.copy(self.images[ind])
        return image, ind

    def get_image_from_nearest_z(self, z:float) -> Tuple[np.ndarray, float, int]:
        """
        Returns image, its exact z, and its associated index nearest the z-height

        Args:
            z (float): z-height to query for the associated image

        Returns:
            image (np.ndarray): Image assoicated with z-height
            ind (int): Index of image and focus
        """
        # Compute difference between all z and passed z and find index of
        # smallest difference (aka the nearest z)
        z_arr = np.asarray(self.z_heights)
        dif_z = np.abs(z_arr - z)
        ind = int(np.argmin(dif_z))
        ind = self.z_heights.index(z)
        image = np.copy(self.images[ind])
        true_z = self.z_heights[ind]
        return image, true_z, ind

    def get_images_as_stack(self) -> np.ndarray:
        """
        Returns all images as a np.ndarray stack with shape
        (num images x rows x columns)
        """
        stack = np.stack(self.images)
        return stack

@dataclass
class ZStackDataWithAnnotations(ZStackData):
    annotations: list[list[float, float]] = field(default_factory=list)

    def append(self, image:np.ndarray, z:float, annotation:Optional[list[float,float]] = None):
        """
        Append image and its associated focus-level/z-height to data attributes.

        Args:
            image (np.ndarray): Image to append
            z (float): Focus level of image to append
            annotation (list): List of annotated target coordinates as [[x, y],...]
        """
        if not isinstance(image, np.ndarray):
            raise TypeError(f"Invalid image type: {type(image)}. Must be np.ndarray.")
        if not val.is_valid_number(z):
            raise ValueError(f"Invalid z-height: {z}. Must be a valid number.")
        if annotation is not None:
            if not isinstance(annotation, list):
                raise TypeError(f"Invalid annotaiton type: {type(annotation)}. Must be a list.")
        super().append(image, z)
        self.annotations.append(annotation)

    def get_data_from_index(self, ind:int) -> Tuple[np.ndarray, float]:
        """
        Returns image and its associated z-height and annotation for the slice index

        Args:
            ind (int): Slice index of image/z-height to return

        Returns:
            image (np.ndarray): Image at index
            z (float): Z-height of image
            annotation (list): List of annotated target coordinates as [[x, y],...]
        """
        image, z = super().get_data_from_index(ind)
        tmp_annot = self.annotations[ind]
        # Only return new list instance of annotation if the annotation is not NOne
        # otherwise will raise an error that NoneType object is not iterable
        if tmp_annot is None:
            annotation = None
        else:
            annotation = list(tmp_annot)
        return image, z, annotation

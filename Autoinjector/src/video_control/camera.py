""" 
Classes and functions for interfacing with camera devices and outputing
captured frames.
"""
import os
import time
import pymmcore
from skimage.util import img_as_ubyte
import cv2
import numpy as np
from PyQt6.QtCore import QObject, QTimer, pyqtSignal
import src.video_control.video_utils as utils
from src.miscellaneous.standard_logger import StandardLogger


class MMCamera():
    """ Interface for MicroManager camera. Permits functionality to get and set camera exposure,
    get image frames from capture, and stop camera. """
    
    def __init__(self,mm_path:str,cam:str,brand:str,val:str,bins:str,rot:int,imagevals:int):
        """
        Arguments:
            mm_path (str): Path to micromanager installation (commonly C:/Program Files/Micro-Manager 2.0)
            cam (str): Name of camera
            brand (str): Brand of camera
            val (str): Device name of camera?
            bins (str): Type of binning
            rot (int): Rotation of caputred image before displaying (degrees)
            imagevals (int): Number of values pixels can take (aka 2^bits)
        """
        self._logger = StandardLogger(__name__)
        #defines camera settings
        self.mm_path = mm_path
        if not os.path.exists(self.mm_path):
            raise utils.CameraError(f"Invalid MicroManager path: {self.mm_path}")
        self.cam = cam
        self.brand = brand
        self.val = val
        self.bins = bins
        self.rot = int(rot)
        self.imagevals = imagevals
        # Start interface with camera
        self._setup_camera()
        self.height, self.width = self._get_image_size()

    def _setup_camera(self):
        """
        Load the camera device and start image acquisition.
        """
        self.cap = pymmcore.CMMCore()
        self.cap.setDeviceAdapterSearchPaths([self.mm_path])
        self.cap.loadDevice(self.cam,self.brand,self.val)
        self.cap.initializeAllDevices()
        self.cap.setCameraDevice(self.cam)
        if self.bins != "none":
            self.cap.setProperty(self.cam, "Binning", self.bins)
        self.cap.startContinuousSequenceAcquisition(1)

    def _get_image_size(self):
        """
        Attempt to read image from camera and get width/height. If can't
        read image in 1 second, raise error

        Raises:
            CameraError if cannot get frame from camera.
        """
        t0 = time.time()
        frame = None
        # Try to get frame from camera
        while frame is None:
            # Raiser error if can't acquire a frame within 1 second
            if time.time() - t0 > 1:
                raise utils.CameraError("Could not establish interface with camera")
            # Try getting a frame, but if no frame available then just keep looping
            try:
                frame = self.get_frame()
            except utils.NoFrameError:
                pass
            except:
                self._logger.exception("Error with camera")
                raise
        # Return image height/width after getting a frame
        height = frame.shape[0]
        width = frame.shape[1]
        return height, width

    def get_exposure(self)->float:
        """
        Get the exposure property from the camera.

        Returns:
            float of camera's exposure value
        """
        return float(self.cap.getProperty(self.cam, 'Exposure'))

    def set_exposure(self, exposure:float):
        """
        Set the exposure property for the camera

        Args:
            exposure (float): Camera exposure to set (artificial units)
        """
        try:
            self.cap.setProperty(self.cam, 'Exposure', float(exposure))
        except:
            self._logger.exception("Error while setting camera exposure")
            raise

    def get_frame(self)->np.ndarray:
        """
        Get a image frame from the camera

        Raises:
            NoFrameError if no images available to stream

        Returns:
            np.ndarray of image from the camera.
        """
        #if video is streaming
        if self.cap.getRemainingImageCount() > 0:
            try:
                frame = self.cap.getLastImage()
            except IndexError as e:
                raise utils.NoFrameError(e)
            except:
                raise
            frame = img_as_ubyte(frame) #convert to 8 bit from 16 bit

            if self.rot > 0: #rotate 
                rows,cols = frame.shape 
                M = cv2.getRotationMatrix2D((cols/2,rows/2),self.rot,1)
                frame = cv2.warpAffine(frame,M,(cols,rows))
            return np.copy(frame)
        else:
            raise utils.NoFrameError('No frame available from camera.')

    def stop(self):
        """
        Stop the continuous camera acquisition.
        """
        self.cap.stopSequenceAcquisition()


class VideoStreamer(QObject):
    """
    Class for handling continous streaming of image frames from camera and
    making these frames available for display. Uses QTimer to periodically
    fetch frames from camera at distinct intervals and emits the image frame.

    Signals:
        new_frame_available(np.ndarray): emits the captured image frame
        new_frame_unavailable(): emitted when no frame available when tried to get frame
    """

    new_frame_available = pyqtSignal(np.ndarray)
    new_frame_unavailable = pyqtSignal()

    def __init__(self, cam:MMCamera, fps:int):
        """
        Args:
            cam (MMCamera): Camera object w/ get_frame() method to return np.ndarray of image
            fps (int): Desired frame rate of video stream
        """
        super().__init__()
        self._logger = StandardLogger(__name__)
        self._cam = cam
        S_TO_MS = 1000
        sampling_period_ms = int((1/fps) * S_TO_MS)
        self._timer = QTimer()
        self._timer.timeout.connect(self._stream_new_frame)
        self._timer.start(sampling_period_ms)

    def _stream_new_frame(self):
        """
        Get new frame from camera and emit the captured frame.

        Emits `new_frame_available(np.ndarray)` if image frame acquired from camera
        otherwise emits `new_frame_unavailable()`
        """
        try:
            frame = self._cam.get_frame()
        except utils.NoFrameError as e:
            self._logger.warning(e)
            self.new_frame_unavailable.emit()
        except Exception as e:
            self._logger.exception("Error while streaming new frame")
            raise
        else:
            self.new_frame_available.emit(frame)

    def stop(self):
        self._timer.stop()
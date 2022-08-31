""" Classes and functions to handle video streaming """
import os
import time
import sys
import pymmcore
import cv2
import numpy as np
from skimage.util import img_as_ubyte
from PyQt6.QtCore import (Qt,
                          QObject,
                          QTimer,
                          pyqtSignal,
                          QThread
                          )
from PyQt6.QtWidgets import (QApplication,
                            QLabel,
                            QVBoxLayout,
                            QWidget,
                            QLineEdit,
                            )
from PyQt6.QtGui import QImage, QPixmap
from src.video_control import video_utils as utils
from src.miscellaneous.standard_logger import StandardLogger


class MMCamera():
    """ Interface for MicroManager camera """
    
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
        Start video stream from camera
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
        """
        t0 = time.time()
        frame = None
        while frame is None:
            if time.time() - t0 > 1:
                raise utils.CameraError("Could not establish interface with camera")
            try:
                frame = self.get_frame()
            except utils.NoFrameError:
                pass
            except:
                self._logger.exception("Error with camera")
                raise
        height = frame.shape[0]
        width = frame.shape[1]
        return height, width

    def get_exposure(self):
        return self.cap.getProperty(self.cam, 'Exposure')

    def set_exposure(self, exposure):
        try:
            self.cap.setProperty(self.cam, 'Exposure', float(exposure))
        except:
            self._logger.exception("Error while setting camera exposure")
            raise

    def get_frame(self):

        #if video is streaming
        if self.cap.getRemainingImageCount() > 0:
            frame = self.cap.getLastImage()
            frame = img_as_ubyte(frame) #convert to 8 bit from 16 bit

            if self.rot > 0: #rotate 
                rows,cols = frame.shape 
                M = cv2.getRotationMatrix2D((cols/2,rows/2),self.rot,1)
                frame = cv2.warpAffine(frame,M,(cols,rows))
            return np.copy(frame)
        else:
            raise utils.NoFrameError('No frame available from camera.')

    def stop(self):
        self.cap.stopSequenceAcquisition()

class VideoDisplay(QWidget):
    """ Class for handling streaming from camera and displaying in GUI """

    clicked_camera_pixel = pyqtSignal(list)
    clicked_canvas_pixel = pyqtSignal(list)
    drawn_camera_pixels = pyqtSignal(list)

    def __init__(self, cam:MMCamera, height:int, fps:int):
        super().__init__()
        self.cam = cam
        self.streamer = VideoStreamer(self.cam, fps)
        self._camera_to_canvas_scaling = height/self.cam.height
        self.height = int(round(height))
        self.width = int(round(self.cam.width * self._camera_to_canvas_scaling))
        self.moved_camera_pixels = []
        self._make_widgets()
        self.streamer.new_frame_available.connect(self.set_new_frame)

    def _make_widgets(self):
        self.canvas = Canvas(self.width, self.height)
        self.canvas.mouse_pressed_pixel.connect(self.handle_mouse_press)
        self.canvas.mouse_released_pixel.connect(self.handle_mouse_release)
        self.canvas.mouse_moved_pixel.connect(self.handle_mouse_move)
        layout = QVBoxLayout()
        layout.addWidget(self.canvas)
        self.setLayout(layout)

    def set_new_frame(self, frame:np.ndarray):
        self.frame = frame
        self.update_display()

    def handle_mouse_press(self,canvas_pixel):
        camera_pixel = self.convert_canvas_to_camera(canvas_pixel)
        self.pressed_camera_pixel = camera_pixel
        self.moved_camera_pixels = []

    def handle_mouse_release(self,canvas_pixel):
        camera_pixel = self.convert_canvas_to_camera(canvas_pixel)
        self.clicked_camera_pixel.emit([camera_pixel[0], camera_pixel[1]])
        self.clicked_canvas_pixel.emit([canvas_pixel[0], canvas_pixel[1]])
        self.drawn_camera_pixels.emit(self.moved_camera_pixels)

    def handle_mouse_move(self,canvas_pixel):
        camera_pixel = self.convert_canvas_to_camera(canvas_pixel)
        self.moved_camera_pixels.append(camera_pixel)

    def update_display(self):
        # Convert camera image to rgb image for display
        canvas_frame = np.copy(self.frame)
        self.canvas.update_(canvas_frame)

    def show_tip_position(self, bool_:bool):
        self.canvas.painter.show_calibrated_tip_points(bool_)

    def set_tip_position(self, x, y):
        camera_pixel = [x,y]
        canvas_pixel = self.convert_camera_to_canvas(camera_pixel)
        self.canvas.painter.calibrated_tip_points = [canvas_pixel]

    def show_drawn_annotation(self, bool_:bool):
        self.canvas.painter.show_drawn_edge(bool_)

    def show_interpolated_annotation(self, bool_:bool):
        self.canvas.painter.show_interpolated_edge(bool_)

    def set_interpolated_annotation(self, annot):
        canvas_annot = [self.convert_camera_to_canvas(pix) for pix in annot.tolist()]
        self.canvas.painter.interpolated_edge = canvas_annot

    def reset_interpolated_annotation(self):
        self.canvas.painter.interpolated_edge = []

    def convert_canvas_to_camera(self, canvas_pixel):
        cam_pixel = [round(int(pix/self._camera_to_canvas_scaling)) for pix in canvas_pixel]
        return cam_pixel

    def convert_camera_to_canvas(self, cam_pixel):
        canvas_pixel = [round(int(pix*self._camera_to_canvas_scaling)) for pix in cam_pixel]
        return canvas_pixel

    def closeEvent(self, event):
        #FIXME remove before inGUI
        self.stop()

    def stop(self):
        self.cam.stop()
        self.streamer.stop()

class VideoStreamer(QObject):
    """ Class for handling streaming from camera and displaying in GUI """

    new_frame_available = pyqtSignal(np.ndarray)
    new_frame_unavailable = pyqtSignal()

    def __init__(self, cam:MMCamera, fps:int):
        super().__init__()
        self._logger = StandardLogger(__name__)
        self._cam = cam
        S_TO_MS = 1000
        sampling_period_ms = int((1/fps) * S_TO_MS)
        self._timer = QTimer()
        self._timer.timeout.connect(self._stream_new_frame)
        self._timer.start(sampling_period_ms)

    def _stream_new_frame(self):
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


class Canvas(QLabel):

    mouse_moved_pixel = pyqtSignal(list)
    mouse_pressed_pixel = pyqtSignal(list)
    mouse_released_pixel = pyqtSignal(list)

    def __init__(self, width, height):
        super().__init__()
        self.width = width
        self.height = height
        self.painter = Painter()
        pixmap = QPixmap(self.width, self.height)
        self.setFixedSize(self.width, self.height)
        self.setPixmap(pixmap)

    def mouseMoveEvent(self, event):
        pixel = [event.pos().x(), event.pos().y()]
        self.mouse_moved_pixel.emit(pixel)
        self.painter.drawn_edge.append(pixel)

    def mousePressEvent(self, event):
        pixel = [event.pos().x(), event.pos().y()]
        self.mouse_pressed_pixel.emit(pixel)
        
    def mouseReleaseEvent(self, event):
        pixel = [event.pos().x(), event.pos().y()]
        self.mouse_released_pixel.emit(pixel)
        self.painter.clicked_points = [pixel]
        self.painter.drawn_edge = []

    def update_(self, image:np.ndarray):
        rgb = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
        resized = cv2.resize(src=rgb, dsize=(self.width, self.height), interpolation = cv2.INTER_AREA)
        # Draw on the image
        resized = self.painter.draw_calibration_points(resized)
        resized = self.painter.draw_calibrated_tip_points(resized)
        resized = self.painter.draw_clicked_points(resized)
        resized = self.painter.draw_drawn_edge(resized)
        resized = self.painter.draw_interpolated_edge(resized)
        canvas_img = QImage(resized, self.width, self.height, self.width*3, QImage.Format.Format_RGB888)
        pixmap= QPixmap.fromImage(canvas_img)
        self.setPixmap(pixmap)

class Painter():

    def __init__(self):
        self._show_calibration_points_bool = False
        self._show_calibrated_tip_points_bool = False
        self._show_clicked_points_bool = True
        self._show_drawn_edge_bool = True
        self._show_interpolated_edge_bool = False
        self.calibration_points = []
        self.calibrated_tip_points = []
        self.clicked_points = []
        self.drawn_edge = []
        self.interpolated_edge = []
        self.set_colors()

    def set_colors(self):
        self.WHITE = [255, 255, 255]
        self.BLACK = [0, 0, 0]
        self.RED = [255, 0, 0]
        self.GREEN = [0, 255, 0]
        self.BLUE = [0, 0, 255]

    def show_calibration_points(self, bool_:bool):
        self._show_calibration_points_bool = bool_

    def show_calibrated_tip_points(self, bool_:bool):
        self._show_calibrated_tip_points_bool = bool_

    def show_clicked_points(self, bool_:bool):
        self._show_clicked_points_bool = bool_

    def show_drawn_edge(self, bool_:bool):
        self._show_drawn_edge_bool = bool_

    def show_interpolated_edge(self, bool_:bool):
        self._show_interpolated_edge_bool = bool_

    def draw_calibration_points(self, image):
        if self._show_calibration_points_bool is True:
            for pixel in self.calibration_points:
                image = utils.display_filled_circle(
                    image = image, 
                    center_xy = pixel,
                    radius = 2,
                    color = self.RED,
                    )
        return image

    def draw_calibrated_tip_points(self, image):
        if self._show_calibrated_tip_points_bool is True:
            for pixel in self.calibrated_tip_points:
                image = utils.display_hollow_circle(
                    image = image, 
                    center_xy = pixel,
                    radius = 5,
                    color = self.BLACK,
                    )
        return image

    def draw_clicked_points(self, image):
        if self._show_clicked_points_bool is True:
            for pixel in self.clicked_points:
                image = utils.display_x(
                    image = image, 
                    center_xy = pixel,
                    radius = 3,
                    color = self.WHITE,
                    thickness=1
                    )
        return image

    def draw_drawn_edge(self, image):
        if self._show_drawn_edge_bool is True:
            for pixel in self.drawn_edge:
                image = utils.display_dual_circle(
                    image = image, 
                    center_xy = pixel,
                    radius = 2, 
                    color_fill = self.WHITE,
                    color_edge = self.BLACK,
                    thickness=1
                    )
        return image

    def draw_interpolated_edge(self, image):
        if self._show_interpolated_edge_bool is True:
            if self.interpolated_edge != []:
                self.interpolated_edge = np.asarray(self.interpolated_edge)
                image = cv2.polylines(
                        img = image,
                        pts = [self.interpolated_edge],
                        isClosed = False,
                        color = self.BLACK,
                        thickness = 2
                    )
                image = cv2.polylines(
                        img = image,
                        pts = [self.interpolated_edge],
                        isClosed = False,
                        color = self.WHITE,
                        thickness = 1
                    )
        return image

if __name__ == "__main__":
    mm_path = os.path.join('C:', os.path.sep, 'Program Files','Micro-Manager-2.0')
    cam = 'HamamatsuHam_DCAM'
    brand = 'HamamatsuHam'
    val = 'HamamatsuHam_DCAM'
    bins = '2x2'
    rot = 180
    imagevals = 256
    cam_MM = MMCamera(mm_path, cam, brand, val, bins, rot, imagevals)
    app = QApplication(sys.argv)
    scale_factor = 1.3
    height = 900
    fps = 60
    win = VideoDisplay(cam_MM, height, fps)
    win.show()
    sys.exit(app.exec())
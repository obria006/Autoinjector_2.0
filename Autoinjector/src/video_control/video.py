"""
Script to handle video streaming in the Autoinjector GUI. Includes a MMCamera class to interface
with a MicroManager camera and get image frames from the camera, a VideoStreamer class which 
handles streaming video frames from the camera to the display, a VideoDisplay class which is the
main hub interfacing between the camera/streamer and the Canvas/Painter classes to display the
video stream.
"""
import os
import sys
import cv2
import numpy as np
from skimage.util import img_as_ubyte
from PyQt6.QtCore import (Qt,
                          QObject,
                          pyqtSignal,
                          )
from PyQt6.QtWidgets import (QApplication,
                            QLabel,
                            QVBoxLayout,
                            QWidget,
                            )
from PyQt6.QtGui import QImage, QPixmap, QColorConstants
from src.video_control.camera import MMCamera, VideoStreamer
from src.video_control import video_utils as utils
from src.miscellaneous.standard_logger import StandardLogger
from src.miscellaneous.utils import MplColorHelper


class VideoDisplay(QWidget):
    """
    Central hub for interfacing with camera and streamer to display the video feed
    in the GUI via the `Canvas`. Provides user interface for interacting with display.

    Updates the video display when recieves `new_frame_available` signal from the
    streamer.

    Signals:
        clicked_camera_pixel(list): [x, y] coordinates of the clicked pixel relative
            to the camera (which can be a different size than the canvas)
        clicked_canvas_pixel(list): [x, y] coordiantes of the clicked pixel relative
            to the canvas (which can be different size than the camera)
        draw_camera_pixels(list): [[x,y], ...] coordinates of the drawn edge annotation
            relative to the camera.
    """

    clicked_camera_pixel = pyqtSignal(list)
    clicked_canvas_pixel = pyqtSignal(list)
    drawn_camera_pixels = pyqtSignal(list)

    def __init__(self, cam:MMCamera, height:int, fps:int):
        """
        Args:
            cam (MMCamera): Camera object
            height (int): Desired pixel height of video display in GUI
            fps (int): Desired frame rate of video stream
        """
        super().__init__()
        self.cam = cam
        self.streamer = VideoStreamer(self.cam, fps)
        self._camera_to_canvas_scaling = height/self.cam.height
        self.height = int(round(height))
        self.width = int(round(self.cam.width * self._camera_to_canvas_scaling))
        self.moved_camera_pixels = []
        self._make_widgets()
        # Update the video display when a new frame available from the camera
        self.streamer.new_frame_available.connect(self.set_new_frame)

    def _make_widgets(self):
        self.canvas = Canvas(self.width, self.height)
        self.canvas.lmb_mouse_pressed_pixel.connect(self.handle_lmb_press)
        self.canvas.lmb_mouse_released_pixel.connect(self.handle_lmb_release)
        self.canvas.lmb_mouse_moved_pixel.connect(self.handle_lmb_move)
        layout = QVBoxLayout()
        layout.addWidget(self.canvas)
        self.setLayout(layout)

    def set_new_frame(self, frame:np.ndarray):
        """
        Set the `frame` attribute from the argument and call to update the display.

        Args:
            frame (np.ndarray): image from camera
        """
        self.frame = frame
        self.update_display()

    def handle_lmb_press(self,canvas_pixel:list):
        """
        Handles what to do when `lmb_mouse_pressed_pixel` from `Canvas`.

        Intializes a list of `moved_camera_pixels` of the drawn edge.

        Args:
            canvas_pixel (list): [x, y] coordiantes of mouse click in canvas
        """
        # Convert canvas pixel location to camera pixel location
        camera_pixel = self.convert_canvas_to_camera(canvas_pixel)
        self.moved_camera_pixels = []

    def handle_lmb_release(self,canvas_pixel:list):
        """
        Handles what to do when `lmb_mouse_released_pixel` from `Canvas`.

        Emits the clicked camera and canvas pixel coordiates as well as
        the list of `moved_camera_pixels` that have been acquired since the
        mouse button was depressed

        Args:
            canvas_pixel (list): [x, y] coordiantes of mouse click in canvas
        """
        # Convert canvas pixel location to camera pixel location
        camera_pixel = self.convert_canvas_to_camera(canvas_pixel)
        # Emit clicked coordinates and list of moved coordiantes
        self.clicked_camera_pixel.emit([camera_pixel[0], camera_pixel[1]])
        self.clicked_canvas_pixel.emit([canvas_pixel[0], canvas_pixel[1]])
        self.drawn_camera_pixels.emit(self.moved_camera_pixels)

    def handle_lmb_move(self,canvas_pixel:list):
        """
        Handles what to do when `lmb_mouse_moved_pixel` from `Canvas`.

        Adds the pixel coordinate to running list of `moved_camera_pixels`

        Args:
            canvas_pixel (list): [x, y] coordiantes of mouse click in canvas
        """
        camera_pixel = self.convert_canvas_to_camera(canvas_pixel)
        self.moved_camera_pixels.append(camera_pixel)

    def update_display(self):
        """
        Update the video display with the new image and annotations
        """
        # Convert camera image to rgb image for display
        canvas_frame = np.copy(self.frame)
        self.canvas.update_(canvas_frame)

    def show_tip_position(self, bool_:bool):
        """
        Tells `Painter` whether to show calibrated tip position.

        Args:
            bool_ (bool): Whehter to show tip position.
        """
        self.canvas.painter.show_calibrated_tip_points(bool_)

    def set_tip_position(self, x:int, y:int, delta_z_um:float):
        """
        Sets the `Painter`'s tip position coordinate

        Args:
            x (int): x coordiante of tip in camera image
            y (int): y coordinate of tip in camera image
            delta_z_um (float): Deviation between tip z and focus plane in um (tip - focus)
        """
        # Covnert from camera pixel to canvas pixel location
        camera_pixel = [x,y]
        canvas_position = self.convert_camera_to_canvas(camera_pixel)
        canvas_position.append(delta_z_um)
        # Set painter coordinate
        self.canvas.painter.calibrated_tip_points = [canvas_position]

    def show_drawn_annotation(self, bool_:bool):
        """
        Tells `Painter` whether to show the raw drawn coordinates

        Args:
            bool_ (bool): Whether to show annotation
        """
        self.canvas.painter.show_drawn_edge(bool_)

    def show_interpolated_annotation(self, bool_:bool):
        """
        Tells `Painter` whether to show the interpolated drawn coordinates

        Args:
            bool_ (bool): Whether to show annotation
        """
        self.canvas.painter.show_interpolated_edge(bool_)

    def set_interpolated_annotation(self, annot:np.ndarray):
        """
        Sets the `Painter`'s interpolated edge coordinates for display.

        Args:
            annot (np.ndarray): Interpolated coordiantes as [[x, y], ...]
        """
        try:
            annot = annot.tolist()
        except:
            pass
        canvas_annot = [self.convert_camera_to_canvas(pix) for pix in annot]
        self.canvas.painter.interpolated_edge = canvas_annot

    def reset_interpolated_annotation(self):
        """
        Sets the `Painter`'s interpolated edge coordinates to empty
        """
        self.canvas.painter.interpolated_edge = []

    def show_tissue_mask(self, bool_:bool):
        """
        Tells `Painter` whether to show tissue mask

        Args:
            bool_ (bool): Whether to show annotation
        """
        self.canvas.painter.show_tissue_mask(bool_)

    def show_edge_mask(self, bool_:bool):
        """
        Tells `Painter` whether to show tissue mask

        Args:
            bool_ (bool): Whether to show annotation
        """
        self.canvas.painter.show_edge_mask(bool_)

    def set_masks(self, tissue_mask:np.ndarray=None, apical_mask:np.ndarray=None, basal_mask:np.ndarray=None):
        """
        Sets the `Painter`'s masks for display.

        Args:
            tissue_mask (np.ndarray): 0-1 binary mask of tissue to display
            apical_mask (np.ndarray): 0-1 binary mask of apical edge to display
            basal_mask (np.ndarray): 0-1 binary mask of basal edge to display
        """
        if tissue_mask is not None:
            tissue_mask = cv2.resize(tissue_mask.astype(np.uint8), (self.width, self.height), cv2.INTER_AREA)
            self.canvas.painter.tissue_mask = utils.make_rgb_mask(tissue_mask, color = self.canvas.painter.TISSUE)
        if apical_mask is not None:    
            apical_mask = cv2.resize(apical_mask.astype(np.uint8), (self.width, self.height), cv2.INTER_AREA)
            self.canvas.painter.apical_mask = utils.make_rgb_mask(apical_mask, color = self.canvas.painter.APICAL)
        if basal_mask is not None:
            basal_mask = cv2.resize(basal_mask.astype(np.uint8), (self.width, self.height), cv2.INTER_AREA)
            self.canvas.painter.basal_mask = utils.make_rgb_mask(basal_mask, color = self.canvas.painter.BASAL)

    def display_masks(self):
        """ Show the tissue/edge masks in the video display """
        self.canvas.show_overlay()

    def hide_masks(self):
        """ Hide the tissue/edge_masks in the video display """
        self.canvas.hide_overlay()

    def reset_masks(self):
        """
        Resets `Painter`'s masks to null.
        """
        self.canvas.painter.tissue_mask = None
        self.canvas.painter.apical_mask = None
        self.canvas.painter.basal_mask = None


    def convert_canvas_to_camera(self, canvas_pixel:list)-> list:
        """
        Convert canvas pixels to camera pixels because they can be different sizes

        Args:
            canvas_pixel (list): [x, y] pixel coordinates in the canvas

        Returns:
            list of [x, y] pixel coordinates in the camera
        """
        cam_pixel = [round(int(pix/self._camera_to_canvas_scaling)) for pix in canvas_pixel]
        return cam_pixel

    def convert_camera_to_canvas(self, cam_pixel:list)-> list:
        """
        Convert camera pixels to canvas pixels because they can be different sizes

        Args:
            cam_pixel (list): [x, y] pixel coordinates in the camera

        Returns:
            list of [x, y] pixel coordinates in the canvas
        """
        canvas_pixel = [round(int(pix*self._camera_to_canvas_scaling)) for pix in cam_pixel]
        return canvas_pixel

    def closeEvent(self, event):
        #FIXME remove before inGUI
        self.stop()

    def stop(self):
        self.cam.stop()
        self.streamer.stop()


class Canvas(QLabel):
    """
    Class to handle showing the display of the image/video and user interaction
    with the GUI's video display. This class emits numerous signals that send
    pixel coordinates. These pixel coordinates are relative to the canvas (not
    the image which can be a different size).

    Uses 2 "canvases". The `dynamic_canvas` is for frequently updating display
    information like the video feed. The `overlay_canvas` is a semi- to fully-
    transparent overlay for showing information that is computationally expensive
    to display frequently (with every new frame from the camera) or information
    that is infrequnelty updated (like the tissue and edge masks).

    Signals:
        lmb_mouse_moved_pixel(list): [x, y] pixel coordinates of a click-and-drag
            left-mouse-button event in the canvas. 
        lmb_mouse_pressed_pixel(list): [x, y] pixel coordinates of the event when
            the left-mouse-button initially depressed.
        lmb_mouse_released_pixel(list):[x, y] pixel coordinates of the event when
            the left-mouse-button is released after being depressed.
    """

    lmb_mouse_moved_pixel = pyqtSignal(list)
    lmb_mouse_pressed_pixel = pyqtSignal(list)
    lmb_mouse_released_pixel = pyqtSignal(list)

    def __init__(self, width:int, height:int):
        """
        Args:
            width (int): Pixel width of the canvas for displaying images
            height (int): Pixel height of the canvas for displaying images
        """
        super().__init__()
        self.width = width
        self.height = height
        self._lmb_track_mouse = False
        self.painter = Painter()
        pixmap = QPixmap(self.width, self.height)
        pixmap.fill(QColorConstants.Transparent)
        self.setFixedSize(self.width, self.height)
        self.dynamic_canvas = QLabel(self)
        self.dynamic_canvas.setFixedSize(self.width, self.height)
        self.dynamic_canvas.setPixmap(pixmap)
        self.overlay_canvas = QLabel(self)
        self.overlay_canvas.setFixedSize(self.width, self.height)
        self.overlay_canvas.setPixmap(pixmap)

    def mouseMoveEvent(self, event):
        """
        Emits pixel of the mouse when it is clicked-and-dragged Also, adds the
        pixel to the painter's list of coordinates to draw for a drawn edge.
        """
        if self._lmb_track_mouse:
            pixel = [event.pos().x(), event.pos().y()]
            self.lmb_mouse_moved_pixel.emit(pixel)
            self.painter.drawn_edge.append(pixel)

    def mousePressEvent(self, event):
        """
        Emits pixel of the mouse when mouse is depressed. Also initializes
        painter's list of coordinates for a drawn edge as empty.
        """
        if event.button() == Qt.MouseButton.LeftButton:
            self._lmb_track_mouse = True
            pixel = [event.pos().x(), event.pos().y()]
            self.lmb_mouse_pressed_pixel.emit(pixel)
            self.painter.drawn_edge = []
        
    def mouseReleaseEvent(self, event):
        """
        Emits pixel of the mouse when mouse button is released. Also sets the
        painter's clicked point and resets painter's drawn edge to empty.
        """
        if event.button() == Qt.MouseButton.LeftButton:
            self._lmb_track_mouse = False
            pixel = [event.pos().x(), event.pos().y()]
            self.lmb_mouse_released_pixel.emit(pixel)
            self.painter.clicked_points = [pixel]
            self.painter.drawn_edge = []

    def update_(self, image:np.ndarray):
        """
        Update the video display with a new image. Also modify image to show
        annotations via `painter` attribute.

        Args:
            image (np.ndarray): Image to display
        """
        # Convert image to rgb and resize to canvas dimensison
        rgb = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
        resized_rgb = cv2.resize(src=rgb, dsize=(self.width, self.height), interpolation = cv2.INTER_AREA)
        # Draw on the image to show annotations
        resized_rgb = self.painter.paint_image(resized_rgb)
        # Convert to pixmap and set canvas's pixmap
        canvas_img = QImage(resized_rgb, self.width, self.height, self.width*3, QImage.Format.Format_RGB888)
        pixmap = QPixmap.fromImage(canvas_img)
        self.dynamic_canvas.setPixmap(pixmap)

    def show_overlay(self):
        """
        Show the overlay canvas if masks are present to display
        """
        # Initialize a image with the same size as the canvas of zeros
        resized_rgb = np.zeros((self.height,self.width,3), np.uint8)
        # Paint the masks onto the zero rgb image
        resized_rgb = self.painter.paint_masks(resized_rgb)
        # Convert to rgb -alpha image (for transparency)
        resized_rgba = cv2.cvtColor(resized_rgb, cv2.COLOR_RGB2RGBA)
        # Where the mask image is non-zero set alpha to 15% else 0% alpha (transparent)
        resized_rgba[...,3] = int(255*0.15)*np.any(resized_rgba[:,:,:3]>0, axis=2)
        # Convert to pixmap and set canvas's pixmap
        canvas_img = QImage(resized_rgba, self.width, self.height, self.width*4, QImage.Format.Format_RGBA8888)
        pixmap = QPixmap.fromImage(canvas_img)
        # Display the image
        self.overlay_canvas.setPixmap(pixmap)

    def hide_overlay(self):
        """
        Hide the overlay canvas. (make fully transparent to see video display)
        """
        pixmap = QPixmap(self.width, self.height)
        pixmap.fill(QColorConstants.Transparent)
        self.overlay_canvas.setPixmap(pixmap)


class Painter():
    """
    Modifies the camera image with the various user and GUI annotations
    """

    def __init__(self):
        self._show_calibration_points_bool = False
        self._show_calibrated_tip_points_bool = False
        self._show_clicked_points_bool = True
        self._show_drawn_edge_bool = True
        self._show_interpolated_edge_bool = False
        self._show_segmented_tissue_bool = False
        self._show_segmented_edges_bool = False
        self.calibration_points = []
        self.calibrated_tip_points = []
        self.clicked_points = []
        self.drawn_edge = []
        self.interpolated_edge = []
        self.tissue_mask = None
        self.apical_mask = None
        self.basal_mask = None
        self.set_colors()

    def set_colors(self):
        self.WHITE = [255, 255, 255]
        self.BLACK = [0, 0, 0]
        self.RED = [255, 0, 0]
        self.GREEN = [0, 255, 0]
        self.BLUE = [0, 0, 255]
        self.TISSUE = [238, 102, 119]
        self.APICAL = [102, 204, 238]
        self.BASAL = [204, 187, 68]
        self.tip_cmap = MplColorHelper('bwr_r',start_val=-1, stop_val=1)

    def show_calibration_points(self, bool_:bool):
        """
        Sets boolean indicator to show calibration points annotation

        Args:
            bool_: Whether to show calibration points annotation
        """
        self._show_calibration_points_bool = bool_

    def show_calibrated_tip_points(self, bool_:bool):
        """
        Sets boolean indicator to show calibrated tip position

        Args:
            bool_: Whether to show calibrated tip position
        """
        self._show_calibrated_tip_points_bool = bool_

    def show_clicked_points(self, bool_:bool):
        """
        Sets boolean indicator to show clicked coordinate in image

        Args:
            bool_: Whether to show the clicked coordinate
        """
        self._show_clicked_points_bool = bool_

    def show_drawn_edge(self, bool_:bool):
        """
        Sets boolean indicator to show raw drawn edge annotation

        Args:
            bool_: Whether to show raw drawn edge
        """
        self._show_drawn_edge_bool = bool_

    def show_interpolated_edge(self, bool_:bool):
        """
        Sets boolean indicator to show interpolated edge annotation

        Args:
            bool_: Whether to show interpolated edge annotation
        """
        self._show_interpolated_edge_bool = bool_

    def show_tissue_mask(self, bool_:bool):
        """
        Sets boolean indicator to show tissue mask annotation

        Args:
            bool_: Whether to show tissue mask annotation
        """
        self._show_segmented_tissue_bool = bool_

    def show_edge_mask(self, bool_:bool):
        """
        Sets boolean indicator to show edge mask annotation

        Args:
            bool_: Whether to show edge mask annotation
        """
        self._show_segmented_edges_bool = bool_

    def paint_masks(self,rgb:np.ndarray)->np.ndarray:
        """
        Modify rgb image with tisue/edge masks

        Args:
            rgb (np.ndarray): rgb image to modify and display

        Returns
            np.ndarray of modified image
        """
        rgb = self.draw_tissue_mask(rgb)
        rgb = self.draw_edge_mask(rgb)
        return rgb

    def paint_image(self, image_rgb:np.ndarray)->np.ndarray:
        """
        Modifies image with annotations.

        Args:
            image_rgb (np.ndarray): rgb image to modify and display

        Returns
            np.ndarray of modified image
        """
        image_rgb = self.draw_calibration_points(image_rgb)
        image_rgb = self.draw_calibrated_tip_points(image_rgb)
        image_rgb = self.draw_clicked_points(image_rgb)
        image_rgb = self.draw_drawn_edge(image_rgb)
        image_rgb = self.draw_interpolated_edge(image_rgb)
        return image_rgb

    def draw_calibration_points(self, image:np.ndarray)->np.ndarray:
        """
        Modifies image with calibration point annotations.

        Args:
            image (np.ndarray): Image to modify

        Returns
            np.ndarray of modified image
        """
        if self._show_calibration_points_bool is True:
            for pixel in self.calibration_points:
                image = utils.display_filled_circle(
                    image = image, 
                    center_xy = pixel,
                    radius = 2,
                    color = self.RED,
                    )
        return image

    def draw_calibrated_tip_points(self, image:np.ndarray)->np.ndarray:
        """
        Modifies image with calibrated tip position.

        Args:
            image (np.ndarray): Image to modify

        Returns
            np.ndarray of modified image
        """
        if self._show_calibrated_tip_points_bool is True:
            for point in self.calibrated_tip_points:
                # Maximum deviation between tip and focal plane in um
                MAX_DZ = 200
                # Max and min radii of display circle in pixel
                MAX_RAD = 50
                MIN_RAD = 10
                # Extract the x, y pixel coordinates
                pixel = point[:2]
                # Exctract delta z between tip and focal plane, and normalize -1 to 1
                dz = point[-1]
                norm_dz = np.clip(dz/MAX_DZ, -1, 1)
                # Compute color and radis from delta z
                col = self.tip_cmap.get_rgb(norm_dz)
                rad = MIN_RAD + int(abs(norm_dz) * (MAX_RAD- MIN_RAD))
                # Display in GUI
                image = utils.display_filled_circle(
                    image = image, 
                    center_xy = pixel,
                    radius = 1,
                    color = self.BLACK
                    )
                image = utils.display_hollow_circle(
                    image = image, 
                    center_xy = pixel,
                    radius = rad,
                    color = col,
                    thickness=1
                    )
        return image

    def draw_clicked_points(self, image:np.ndarray)->np.ndarray:
        """
        Modifies image with clicked point annotation.

        Args:
            image (np.ndarray): Image to modify

        Returns
            np.ndarray of modified image
        """
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

    def draw_drawn_edge(self, image:np.ndarray)->np.ndarray:
        """
        Modifies image with drawn edge annotation.

        Args:
            image (np.ndarray): Image to modify

        Returns
            np.ndarray of modified image
        """
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

    def draw_interpolated_edge(self, image:np.ndarray)->np.ndarray:
        """
        Modifies image with interpolated edge annotations.

        Args:
            image (np.ndarray): Image to modify

        Returns
            np.ndarray of modified image
        """
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

    def draw_tissue_mask(self,rgb:np.ndarray)->np.ndarray:
        """
        Draw the tissue mask on the rgb image (if tissue mask exists)

        Args:
            rgb (np.ndarray): rgb image to modify and display

        Returns
            np.ndarray of modified image
        """
        if self.tissue_mask is not None:
            rgb = utils.alpha_compost_A_over_B(self.tissue_mask, rgb, 1)
        return rgb

    def draw_edge_mask(self,rgb:np.ndarray)->np.ndarray:
        """
        Draw the edge masks on the rgb image (if edge masks exists)

        Args:
            rgb (np.ndarray): rgb image to modify and display

        Returns
            np.ndarray of modified image
        """
        if self.apical_mask is not None:
            rgb = utils.alpha_compost_A_over_B(self.apical_mask, rgb, 1)
        if self.basal_mask is not None:
            rgb = utils.alpha_compost_A_over_B(self.basal_mask, rgb, 1)
        return rgb
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
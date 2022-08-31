from scipy.interpolate import UnivariateSpline
import numpy as np
import cv2

class AnnotationError(Exception):
    '''
    Exception raised for problems with annotation

    Attributes:
        msg (str): Explanation of error
    '''
    def __init__(self, msg="Error while annotating video"):
        self.msg = msg
        super().__init__(self.msg)

class CameraError(Exception):
    '''
    Exception raised for problems with camera

    Attributes:
        msg (str): Explanation of error
    '''
    def __init__(self, msg="Error while interfacing with camera"):
        self.msg = msg
        super().__init__(self.msg)

class NoFrameError(Exception):
    '''
    Exception raised when can't sample frame from camera

    Attributes:
        msg (str): Explanation of error
    '''
    def __init__(self, msg="No frame from camera"):
        self.msg = msg
        super().__init__(self.msg)

def display_filled_circle(image, center_xy, radius, color):
    return cv2.circle(image,center_xy, radius, color, thickness=-1)

def display_hollow_circle(image, center_xy, radius, color, thickness=1):
    return cv2.circle(image,center_xy, radius, color,thickness)

def display_dual_circle(image, center_xy, radius, color_fill, color_edge, thickness):
    assert radius - thickness >= 1, "Thickness must not exceed radius"
    image = cv2.circle(image, center_xy, radius, color_edge, thickness)
    image = cv2.circle(image, center_xy, radius - thickness, color_fill, thickness=-1)
    return image

def display_cross(image, center_xy, radius, color, thickness):
    c_x, c_y = center_xy
    min_x = max(0, c_x - radius)
    max_x = min(image.shape[1]-1, c_x + radius)
    min_y = max(0, c_y - radius)
    max_y = min(image.shape[0]-1, c_y + radius)
    image = cv2.line(image, (c_x, min_y), (c_x, max_y), color, thickness)
    image = cv2.line(image, (min_x, c_y), (max_x, c_y), color, thickness)
    return image

def display_x(image, center_xy, radius, color, thickness):
    c_x, c_y = center_xy
    min_x = max(0, c_x - radius)
    max_x = min(image.shape[1]-1, c_x + radius)
    min_y = max(0, c_y - radius)
    max_y = min(image.shape[0]-1, c_y + radius)
    image = cv2.line(image, (min_x, min_y), (max_x, max_y), color, thickness)
    image = cv2.line(image, (min_x, max_y), (max_x, min_y), color, thickness)
    return image

def display_point(image, center_xy, color):
    return cv2.circle(image, center_xy, radius=1, color=color, thickness=-1)
    
def interpolate_vertical(drawn_pixels:list):
    pixel_arr = np.array(drawn_pixels)
    xvals = pixel_arr[:,0]
    yvals = pixel_arr[:,1]
    order = np.argsort(yvals)        
    height = max(yvals)
    spl = UnivariateSpline(yvals[order],xvals[order], s = len(yvals)*3)
    ynew = np.arange(min(yvals),height,1)
    xnew = (spl(ynew))

    try:
        interpolated_pixels = []
        for i in range(0,len(xnew)-1):
            x = int(xnew[i])
            y = int(ynew[i])
            interpolated_pixels.append([x,y])
            if i == len(xnew)-2:
                interpolated_pixels= np.asarray(interpolated_pixels)
                return interpolated_pixels
    except Exception as e:
        raise AnnotationError(f'Error while interpolating annotation: {e}')

def interpolate_horizontal(drawn_pixels:list):
    pixel_arr = np.array(drawn_pixels)
    xvals = pixel_arr[:,0]
    yvals = pixel_arr[:,1]
    order = np.argsort(xvals)        
    height = max(xvals)
    spl = UnivariateSpline(xvals[order],yvals[order], s = len(xvals)*3)
    xnew = np.arange(min(xvals),height,1)
    ynew = (spl(xnew))

    try:
        interpolated_pixels = []
        for i in range(0,len(ynew)-1):
            x = int(xnew[i])
            y = int(ynew[i])
            interpolated_pixels.append([x,y])
            if i == len(xnew)-2:
                interpolated_pixels= np.asarray(interpolated_pixels)
                return interpolated_pixels
    except Exception as e:
        raise AnnotationError(f'Error while interpolating annotation: {e}')
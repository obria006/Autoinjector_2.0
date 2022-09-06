from collections.abc import Iterable
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

def display_transparent_mask(image:np.ndarray, rgb:np.ndarray, mask:np.ndarray, color:list):
    """
    Display transparent mask on image with color. Returns RGB image with colored
    and transparent mask.

    Args:
        image (np.ndarray): 2d gray scale image with image intensity values
        rgb (np.ndarray): 3d rgb image to insert the colored mask
        mask (np.ndarray): 0-1 valued binary mask image
        color (list): 3 element list of uint8 colored values

    Returns:
        3d rgb image with colored mask
    """
    # Validate
    if image.shape != mask.shape:
        raise ValueError(f"image and mask must have same width and height. {image.shape} does not match {mask.shape}")
    if np.amax(mask) > 1 or np.amin(mask) < 0 or len(np.unique(mask)) > 2:
        breakpoint()
        raise ValueError(f"Mask must be binary valued on 0-1")
    if image.dtype != np.uint8:
        raise TypeError(f"Image must be 8bit image")
    if not isinstance(color, Iterable):
        raise TypeError(f"Invalid `color` type: {type(color)}. Color must be Iterable type.")
    if len(color) !=3:
        raise ValueError(f"Invalid `color`: {color}. `color` must be a 3 element rgb triplet")
    for ele in color:
        assert ele>=0 and ele<=255 and isinstance(ele, int)
    # Compute 0-1 normalized intensity mask and multiply by color value to get rgb mask
    normalized_intensity_mask = np.multiply(image.astype(np.float32)/255,mask.astype(np.float32))
    rgb_mask = (cv2.cvtColor(normalized_intensity_mask, cv2.COLOR_GRAY2RGB)*color).astype(np.uint8)
    # insert rgb mask into rgb image where mask exists
    rgb = np.copy(rgb)
    rgb[np.where(mask)] = rgb_mask[np.where(mask)]

    return rgb

def interpolate(drawn_pixels):
    pixel_arr = np.array(drawn_pixels)
    n = len(pixel_arr)
    tvals = np.arange(len(pixel_arr)).reshape(-1,1) / (len(pixel_arr)-1)
    xvals = pixel_arr[:,0]
    yvals = pixel_arr[:,1]
    
    # Fit splines for data
    unispl_x = UnivariateSpline(tvals, xvals, s=5*n)
    unispl_y = UnivariateSpline(tvals, yvals, s=5*n)

    # Approximate total pixel distance of annotation using inf norm
    # This should over estimate the total number of pixels in a contiguous annotation
    dist = 0
    for ind, pix in enumerate(pixel_arr):
        if ind == 0:
            continue
        dist += np.linalg.norm(pix - pixel_arr[ind-1], ord=np.inf)
    # Generate datapoints from the total distance to make contigous pline
    tnew = np.linspace(0,1,int(dist))
    xnew = (unispl_x(tnew)).astype(int).reshape(-1,1)
    ynew = (unispl_y(tnew)).astype(int).reshape(-1,1)
    interpolated = np.concatenate([xnew,ynew],axis=1)
    # Remove duplicate values in interpolated
    sequential_differences = np.diff(interpolated, axis=0).astype(np.bool)
    is_sequentially_different = np.any(sequential_differences,axis=1)
    non_duplicate_indices = np.insert(is_sequentially_different,0,True)
    interpolated = interpolated[non_duplicate_indices]

    return interpolated
    
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
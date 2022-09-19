""" Scripts for post-processing detected edges of tissue """
import numpy as np
import cv2
import matplotlib.pyplot as plt
from src.deep_learning.edge_utils.error_utils import EdgeNotFoundError
from src.miscellaneous.validify import val_binary_image

def reachable_edges(edge_mask:np.ndarray, tiss_mask:np.ndarray, pip_orient_RH:float, edge_type:str, ang_thresh:float=20, bound_perc:float=0.1)-> np.ndarray:
    """
    Extract the reachable edges from the edge mask. Reachable meaning that
    the edges make a desired angle w/ the pipette (like 90 deg), don't 
    extend to the edges of the FOV, and are not obsucured by tissue (aka
    doesn't try to inject an edge behind a wall of tissue).

    Downsizes masks to 128x128 to speed up processing, then resizes up 
    to original size upon return.

    Args:
        edge_mask (np.ndarray): 0-1 binary edge mask
        tiss_mask (np.ndarray): 0-1 binary tissue mask
        pip_orient_RH (float): Orientation of pipette x-axis (projecting out and
            away from pipette tip) in degrees relative to RH coordinate system (positve is
            CCW from pointing to the right)
        edge_type (str): name of edge to be detected
        ang_thresh (float): Mininum appropriate angle between the pipette and the
            edges in degrees. Edges below the angle will be removed
        bound_perc (np.ndarray): Perecentage of width/height to set to 0

    Returns:
        (np.ndarray) 0-1 binary mask of reachable edges
    """
    # Validate masks
    val_binary_image(edge_mask)
    val_binary_image(tiss_mask)
    if edge_mask.shape != tiss_mask.shape:
        raise ValueError(f"Mask shapes must match. Edge mask shape ({edge_mask.shape}) different than tissue mask shape ({tiss_mask.shape}).")
    # Copy arrays not to modify arguments
    edge_mask = np.copy(edge_mask).astype(np.uint8)
    tiss_mask = np.copy(tiss_mask).astype(np.uint8)
    # Resize to 128
    SQ_DIM = 128
    ksz = (int(edge_mask.shape[0]/SQ_DIM)+1, int(edge_mask.shape[1]/SQ_DIM)+1)
    kern = cv2.getStructuringElement(shape=cv2.MORPH_RECT, ksize=ksz)
    dil = cv2.dilate(edge_mask, kern)
    edge_mask_rsz = cv2.resize(dil, (SQ_DIM, SQ_DIM), interpolation=cv2.INTER_AREA)
    tiss_mask_rsz = cv2.resize(tiss_mask, (SQ_DIM, SQ_DIM), interpolation=cv2.INTER_AREA)
    # Rotate image as if the pipette was at the right side of the screen point left
    rot_ang = 180-pip_orient_RH
    edge_mask_rsz_rot, inv = rotate_image(edge_mask_rsz, rot_ang)
    tiss_mask_rsz_rot, inv = rotate_image(tiss_mask_rsz, rot_ang)
    # Threshold to edge that makes angles with pipette
    edge_mask_rsz_rot = edge_angle_thresholding(edge_mask_rsz_rot, min_ang_deg = ang_thresh)
    # Get only edges that are not obscurred by tissue or mask
    edge_mask_rsz_rot = unobscurred_rightmost_edges(edge_mask_rsz_rot, tiss_mask_rsz_rot)
    # Rotate back to the original orientaion
    edge_mask_rsz = invert_rotation(edge_mask_rsz_rot, inv, SQ_DIM, SQ_DIM)
    # Elminate edges near image boundary
    edge_mask_rsz = zeroify_boundaries(edge_mask_rsz, perc=bound_perc)
    # Get the longest contigous edge
    if len(np.unique(edge_mask_rsz)) == 1:
        raise EdgeNotFoundError(f"Could not find 'reachable' {edge_type} edge given the orientation between the tissue and pipette.")
    edge_mask_rsz = largest_connected_component(edge_mask_rsz)
    # Dialate the smaller image so when intersect upsized version and orignal edge, there will definintly be an intersection
    kern = cv2.getStructuringElement(shape=cv2.MORPH_RECT, ksize=(3,3))
    edge_mask_rsz = cv2.dilate(edge_mask_rsz.astype(np.uint8), kern)
    # Resize to original
    reachable = cv2.resize(edge_mask_rsz.astype(np.uint8),edge_mask.shape,interpolation=cv2.INTER_LINEAR)
    reachable = np.logical_and(reachable, edge_mask)
    # Evaluate connectivity (aka that edge is single pixel width so each True pixel
    # has at most 3 instances of 8-point connectivity)
    if not is_single_pixel_width(reachable):
        raise ValueError("Edge mask is not single pixel width")
    return reachable
    
def is_single_pixel_width(mask:np.ndarray)->bool:
    """
    Return true if mask only contains edges of single pixel width

    Args:
        mask (np.ndarray): 0-1 binary mask
    
    Returns:
        np.ndarray w/ number of connectivity pixels at each location
    """
    # Validate
    val_binary_image(mask)
    # Convolve kernel
    kernel = np.ones((2,2))
    conn_img = cv2.filter2D(mask.astype(np.int16), ddepth=-1, kernel=kernel, borderType=cv2.BORDER_CONSTANT)
    return np.all(conn_img<4)

def edge_angle_thresholding(edge_mask:np.ndarray, min_ang_deg:float = 20)->np.ndarray:
    """
    Return binary edge mask only with edges above the angle threshold.

    Args:
        edge_mask (np.ndarray): 0-1 binary edge mask
        min_ang_deg (float): Mininum appropriate angle between the pipette and the
            edges in degrees. Edges below the angle will be removed

    Returns:
        (np.ndarray) 0-1 binary mask of edges with angle above threshold
    """
    val_binary_image(edge_mask)
    if min_ang_deg < 0 or min_ang_deg > 90:
        raise ValueError("Invalid angle threshold: {min_ang_deg}. Angle threshold must be 0-90.")
    # Compute gradients
    grad_x = cv2.Sobel(edge_mask.astype(float), -1, 1, 0, ksize=7, scale=1, delta=0, borderType=cv2.BORDER_DEFAULT)
    grad_y = cv2.Sobel(edge_mask.astype(float), -1, 0, 1, ksize=7, scale=1, delta=0, borderType=cv2.BORDER_DEFAULT)
    # Compute absolute angle of gradients (which is perpendicualr to edges)
    grad_x = grad_x + 1e-6 # Prevent 0 division
    ang = np.rad2deg(np.abs(np.arctan(np.divide(grad_y, grad_x))))
    ang = cv2.blur(ang,(5,5))
    # Convert angle of gradient to angle of edges by negate and add 90 deg
    ang = ang* -1 + 90
    # Limit only to angle on the single pixel edge
    ang[edge_mask==0]=0
    # Return angle above threshold
    return ang > min_ang_deg

def zeroify_boundaries(mask:np.ndarray, perc:float=0.1)->np.ndarray:
    """
    Sets boundary pixels of mask to 0 based on percentage of width/height.

    Args:
        mask (np.ndarray): 0-1 binary mask
        perc (np.ndarray): Perecentage of width/height to set to 0

    Returns:
        np.ndarray with boundaries set to 0
    """
    # Validate
    val_binary_image(mask)
    if perc<0 or perc>1:
        raise ValueError(f"Invalid zeroify percentage {perc}. Must be 0-1.")
    # Compute boundaires of setting 0
    del_x = int(perc/2*mask.shape[1])
    del_y = int(perc/2*mask.shape[0])
    # Set 0 boundaries
    mask = mask.astype(np.uint8)
    mask[:del_y,:] = 0
    mask[-del_y:,:] = 0
    mask[:,:del_x] = 0
    mask[:,-del_x:] = 0
    return mask

def unobscurred_rightmost_edges(edge_mask:np.ndarray, tiss_mask:np.ndarray)->np.ndarray:
    """
    Return binary image of rightmost edges in edge mask that dont have any
    tissue or other edges to their right

    Args:
        edge_mask (np.ndarray): 0-1 binary edge mask
        tiss_mask (np.ndarray): 0-1 binary tissue mask
    """
    #Validate
    val_binary_image(edge_mask)
    val_binary_image(tiss_mask)
    # Get rightmost connected edges
    right_edge = contiguous_rightmost_edges(edge_mask)
    # Get rightmost connected edges of tissue+edge
    tiss_edge_mask = np.logical_or(right_edge, tiss_mask)
    right_tiss_edge = contiguous_rightmost_edges(tiss_edge_mask, alt_mask=right_edge)
    # Unobscurred edges are those that exist in the rightmost tissue and rightmost edge image
    unobscurred = np.logical_and(right_edge,right_tiss_edge)
    return unobscurred

def contiguous_rightmost_edges(mask:np.ndarray, alt_mask:np.ndarray=None)->np.ndarray:
    """
    Return binary mask of right most pixels in `mask` (with left-connected pixels in
    `alt_mask` or `mask` if `alt_mask` is None) that don't have any nonzero pixels to
    their right in `mask` or `alt_mask`.

    Args:
        mask (np.ndarray): Binary image to assess
        alt_mask (np.ndarray): Alternate binary image to assess leftside connectivity

    Returns:
        np.ndarray binary array of rightmost w/ left connected pixels in `mask`
    """
    # validate
    val_binary_image(mask)
    if alt_mask is not None:
        val_binary_image(alt_mask)
    # Make a image of the row-priority pixel indices w/ same size as mask
    mask_inds = np.arange(mask.shape[0]*mask.shape[1]).reshape(mask.shape[0],mask.shape[1])
    # Get pixel indices of right most pixels
    rightmost_inds = np.max(mask*mask_inds, axis=1)
    # Generate pixel indieces that are connected ot the rightmost pixel in alt mask
    if alt_mask is None:
        alt_mask = mask
    rightmost_inds = maintain_left_conn(alt_mask, rightmost_inds)
    # Create mask fo only rightmost (and left connected) pixels
    new_mask = np.zeros_like(mask)
    np.put(new_mask,rightmost_inds,1)
    # Supress all detected edges on the left boundary (spurious results from np.max)
    new_mask[:,0] = 0
    return new_mask.astype(bool)

def maintain_left_conn(mask:np.ndarray, inds:np.ndarray)->np.ndarray:
    """
    Appends indices of pixels (from `mask`) that are continuously left-connected
    the pixel at each index specified in `inds`.

    Args:
        mask (np.ndarray): Binary image to assess
        inds (np.ndarray): Flat array of flattend pixel indices of mask

    Returns:
        np.ndarray of inds w/ connected flattend indices
    """
    # Number of rows and columsn in mask
    rows = mask.shape[0]
    cols = mask.shape[1]
    new_inds = []
    for ind in np.unique(inds):
        row = ind//cols
        is_true = True
        new_ind = ind - 1
        # while pixels are connected to pixel in same row as ind, add to new inds
        while is_true:
            if new_ind//cols < row:
                is_true = False
            elif mask.flat[new_ind] > 0:
                new_inds.append(new_ind)
                new_ind -= 1
            else:
                is_true = False
    return np.concatenate((inds, np.array(new_inds).astype(int)))

def largest_connected_component(mask:np.ndarray)->np.ndarray:
    """
    Return the largest connected component in the binary mask

    Args:
        mask (np.ndarray): Binary image to assess

    Returns:
        np.ndarray binary 0-1 like mask w/ largest component 
    """
    num_labels, mask_cc, mask_stats, _ = cv2.connectedComponentsWithStats(mask.astype(np.uint8))
    mask_areas = list(mask_stats[1:, cv2.CC_STAT_AREA])
    max_area = max(mask_areas)
    mask_size = {area: label for label, area in enumerate(mask_areas, start=1)}
    longest = mask_cc == mask_size[max_area]
    return longest

def rotate_image(image:np.ndarray, angle:float, bound:bool = True):
    '''
    Rotates an image by desired angle where positive angle is CW rotation
    Args:
        image: Single channel 8-bit image (numpy array)
        angle: Rotaion angle in degrees
        bound: Boolean whether image is kept at orignal size (False) or bounded in larger image (True)

    Returns:
        rot: Rotated image
        inv_rot_matrix: Inverse of rotation matrix used to originally rotate the image
    '''

    # Dimensions of the image and then determine the center
    (h, w) = image.shape[:2]
    (cX, cY) = (w // 2, h // 2)
    # grab the rotation matrix (applying the negative of the
    # angle to rotate clockwise), then grab the sine and cosine
    # (i.e., the rotation components of the matrix)
    M = cv2.getRotationMatrix2D((cX, cY), angle, 1.0)
    Mp = M
    cos = np.abs(M[0, 0])
    sin = np.abs(M[0, 1])
    # compute the new bounding dimensions of the image
    nW = int((h * sin) + (w * cos))
    nH = int((h * cos) + (w * sin))
    # adjust the rotation matrix to take into account translation
    M[0, 2] += (nW / 2) - cX
    M[1, 2] += (nH / 2) - cY

    # Construct the inverse rotation matrix
    # Rotate about the new image center, aka (nW/2, nH/2), in the
    # opposite direction as the initial rotation.
    M2 = cv2.getRotationMatrix2D((nW//2, nH//2), -angle, 1.0)
    # Translate the re-rotated image center back to the original image center
    M2[0, 2] += (w/2) - nW//2
    M2[1, 2] += (h/2) - nH//2
    inv_rot_mat = M2
    
    # perform the actual rotation and return the image
    if bound is True:
        rot = cv2.warpAffine(image, M, (nW, nH))
    # Unbounded image
    else:
        rot = cv2.warpAffine(image, Mp, (w, h))
    
    return rot, inv_rot_mat

def invert_rotation(image:np.ndarray,inv_rot_matrix:np.ndarray,w:int,h:int):
    '''
    Function to undo rotation of the rotated image.

    Keyword Arguements:
        image: Single channel 8-bit image (numpy array)
        inv_rot_matrix: Inverse of rotation matrix used to originally rotate the image
        w: Desired width of output image in pixels
        h: Desired height of output image in pixels

    Returns:
        inv_rot: Rotated single channel image from input image
    '''

    inv_rot = cv2.warpAffine(image.astype(np.uint8), inv_rot_matrix, (w, h))
    return inv_rot
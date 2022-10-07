""" General utility functions used in edge classification """
import numpy as np
import cv2
from src.deep_learning.edge_utils.error_utils import EdgeNotFoundError
from src.deep_learning.edge_utils.img_utils import is_single_pixel_width

def inv_dictionary(dict_: dict) -> dict:
    """
    Inverts dictionary mapping. Each value in inverted dictionary is a list.

    Args:
        dict_: dictionary to be inverted

    Returns:
        inv_map: dictionary with inverse mapping from `dict_`
    """
    inv_map = {}
    for k, v in dict_.items():
        inv_map[v] = inv_map.get(v, []) + [k]
    return inv_map

def sort_paths_along_binary_trajectory(img:np.ndarray)->list[ list[ list[int, int]]]:
    """
    Sorts ALL CONTIGUOUS PATHS in binary image so that the sorted points could
    be traced consecutively from point to point. The path must have single
    pixel width and the path must have a value of 1 (with background as 0).

    !! IMPORTANT: MUST HAVE EDGES OF ONLY SINGLE PIXEL WIDTH WITH 8-WAY CONNECTIVITY !!

    Starts by finding the extremum point of path (a pixel with a single 8-way connection).
    Then trace coordinates from extremum point to nearest neighbors to construct the path.

    Args:
        img (np.ndarray): Binary image w/ single pixel width paths to be sorted (path value == 1)
    
    Returns:
        sorted_ (list): List of sorted coordinates for each contiguous edge like:
            [
                [[x11, y11], [x12, y12], ...],
                [[x21, y21], [x22, y22], ...],
                ...,
            ]
    """
    # Validate image is a binary image
    if not isinstance(img, np.ndarray):
        raise TypeError("img must be numpy array")
    if not is_single_pixel_width(img, conn=8):
        raise ValueError(f"Image must have single pixel width edge with strictly 8-way connectivity")
    for ele in np.unique(img):
        if ele not in [0,1]:
            raise ValueError(f"img must be binary valued on 0-1")
    # Evaluate number of edges. Should have 2 components 0 is bg and 1 is edge
    n_labels, mask = cv2.connectedComponents(img.astype(np.uint8))
    if n_labels - 1 < 1:
        raise ValueError(f'Invalid number of edges to sort for trajectory: {n_labels -1}. Must have at least 1 edge to sort.')

    # sort each connected edge in the connnected compoennt mask but ignore 0 lable which is bg
    sorted_ = []
    for label in range(1,n_labels):
        single_edge_mask = mask == label
        sorted_edge = sort_path_along_binary_trajectory(single_edge_mask)
        sorted_.append(sorted_edge)

    return sorted_
    

def sort_path_along_binary_trajectory(img:np.ndarray)->list[ list[int, int]]:
    """
    Sorts A SINGLE CONTIGUOUS PATH in binary image so that the sorted points could
    be traced consecutively from point to point. The path must have single
    pixel width and the path must have a value of 1 (with background as 0).

    !! IMPORTANT: MUST HAVE EDGES OF ONLY SINGLE PIXEL WIDTH WITH 8-WAY CONNECTIVITY !!

    Starts by finding the extremum point of path (a pixel with a single 8-way connection).
    Then trace coordinates from extremum point to nearest neighbors to construct the path.

    Args:
        img (np.ndarray): Binary image w/ single pixel width path to be sorted (path value == 1)
    
    Returns:
        sorted_ (list): List of sorted coordinates for single edge like:
            [[x11, y11], [x12, y12], ...]
    """
    # Validate image is a binary image
    if not isinstance(img, np.ndarray):
        raise TypeError("img must be numpy array")
    if not is_single_pixel_width(img, conn=8):
        raise ValueError(f"Image must have single pixel width edge with strictly 8-way connectivity")
    for ele in np.unique(img):
        if ele not in [0,1]:
            raise ValueError(f"img must be binary valued on 0-1")
    # Evaluate number of edges. Should have 2 components 0 is bg and 1 is edge
    n_labels, _ = cv2.connectedComponents(img.astype(np.uint8))
    if n_labels - 1 != 1:
        raise ValueError(f'Invalid number of edges to sort for trajectory: {n_labels -1}. Must have 1 edge to sort.')
    
    # get list of mask points
    pnts = np.transpose(np.where(img == 1))
    num_points = len(pnts)
    extremum_found = False
    
    # Get extreme point if it exists
    for p in pnts:
        x = p[1]
        y = p[0]
        neighbors = get_neighbors_8way(img, x, y)
        # An extremum point has only 1 neighbor
        if len(neighbors) == 1:
            extremum_found = True
            break
    # If no extremum found, then designate the starting point as an arbitray point (and
    # delete one of its nearest neighbors to turn it into an extremum point)
    if extremum_found is False:
        x = pnts[0][1]
        y = pnts[0][0]
        neighbors = get_neighbors_8way(img, x, y)
        # compute distance between current pixel and its neighbors
        norms = [np.linalg.norm(np.array([y,x]) - np.array([ele[0],ele[1]])) for ele in neighbors]
        try:
            nn_ind = np.argmin(np.asarray(norms))
        except:
            raise
        # Remove nearest pixel to remove the 4-way (or 8way) connectivity
        nearest_neigh = neighbors[nn_ind]
        img = np.copy(img)
        img[nearest_neigh[0], nearest_neigh[1]] = 0
    # trace path to build a sorted list
    sorted_ = sort_path_from_start(img, x_start=x, y_start=y)

    return sorted_

def sort_path_from_start(img:np.ndarray, x_start:int, y_start:int)->list[ list[int, int]]:
    """
    Sort along path in binary image w/ single pixel width path starting at an
    extremum point.

    Args:
        img (np.ndarray): Binary image w/ single pixel width path to be sorted (path value == 1)
        x_start (int): x coordinate of extreumum point to start sorting
        y_start (int): y coordinate of extremum point to start sorting
        start_point (list): Extremum point to start sorting as [x, y]
    
    Returns:
        sorted_ (list): nx2 list of sorted coordinates as [x, y]
    """
    img = np.copy(img)
    sorted_ = []
    num_points = np.sum(img>0)
    # Move from starting point along path to nearest neighbors while deleting previous pixels
    for i in range(num_points):
        # If starting sort, set first sorted position as the passed startign point
        if i == 0:
            x = x_start
            y = y_start
        # If at last positoin break
        elif i == num_points -1:
            break
        # Compare distance between neighbor pixels and choose closest as the next position
        else:
            neigh = get_neighbors_8way(img, x, y)
            # If only 1 neighbor pixel set next sorted pixel as the neigbor as such
            if len(neigh) == 1:
                y = neigh[0][0]
                x = neigh[0][1]
            # Otherwise se tto the closest one
            else:
                norms = [np.linalg.norm(np.array([y,x]) - np.array([ele[0],ele[1]])) for ele in neigh]
                try:
                    nn_ind = np.argmin(np.asarray(norms))
                except ValueError as e:
                    raise EdgeNotFoundError(f"Error while sorting detected edges: {e}")
                if len(np.where(norms == norms[nn_ind])) > 1:
                    raise EdgeNotFoundError(f"Multiple norms with the minimum value. Norms: {norms}")
                y = neigh[nn_ind][0]
                x = neigh[nn_ind][1]
        img[y,x] = 0
        sorted_.append([x, y])
    return sorted_

def get_neighbors_4way(img:np.ndarray, x:int, y:int)->list:
    """
    Return pixel coordinates of nonzero neighboring pixels to `x` and `y` in
    `img` with 4-way connectivity as a list like [[y,x]..].
    
    Args:
        img (np.ndarray): Image to assess
        x (int): x-coordiante to assess for neighbors
        y (int): y-coordinate to assess for neightbors
    
    Returns:
        list of pixel coords as [row, col] (y, x)
    """
    try:
        if img[y - 1,x] > 0: n_.append((y-1, x))
    except:
        pass
    try:
        if img[y,x - 1] > 0: n_.append((y, x - 1))
    except:
        pass
    try:
        if img[y,x + 1] > 0: n_.append((y, x + 1))
    except:
        pass
    try:
        if img[y + 1,x] > 0: n_.append((y+1, x))
    except:
        pass
    return n_

def get_neighbors_8way(img:np.ndarray, x:int, y:int)->list:
    """
    Return pixel coordinates of nonzero neighboring pixels to `x` and `y` in
    `img` with 8-way connectivity as a list like [[y,x]..].
    
    Args:
        img (np.ndarray): Image to assess
        x (int): x-coordiante to assess for neighbors
        y (int): y-coordinate to assess for neightbors
    
    Returns:
        list of pixel coords as [row, col] (y, x)
    """
    n_ = []
    try:
        if img[y - 1,x] > 0: n_.append((y-1, x))
    except:
        pass
    try:
        if img[y - 1,x - 1] > 0: n_.append((y-1, x - 1))
    except:
        pass
    try:
        if img[y - 1,x + 1] > 0: n_.append((y-1, x + 1))
    except:
        pass
    try:
        if img[y,x - 1] > 0: n_.append((y, x - 1))
    except:
        pass
    try:
        if img[y,x + 1] > 0: n_.append((y, x + 1))
    except:
        pass
    try:
        if img[y + 1,x] > 0: n_.append((y+1, x))
    except:
        pass
    try:
        if img[y + 1,x - 1] > 0: n_.append((y+1, x - 1))
    except:
        pass
    try:
        if img[y + 1,x + 1] > 0: n_.append((y+1, x + 1))
    except:
        pass
    return n_
""" General utility functions used in edge classification """
import numpy as np
from src.deep_learning.edge_utils.error_utils import EdgeNotFoundError

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

def sort_path_along_binary_trajectory(img:np.ndarray)->list:
    """
    Sorts points in binary image with a single pixel width path (value == 1), so
    that the sorted points could be traced consecutively.

    Starts by finding the extremum point of path (a pixel with a single 4-way connection).
    Then trace coordinates from extremum point to nearest neighbors to construct the path.
    All pixels should have 4-way connectivity so the nearest neighbor (ignoring the traced
    path) is the pixel w/ 4-way connectivity to the pixel of interest

    Args:
        img (np.ndarray): Binary image w/ single pixel width path to be sorted (path value == 1)
    
    Returns:
        sorted_ (list): nx2 list of sorted coordinates as [y, x]
    """
    # Validate image is a binary image
    if not isinstance(img, np.ndarray):
        raise TypeError("img must be numpy array")
    for ele in np.unique(img):
        if ele not in [0,1]:
            raise ValueError(f"img must be binary valued on 0-1")
    
    # get list of mask points
    pnts = np.transpose(np.where(img == 1))
    num_points = len(pnts)
    extremum_found = False
    
    # Get extreme point if it exists
    for p in pnts:
        x = p[1]
        y = p[0]
        neighbors = get_neighbors_4way(img, x, y)
        # An extremum point has only 1 neighbor
        if len(neighbors) == 1:
            ex = [y, x]
            extremum_found = True
            break
    # If no extremum found, then designate the starting point as an arbitray point (and
    # delete one of its nearest neighbors to turn it into an extremum point)
    if extremum_found is False:
        x = pnts[0][1]
        y = pnts[0][0]
        neighbors = get_neighbors_4way(img, x, y)
        # compute distance between current pixel and its neighbors
        norms = [np.linalg.norm(np.array([y,x]) - np.array([ele[0],ele[1]])) for ele in neighbors]
        try:
            nn_ind = np.argmin(np.asarray(norms))
        except:
            raise
        # Set extremum to be nearest neighbor and remove original pixel to remove the 4-way connectivity
        ex = neighbors[nn_ind]
        img = np.copy(img)
        img[y,x] = 0
    # trace path to build a sorted list
    sorted_ = sort_path_from_start(img,ex)

    return sorted_

def sort_path_from_start(img:np.ndarray, start_point:list)->list:
    """
    Sort along path in binary image w/ single pixel width path starting at an
    extremum point.

    Args:
        img (np.ndarray): Binary image w/ single pixel width path to be sorted (path value == 1)
        start_point (list): Extremum point to start sorting as [y, x]
    
    Returns:
        sorted_ (list): nx2 list of sorted coordinates as [y, x]
    """
    img = np.copy(img)
    sorted_ = []
    num_points = np.sum(img>0)
    # Move from starting point along path to nearest neighbors while deleting previous pixels
    for i in range(num_points):
        # If starting sort, set first sorted position as the passed startign point
        if i == 0:
            y = start_point[0]
            x = start_point[1]
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
        sorted_.append([y,x])
    return sorted_

def get_neighbors_4way(img,x,y):
    n_ = []
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

def get_neighbors_8way(img,x,y):
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
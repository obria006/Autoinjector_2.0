''' Functions for miscellaneous validations (for arugments/ inputs) '''
import math
import numpy as np

def is_valid_number(val, inc_nan:bool=False)->bool:
    '''
    Returns true if can convert argument to a float. If inc_nan is True then 
    NaN is considered a valid number.
    
    Note, this doesn't indicate a valid Type, just that the argument can be
    cast to a float. For instance str(5) is considered a valid number.
    
    Arguments:
        val (Any): Value to test if it is a valid number
        inc_nan (bool): Boolean for considering NaN as a valid number
        
    Returns:
        Boolean indicator whether arument is a valid number
    '''
    try:
        float(val)
    except:
        return False
    else:
        if inc_nan is False and math.isnan(float(val)) is True:
            return False
        else:
            return True

def is_of_types(obj, types:list) -> bool:
    '''
    Returns true if object is an instance of any type in 'types'

    Arguments:
        obj (Any): Object to test against types
        types (list): List of types to evalute obj against

    Returns:
        Boolean indicator whether obj is consistent with any of the 'types'
    '''
    is_types = False
    for type_ in types:
        if isinstance(obj,type_):
            is_types = True
    return is_types

def val_binary_image(mask:np.ndarray):
    """
    Returns true if binary image else false

    Args:
        mask (np.ndarray): image to evaluate
    """
    if not isinstance(mask, np.ndarray):
        raise TypeError(f"Invalid mask type: {type(mask)}. Must be np.ndarray")
    if len(mask.shape)>2:
        raise ValueError("Mask image must have depth=1.")
    vals = np.unique(mask)
    for val in vals:
        if val not in [0, 1]:
            raise ValueError("Mask must be binary image")
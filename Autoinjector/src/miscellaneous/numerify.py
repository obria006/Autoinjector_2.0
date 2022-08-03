""" Functions for making data numerical """
from src.miscellaneous.validify import is_valid_number

def flint_type(number):
    '''
    Casts a valid number (str, int, float) to a float or int depending on the
    precision of the number. Non-zero numbers after decimal place are floats
    while strictly zeros after decimal are ints.
    
    flint_type(12.2) -> 12.2

    flint_type(12.0) -> 12
    
    flint_type(12)   -> 12
    '''
    if not is_valid_number(number):
        raise TypeError(f"flint_type accepts only valid number. {number} is an invalid number.")
    return int(float(number)) if int(float(number)) == float(number) else float(number)

def str_or_flint_type(value):
    '''
    Casts value to string, int, or float type
    '''
    if not (isinstance(value,int) or isinstance(value,float) or isinstance(value,str)):
        raise TypeError(f"Value {value} must be str, int, or float type. Not {type(value)}.")
    
    if is_valid_number(value,inc_nan=False):
        ret = flint_type(value)
    elif is_valid_number(value, inc_nan=True):
        ret = float(value)
    else:
        ret = value

    return ret
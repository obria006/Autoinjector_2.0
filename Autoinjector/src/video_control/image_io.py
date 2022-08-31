''' Functions to handle loading/saving images '''
import os
from pathlib import Path
from PIL import Image
import numpy as np
from src.miscellaneous import validify as val

def save_image(image:np.ndarray, filepath:str):
    '''
    Saves image to path

    Arguments: 
        image (np.ndarray): Image to save to file
        filepath (str): Location to save image
    '''
    # Validate args
    if val.is_of_types(image, [np.ndarray]) is False:
        raise TypeError('image to save must be np.ndarray')
    if val.is_of_types(filepath, [str]) is False:
        raise TypeError('filepath must be string')
    if os.path.isdir(Path(filepath).parent.absolute()) is False:
        raise IOError(f'filepath parent directory not found: {filepath}')
    # Save image
    im = Image.fromarray(image)
    im.save(filepath,format='jpeg',quality=90)
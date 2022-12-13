""" Configuration IO functions """

import os
import yaml
from src.miscellaneous.standard_logger import StandardLogger as logr

def dict_to_yaml(filepath:str, data:dict):
    """
    Writes data in dictionary to a yaml file.
    
    Args:
        filepath (str): path to save yaml file. Must be in valid directory
        data (dict): data to write to file
    """
    if not os.path.isdir(os.path.dirname(filepath)):
        raise OSError(f"Invalid parent directory for YAML filepath: {filepath}. Directory does not exist.")
    if not isinstance(data, dict):
        raise TypeError(f"Invalid data type for data: {type(data)}. Must be a dictionary.")
    with open(filepath,'w') as stream:
        yaml.safe_dump(data, stream)

def yaml_to_dict(filepath:str) -> dict:
    """
    Reads data from a YAML file and returns a dictionary
    
    Args:
        filepath (str): path to save yaml file. Must be in valid directory
    
    Returns:
        dict of data from YAML file
    """
    if not os.path.exists(filepath):
        raise OSError(f"Invalid YAML filepath: {filepath}. File does not exist.")
    with open(filepath,'r') as stream:
        data = yaml.safe_load(stream)
    return data
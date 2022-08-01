''' For importing/exporting GUI configuration '''
from datetime import datetime
import traceback
import yaml
from src.miscellaneous.standard_logger import StandardLogger as logr

class YamlIOHandler():
    ''' Class for handling read-write of setup configuration'''

    def __init__(self,filepath:str=None):
        self.filepath = filepath
        self.logger = logr(__name__)

    def write_to_yaml_cfg(self,data=None):
        ''' dumps data to yaml filepath '''
        if self.filepath is None:
            self.logger.critical(f"YamlIOHandler filepath is none.")
            raise ValueError("YamlIOHandler filepath is none. filepath must not be None")
        if data is None:
            self.logger.critical(f"YamlIOHandler data is none.")
            raise ValueError("data is None. Must pass data to write_to_yaml_cfg()")

        try:
            with open(self.filepath,'w') as stream:
                yaml.safe_dump(data, stream)    # Write a YAML representation of data to 'document.yaml'.
            return True
        except Exception as e:
            self.logger.exception('Error writing cfg to yaml')
            print(traceback.format_exc())
            return False

    def read_from_yaml_cfg(self):
        ''' reads data to yaml filepath '''
        if self.filepath is None:
            self.logger.critical(f"YamlIOHandler filepath is none.")
            raise ValueError("YamlIOHandler filepath is none. filepath must not be None")

        try:
            with open(self.filepath,'r') as stream:
                data = yaml.safe_load(stream)    # Write a YAML representation of data to 'document.yaml'.
            return data
        except Exception as e:
            self.logger.exception('Error reading cfg from yaml')
            print(traceback.format_exc())
            return None

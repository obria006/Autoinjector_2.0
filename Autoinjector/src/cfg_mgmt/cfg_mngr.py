''' For managing the configurations '''
import os
from datetime import datetime
from pathlib import Path
import traceback
from src.cfg_mgmt.cfg_io import YamlIOHandler
from src.miscellaneous import validify as val
from src.miscellaneous.standard_logger import StandardLogger as logr

class CfgPointer():
    '''
    Class that points to the GUI configuration.

    All config files must be located in project_directory/configs
    '''

    def __init__(self):
        ''' Initializes pointer from filepath '''
        # Confirm a configs directory exists
        ptr_path = "configs/pointer.yaml"
        ptr_dir = Path(ptr_path).parent.absolute()
        if os.path.isdir(ptr_dir) is False:
            self.logger.critical(f'No "configs" directory found at: {ptr_dir}')
            raise IOError(f'No "configs" directory found at: {ptr_dir}')
        # Set attributes
        self.logger = logr(__name__)
        self.ptr_path = ptr_path
        self.ptr_data = {"Latest config path":"",
                        "Use default config":True}
        self.ptr_io = YamlIOHandler(filepath=self.ptr_path)

    def load_pointer(self):
        '''
        Load the pointer data
        '''
            
        # Attempt to load the pointer file
        try:
            self.ptr_data = self.ptr_io.read_from_yaml_cfg()
        # Error while loading pointer file so use hardcoded values
        except Exception as e:
            self.logger.exception(f'Error loading config pointer: {self.ptr_path}')
            print(traceback.format_exc())
            # TODO load defaults
        # Try loading the configuration file that was pointed to
        else:
            self.logger.info(f"Loaded cfg pointer: {self.ptr_path}")
    
    def save_pointer(self):
        '''
        Saves current configuration name in pointer file.
        '''
        try:
            self.ptr_io.write_to_yaml_cfg(data=self.ptr_data)
        except Exception as e:
            self.logger.exception(f'Error saving configuration pointer: {self.ptr_path}')
            print(traceback.format_exc())
        else:
            self.logger.info("Saved config pointer: "+self.ptr_path)
            self.logger.info(self.ptr_path + " --> " + self.ptr_data["Latest config path"])

    def set_pointer_data(self,filepath=None,defaults=None):
        '''
        Sets the configuration pointer data

        Arguments:
            filepath (str): Filepath of config file
            defaults (bool): True = use defaults. False = use file
        '''
        if filepath is not None:
            filepath = filepath.replace("\\","/")
            self.ptr_data["Latest config path"] = filepath
        if defaults is not None:
            if val.is_of_types(defaults, [bool]) is False:
                self.logger.error(f'defaults in set_pointer_data must be bool. {type(defaults)} is invalid.')
                raise ValueError(f'defaults in set_pointer_data must be bool. {type(defaults)} is invalid.')
            else:
                self.ptr_data["Use default config"] =  defaults
            
class CfgGUI():
    '''
    Class for holding configuration of the GUI
    '''

    def __init__(self):
        # Confirm a configs directory exists
        cfg_dir = Path("configs/pointer.yaml").parent.absolute()
        if os.path.isdir(cfg_dir) is False:
            self.logger.critical(f'No "configs" directory found at: {cfg_dir}')
            raise IOError(f'No "configs" directory found at: {cfg_dir}')
        # Set attributes
        self.logger = logr(__name__)
        self.cfg_dir = cfg_dir
        self.cfg_io = YamlIOHandler()
        self.values = {}
        self.default_cfg()

    def default_cfg(self):
        ''' Set config values to the default values '''
        self.values = {
            'data directory':'C:/Users/Public/Documents/envs/Autoinjector_2/Autoinjector/data'
        }
        self.logger.info('Set config values to defaults')
    
    def load_cfg(self, filename:str):
        ''' Load config values from a file '''
        # Valid type for filename
        if val.is_of_types(filename, [str]) is False:
            self.logger.error(f'filename must be string. {type(filename)} is invalid')
            raise TypeError('filename must be string. {type(filename)} is invalid')
        # Load the config values
        try:
            self.cfg_io.filepath = os.path.join(self.cfg_dir,filename).replace('\\', '/')
            self.values = self.cfg_io.read_from_yaml_cfg()
            self.logger.info(f'Loaded configuration from {self.cfg_io.filepath}')
        except:
            self.logger.exception(f'Failed to load cfg: {self.cfg_io.filepath}')
            print(traceback.format_exc())

    def save_cfg(self, filename:str):
        ''' Save config values to a file '''
        # Valid type for filename
        if val.is_of_types(filename, [str]) is False:
            self.logger.error(f'filename must be string. {type(filename)} is invalid')
            raise TypeError('filename must be string. {type(filename)} is invalid')
        # Save the values to the file
        try:
            self.cfg_io.filepath = os.path.join(self.cfg_dir,filename).replace('\\', '/')
            self.cfg_io.write_to_yaml_cfg(data=self.values)
            print('here')
            self.logger.info(f'Saved configuration to {self.cfg_io.filepath}')
        except:
            self.logger.exception(f'Failed to save cfg: {self.cfg_io.filepath}')
            print(traceback.format_exc())
        


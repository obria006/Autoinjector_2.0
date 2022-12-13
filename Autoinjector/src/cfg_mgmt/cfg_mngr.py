''' For managing the configurations '''
import os
from datetime import datetime
from pathlib import Path
from dataclasses import dataclass, asdict
from src.cfg_mgmt.definitions import CONFIG_DIR, CONFIG_PATH, CAMERA_PARAMS
from src.cfg_mgmt.io import yaml_to_dict, dict_to_yaml
from src.cfg_mgmt.cfg_io import YamlIOHandler
from src.miscellaneous import validify as val
from src.miscellaneous.standard_logger import StandardLogger as logr

class CfgManager():
    '''
    Class for managing the configuration values.
    '''

    def __init__(self):
        self._logger = logr(__name__)

        # Default the configuration dir
        cfg_dir = CONFIG_DIR
        if os.path.isdir(cfg_dir) is False:
            self._logger.critical(f'No "configs" directory found at: {cfg_dir}')
            raise IOError(f'No "configs" directory found at: {cfg_dir}')
        self.cfg_dir = cfg_dir
        self._cfg_io = YamlIOHandler()
        self._cfg_ptr = CfgPointer()
        self.values = Configuration() # the dataclass
        

    def from_pointer_file(self):
        '''
        Load the configuration from the pointer. If "defaults" = true in
        pointer, then loads default values
        '''
        # Load the pointer file
        self._cfg_ptr.load_pointer()
        # `Configuration` already has default values, so only set values if not using default
        if self._cfg_ptr.ptr_data["Use default config"] is False:
            cfg_path = self._cfg_ptr.ptr_data["Latest config path"]
            cfg_fname = os.path.basename(cfg_path)
            self.from_data_file(cfg_fname)        

    def from_data_file(self, filename:str, set_pointer:bool = False):
        '''
        Load config values from a YAML file

        Args:
            filename (str): Path to YAML configuration file
            set_pointer (bool): Whether to update pointer values. If true updates values
            with `filename` and default to False.
        '''
        # Read the configuraiton dictionary from file
        try:
            self._cfg_io.filepath = os.path.join(self.cfg_dir,filename).replace('\\', '/')
            cfg_dict = self._cfg_io.read_from_yaml_cfg()
        except:
            self._logger.exception(f'Failed to load cfg: {self._cfg_io.filepath}')
            return

        # Set the configuration values
        try:
            self.values.set_values_from_dict(cfg_dict)
        except:
            self._logger.exception(f'Failed set configuration values.')
        else:
            self._logger.info(f'Loaded configuration from {self._cfg_io.filepath}')

            # Set the pointer data if desired
            if set_pointer is True:
                fpath = self._cfg_io.filepath
                self._cfg_ptr.set_pointer_data(filepath=fpath, defaults=False)
                self._cfg_ptr.save_pointer()

    def to_data_file(self, filename:str = False, set_pointer:bool = False):
        '''
        Save config values to a YAML file

        Args:
            filename (str): Path to YAML configuration file
            set_pointer (bool): Whether to update pointer values. If true updates values
            with `filename` and default to False.
        '''
        try:
            self._cfg_io.filepath = os.path.join(self.cfg_dir,filename).replace('\\', '/')
            data_dict = asdict(self.values)
            self._cfg_io.write_to_yaml_cfg(data=data_dict)
        except:
            self._logger.exception(f'Failed to save cfg: {self._cfg_io.filepath}')
        else:
            self._logger.info(f'Saved configuration to {self._cfg_io.filepath}')

            # Set the pointer data if desired
            if set_pointer is True:
                fpath = self._cfg_io.filepath
                self._cfg_ptr.set_pointer_data(filepath=fpath, defaults=False)
                self._cfg_ptr.save_pointer()
        
        
class CfgPointer():
    '''
    Class that points to the GUI configuration.

    All config files must be located in project_directory/configs
    '''

    def __init__(self):
        ''' Initializes pointer from filepath '''
        self._logger = logr(__name__)
        # Confirm a configs directory exists
        ptr_path = f"{CONFIG_DIR}/pointer.yaml"
        ptr_dir = Path(ptr_path).parent.absolute()
        if os.path.isdir(ptr_dir) is False:
            self._logger.critical(f'No "configs" directory found at: {ptr_dir}')
            raise IOError(f'No "configs" directory found at: {ptr_dir}')
        # Set attributes
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
            self._logger.exception(f'Error loading config pointer: {self.ptr_path}')
            # TODO load defaults
        # Try loading the configuration file that was pointed to
        else:
            self._logger.info(f"Loaded cfg pointer: {self.ptr_path}")
    
    def save_pointer(self):
        '''
        Saves current configuration name in pointer file.
        '''
        try:
            self.ptr_io.write_to_yaml_cfg(data=self.ptr_data)
        except Exception as e:
            self._logger.exception(f'Error saving configuration pointer: {self.ptr_path}')
        else:
            self._logger.info("Saved config pointer: "+self.ptr_path)
            self._logger.info(self.ptr_path + " --> " + self.ptr_data["Latest config path"])

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
                self._logger.error(f'defaults in set_pointer_data must be bool. {type(defaults)} is invalid.')
                raise ValueError(f'defaults in set_pointer_data must be bool. {type(defaults)} is invalid.')
            else:
                self.ptr_data["Use default config"] =  defaults

@dataclass
class Configuration:
    """ Handles parameters for gui configuration """

    # The default values for Configuration
    micromanager_path: str = 'C:/Program Files/Micro-Manager-2.0'
    camera: str = "Hamamatsu Orca DCAM"
    com: str = "com3"
    sensapex_id: int = 1
    z_polarity: int = -1
    z_scaling: float = 0.741
    pullout_nm: int = 200000

    def __post_init__(self):
        """
        Called after initialization. Make the configuration directory if
        it doesn't exist.
        """
        if not os.path.isdir(CONFIG_DIR):
            os.makedirs(CONFIG_DIR)

    @classmethod
    def init_from_file(cls, filename:str = None):
        # Read the configuraiton dictionary from file
        if filename is None:
            fpath = CONFIG_PATH
        else:
            fpath = os.path.join(CONFIG_DIR, filename)
        data = yaml_to_dict(fpath)
        return cls(**data)

    def set_values_from_dict(self, dict_:dict)->None:
        """
        Set configuration values (attribtues of the `Configuration` dataclass) using
        keyword:value pairs in `dict_`. Keywords in `dict_` much match the attributes

        Args:
            dict_: dictionary of values to put as configuration values
        """
        # Preliminary validate all keys to enforce that all values in `dict_` are
        # set in the `Configuration` attrs. Without this, some but not all values
        # in `dict_` could be set before the erroneous key is caughts
        for key in list(dict_.keys()):
            if not hasattr(self, key):
                raise KeyError(f"Invalid configuration key: {key}. No configuration values set because the key does not exist in `Configuration` attributes")
        
        for key, value in dict_.items():
            setattr(self,key,value)

    def from_file(self, filename:str = None):
        '''
        Load config values from a YAML file

        Args:
            filename (str): Path to YAML configuration file
        '''
        # Read the configuraiton dictionary from file
        if filename is None:
            fpath = CONFIG_PATH
        else:
            fpath = os.path.join(CONFIG_DIR, filename)
        data = yaml_to_dict(fpath)
        self.set_values_from_dict(data)

    def to_file(self, filename:str = None):
        '''
        Save config values to a YAML file

        Args:
            filename (str): Path to YAML configuration file
        '''
        if filename is None:
            fpath = CONFIG_PATH
        else:
            fpath = os.path.join(CONFIG_DIR, filename)
        data = asdict(self)
        dict_to_yaml(fpath, data)

@dataclass
class CameraConfiguration:
    """ Handles parameters for gui configuration """

    # The default values for Configuration
    devicename: str
    brand: str
    devicevalue: str
    bins: str
    rotate: int
    bits: int
    scalefactor: float
    
    @classmethod
    def init_from_camera_name(cls, name:str):
        """
        Load camera configuration from defined camera parameters in
        definitions.py.
        
        Args:
            name (str): name of the camera in `CAMERA_PARAMS`
        """
        logger = logr(__name__)
        try:
            params = CAMERA_PARAMS[name]
            return cls(**params)
        except Exception as e:
            logger.exception(f"Could not load camera configuration. {e}")


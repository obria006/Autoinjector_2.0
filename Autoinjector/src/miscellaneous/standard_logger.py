''' Class for handling logging '''
import logging
from datetime import datetime
from src.miscellaneous import validify as val

class StandardLogger(logging.Logger):
    '''
    Defines a standard logger that can be used for most in autoinjector GUI.
    Streams all logger levels to the console, and streams warnings and above
    to a file. File is located at logs/YYYYMMDD.log

    Usage:
        # In the script/function that you want logging
        from src.miscellaneous.standard_logger import StandardLogger
        logger = StandardLogger(__name__)
    '''

    def __init__(self, name:str):
        '''
        Creates logging.Logger instance with the passed name

        Arguments:
            name (str): Name of Logger (usually passed as __name__ of the file
                the logger is located in)
        
        Returns:
            instance of logging.Logger
        '''
        super().__init__(name=name)
        if val.is_of_types(name, [str]) is False:
            raise TypeError('name of logger must be a string')
        # self = logging.getLogger(name)
        s_handler = logging.StreamHandler()
        s_handler.setLevel(logging.DEBUG)
        s_formatter = logging.Formatter('%(levelname)s - %(name)s: %(message)s')
        s_handler.setFormatter(s_formatter)
        self.addHandler(s_handler)
        now = datetime.now().strftime("%Y%m%d")
        log_path = f"Autoinjector/logs/{now}.log"
        f_handler = logging.FileHandler(log_path)
        f_handler.setLevel(logging.WARNING)
        f_formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s: %(message)s')
        f_handler.setFormatter(f_formatter)
        self.addHandler(f_handler)
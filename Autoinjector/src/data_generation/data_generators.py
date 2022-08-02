''' Generates various types of data from GUI operation '''
import os
from datetime import datetime
from PIL import Image
import pandas as pd
import numpy as np
from src.imageprocessing.image_io import save_image
from src.miscellaneous.standard_logger import StandardLogger as logr
from src.miscellaneous import validify as val

class PipTipData():
    ''' Class handles coordianted saving of pipette image and annotated tip '''

    def __init__(self, pip_data_dir:str):
        '''
        Initializes class for saving pipette data. Creates directory with date
        as name in the pip_data_DIR

        Arguments:
            pip_data_dir (str): Parent directory of where to save data
        '''

        self.logger = logr(__name__)
        
        # Ensure parent directory exists
        if os.path.isdir(pip_data_dir) is False:
            self.logger.error(f"Pipette image directory doesn't exist: {pip_data_dir}")
            raise ValueError(f"Pipette image directory doesn't exist: {pip_data_dir}")
        self.parent_dir = pip_data_dir.replace('\\','/')

        # Directory where images will be saved
        ymd = datetime.now().strftime("%Y%m%d")
        self.pip_tip_dir = f"{self.parent_dir}/{ymd}"

        # Make date directory if necessary
        if os.path.isdir(self.pip_tip_dir) is False:
            os.mkdir(self.pip_tip_dir)
            self.logger.info(f"Created pipette image directory: {self.pip_tip_dir}")

    def save_data(self, image:np.ndarray, tip_position:dict):
        '''
        Save the pipette image and coordinate of clicked pipette tip. Saves pipette
        coordinate dataframe of: [image path, x, y, norm x, norm y, img width, img height].
        Coordinate data saved to csv

        Arguments:
            image (np.ndarray): Image that was saved
            tip_position (dict): Coords of tip annotation in image {'x':x, 'y':y}
        '''

        # Validate arguments
        if val.is_of_types(image, [np.ndarray]) is False:
            raise TypeError('image must be an np.ndarray')
        if val.is_of_types(tip_position, [dict]) is False:
            raise TypeError('tip_postition must be a dict')
        tip_keys = list(tip_position.keys())
        if 'x' not in tip_keys or 'y' not in tip_keys:
            raise ValueError(f"tip_position must 'x' an 'y' keys. {tip_keys} is invalid.")

        # Time of saving
        ymd_hms = datetime.now().strftime("%Y%m%d_%H%M%S")
        ymd = datetime.now().strftime("%Y%m%d")

        # Save the pipette image
        try:
            img_path = f"{self.pip_tip_dir}/{ymd_hms}_pipette.jpeg"
            save_image(image, img_path)
        except:
            self.logger.exception(f'Failed to save pipette image.')
            raise
        else:
            self.logger.info(f'Saved pipette image: {img_path}')

        # Save the pipette data for individual image
        try:
            tip_df = self._make_coord_df(img_path, image, tip_position)
            coord_path = f"{self.pip_tip_dir}/{ymd_hms}_coordinates.csv"
            tip_df.to_csv(coord_path)
        except:
            self.logger.exception(f'Failed to save pipette coordinates.')
            raise
        else:
            self.logger.info(f'Saved pipette coords: {coord_path}')

        # Save the pipette to master file (compilation of )
        try:
            tip_df = self._make_coord_df(img_path, image, tip_position)
            coord_path = f"{self.pip_tip_dir}/{ymd}_master_coordinates.csv"
            if os.path.exists(coord_path):
                tip_df.to_csv(coord_path, mode='a', index=False, header=False)
            else:
                tip_df.to_csv(coord_path, index=False)
        except:
            self.logger.exception(f'Failed to save pipette coordinates.')
            raise
        else:
            self.logger.info(f'Saved pipette coords: {coord_path}')

    def _make_coord_df(self, img_path:str, image:np.ndarray, tip_position:dict):
        '''
        Makes dataframe of: [image path, x, y, norm x, norm y, img width, img height]

        Arguments:
            img_path (str): Path for where image was saved
            image (np.ndarray): Image that was saved
            tip_position (dict): Coords of tip annotation in image {'x':x, 'y':y}
        
        Returns:
            Dataframe of [image path, x, y, norm x, norm y, img width, img height]
        '''
        # Get x and y coordinates of annoation
        x = tip_position['x']
        y = tip_position['y']
        # Get image size
        rows = image.shape[0]
        cols = image.shape[1]
        # Normalize the x and y coords by image size
        norm_x = x / cols
        norm_y = y / rows
        # Make data dict and DataFrame
        data = {'filepath':[img_path],
                'x':[x],
                'y':[y],
                'normalized x':[norm_x],
                'normalized y':[norm_y],
                'image width':[cols],
                'image height':[rows]}
        df = pd.DataFrame(data=data)
        return df

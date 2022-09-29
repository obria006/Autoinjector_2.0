''' Generates various types of data from GUI operation '''
import os
from datetime import datetime
from PIL import Image
import pandas as pd
import numpy as np
import tifffile
from src.video_control.image_io import save_image
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

        # Save the data
        try:
            # Pipette image
            img_path = f"{self.pip_tip_dir}/{ymd_hms}_pipette.jpeg"
            save_image(image, img_path)

            # Pipette coordinates
            tip_df = self._make_coord_df(img_path, image, tip_position)
            coord_path = f"{self.pip_tip_dir}/{ymd_hms}_coordinates.csv"
            tip_df.to_csv(coord_path)

            # master file
            ip_df = self._make_coord_df(img_path, image, tip_position)
            coord_path = f"{self.pip_tip_dir}/{ymd}_master_coordinates.csv"
            if os.path.exists(coord_path):
                tip_df.to_csv(coord_path, mode='a', index=False, header=False)
            else:
                tip_df.to_csv(coord_path, index=False)
        except:
            self.logger.exception(f'Error during saving pipette data.')
            raise
        else:
            self.logger.info(f'Saved pipette data: {coord_path}')

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

class TissueEdgeData():
    ''' Class handles coordinated saving of tissue image and annotated trajectory '''

    def __init__(self, tis_data_dir:str):
        '''
        Initializes class for saving tissue data. Creates directory with date
        as name in the tis_data_dir

        Arguments:
            tis_data_dir (str): Parent directory of where to save data
        '''

        self.logger = logr(__name__)

        # Ensure parent directory exists
        if os.path.isdir(tis_data_dir) is False:
            self.logger.error(f"Tissue image directory doesn't exist: {tis_data_dir}")
            raise ValueError(f"Tissue image directory doesn't exist: {tis_data_dir}")
        self.parent_dir = tis_data_dir.replace('\\','/')

        # Directory where images will be saved
        ymd = datetime.now().strftime("%Y%m%d")
        self.tis_anot_dir = f"{self.parent_dir}/{ymd}"

        # Make date directory if necessary
        if os.path.isdir(self.tis_anot_dir) is False:
            os.mkdir(self.tis_anot_dir)
            self.logger.info(f"Created pipette image directory: {self.tis_anot_dir}")

    def save_data(self, image:np.ndarray, raw_annot:np.ndarray, interpolate_annot:np.ndarray):
        '''
        Save tissue image and annotation of edge directory. Saves tissue coordinates in dataframe
        of [image path, x, y, norm x, norm y, img width, img height]. Coordinate data saved to csv.

        Arguments: 
            image (np.ndarray): Image that was saved
            raw_annot (np.ndarray): nx2 array [x, y] of drawn annotation coordinates in image
            interpolate_annot (np.ndarray): nx2 array [x, y] of interpolated annotation coordinates in image
        '''

        # Validate input arguments
        if val.is_of_types(image, [np.ndarray]) is False:
            raise TypeError('image must be an np.ndarray')
        if val.is_of_types(raw_annot, [np.ndarray]) is False:
            raise TypeError('raw annotation must be an np.ndarray')
        if raw_annot.shape[1] != 2:
            raise ValueError('raw annotation must be nx2 array')
        if val.is_of_types(interpolate_annot, [np.ndarray]) is False:
            raise TypeError('interpolated annotation must be an np.ndarray')
        if interpolate_annot.shape[1] != 2:
            raise ValueError('interpolated  annotation must be nx2 array')

        # Time of saving
        ymd_hms = datetime.now().strftime("%Y%m%d_%H%M%S")
        ymd = datetime.now().strftime("%Y%m%d")

        # Tissue data
        try:
            # Image
            img_path = f"{self.tis_anot_dir}/{ymd_hms}_tissue.jpeg"
            save_image(image, img_path)

        # Save the tissue data for the raw edge annotation
            tis_df = self._make_coord_df(img_path, image, raw_annot)
            coord_path = f"{self.tis_anot_dir}/{ymd_hms}_raw_annotation.csv"
            tis_df.to_csv(coord_path)

        # Save the tissue data for the interpolated edge annotation
            tis_df = self._make_coord_df(img_path, image, interpolate_annot)
            coord_path = f"{self.tis_anot_dir}/{ymd_hms}_interpolate_annotation.csv"
            tis_df.to_csv(coord_path)
        except:
            self.logger.exception(f'Failed to save tissue data.')
            raise
        else:
            self.logger.info(f'Saved tissue data: {coord_path}')


    def _make_coord_df(self, img_path:str, image:np.ndarray, annot:np.ndarray):
        '''
        Makes dataframe of: [image path, x, y, norm x, norm y, img width, img height]

        Arguments:
            img_path (str): Path for where image was saved
            image (np.ndarray): Image that was saved
            annot (np.ndarray): nx2 annotation coordinates [x, y]
        
        Returns:
            Dataframe of [image path, x, y, norm x, norm y, img width, img height]
        '''

        # Get x and y coordinates of annoation
        x = annot[:,0]
        y = annot[:,1]
        # Get image size
        rows = image.shape[0]
        cols = image.shape[1]
        # Normalize the x and y coords by image size
        norm_x = x / cols
        norm_y = y / rows
        # Make data dict and DataFrame
        data = {'filepath':[img_path]*len(x),
                'x':x.tolist(),
                'y':y.tolist(),
                'normalized x':norm_x.tolist(),
                'normalized y':norm_y.tolist(),
                'image width':[cols]*len(x),
                'image height':[rows]*len(x)}
        df = pd.DataFrame(data=data)
        return df

class ImageStackData():
    ''' Class handles coordianted saving image stack '''

    def __init__(self, data_dir:str):
        '''
        Initializes class for saving data. Creates directory with date
        as name in the data_dir

        Arguments:
            data_dir (str): Parent directory of where to save data
        '''

        self.logger = logr(__name__)
        
        # Ensure parent directory exists
        if os.path.isdir(data_dir) is False:
            self.logger.error(f"Z-stack directory doesn't exist: {data_dir}")
            raise ValueError(f"Z-stack directory doesn't exist: {data_dir}")
        self.parent_dir = data_dir.replace('\\','/')

        # Directory where images will be saved
        ymd = datetime.now().strftime("%Y%m%d")
        self.ymd_data_dir = f"{self.parent_dir}/{ymd}"

        # Make date directory if necessary
        if os.path.isdir(self.ymd_data_dir) is False:
            os.mkdir(self.ymd_data_dir)
            self.logger.info(f"Created z-stack directory: {self.ymd_data_dir}")

    def save_data(self, image_stack:np.ndarray, z_height:list):
        '''
        Save the image stack as a tiff and an associated csv file that contains
        the z-coordinates of each image in the stack.

        Arguments:
            image (np.ndarray): Image to save
            z_height (list): z-coordinates of each image in the z-stack
        '''

        # Validate arguments
        if val.is_of_types(image_stack, [np.ndarray]) is False:
            raise TypeError('image must be an np.ndarray')
        if val.is_of_types(z_height, [list]) is False:
            raise TypeError('z_height must be alist')
        if image_stack.shape[0] != len(z_height):
            raise ValueError(f"the depth of the image stack ande elements of the z-height must be equivalent")

        # Time of saving
        ymd_hms = datetime.now().strftime("%Y%m%d_%H%M%S")
        ymd = datetime.now().strftime("%Y%m%d")

        # Save the data
        try:
            img_path = f"{self.ymd_data_dir}/{ymd_hms}_zstack.tif"
            tifffile.imwrite(img_path, image_stack, photometric='minisblack')
            df = self._make_coord_df(img_path, z_height)
            df_path = f"{self.ymd_data_dir}/{ymd_hms}_zstack_heights.csv"
            df.to_csv(df_path)

        except:
            self.logger.exception(f'Error during saving z-stack data.')
            raise
        else:
            self.logger.info(f'Saved z-stack data: {df_path}')

    def _make_coord_df(self, img_path:str, z_height:list):
        '''
        Makes dataframe of: [image path, slice, z_height]

        Arguments:
            img_path (str): Path for where image was saved
            tz_height (list): z-coordinates of each image in the z-stack
        
        Returns:
            Dataframe of [image path, slice, z_height]
        '''
        # Make data dict and DataFrame
        img_paths = [img_path]*len(z_height)
        slices = list(range(len(z_height)))
        data = {'filepath':img_paths,
                'slice':slices,
                'z':z_height}
        df = pd.DataFrame(data=data)
        return df

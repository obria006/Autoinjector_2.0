""" Script for MVC impimentation of tissue detection """
import sys
import os
import numpy as np
import pandas as pd
import tifffile
from matplotlib import pyplot as plt
from PyQt6.QtCore import QObject
from src.deep_learning.edge_utils.error_utils import EdgeNotFoundError
from src.miscellaneous.standard_logger import StandardLogger
from src.deep_learning.edge_postprocessing import reachable_edges, largest_connected_component
from src.deep_learning.edge_utils.general import sort_path_along_binary_trajectory
from src.deep_learning.predict import Predicter
from src.deep_learning.inference import TissueEdgeClassifier, SegmenterWrapper


class ModelTissueDetection():
    """ Tissue detection model for MVC architecture """

    def __init__(self, model_ckpt:str):
        """
        Args:
            model_ckpt (str): Path to file containing model weights
        """
        self._logger = StandardLogger(__name__)
        if not os.path.exists(model_ckpt):
            raise OSError(f"Invalid segmenter model directory: {model_ckpt}")
        segmenter = SegmenterWrapper(Predicter.from_ckpt(model_ckpt), in_size=(128, 128))
        self._classifier = TissueEdgeClassifier(segmenter)

    def detect(self, image:np.ndarray)->dict:
        """
        Perform tissue detection and edge classification

        Args:
            image (np.ndarray): Image to detect tissue/edges

        Returns:
            dictionary of:
                {'detection_data': pd.DataFrame,
                'segmentation_image': np.ndarray,
                'edge_image': np.ndarray}
        """
        if not isinstance(image, np.ndarray):
            raise TypeError(f"Invalid image type: {type(image)}. It must be numpy ndarray.")
        edge_cc, mask_cc, edge_df, new_mask = self._classifier.classify_img(image)
        detection_dict = {
            'detection_data': edge_df,
            'segmentation_image': mask_cc,
            'edge_image': edge_cc}
        return detection_dict
    
    def longest_reachable_edge_of_type(self, detection_dict:dict, edge_type:str, pip_orient_RH:float=None):
        """
        Post-process the detected edge and return the longest edge of a specific
        type (like 'apical' or 'basal') that can be reached by the pipette

        Args:
            detection_dict (dict): Dictionary output from `detect` as:
                {'detection_data': pd.DataFrame,
                'segmentation_image': np.ndarray,
                'edge_image': np.ndarray}
            edge_type (str): type of edge to return (like 'apical' or 'basal')
            pip_orient_RH (float): Orientation of pipette x-axis (projecting out and
            away from pipette tip) in degrees relative to RH coordinate system (positve is
            CCW from pointing to the right)

        Returns:
            edge_coords (np.ndarray): numpy array of pixel coordinates as [[x,y],...] (or None)
            err: Error raised during post-processing (or None)
        """
        # Get edge mask
        if edge_type.lower() == 'reachable':
            edge_mask = detection_dict['edge_image']>0
        else:
            edge_mask = self._edge_mask_of_type(detection_dict, edge_type)
        # Only return reachable edges
        mask_cc = detection_dict['segmentation_image']
        edge_mask = reachable_edges(edge_mask=edge_mask,
                                    tiss_mask=mask_cc>0,
                                    pip_orient_RH=pip_orient_RH,
                                    ang_thresh=20,
                                    bound_perc=0.1)
        # Get coordinates of edge from connected component edge image
        edge_coords = np.asarray(sort_path_along_binary_trajectory(edge_mask))
        # Convert from [rows (y), columns (x)] to [x, y] array
        edge_coords[:,[0,1]] = edge_coords[:,[1,0]]
        return edge_coords

    def longest_edge_of_type(self, detection_dict:dict, edge_type:str):
        """
        Post-process the detected edge and return the longest edge of a specific
        type (like 'apical' or 'basal')

        Args:
            detection_dict (dict): Dictionary output from `detect` as:
                {'detection_data': pd.DataFrame,
                'segmentation_image': np.ndarray,
                'edge_image': np.ndarray}
            edge_type (str): type of edge to return (like 'apical' or 'basal')

        Returns:
            edge_coords (np.ndarray): numpy array of pixel coordinates as [[x,y],...] (or None)
            err: Error raised during post-processing (or None)
        """
        # Get edge mask
        if edge_type.lower() == 'reachable':
            edge_mask = detection_dict['edge_image']>0
        else:
            edge_mask = self._edge_mask_of_type(detection_dict, edge_type)
        edge_mask = largest_connected_component(edge_mask)
        # Get coordinates of edge from connected component edge image
        edge_coords = np.asarray(sort_path_along_binary_trajectory(edge_mask))
        # Convert from [rows (y), columns (x)] to [x, y] array
        edge_coords[:,[0,1]] = edge_coords[:,[1,0]]
        return edge_coords

    def _edge_mask_of_type(self, detection_dict:dict, edge_type:str):
        """
        Return binary mask of the longest edge of specific type.

        Args:
            detection_dict (dict): Dictionary output from `detect` as:
                {'detection_data': pd.DataFrame,
                'segmentation_image': np.ndarray,
                'edge_image': np.ndarray}
            edge_type (str): type of edge to return (like 'apical' or 'basal')

        Returns:
            edge_mask (np.ndarray): Binary mask containing longest edge of type
        """
        # Unpack dictionary
        edge_df = detection_dict['detection_data']
        edge_cc = detection_dict['edge_image']
        # Validate that desired edge was detected
        existing_edges = np.unique(edge_df['semantic'].to_numpy())
        if edge_type not in existing_edges:
            raise EdgeNotFoundError(f"Could not find {edge_type} edge in tissue.")
        # Get the longest edge label
        desired_edge_df = edge_df[edge_df['semantic']==edge_type]
        largest_edge_size = np.amax(desired_edge_df['size'].to_numpy())
        ind = desired_edge_df.index[desired_edge_df['size']==largest_edge_size].tolist()[0]
        desired_edge_label = desired_edge_df.loc[ind,'edge']
        # make mask 
        edge_mask = edge_cc==desired_edge_label

        return edge_mask

if __name__ == "__main__":
    ckpt_path = "Autoinjector/src/deep_learning/weights/20220824_180000_Colab_gpu/best.pth"
    img_path = "T:/Autoinjector/latop_to_desktop_transfer/deep_learning/data/e15 g_000012.tif"
    # img_path = "T:/Autoinjector/latop_to_desktop_transfer/deep_learning/data/E4 b_000017.tif"
    # img_path = "T:/Autoinjector/latop_to_desktop_transfer/deep_learning/data/E4c_000033.tif"
    img_path = "T:/Autoinjector/latop_to_desktop_transfer/deep_learning/data/E4g_000019.tif"
    img_path = "C:/Users/Public/Documents/envs/Autoinjector_2/Autoinjector/data/tissue/annotation_images/20220915/20220915_152001_tissue.jpeg"
    from PIL import Image
    real_img = np.array(Image.open(img_path))
    MTC = ModelTissueDetection(ckpt_path)
    results = MTC.detect(real_img)
    # path to image
    # path to model
    # instantiate tissue detector
    # make prediction
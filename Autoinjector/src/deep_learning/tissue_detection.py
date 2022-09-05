""" Script for MVC impimentation of tissue detection """
import sys
import os
import numpy as np
import pandas as pd
import tifffile
from matplotlib import pyplot as plt
from PyQt6.QtCore import QObject
from src.deep_learning.edge_utils.error_utils import EdgeNotFoundError
from src.deep_learning.predict import Predicter
from src.deep_learning.inference import TissueEdgeClassifier, SegmenterWrapper


class ModelTissueDetection():
    """ Tissue detection model for MVC architecture """

    def __init__(self, model_ckpt:str):
        """
        Args:
            model_ckpt (str): Path to file containing model weights
        """
        if not os.path.exists(model_ckpt):
            raise OSError(f"Invalid segmenter model directory: {model_ckpt}")
        segmenter = SegmenterWrapper(Predicter.from_ckpt(model_ckpt), in_size=(128, 128))
        self._classifier = TissueEdgeClassifier(segmenter)

    def detect(self, image:np.ndarray, edge_type:str)->dict:
        """
        Perform tissue detection and edge classification

        Args:
            image (np.ndarray): Image to detect tissue/edges
            edge_type (str): Type of edge to detect like 'apical' or 'basal'

        Returns:
            dictionary of:
                {'detection_data': pd.DataFrame,
                'segmentation_image': np.ndarray,
                'edge_image': np.ndarray,
                'edge_coordinates': np.ndarray}
        """
        if not isinstance(image, np.ndarray):
            raise TypeError(f"Invalid image type: {type(image)}. It must be numpy ndarray.")
        edge_cc, mask_cc, edge_df, new_mask = self._classifier.classify_img(image)
        edge_coords = self._postprocess_edges(edge_cc=edge_cc, edge_df=edge_df, edge_type=edge_type)
        detection_dict = {
            'detection_data': edge_df,
            'segmentation_image': mask_cc,
            'edge_image': edge_cc,
            'edge_coordinates': edge_coords
        }

        return detection_dict

    def _postprocess_edges(self, edge_cc:np.ndarray, edge_df:pd.DataFrame, edge_type:str):
        """
        Process the detected edges and return the longest desired edge.

        Args:
            edge_cc (np.ndarray): Labeled image of detected edges
            edge_df (pd.DataFrame): Data of detection
            edge_type (str): Semantic label of edge to extract (like 'basal' or 'apical')
        """
        # Validate that desired edge was detected
        existing_edges = np.unique(edge_df['semantic'].to_numpy())
        if edge_type not in existing_edges:
            raise EdgeNotFoundError(f"Edge not found: {edge_type}. Detected edges include: {existing_edges.tolist()}")
        # Get the longest edge label
        desired_edge_df = edge_df[edge_df['semantic']==edge_type]
        largest_edge_size = np.amax(desired_edge_df['size'].to_numpy())
        ind = desired_edge_df.index[desired_edge_df['size']==largest_edge_size].tolist()[0]
        desired_edge_label = desired_edge_df.loc[ind,'edge']
        # Get coordinates of edge from connected component edge image
        # FIXME - convert to path of coordiantes that follows one after another vs left-to-right
        edge_coords = np.transpose(np.where(edge_cc==desired_edge_label)) # first column is row index and second column i
        # Convert from [rows (y), columns (x)] to [x, y] array
        edge_coords[:,[0,1]] = edge_coords[:,[1,0]]

        return edge_coords

if __name__ == "__main__":
    ckpt_path = "T:/Autoinjector/latop_to_desktop_transfer/deep_learning/dl4mia_tissue_unet/results/20220824_181000_Colab_gpu/best.pth"
    img_path = "T:/Autoinjector/latop_to_desktop_transfer/deep_learning/data/e15 g_000012.tif"
    real_img = tifffile.imread(img_path)
    MTC = ModelTissueDetection(ckpt_path)
    results = MTC.detect(real_img)
    # path to image
    # path to model
    # instantiate tissue detector
    # make prediction
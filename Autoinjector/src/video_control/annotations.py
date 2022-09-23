""" Classes and functions for handling injection target annotations """
from collections.abc import Iterable
from PyQt6.QtCore import QObject
from src.video_control import video_utils as utils


class AnnotationManager(QObject):
    """
    Class to manage the annotations in the GUI video display
    """

    def __init__(self):
        """
        Args:
        """
        super().__init__()
        self.reset_annotations()

    def add_annotation(self, annot:list):
        """
        Add annotation to annotation annotation dict

        Args:
            annot (list): Ordered list of annotation as [[x1,y1],[x2,y2],...]
        """
        inter_annot = self._interpolate_annotation(annot)
        self.annotation_dict['raw'].append(annot)
        self.annotation_dict['interpolated'].append(inter_annot)

    def rm_annotation_by_inds(self, inds):
        """
        Remove annotations from `annot_dict` by their indices

        Args:
            inds (int or Iterable): Indices of annotations to remove
        """
        # Validate int or iterable
        if not val.is_of_types(inds, [int, Iterable]):
            raise TypeError(f"Invalid index type: {type(inds)}. Must be int or Iterable.")
        # COnvert int to iterable
        if isinstance(inds, int):
            inds = [inds]
        # Validate indices
        n_inds = len(self.annotation_dict['raw'])
        for ind in inds:
            if ind<0 or ind>n_inds-1:
                raise IndexError(f"Invalid index value: {ind}. Index must be positive and less than the size of the annotation list ({n_inds})")
        # Remove annotations
        for ind in sorted(inds, reverse=True):
            del self.annotation_dict['raw'][ind]
            del self.annotation_dict['interpolated'][ind]

    def reset_annotations(self):
        """ Set annotations to be empty """
        self.annotation_dict = {'raw':[], 'interpolated':[]}

    def _interpolate_annotation(self, annot:list):
        """
        Conducts univariate spline interpolation on ordered x,y coordinates in
        `annot` list.

        Args:
            annot (list): Ordered list to interpolate as [[x1,y1],[x2,y2],...]

        Returns:
            list of interpolated coordinates as [[x1,y1],[x2,y2],...]
        """
        if len(annot) <=3:
            raise utils.AnnotationError("Annotation is too short.\n\nPlease make a longer annotation.")
        interpolated_pixels = utils.interpolate(annot)
        return interpolated_pixels
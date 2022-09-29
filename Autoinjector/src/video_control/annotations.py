""" Classes and functions for handling injection target annotations """
from collections.abc import Iterable
import numpy as np
from PyQt6.QtCore import QObject, pyqtSignal
from src.miscellaneous import validify as val
from src.video_control import video_utils as utils


class AnnotationManager(QObject):
    """
    Class to manage the annotations in the GUI video display

    Signals:
        annotation_changed (list): List of lists of interpolated annoation coordinates
            [[[x11,y11,z11], [x12,y12,z12]... ], [[x21,y21,z21],[x22,y22,z22],...], ... ]
        new_annotation: Emitted when a new annotation is added
    """
    annotation_changed = pyqtSignal(list)
    new_annotation = pyqtSignal()

    def __init__(self):
        """
        Args:
        """
        super().__init__()
        self.annotation_dict = {'raw':[], 'interpolated':[]}

    def add_annotation(self, annot:list):
        """
        Add annotation to annotation annotation dict

        Args:
            annot (list): Ordered list of annotation as [[x1,y1,z1],[x2,y2,z2],...]
        """
        if isinstance(annot, np.ndarray):
            annot = np.copy(annot).tolist()
        annot_dims = np.asarray(annot).shape
        if annot_dims[1] != 3:
            raise ValueError(f"Invalid annotation because not 3D. Annotation shape is {annot_dims}")
        inter_annot = self._interpolate_annotation(annot)
        self.annotation_dict['raw'].append(annot)
        self.annotation_dict['interpolated'].append(inter_annot)
        self.new_annotation.emit()
        self.annotation_changed.emit(self.annotation_dict['interpolated'])

    def get_annotations(self, type_:str, coords:str)->list:
        """
        Return list of all annotaitons of given `type_` with `coords`.

        Args:
            type_ (str): Type of annotation as 'raw' or 'interpolated'
            coords (str): Which coordinates to return as 'xy' or 'xyz'
        
        Returns
            list of annotations as [[x,y], ...] or [[x,y,z], ...] as
            depending on `coords`
        """
        # Validate correct args for annotation type and coordinates
        if type_ not in ['raw', 'interpolated']:
            raise ValueError(f"Invalid annotation type: {type_}. Must be in ['raw', 'interpolated']")
        if coords not in ['xy', 'xyz']:
            raise ValueError(f"Invalid annotation coords: {coords}. Must be in ['xy', 'xyz']")
        
        # Return empty list if the annotation is empty
        if len(self.annotation_dict) ==0:
            return list(self.annotation_dict[type_])

        # Return the xyz or xy coordinates 
        if coords == 'xyz':
            return list(self.annotation_dict[type_])
        elif coords == 'xy':
            # Only want to return xy coordiantes, but annot is 3D as [[x,y,z],...], so
            # slice out the only the xy coordiantes
            annots_xy = []
            for annot_xyz in self.annotation_dict[type_]:
                annot_xy = [xyz[0:2] for xyz in annot_xyz]
                annots_xy.append(annot_xy)
            return annots_xy
        else:
            raise NotImplementedError(f"Invalid annotation coords: {coords}. Must be in ['xy', 'xyz']")

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
        self.annotation_changed.emit(self.annotation_dict['interpolated'])
        

    def reset_annotations(self):
        """ Set annotations to be empty """
        self.annotation_dict = {'raw':[], 'interpolated':[]}
        self.annotation_changed.emit(self.annotation_dict['interpolated'])

    def _interpolate_annotation(self, annot:list):
        """
        Conducts univariate spline interpolation on ordered x,y coordinates in
        `annot` list.

        Args:
            annot (list): Ordered list to interpolate as [[x1,y1,z1],[x2,y2,z2],...]

        Returns:
            list of interpolated coordinates as [[x1,y1,z1],[x2,y2,z2],...]
        """
        if len(annot) <=3:
            raise utils.AnnotationError("Annotation is too short. Please make a longer annotation.")
        interpolated_pixels = utils.interpolate_3D(annot)
        return interpolated_pixels
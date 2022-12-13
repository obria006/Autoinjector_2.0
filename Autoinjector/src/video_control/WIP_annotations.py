"""
WORK IN PROGRESS annotations for converting the existing annotation class
that uses a dict to store annotations to dataclasses. Each annotation would
be an instance of the `Annotation` dataclass and these annotations would
be stored sequentially in `AnnotationSequence` dataclass.
"""

from dataclasses import dataclass, field
from typing import Union, List
import numpy as np
from src.video_control import video_utils as utils

@dataclass
class Annotation:
    """
    Data class for containing annotaion information including the name, the 
    raw annotation coordinates, and the interpolated coordinates.
    """
    name:str
    raw_x:list
    raw_y:list
    raw_z:list
    interpolated_x:list
    interpolated_y:list
    interpolated_z:list

    @classmethod
    def from_raw_annotation(cls, name:str, raw_annot:Union[list,np.ndarray]):
        """
        Instantiate an `Annotation` from a raw annotation and interpolate the 
        raw annotation to make an interpolated annotation for instantiation.

        Args:
            name (str): Name/type/mode of the annotation (like 'line' or 'points')
            raw_annot (list|np.ndarray): 3D annotation coordinates as [x, y, z] to be
            interpolated
        """ 
        # Validate 3D annotation
        annot_dims = np.asarray(raw_annot).shape
        if annot_dims[1] != 3:
            raise ValueError(f"Invalid annotation because not 3D. Annotation shape is {annot_dims}")

        # Interpolate the annotation
        inter_annot = utils.interpolate_3D(raw_annot)

        # Parse the annotations into x, y, z, coords
        raw_arr = np.asarray(raw_annot).reshape(-1,3)
        raw_x = raw_arr[:,0]
        raw_y = raw_arr[:,1]
        raw_z = raw_arr[:,2]
        inter_arr = np.asarray(inter_annot).reshape(-1,3)
        inter_x = inter_arr[:,0]
        inter_y = inter_arr[:,1]
        inter_z = inter_arr[:,2]

        return cls(
            name = name,
            raw_x = raw_x,
            raw_y = raw_y,
            raw_z = raw_z,
            interpolated_x = inter_x,
            interpolated_y = inter_y,
            interpolated_z = inter_z
        )

    def get_raw_coordinates(self, order_:str='xyz')->list:
        """
        Return list of raw annotation coordinates as specified by `order_`.
        Coordinates will be returned as list of lists (eg[[x,y], [x,y],.. ])
        even if a single coordinate is requested.

        Args:
            order_ (str): order of coordinates to return like 'xyz', or 'z'

        Returns:
            list of lists of raw coordinates
        """
        annot_dict = {'x':self.raw_x, 'y':self.raw_y, 'z':self.raw_z}
        arr = None
        for coord in order_.lower():
            assert ele in 'xyz', f'Invalid order: {order_}. Order must only contain "x", "y", and/or "z"'
            if arr is None:
                arr = np.array(annot_dict[coord]).reshape(-1,1)
            else:
                arr = np.conceatenate([arr, np.array(annot_dict[coord]).reshape(-1,1)], axis=1)
        return arr.tolist()

    def get_interpolated_coordinates(self, order_:str='xyz')->list:
        """
        Return list of interpolated annotation coordinates as specified by `order_`.
        Coordinates will be returned as list of lists (eg[[x,y], [x,y],.. ])
        even if a single coordinate is requested.

        Args:
            order_ (str): order of coordinates to return like 'xyz', or 'z'

        Returns:
            list of lists of interpolated coordinates
        """
        annot_dict = {'x':self.interpolated_x, 'y':self.interpolated_y, 'z':self.interpolated_z}
        arr = None
        for coord in order_.lower():
            assert ele in 'xyz', f'Invalid order: {order_}. Order must only contain "x", "y", and/or "z"'
            if arr is None:
                arr = np.array(annot_dict[coord]).reshape(-1,1)
            else:
                arr = np.conceatenate([arr, np.array(annot_dict[coord]).reshape(-1,1)], axis=1)
        return arr.tolist()


@dataclass
class AnnotationSequence:
    """ Data class for holding sequence of annotations for injection """
    annotations:List[Annotation] = field(default_factory=list)

    def add_annotation(self, annot_instance:Annotation):
        """ Append annotation to annotation sequence """
        self.annotations.append(annot_instance)

    def is_empty(self)->bool:
        """Returns true if `annotations` list is empty"""
        return len(self.annotations)==0

    def get_raw_coordinates(self, order_:str='xyz')->list:
        """
        Return list of raw annotation coordinates as specified by `order_`.
        Coordinates will be returned as list of lists (eg[[x,y], [x,y],.. ])
        even if a single coordinate is requested.

        Args:
            order_ (str): order of coordinates to return like 'xyz', or 'z'

        Returns:
            list of lists of raw coordinates
        """
        coords = []
        for annotation in self.annotations:
            coords.append(annotation.get_raw_coordinates(order_=order_))
        return coords

    def get_interpolated_coordinates(self, order_:str='xyz')->list:
        """
        Return list of interpolated annotation coordinates as specified by `order_`.
        Coordinates will be returned as list of lists (eg[[x,y], [x,y],.. ])
        even if a single coordinate is requested.

        Args:
            order_ (str): order of coordinates to return like 'xyz', or 'z'

        Returns:
            list of lists of interpolated coordinates
        """
        coords = []
        for annotation in self.annotations:
            coords.append(annotation.get_interpolated_coordinates(order_=order_))
        return coords


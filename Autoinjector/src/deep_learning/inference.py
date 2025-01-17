""" Functions/classes to perform inference and label tissue edges as apical vs basal """

import glob
import time
import random
import numpy as np
import cv2
from skimage.morphology import convex_hull_image
import tifffile
import pandas as pd
from matplotlib import pyplot as plt
from src.miscellaneous.standard_logger import StandardLogger
from src.deep_learning.predict import Predicter
from src.deep_learning.edge_utils.general import inv_dictionary
from src.deep_learning.edge_utils.error_utils import EdgeNotFoundError
from src.deep_learning.edge_utils.data_utils import mask_statistics
from src.deep_learning.edge_utils.img_utils import (
    overlay_image,
    rm_components_by_size,
)


class TissueEdgeClassifier:
    """
    Labels tissue edge as apical or basal

    Steps:
        1. Use segmentation model to generate mask of image
        2. Preprocess the mask to fill in holes and smooth edges
        3. Use gradient or dilation/subtration to get edges of mask
        4. Connected components to isolate distinct edges in image
        5. Convex hull for each connected component edge
        6. Copmute overlap of hull with mask for each edge convex hull
        7. classify edges hull-mask overlap > 50% = basal else apical
        8. Relabel edges with apical=1 basal=2
        9. Extract coordinates of the edges
    """

    def __init__(self, segmenter: Predicter):
        """
        Args:
            segmenter: Model to perform tissue segmentation
        """
        self._logger = StandardLogger(__name__)
        self.segmenter = segmenter
        self.edge_dict = {"apical": 1, "basal": 2}
        self._BG_5TH_PTILE = 0.038 # From segmentation statistics
        self._FG_5TH_PTILE = 0.043 # From segmentation statistics

    def classify_img(self, img: np.ndarray) -> dict:
        """
        Classify an image to label apical /basal edges and return dict
        of {'apical':apical_edge_list, 'basal':basal_edge_list}.

        Args:
            img (np.ndarray): Image of tissue to classify

        Returns:
            edge_cc: connected component labeled edge image
            mask_cc: connected component labeled mask/tissue image
            edge_df: dataframe of edge info (label, mask, overlap, size, and semantic)
            mask: segmented tissue mask
        """
        try:
            # Use segmenter to segment tissue into binary mask
            mask, activation = self.segmenter.predict(img)
            mask = mask.astype(np.uint8)

            # Classify edges of the mask as apical/basal
            edge_cc, mask_cc, edge_df = self.classify_mask(mask)
        except EdgeNotFoundError as e:
            raise
        except:
            raise
        else:

            return edge_cc, mask_cc, edge_df, mask

    def classify_mask(self, mask: np.ndarray) -> dict:
        """
        Classify an mask to label apical /basal edges and return dict
        of {'apical':apical_edge_list, 'basal':basal_edge_list}.

        Args:
            mask (np.ndarray): Tissue mask to classify

        Returns:
            edge_cc: connected component labeled edge image
            mask_cc: connected component labeled mask/tissue image
            edge_df: dataframe of edge info (label, mask, overlap, size, and semantic)
        """
        try:
            # preprocess mask to fill in holes
            processed_mask = self._preprocess_mask(mask)

            # Label edges of binary mask with 1 for apical and 2 for basal
            (edge_cc, mask_cc, edge_df) = self._label_mask_edge(processed_mask)

        except EdgeNotFoundError as e:
            raise
        except:
            raise
        else:

            return edge_cc, mask_cc, edge_df

    def _preprocess_mask(self, mask: np.ndarray) -> np.ndarray:
        """
        Preprocess mask by removing small fg/bg regions based on training mask statistics
        and smooth the mask edges

        Args:
            mask (np.ndarray): Tissue mask to preprocess

        Returns:
            preprocced: preprocessed mask image
        """
        # Remove spurious fg/bg regions based on statistics of training data
        preprocessed = rm_components_by_size(
            mask=mask, size_frac=self._BG_5TH_PTILE, smaller=True, region="bg"
        )
        preprocessed = rm_components_by_size(
            mask=preprocessed, size_frac=self._FG_5TH_PTILE, smaller=True, region="fg"
        )
        preprocessed = preprocessed.astype(np.uint8)

        # smooth edges
        preprocessed = cv2.medianBlur(preprocessed, ksize=7)

        return preprocessed

    def _associate_edge_to_mask(
        self, mask: np.ndarray, num_edge_labels: int, edge_cc: np.ndarray
    ):
        """
        Associates and edge label with a detected foreground region in `mask`.
        Returns dictionary associating labels and a connected component labeled
        `mask` image.

        Args:
            mask: detected foreground image
            num_edge_labels: number of labeled edges from connected components
            edge_cc: connected component edge labels

        Returns:
            edge_to_mask: dictionary associating edge label to mask label
            mask_cc: np.ndarray of connected components in `mask`
        """
        # Get mask and edge connected components
        num_mask_labels, mask_cc = cv2.connectedComponents(mask)

        # init dictionary to associate edge with mask
        edge_to_mask = {}

        # Loop over edges. If edge intersects with mask region, then associate
        # edge label with mask label
        for edge_label in range(1, num_edge_labels):
            roi_edge = edge_cc == edge_label
            for mask_label in range(1, num_mask_labels):
                roi_mask = mask_cc == mask_label
                if np.any(np.bitwise_and(roi_edge, roi_mask)):
                    edge_to_mask[edge_label] = mask_label
                    break

        return edge_to_mask, mask_cc

    def _label_mask_edge(self, mask: np.ndarray) -> np.ndarray:
        """
        Labels edges of mask as apical or basal.

        Apical edges are assumed to be concave (curves in towards tissue) and
        basal edges are assumed to be convex (curves away from tissue).
        Evaluated by computing area overlap of convex hull for each edge with
        the tissue mask.

        Classification scheme:
            2 edges: larger overlap is basal and smaller overlap is apical
            otherswise: overlap > 50% is basal and overlap < 50% is apical
            larger hull and mask overlap -> basal (other apical)

        Args:
            mask: binary mask of segmented tissue

        Returns:
            edge_cc: connected component labeled edge image
            mask_cc: connected component labeled mask/tissue image
            edge_df: dataframe of edge info (label, mask, overlap, size, and semantic)
        """
        # Get binary mask of mask edges
        edges = self._get_mask_edges(mask=mask)

        # get labeled connected components in image (can ignore background @ 0 index)
        num_labels, edge_cc, edge_stats, _ = cv2.connectedComponentsWithStats(edges)
        if num_labels == 1:
            raise EdgeNotFoundError("Could not detect any tissue edges.")
        edge_areas = list(edge_stats[1:, cv2.CC_STAT_AREA])
        edge_size = {label: area for label, area in enumerate(edge_areas, start=1)}

        # Associate edge with a mask region
        edge_to_mask, mask_cc = self._associate_edge_to_mask(
            mask=mask, num_edge_labels=num_labels, edge_cc=edge_cc
        )

        # Iterate through the first 2 labels (ignore 0 label since its background)
        # create dict of {'label1': overlap_ratio1, 'label2': overlap_ratio2}
        edge_to_overlap = {}
        for label in np.unique(edge_cc):
            if label == 0:
                continue
            edge_mask = edge_cc == label
            hull = convex_hull_image(edge_mask)
            overlap_ratio = self._hull_overlap_ratio(
                edge=edge_mask, hull=hull, mask=mask
            )
            edge_to_overlap[label] = overlap_ratio

        # Relabel the edges according to their overlap ratio and the edge_dict
        # attribute.
        edge_to_semantic = self._label_by_overlap_ratio(
            edge_cc=edge_cc, edge_to_overlap=edge_to_overlap, edge_to_mask=edge_to_mask
        )
        semantic_edges = self._semantic_edge_image(
            edge_cc=edge_cc, edge_to_semantic=edge_to_semantic
        )

        # Create dataframe of information
        edge_df = self._to_df(
            edge_size=edge_size,
            edge_to_mask=edge_to_mask,
            edge_to_overlap=edge_to_overlap,
            edge_to_semantic=edge_to_semantic,
        )

        return edge_cc, mask_cc, edge_df

    def _to_df(
        self,
        edge_size: dict,
        edge_to_mask: dict,
        edge_to_overlap: dict,
        edge_to_semantic: dict,
    ):
        """
        Create DataFrame of where each edge associated to its edge label, its size, the mask
        region it belongs to, its convex hull overlap with the mask, and its semanitc label.

        Args:
            edge_size (dict): Dictionary of edge sizes
            edge_to_mask (dict): Dictionary of accociating edge label to mask label
            edge_to_overlap (dict): Dictionary of accociating edge label to overlap ratio
            edge_to_semantic (dict): Dictionary of accociating edge label to semantic label

        Returns:
            pandas DataFrame indexed by edge label with corresponding edge information
                columns: ["edge", "size", "mask", "overlap", "semantic]
        """
        # Validate same keys
        size_keys = np.sort(np.array(list(edge_size.keys())))
        mask_keys = np.sort(np.array(list(edge_to_mask.keys())))
        overlap_keys = np.sort(np.array(list(edge_to_overlap.keys())))
        sem_keys = np.sort(np.array(list(edge_to_semantic.keys())))
        if not np.all(
            (size_keys == mask_keys)
            & (mask_keys == overlap_keys)
            & (overlap_keys == sem_keys)
        ):
            raise KeyError(f"All keys in must be same")

        # Create master dictionary
        edge_labels = [key for key in mask_keys]
        sizes = [edge_size[key] for key in size_keys]
        mask_labels = [edge_to_mask[key] for key in mask_keys]
        overlaps = [edge_to_overlap[key] for key in overlap_keys]
        sem_labels = [edge_to_semantic[key] for key in sem_keys]
        master_dict = {
            "edge": edge_labels,
            "size": sizes,
            "mask": mask_labels,
            "overlap": overlaps,
            "semantic": sem_labels,
        }

        # Dictioanry to dataframe
        df = pd.DataFrame(data=master_dict, index=edge_labels)

        return df

    def _get_mask_edges(self, mask: np.ndarray) -> np.ndarray:
        """
        Erode mask w/ 3x3 kernel and subtract it from the original mask.
        Results in the edge being the pixels exactly on the edge of the mask.

        Args:
            mask: The mask to dilate.

        Return:
            edges: numpy.ndarray of mask edges
        """
        kernel = cv2.getStructuringElement(shape=cv2.MORPH_RECT, ksize=(3, 3))
        eroded = cv2.erode(src=mask, kernel=kernel)
        edges = mask - eroded
        return edges

    def _hull_overlap_ratio(
        self, edge: np.ndarray, hull: np.ndarray, mask: np.ndarray
    ) -> float:
        """
        Computes area overlap between (edgeless) convex hull and (edgeless) mask.

        Args:
            edge: binary mask of edges
            hull: binary mask of convex hull
            mask: binary mask of tissue

        Return:
            area_ratio: ratio of overlap between convex hull and mask
        """
        # compute the edgeless hull and its area
        ne_hull = (hull == 1) & (edge == 0)
        ne_hull_area = np.sum(ne_hull)

        # Compute overlap area between hull and mask and ratio of overlap to hull
        ne_overlap_area = np.sum((mask == 1) & (ne_hull == 1))
        area_ratio = ne_overlap_area / ne_hull_area

        return area_ratio

    def _label_by_overlap_ratio(
        self, edge_cc: np.ndarray, edge_to_overlap: dict, edge_to_mask: dict
    ) -> dict:
        """
        Relabel connected component edges based on the convex hull and mask overlap.

        if 2 edges on a single tissue region:
            basal = larger overlap and apical = smaller overlap
        else:
            50% overlap = basal else apical

        Args:
            edge_cc: np.ndarray of connected component edges (can have at most 2 edges)
            edge_to_overlap: dict of overlap ratios for each edge label
            edge_to_mask: dict associating edge label with a fg region (mask) label

        Returns:
            dict assoicating connected components label with tissue type label ('apical' or 'basal')
        """
        # Ensure each edge has a overlap ratio
        edge_labels = np.sort(np.unique(edge_cc))[1:]  # ignore the 0 index since bg
        overlap_keys = np.sort(np.array(list(edge_to_overlap.keys())))
        mask_keys = np.sort(np.array(list(edge_to_mask.keys())))
        if not np.all((edge_labels == overlap_keys) & (overlap_keys == mask_keys)):
            raise ValueError("All edges must have an overlap ratio and region")

        # Invert dictionary to get dictionary of {mask_label:[edge_label1, edge_label2,...],...}
        region_to_edge = inv_dictionary(edge_to_mask)

        # Dictionary to store the edge type for each edge
        edge_to_semantic = {}

        # Iterate through each edge and assign a semantic label
        for label, overlap in edge_to_overlap.items():
            edge_region = edge_to_mask[label]
            regions_edges = region_to_edge[edge_region]
            # 1 edge on tissue: 50% overlap or more = basal, else apical
            if len(regions_edges) == 1:
                if overlap > 0.5:
                    edge_to_semantic[label] = "basal"
                else:
                    edge_to_semantic[label] = "apical"
            # 2 edge on tissue: larger overlap = basal, smaller overlap = apical
            elif len(regions_edges) == 2:
                other_edge_label = np.setxor1d(
                    np.array(regions_edges), np.array(label)
                )[0]
                other_overlap = edge_to_overlap[other_edge_label]
                if overlap > other_overlap:
                    edge_to_semantic[label] = "basal"
                else:
                    edge_to_semantic[label] = "apical"
            # more than 1 edge on tissue: 50% overlap or more = basal, else apical
            else:
                if overlap > 0.5:
                    edge_to_semantic[label] = "basal"
                else:
                    edge_to_semantic[label] = "apical"
                self._logger.warning(
                    f"Unusual edge detection having {len(regions_edges)} edges. Usually a piece of tissue has 1 or 2 edges."
                )

        return edge_to_semantic

    def _semantic_edge_image(self, edge_cc: np.ndarray, edge_to_semantic: dict):
        """
        Create a new image of where each edge in the connected component gets
        assinged an new value based on its classification as 'apical' or 'basal'
        in the edge_to_semantic dictionary.

        Args:
            edge_cc: connected component edge image
            edge_to_semantic: Association between labels in `edge_cc` with 'apical' or 'basal'

        Returns:
            np.ndarray where each edge is semantically labeled
        """
        # Ensure each edge has a overlap ratio
        edge_labels = np.sort(np.unique(edge_cc))[1:]  # ignore the 0 index since bg
        dict_labels = np.sort(np.array(list(edge_to_semantic.keys())))
        for label in dict_labels:
            if label not in edge_labels.tolist():
                raise ValueError(
                    f"Invalid semantic label ({label}) for labels in edges ({edge_labels})"
                )

        # Relabel each edge with appropriate value corresponding to basal or apical
        semantic_edges = np.zeros_like(edge_cc)
        for label, edge_type in edge_to_semantic.items():
            edge_mask = edge_cc == label
            semantic_edges[edge_mask] = self.edge_dict[edge_type]

        return semantic_edges


class SegmenterWrapper:
    """
    Wraps the segmenter class to handle input and output image resizing
    for the segmentation network when used for inference.
    """

    def __init__(self, segmenter: Predicter, in_size: tuple = (128, 128)):
        """
        Args:
            segmenter: Model to perform tissue segmentation
            in_size: size to scale image to before segmentation
        """
        self.segmenter = segmenter
        self.in_size = in_size

    def predict(self, img: np.ndarray, scale_out: bool = True):
        """
        Predict the segmentation of an input image after resizing to `in_size`
        attribute. If `scale_out` is true, scales segmentation to match the size
        of the `img`.

        Args:
            img: Image to segment
            scale_out: If true, scales segmentation to match the of `img`

        Returns
            segmented image
        """
        # Get size of image and resize for input to model
        img_shape = img.shape
        if img_shape != self.in_size:
            in_img = cv2.resize(
                np.copy(img), dsize=self.in_size, interpolation=cv2.INTER_LINEAR
            )
        else:
            in_img = np.copy(img)

        # Predict segmentation
        mask, activation = self.segmenter.predict(image=in_img)

        # Re scale to input image size if desired
        if scale_out is True and mask.shape != img_shape:
            mask = cv2.resize(
                mask.astype(np.uint8), dsize=img_shape, interpolation=cv2.INTER_LINEAR
            ).astype(mask.dtype)
            activation = cv2.resize(
                activation, dsize=img_shape, interpolation=cv2.INTER_LINEAR
            )

        return mask, activation


def make_plot(imgs: list):
    from math import ceil, sqrt

    n = len(imgs)
    ASP = 16 / 9
    rows = ceil(sqrt(n / ASP))
    cols = int(ASP * rows)
    fig, ax = plt.subplots(rows, cols)
    iter = 0
    for i in range(rows):
        for j in range(cols):
            if iter < len(imgs):
                ax[i, j].imshow(imgs[iter])

            ax[i, j].axis("off")
            iter += 1
    plt.show()


if __name__ == "__main__":
    unet_ckpt = (
        "src/models/dl4mia_tissue_unet/results/20220824_181000_Colab_gpu/best.pth"
    )
    unet_model = Predicter.from_ckpt(unet_ckpt)
    unet_model = SegmenterWrapper(unet_model)
    TRAIN_MASK_DIR = "data/processed/uncropped/train/masks"
    TC = TissueEdgeClassifier(segmenter=unet_model, train_mask_dir=TRAIN_MASK_DIR)
    MASK_DIR = "data/processed/uncropped/test/masks"
    IMAGE_DIR = "data/processed/uncropped/test/images"
    INAME = "E14a_000008_crop00.tif"
    INAME = "e15 20x p_000011_crop00.tif"
    INAME = "e15 c_000011_crop00.tif"
    INAME = "e15 g_000012_crop00.tif"
    mask_paths = glob.glob(f"{MASK_DIR}/*.tif")
    img_paths = glob.glob(f"{IMAGE_DIR}/*.tif")
    paths = zip(mask_paths, img_paths)
    mask_path = random.choice(mask_paths)
    # mask_path = f"{MASK_DIR}/{INAME}"
    imgs = []
    # for mask_path in mask_paths:
    for mask_path, img_path in list(paths):
        mask_img = tifffile.imread(mask_path)
        real_img = tifffile.imread(img_path)
        t0 = time.time()
        try:
            edge_cc, mask_cc, edge_df, new_mask = TC.classify_img(real_img)
            edge_to_semantic = {
                ind: edge_df.loc[ind, "semantic"] for ind in list(edge_df.index)
            }
            t1 = time.time()
            print(f"Total time:\t{t1 - t0}")
            overlay = overlay_image(
                image=real_img,
                edge_label_dict=edge_to_semantic,
                mask=new_mask,
                edge_labels=edge_cc,
            )
            imgs.append(overlay)
        except Exception as e:
            print(f"exception passed: {e}")
    make_plot(
        imgs,
    )

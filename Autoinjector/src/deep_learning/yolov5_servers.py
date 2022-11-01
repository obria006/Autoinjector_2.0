import torch
import os
import cv2
import numpy as np
import onnxruntime
import src.deep_learning.yolov5_utils.processing as utils


class Yolov5ONNXServer:
    """
    Class to perform yolov5 model serving (loading and inference) using ONNX weights
    and onnxruntime
    """

    def __init__(self, onnx_weight_path, id2classname_dict=None):
        """
        Initialize yolov5 model server by loading model

        Args:
            onnx_weight_path (str): Full filepath to saved onnx weights (extenstion *.onnx)
            id2classname_dict (dict): Dictionary relating yolo ID # to class name
        """

        # Check filepaths
        self.onnx_weight_path = onnx_weight_path.replace("\\", "/")
        assert os.path.exists(self.onnx_weight_path), (
            "Path to class name file not found(%r)" % self.onnx_weight_path
        )

        # Set relationship between ID# and classname
        self.id2classname_dict = id2classname_dict

        # Load neural network
        self.load_onnx_model()

    def load_onnx_model(self):
        """
        Load onnxruntime model from onnx weights and create internal attribtues
        for model.
        """
        # Create neural network from onnx weights filepath
        print(
            "Loading onnxruntime NN from onnx model weights: " + self.onnx_weight_path
        )
        providers = ["CPUExecutionProvider"]
        self.session = onnxruntime.InferenceSession(
            self.onnx_weight_path, providers=providers
        )
        self.output_names = [x.name for x in self.session.get_outputs()]
        meta = self.session.get_modelmeta().custom_metadata_map  # metadata
        if "stride" in meta:
            stride, names = int(meta["stride"]), eval(meta["names"])
        print("onnxruntime NN loaded!")

    def image_inference(self, img, img_size=640):
        """
        Perform yolov5 inference using cv2.dnn

        Args:
            img (np.ndarray): BGR 8-bit numpy array image for inference
            img_size (int): Dimension to resize image. Should match traned model (640 for yolov5s).

        Returns:
        detections (list): List of NMS detections [(x1, y1, x2, y2, confidence, classID),...]
        """

        # Preprocess the image to be input into nerual network
        img_blob, img0 = utils.preprocess_image(img0=img, img_size=img_size)
        im = img_blob

        # Pass inputs to model and perform inference
        outputs = self.session.run(
            self.output_names, {self.session.get_inputs()[0].name: im}
        )

        # Run input through NN for inference
        if isinstance(outputs, (list, tuple)):
            if len(outputs) == 1:
                outputs = torch.from_numpy(outputs[0])
            else:
                raise ValueError(
                    "See DetctMultiBackend.forward for how to handle multiple outputs in list"
                )
        elif isinstance(outputs, np.ndarray):
            outputs = torch.from_numpy(outputs)
        elif not isinstance(outputs, torch.Tensor):
            raise TypeError("Outputs must are not convertable to torch.Tensor")


        # Do non-max suppression to filter outputs
        conf_thres = 0.25  # confidence threshold
        iou_thres = 0.45  # NMS IOU threshold
        classes = None  # filter by class: --class 0, or --class 0 2 3
        agnostic_nms = False  # class-agnostic NMS
        max_det = 5  # maximum detections per image
        pred = utils.non_max_suppression(
            prediction=outputs,
            conf_thres=conf_thres,
            iou_thres=iou_thres,
            classes=classes,
            agnostic=agnostic_nms,
            max_det=max_det,
        )

        # Process detections
        detections = []
        for i, det in enumerate(pred):
            if len(det):
                det = det.cpu().numpy()
                # Confidence score
                conf = round(float(det[0, 4]), 3)
                # Class
                classID = int(det[0, 5])
                # Rescale boxes from img_size to im0 size
                x1y1x2y2 = utils.scale_coords(
                    img_blob.shape[2:], det[:, :4], img0.shape
                ).round()
                x1 = int(x1y1x2y2[0, 0])
                y1 = int(x1y1x2y2[0, 1])
                x2 = int(x1y1x2y2[0, 2])
                y2 = int(x1y1x2y2[0, 3])
                detections.append((x1, y1, x2, y2, conf, classID))

        return detections

    def max_scoring_detection(self, dets):
        """
        Extract the maximum scoring detection

        Keyword Arguements:
        dets (list): List of detections

        Returns:
        max_det (list): Maximum scoring detection
        det_index (int): Index of max scoring detecion in dets
        """

        # Convert detections to np.array for easy processing
        dets = np.array(dets) if isinstance(dets, list) else dets

        # Find the max scoring detection in all the scaled predicions
        max_score = dets.max(axis=0)[4]
        max_score_index = np.where(dets[:, 4] == max_score)[0][0]
        max_det = dets[max_score_index, :].tolist()

        return max_det, max_score_index

    def scale_dets(self, dets, img_xy_size, disp_xy_size):
        """
        Scale detections from input image dimensions to the display image display image dimensions.

        |x1 scaled| = |x1 y1 x2 y2 score class| * |x scaling|
        |y1 scaled| = |x1 y1 x2 y2 score class| * |y scaling|
        |x2 scaled|                               |x scaling|
        |y2 scaled|                               |y scaling|
        |score    |                               |   1     |
        |class    |                               |   1     |

        Keyword Arguments:
        dets (list): List of detections in input image
        xy_img_size (tuple): Size of input image used for detections as (x,y)
        disp_img_size (tuple): Size of display image used for detections as (x,y)

        Returns:
        det_scaled (list): List containing detections scaled to display image size
        """

        # Convert detections to np.array if not already
        det_arr = np.array(dets) if isinstance(dets, list) else dets

        # Parse the input image size
        img_x, img_y = img_xy_size
        # Parse display image size
        display_x, display_y = disp_xy_size

        # Compute scaling from input size to display size
        if display_x is not None and display_y is not None:
            input2display_x = display_x / img_x
            input2display_y = display_y / img_y
        else:
            input2display_x = 1
            input2display_y = 1

        # Set scaling array for matrix multiplication scaling
        scaling_arr = np.array(
            [input2display_x, input2display_y, input2display_x, input2display_y, 1, 1]
        ).reshape((1, -1))

        # Compute scaled detection via matrix multiplicaion
        det_scaled = np.multiply(det_arr, scaling_arr).tolist()

        return det_scaled


class Yolov5PipetteDetector(Yolov5ONNXServer):
    """Class to load and serve pipette tip detectorr"""

    def __init__(
        self,
        onnx_weight_path,
        id2classname_dict={0:"below", 1:"in", 2:"above"},
        xy_display_size=(None, None),
    ):
        """
        Initialize yolov5 tip detector by loading model from weights

        Args:
            onnx_weight_path (str): Full filepath to saved onnx weights (extenstion *.onnx)
            id2classname_dict (dict): Dictionary relating yolo ID # to class name
            xy_display_size (tuple): Size of display in as (x,y)
        """

        # Initalize imp. detector by inherting YoloV5Server and using its methods to load the imp_detect weights
        super().__init__(
            onnx_weight_path=onnx_weight_path, id2classname_dict=id2classname_dict
        )
        # Set display size
        self.display_x, self.display_y = xy_display_size

    def detect(self, img:np.ndarray, img_size:int = 640):
        """
        Detect tip location in input image

        Args:
            img (np.ndarray): BGR 8-bit numpy array image for inference
            img_size (int): Dimension to resize image. Should match traned model (640 for yolov5s).

        Returns:
            dict of tip position (og & scaled), max scoring detection (og & scaled),
                class, and score
        """
        if len(img.shape) != 3:
            raise ValueError(f"Invalid image shape {img.shape}. Must have 3 dimensions")
        if img.shape[2] != 3:
            raise ValueError(f"Invalid image shape {img.shape}. Must be BGR image")
        
        # Conduct inference for pipette tip detection
        img_xy_size = (img.shape[1], img.shape[0])
        dets_input = self.image_inference(img=img, img_size=img_size)
        # Scale detections to display size and get maximum scoring detection
        if len(dets_input) > 0:
            dets_scaled = self.scale_dets(
                dets=dets_input,
                img_xy_size=img_xy_size,
                disp_xy_size=(self.display_x, self.display_y),
            )
            max_det_scaled, det_index = self.max_scoring_detection(dets=dets_scaled)
            class_id = max_det_scaled[-1]
            class_name = self.id2classname_dict[int(class_id)]
            score = max_det_scaled[4]
            max_det_input = dets_input[det_index]
            tip_input = self.tip_as_bb_center(max_det_input)
            tip_scaled = self.tip_as_bb_center(max_det_scaled)
        else:
            tip_input = None
            max_det_input = None
            tip_scaled = None
            max_det_scaled = None
            class_name = None
            score = None

        detection_output = {
            'xy_og':tip_input,
            'xy_scaled':tip_scaled,
            'detection_og':max_det_input,
            'detection_scaled':max_det_scaled,
            'class':class_name,
            'score':score
        }

        return detection_output

    def tip_as_bb_center(self, det):
        """
        Compute the tip coordinate as the center of the bounding box

        Keyword Arguements:
        det (list): 1D List of detection

        Returns:
        tip_coord (tuple): (x,y) coordiante of tip as center of bounidng box
        """
        assert isinstance(det, (list, tuple)), "Detection must be a list or tuple"
        assert len(det) == 6, "Detection must contain 6 entries"
        assert det[0] <= det[2], "Invalid x0 and x1 coordiantes"
        assert det[1] <= det[3], "Invalid y0 and y1 coordiantes"
        assert det[4] >=0 and det[4]<=1, "Invalid detection confidence"

        # Compute center of bounding box to be tip coordinate
        x_c = int((det[0] + det[2]) / 2)
        y_c = int((det[1] + det[3]) / 2)
        tip_coord = (x_c, y_c)
        return tip_coord


# if __name__ == "__main__":
#     from PIL import Image

#     onnx_pth = "/home/jacob.obrien/envs/HT_object_detection/yolov5/runs/train/exp8/weights/best.onnx"
#     classes = {0: "below", 1: "in", 1: "above"}
#     model = Yolov5Server(onnx_pth, classes)
#     test_dir = "/home/jacob.obrien/datasets/yolov5_pipette/processed/tip_xyz_yolo_detection/images/test"
#     imgs = os.listdir(test_dir)
#     imgs = os.listdir(test_dir)
#     im = cv2.imread(f"{test_dir}/{imgs[0]}")
#     model.image_inference(im)

    # onnx_pth = "/home/jacob.obrien/envs/HT_object_detection/yolov5/runs/train/exp6/weights/best.onnx"
    # classes = {0: "below", 1: "in", 1: "above"}
    # model = Yolov5ONNXServer(onnx_pth, classes)
    # test_dir = "/home/jacob.obrien/datasets/yolov5_pipette/processed/tip_xyz_yolo_detection/images/test"
    # imgs = os.listdir(test_dir)
    # im = cv2.imread(f"{test_dir}/{imgs[0]}")
    # model.image_inference(im)

    # weights_pth = "/home/jacob.obrien/envs/HT_object_detection/yolov5/runs/train/exp6/weights/best.pt"
    # classes = {0: "below", 1: "in", 1: "above"}
    # model = Yolov5TorchServer(weights_pth, classes)
    # test_dir = "/home/jacob.obrien/datasets/yolov5_pipette/processed/tip_xyz_yolo_detection/images/test"
    # imgs = os.listdir(test_dir)

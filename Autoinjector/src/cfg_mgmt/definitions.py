""" File containing static definitions to be used by the Autoinjector project.
These definitions should be constant once the project is installed and running.
Conversely, the configuration values may change peiodically, so those values
can be changed with the configuration application. """
import os

# This definitions.py file should be located in ROOT/src/cfg_mgmt so it is
# located 2 directories below the root directory. Hence, we need to navigate
# up two directories from this file's absolute path (__file__) to get the
# root directory. Navigate up two directories with two ".."
ROOT_DIR = os.path.realpath(os.path.join(os.path.dirname(__file__), '..', '..'))

# Specify other paths in project relative to ROOT_DIR
LOG_DIR = os.path.join(ROOT_DIR, 'logs')
DATA_DIR = os.path.join(ROOT_DIR, 'data')
CONFIG_DIR = os.path.join(ROOT_DIR, 'configs')
CONFIG_PATH = os.path.join(CONFIG_DIR, 'config.yaml')
UNET_WEIGHTS_PATH = os.path.join(ROOT_DIR, 'src/deep_learning/weights/20220824_180000_Colab_gpu/best.pth')
YOLO_WEIGHTS_PATH = os.path.join(ROOT_DIR, 'src/deep_learning/weights/yolov5_train_exp6/best.onnx')

# Camera parameters for Micro-Manager compatible cameras
# Parameters taken from Autoinjector 1 `launchapp.py`
CAMERA_PARAMS = {
    "PVCam":{
        "devicename": 'Cool Snap Dyno',
        "brand": 'PVCAM',
        "devicevalue": 'Camera-1',
        "bins": "none",
        "rotate": 180,
        "bits": 8,
        "scalefactor": 2.4,
    },
    "Hamamatsu Orca DCAM":{
        "devicename": 'HamamatsuHam_DCAM',
        "brand": 'HamamatsuHam',
        "devicevalue": 'HamamatsuHam_DCAM',
        "bins": "2x2",
        "rotate": 180,
        "bits": 8,
        "scalefactor": 1.3,
    },
    "Zeiss Axiocam":{
        "devicename": 'Zeiss AxioCam',
        "brand": 'AxioCam',
        "devicevalue": 'Zeiss AxioCam',
        "bins": "none",
        "rotate": 270,
        "bits": 16,
        "scalefactor": 1.5,
    }
}
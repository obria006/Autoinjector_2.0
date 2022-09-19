import os
import glob
import cv2
import matplotlib.pyplot as plt
from PIL import Image
import numpy as np
import traceback
from src.video_control import video_utils as utils
from src.deep_learning.tissue_detection import ModelTissueDetection

ckpt_path = "Autoinjector/src/deep_learning/weights/20220824_180000_Colab_gpu/best.pth"
img_dir = "T:/Autoinjector/tissue_images/mouse/tissue_seg/jpeg/images"
out_dir = "C:/Users/Public/Documents/envs/Autoinjector_2/Autoinjector/results/edge_classification"

# make results directory
if not os.path.isdir(out_dir):
    os.makedirs(out_dir)

# instantate tissue detector
MTC = ModelTissueDetection(ckpt_path)

img_paths= glob.glob(f"{img_dir}/*.jpg")
for img_path in img_paths:
    img = np.array(Image.open(img_path))
    for edge_type in ['apical', 'basal']:
        try:
            res = MTC.detect(img,edge_type)
        except:
            print()
            print(f"Error during: {img_path}")
            print(traceback.format_exc())
        else:
            df = res['detection_data']
            edge_cc = res['edge_image']
            seg_rgb = utils.make_rgb_mask(res['segmentation_image']>0, color = [238, 102, 119])
            if edge_type in df['semantic'].to_numpy():
                edge_num_labels = df.loc[df['semantic']==edge_type,'edge'].to_numpy()
                bin_mask = np.zeros(edge_cc.shape, dtype=np.uint8)
                for ind, label in enumerate(edge_num_labels):
                    bin_mask = np.bitwise_or(bin_mask, edge_cc == label)
                kernel = cv2.getStructuringElement(shape=cv2.MORPH_RECT, ksize=(21, 21))
                bin_mask = cv2.dilate(bin_mask, kernel)
                apical_rgb = utils.make_rgb_mask(bin_mask, color = [102, 204, 238])
                seg_rgb = utils.alpha_compost_A_over_B(apical_rgb, seg_rgb, 1)
            if 'basal' in df['semantic'].to_numpy():
                edge_num_labels = df.loc[df['semantic']=='basal','edge'].to_numpy()
                bin_mask = np.zeros(edge_cc.shape, dtype=np.uint8)
                for ind, label in enumerate(edge_num_labels):
                    bin_mask = np.bitwise_or(bin_mask, edge_cc == label)
                kernel = cv2.getStructuringElement(shape=cv2.MORPH_RECT, ksize=(21, 21))
                bin_mask = cv2.dilate(bin_mask, kernel)
                basal_rgb = utils.make_rgb_mask(bin_mask, color = [204, 187, 68])
                seg_rgb = utils.alpha_compost_A_over_B(basal_rgb, seg_rgb, 1)
            
            rgb = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
            rgb = utils.alpha_compost_A_over_B(seg_rgb, rgb, 0.25)
            rgb = cv2.polylines(
                    img = rgb,
                    pts = [res['edge_coordinates']],
                    isClosed = False,
                    color = [0,0,0],
                    thickness = 9
                )
            rgb = cv2.polylines(
                    img = rgb,
                    pts = [res['edge_coordinates']],
                    isClosed = False,
                    color = [255,255,255],
                    thickness = 5
                )
            im = Image.fromarray(rgb)
            out_path = img_path.replace(img_dir,out_dir)
            out_path.replace(".jpg",f"_{edge_type}.jpg")
            im.save(out_path)
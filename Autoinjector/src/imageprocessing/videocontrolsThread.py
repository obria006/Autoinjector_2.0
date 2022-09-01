from PyQt6.QtWidgets import *
from PyQt6.QtCore import *
from PyQt6.QtGui import QImage, QPixmap
import cv2
import numpy as np
import time
import pymmcore
import os
import sys
from skimage.util import img_as_ubyte
import traceback


class vidcontrols(QThread):
    #vidout = pyqtSignal()
    clicked_pos = pyqtSignal([float, float])
    def __init__(self,cam,brand,val,bins,rot,imagevals,scalefactor):
        #defines camera settings
        self.cam = cam
        self.brand = brand
        self.val = val
        self.bins = bins
        self.rot = int(rot)
        self.imagevals = imagevals
        self.scalefactor = scalefactor
        #self.exposure = 30

        #error message box
        self.error_msg = QMessageBox()
        self.error_msg.setIcon(QMessageBox.Icon.Critical)
        self.error_msg.setWindowTitle("Error")
        
        #initiates stream of video 
        QThread.__init__(self)
        self.image_analysis_window = QLabel() #creates place to put stream of vid
        self.CAMsetup() #set up camera

        self.keeptracking = False #initialize variable for later
        self.vidnum = 0 #count number of recordings
        self.gotopos = False
        self.hideshapecommand = False
        self.display_tip_pos = False

    # ---------------------- Camera Setup ------------------------------------------
    def CAMsetup(self):
        try:
            self.cap = pymmcore.CMMCore()
            MM_PATH = os.path.join('C:', os.path.sep, 'Program Files','Micro-Manager-2.0')
            self.cap.setDeviceAdapterSearchPaths([MM_PATH])
            self.cap.loadDevice(self.cam,self.brand,self.val)
            self.cap.initializeAllDevices()
            self.cap.setCameraDevice(self.cam)
            if self.bins != "none":
                self.cap.setProperty(self.cam, "Binning", self.bins)
            self.cap.startContinuousSequenceAcquisition(1)
            self.timer = QTimer()
            self.timer.timeout.connect(self.streamtranslate)
            self.timer.start(30)
            self.startcap = 0
            self.endcap = 0
            self.calib = 0
            self.camerafound = True
            self.streamtranslate() #stream video
            
        except:
            self.error_msg.setText("Camera not detected yet, close window and try again")
            self.error_msg.exec()
            self.camerafound = False
            self.image_analysis_window.setText("CAMERA ERROR. Verify camera is detected in Device Manager, correct camera settings are applied, and restart program. \n Python error = \n" + str(sys.exc_info()[1]))

    def changeexposure(self,value):
        self.exposure = float(value)
        print("self exposure =" + str(self.exposure))

    def streamtranslate(self):
        #this function controls the streaming video
        
        #check exposure value
        try:
            self.cap.setProperty(self.cam, 'Exposure', self.exposure)
        except:
            #print('camera does not have property, exposure')
            s =1 

        #if video is streaming
        if self.cap.getRemainingImageCount() > 0:
            self.frame = self.cap.getLastImage()
            self.frame = img_as_ubyte(self.frame) #convert to 8 bit from 16 bit

            if self.rot > 0: #rotate 
                rows,cols = self.frame.shape 
                M = cv2.getRotationMatrix2D((cols/2,rows/2),self.rot,1)
                self.frame = cv2.warpAffine(self.frame,M,(cols,rows))

            self.unmod_frame = np.copy(self.frame)
            self.width = int(self.frame.shape[0])
            self.height = int(self.frame.shape[1])


            #Look to see if line/point is drawn, show if it is
            try:
                if self.hideshapecommand == False:
                    cv2.circle(self.frame, (self.self.pixelclicked.x(), self.self.pixelclicked.y()), 1, 255, -1)
            except:
                s = 1

            
            try: #draws position clicked
                cv2.circle(self.frame,(int(self.scalefactor*self.pixelclicked.x()),int(self.scalefactor*self.pixelclicked.y())), 3,255, -1)
            except:
                s = 1

            if self.hideshapecommand == False:
            	try:#Look to see if edge coordinates exist, display them if they do, otherwise display vid
            		cv2.polylines(self.frame, [self.edgearray], False, (255,255,255),1)
            	except:
                    s = 1

            if self.display_tip_pos == True:
                try:
                    x_img = int(self.tip_x)
                    y_img = int(self.tip_y)
                    cv2.circle(self.frame, (x_img, y_img), 3, 0, -1)
                except:
                    print(traceback.format_exc())


            #video display
            if self.startcap == 1:
                self.out.write(self.frame)
                if self.endcap == 1:
                    self.out.release()
                    self.startcap = 0
            
            #display frame
            self.image = QImage(self.frame, self.frame.shape[1], self.frame.shape[0], self.frame.strides[0], QImage.Format.Format_Indexed8)
            self.image_analysis_window.setPixmap(QPixmap.fromImage(self.image.scaledToWidth(int(self.frame.shape[1]/self.scalefactor),Qt.TransformationMode.SmoothTransformation)))
            self.image_analysis_window.setFixedSize(int(self.frame.shape[1]/self.scalefactor),int(self.frame.shape[0]/self.scalefactor))
            self.image_analysis_window.mousePressEvent = self.getPos

    #-------------------------- Motor calibration command
    def calcymotortheta(self,p1,p2):
        self.position1 = p1
        self.position2 = p2

    def getPos(self , QMouseEvent):
        #get position of click in qpixmap
        self.pixelclicked = (QMouseEvent.pos())
        self.tipcircle = (self.pixelclicked*self.scalefactor)
        x = self.tipcircle.x()
        y = self.tipcircle.y()
        self.positionnow = (x,y)
        print(self.positionnow)
        self.clicked_pos.emit(x,y)

    def showshapes(self):
        self.hideshapecommand = False

    def hideshapes(self):
        #hides the shapes from image when trajectory is in progress as triggered in runalongedgetrajectory
        self.hideshapecommand = True

    #------------------------ Edge/Tip functions---------------------------------
    def drawinwindow(self):
        #puts image frame in GUI
        self.image = QImage(self.frame, self.frame.shape[1], self.frame.shape[0], self.frame.strides[0], QImage.Format_Indexed8)
        self.image_analysis_window.setPixmap(QPixmap.fromImage(self.image.scaledToWidth(800,Qt.SmoothTransformation)))

    def edgearraypointer(self,edgearray):
        self.edgearray = edgearray

    def show_tip_pos(self, x, y):
        self.tip_x = x
        self.tip_y = y

    def displaytip(self,tipx,tipy):
        #draws coordinates output from tipdetector
        self.tipx = tipx
        self.tipy = tipy

    def vid_stop(self):
        '''
        Stops video so GUI can close. Specifically, stops the continous image
        sequence acquisition from the camera and quits this QThread. 
        '''
        self.cap.stopSequenceAcquisition()
        self.quit()


# -*- coding: utf-8 -*-
import time
import os
import sys
import traceback
from functools import partial
import cv2
import serial
import numpy as np
from PyQt6.QtWidgets import *
from PyQt6.QtCore import *
from PyQt6.QtGui import QIcon, QPalette, QColor
import pandas as pd
from src.imageprocessing.videocontrolsThread import vidcontrols as vc
from src.imageprocessing.draw import drawobj
from src.motorcontrol.altermotorposition import delmotor
from src.motorcontrol.motorlocationThread import motorpositionThread
from src.motorcontrol.trajectorythread_minimal import trajectoryimplementor
from src.pythonarduino.injectioncontrolmod import injection
from src.resolutiontest.gotoposition import GetPos
from src.cfg_mgmt.cfg_mngr import CfgManager
from src.miscellaneous.standard_logger import StandardLogger as logr
from src.miscellaneous import validify as val
from src.data_generation.data_generators import PipTipData, TissueEdgeData
from src.manipulator_control.calibration import Calibrator
from src.manipulator_control.sensapex_utils import SensapexDevice
from src.manipulator_control.injection_trajectory import SurfaceLineTrajectory3D
from src.ZEN_interface.ZEN_App import ZenGroup



class ControlWindow(QMainWindow):
    """ QWidget class to control video stream and capture
    This class controls the GUI of the autoinjector and all subsequent controls including:
    - motor controls and obtaining motor position information
    - injection protocols and triggering 
    - video streaming and acquisition 
    Calling this class will initiate all functions and also present user with GUI (hence bioler plate at bottom of file)
    """

    def __init__(self,cam,brand,val,bins,rot,imagevals,scale,restest,com,fourtyxcalibdist, parent=None):
        super().__init__(parent)
        self.logger = logr(__name__)
        self._central_widget = QWidget(self)
        self.setCentralWidget(self._central_widget)
        QApplication.setStyle(QStyleFactory.create("Fusion"))
        self.error_msg = QMessageBox()
        self.error_msg.setIcon(QMessageBox.Icon.Critical)
        self.error_msg.setWindowTitle("Error")
        self.fourtyxmag = fourtyxcalibdist #distance manipulators move for calibration based on camera FOV and mag

        # initiate thread to poll position of motors and report error if they are not found
        try:
            self.getposition = motorpositionThread()
            self.getposition.start()
            self.getposition.motorpos.connect(self.showmotorposition)
            self.motorfound = True
        except:
            self.error_msg.setText("Manipulators not detected. Wait 2 minutes then relaunch the app. If this does not work, replug manipulators into computer.")
            self.error_msg.exec()
            print("Manipulators not detected")
            self.motorfound = False

        # open arduino port and report error if it is not found
        global arduino
        try:
            arduino = serial.Serial(str(com), 9600,timeout=5)
            self.arduinofound = True
            self.com = com
        except:
            arduino = None
            self.error_msg.setText("Arduino not detected, make sure you selected the correct com port, plug in, and try again.")
            self.error_msg.exec()
            self.arduinofound = False

        #initiate video stream thread using camera settings
        self.vidctrl = vc(cam,brand,val,bins,rot,imagevals,scale,restest)
        self.vidctrl.start()
        self.file_selected = 0
        self.restest = restest
        self.setup_gui()

        #initiate parameters for injection
        self.ninjection = 0 
        self.injectpressurevoltage = 0
        self.pulseduration = 0
        self.edgedetected = False
        self.overrideon = 'No'

        self.i = 0 #restest point number
    
    # ---------- Initialize GUI -------------------------------------------------------
    def setup_gui(self):
        '''
        Function for initializing the GUI
        
        General order of
        get config
        initialze variables
        do imports
        make widgets
        make connections
        set widget states
        '''
        # Load the configuration values
        self.get_gui_cfg()
        self.pipette_calibrator_widgets()
        self.data_generator_widgets()
        self.zen_group = ZenGroup()
        self.pip_cal = Calibrator()
        self.zen_group.obj_changed.connect(self.obj_changed)
        self.zen_group.opto_changed.connect(self.opto_changed)
        self.zen_group.ref_changed.connect(self.ref_changed)
        self.GUIsetup()
        self.init_from_ZEN()
        self.vidctrl.clicked_pos.connect(self.add_cal_positions)
        self.stateify_pipette_calibrator_widgets()
        self.stateify_data_generator_widgets()

    def obj_changed(self, mag_level:float):
        if mag_level in list(self.zen_group.zen.objectives['magnification']):
            self.motorcalibdist = self.fourtyxmag*(40/mag_level)
            self.response_monitor_window.append(">> Magnification set to " +str(mag_level))

    def opto_changed(self, mag_level:float):
        if mag_level in list(self.zen_group.zen.optovars['magnification']):
            self.response_monitor_window.append(">> Optovar set to " +str(mag_level))

    def ref_changed(self, ref_name:str):
        if ref_name in list(self.zen_group.zen.reflectors['name']):
            self.response_monitor_window.append(">> Reflector set to " +str(ref_name))
    
    def get_gui_cfg(self):
        ''' Loads the configuration values for the GUI '''
        self.cfg = CfgManager()
        self.cfg.cfg_from_pointer()

    def pipette_calibrator_widgets(self):
        ''' Creates widgets for pipette calibration '''
        # Pipette angle
        angle_label = QLabel("Pipette \n Angle", parent=self)
        self.angle_mode_box = QComboBox(parent=self)
        self.angle_mode_box.setPlaceholderText('Angle Mode') # Connect currentTextChanged to set angle
        self.angle_entry = QLineEdit(self)
        self.set_angle_button = QPushButton("Set", self)
        grid_layout1 = QGridLayout()
        grid_layout1.addWidget(angle_label, 0, 0, 2, 1)
        grid_layout1.addWidget(self.angle_mode_box, 0, 1, 1, 2)
        grid_layout1.addWidget(self.angle_entry, 1, 1, 1, 1)
        grid_layout1.addWidget(self.set_angle_button, 1, 2, 1, 1)
        # My calibration buttons
        self.conduct_calibration_but = QCheckBox("Calibrate")
        self.display_calibration_but = QCheckBox("Display")
        h_layout1 = QHBoxLayout()
        h_layout1.addWidget(self.conduct_calibration_but)
        h_layout1.addWidget(self.display_calibration_but)
        # Groupbox and master layout
        pip_cal_layout = QVBoxLayout()
        pip_cal_layout.addLayout(grid_layout1)
        pip_cal_layout.addLayout(h_layout1)
        self.pip_cal_group = QGroupBox('Pipette Calibration')
        self.pip_cal_group.setLayout(pip_cal_layout)
        # Set connections
        self.conduct_calibration_but.clicked.connect(self.compute_calibration)
        self.display_calibration_but.clicked.connect(self.display_calibration)
        self.angle_mode_box.currentTextChanged.connect(self.set_angle_mode)
        self.set_angle_button.clicked.connect(self.set_pipette_angle)

    def stateify_pipette_calibrator_widgets(self):
        ''' Set initial states for the pipette calirbator widgets '''
        self.angle_mode_box.insertItems(0, ['Automatic','Manual','Load'])
        self.angle_mode_box.setCurrentText('Automatic')
        self.pip_disp_timer = QTimer()
        self.pip_disp_timer.timeout.connect(self.display_calibration)
        self.pip_disp_timeout = 25

    def data_generator_widgets(self):
        ''' 
        Creates widgets for data generation 
        
        Workflow:
        wid1 = QtWidget
        wid2 = QtWidget
        layout = QTTYPELAYOUT
        layout.addWidget(wid1)
        layout.addWidget(wid2)
        groupbox = QGroupBox
        groupbox.setLayout(layout)
        masterlayout.addWidget(groupbox)
        '''
        self.save_tip_annot = QCheckBox("Save tip annotation")
        self.save_tiss_annot = QCheckBox("Save tissue annotation")
        data_gen_layout = QVBoxLayout()
        data_gen_layout.addWidget(self.save_tip_annot)
        data_gen_layout.addWidget(self.save_tiss_annot)
        self.data_gen_group = QGroupBox('GUI Data Acquisition')
        self.data_gen_group.setLayout(data_gen_layout)

    def stateify_data_generator_widgets(self):
        ''' Set initial states for data generator widgets '''
        self.save_tip_annot.setChecked(True)
        self.save_tiss_annot.setChecked(False)

    def init_from_ZEN(self):
        # Set magnification of objective
        mag_level = self.zen_group.zen.get_obj_info('magnification')
        if mag_level in list(self.zen_group.zen.objectives['magnification']):
            self.motorcalibdist = self.fourtyxmag*(40/mag_level)
            self.response_monitor_window.append(">> Magnification set to " +str(mag_level))
        # Set magnificaiton of optovar
        opto_mag_level = self.zen_group.zen.get_opto_info('magnification')
        self.response_monitor_window.append(">> Optovar set to " +str(opto_mag_level))
        # Set name of reflector
        ref_name = self.zen_group.zen.get_ref_info('name')
        self.response_monitor_window.append(">> Reflector set to " +str(ref_name))

    def GUIsetup(self):
        #Create widgets for image display
        self.image_analysis_window_box = QVBoxLayout()
        self.image_analysis_window_box.addWidget(self.vidctrl.image_analysis_window)
        self.image_analysis_window_box.addStretch()
        groupbox_image_analysis_window= QGroupBox('Microscope Video Stream')
        groupbox_image_analysis_window.setLayout(self.image_analysis_window_box)


        #manual image processing controls
        image_processing_windowmanual_detectedge = QPushButton("Draw Edge")
        image_processing_windowmanual_detectedge.clicked.connect(self.drawedge)
        image_processing_windowmanual = QHBoxLayout()
        image_processing_windowmanual.addWidget(image_processing_windowmanual_detectedge)
        groupbox_image_processing_windowmanual = QGroupBox('Draw Desired Trajectory')
        groupbox_image_processing_windowmanual.setLayout(image_processing_windowmanual)

        #view drawn edge
        misc = QVBoxLayout()
        misc_hideshape = QPushButton("Hide Edge")
        misc_hideshape.clicked.connect(self.vidctrl.hideshapes)
        misc_showshape = QPushButton("Show Edge")
        misc_showshape.clicked.connect(self.vidctrl.showshapes)
        exposurelabel = QLabel("Camera Exposure")
        self.exposureslider = QSlider(Qt.Orientation.Horizontal)
        self.exposureslider.setMinimum(4)
        self.exposureslider.setMaximum(30)
        self.exposureslider.setValue(8)
        self.exposureslider.setTickPosition(QSlider.TickPosition.TicksBelow)
        self.exposureslider.setTickInterval(0)
        self.exposureslider.valueChanged.connect(self.exposurevaluechange)
        misc.addWidget(exposurelabel)
        misc.addWidget(self.exposureslider)
        misc.addWidget(misc_showshape)
        misc.addWidget(misc_hideshape)
        groupbox_misc = QGroupBox('Display Settings')
        groupbox_misc.setLayout(misc)
        
        #automated injection controls
        # -*- coding: utf-8 -*-
        self.mu = "µ"
        self.trajectoryplan = QVBoxLayout()
        self.trajectoryplan_labelaproachdist = QLabel("Approach Distance ("+  self.mu +"m)   ")
        self.trajectoryplan_labeldepth = QLabel("Depth ("+  self.mu +"m)                      ")
        self.trajectoryplan_labelspace = QLabel("Spacing ("+  self.mu +"m)                   ")
        self.trajectoryplan_labelspeed = QLabel("Speed (%)                       ")
        self.trajectoryplan_approach = QLineEdit(self)
        self.trajectoryplan_injectiondepth= QLineEdit(self)
        self.trajectoryplan_spacingbtwn = QLineEdit(self)
        self.trajectoryplan_speed = QLineEdit(self)
        self.trajectoryplan_runbutton = QPushButton("Run Trajectory")
        self.trajectoryplan_runbutton.clicked.connect(self.runalongedgetrajectory)
        self.trajectoryplan_stopbutton = QPushButton("Stop Process")
        self.trajectoryplan_stopbutton.clicked.connect(self.stoptrajectory)
        approach = QHBoxLayout()
        depth = QHBoxLayout()
        space = QHBoxLayout()
        speed = QHBoxLayout()       
        approach.addWidget(self.trajectoryplan_labelaproachdist)
        approach.addWidget(self.trajectoryplan_approach)
        depth.addWidget(self.trajectoryplan_labeldepth)
        depth.addWidget(self.trajectoryplan_injectiondepth)
        space.addWidget(self.trajectoryplan_labelspace)
        space.addWidget(self.trajectoryplan_spacingbtwn)
        speed.addWidget(self.trajectoryplan_labelspeed)
        speed.addWidget(self.trajectoryplan_speed)
        self.trajectoryplan.addLayout(approach)
        self.trajectoryplan.addLayout(depth)
        self.trajectoryplan.addLayout(space)
        self.trajectoryplan.addLayout(speed)
        self.sl = QSlider(Qt.Orientation.Horizontal)
        self.sl.setMinimum(10)
        self.sl.setMaximum(255)
        self.sl.setValue(10)
        self.sl.setTickPosition(QSlider.TickPosition.TicksBelow)
        self.sl.setTickInterval(30)
        self.sl.valueChanged.connect(self.valuechange)
        self.automatedcontrol_window_left = QVBoxLayout()
        self.automatedcontrol_window_right = QVBoxLayout()
        self.automatedcontrol_window_controls = QHBoxLayout()
        self.automatedcontrol_window_controls_merged = QVBoxLayout()
        compensatpresslabel = QLabel("Pressure")
        self.compensatpres = QLineEdit(self)
        self.automatedcontrol_window_setvalues = QPushButton("Set Values")
        self.automatedcontrol_window_setvalues.clicked.connect(self.setautomatedparameters)
        self.automatedcontrol_window_left.addWidget(self.sl)
        self.automatedcontrol_window_right.addWidget(self.compensatpres)
        self.automatedcontrol_window_controls.addLayout(self.automatedcontrol_window_left)
        self.automatedcontrol_window_controls.addLayout(self.automatedcontrol_window_right)
        self.automatedcontrol_window_controls_merged.addWidget(compensatpresslabel)
        self.automatedcontrol_window_controls_merged.addLayout(self.automatedcontrol_window_controls)
        self.automatedcontrol_window_bottomwindow = QVBoxLayout()
        self.automatedcontrol_window_bottomwindow.addWidget(self.automatedcontrol_window_setvalues)
        self.automatedcontrol_window_bottomwindow.addWidget(self.trajectoryplan_runbutton)
        self.automatedcontrol_window_bottomwindow.addWidget(self.trajectoryplan_stopbutton)
        self.automatedcontrol_window_controls_merged.addLayout(self.automatedcontrol_window_bottomwindow)
        self.trajectoryplan.addLayout(self.automatedcontrol_window_controls_merged)
        groubox_trajectory = QGroupBox('Automated Microinjection Controls')
        groubox_trajectory.setLayout(self.trajectoryplan)

        #Manipulator Status 
        # -*- coding: utf-8 -*-
        self.mu = "µ"
        self.motorchangeincrementtext = QLabel("Increment (" + self.mu + "m)    ")
        self.motorchangespeedtext = QLabel("    Speed (%)       ")
        self.motorchangeincrement = QLineEdit(self)
        self.motorchangespeed = QLineEdit(self)
        self.motorxpositiontext = QLabel("   X Position     ")
        self.motorypositiontext = QLabel("   Y Position     ")
        self.motorzpositiontext = QLabel("   Z Position     ")
        self.motorxposition = QLineEdit(self)
        self.motoryposition = QLineEdit(self)
        self.motorzposition = QLineEdit(self)
        self.motorxposition.setReadOnly(True)
        self.motoryposition.setReadOnly(True)
        self.motorzposition.setReadOnly(True)
        self.motorxposition_increase = QPushButton("+")
        self.motorxposition_increase.clicked.connect(lambda: self.advancemotor(axis='x',direction='increase'))
        self.motorxposition_decrease = QPushButton("-")
        self.motorxposition_decrease.clicked.connect(lambda: self.advancemotor(axis='x',direction='decrease'))
        self.motoryposition_increase = QPushButton("+")
        self.motoryposition_increase.clicked.connect(lambda: self.advancemotor(axis='y',direction='increase'))
        self.motoryposition_decrease = QPushButton("-")
        self.motoryposition_decrease.clicked.connect(lambda: self.advancemotor(axis='y',direction='decrease'))
        self.motorzposition_increase = QPushButton("+")
        self.motorzposition_increase.clicked.connect(lambda: self.advancemotor(axis='z',direction='increase'))
        self.motorzposition_decrease = QPushButton("-")
        self.motorzposition_decrease.clicked.connect(lambda: self.advancemotor(axis='z',direction='decrease'))
        self.motorchange_inc_param = QHBoxLayout()
        self.motorchange_speed_param = QHBoxLayout()
        self.motorxposition_change_window = QHBoxLayout()
        self.motoryposition_change_window = QHBoxLayout()
        self.motorzposition_change_window = QHBoxLayout()
        self.motorxposition_change_window.addStretch()
        self.motorxposition_increase.setFixedWidth(75)
        self.motorxposition_decrease.setFixedWidth(75)
        self.motoryposition_increase.setFixedWidth(75)
        self.motoryposition_decrease.setFixedWidth(75)
        self.motorzposition_increase.setFixedWidth(75)
        self.motorzposition_decrease.setFixedWidth(75)
        self.motorchange_inc_param.addWidget(self.motorchangeincrementtext)
        self.motorchange_inc_param.addWidget(self.motorchangeincrement)
        self.motorchange_speed_param.addWidget(self.motorchangespeedtext)
        self.motorchange_speed_param.addWidget(self.motorchangespeed)
        self.motorxposition_change_window.addWidget(self.motorxposition_increase)
        self.motorxposition_change_window.addWidget(self.motorxposition_decrease)
        self.motoryposition_change_window.addStretch()
        self.motoryposition_change_window.addWidget(self.motoryposition_increase)
        self.motoryposition_change_window.addWidget(self.motoryposition_decrease)
        self.motorzposition_change_window.addStretch()
        self.motorzposition_change_window.addWidget(self.motorzposition_increase)
        self.motorzposition_change_window.addWidget(self.motorzposition_decrease)
        self.motorposition_change_window = QGridLayout()
        self.motorposition_change_window.addWidget(self.motorxpositiontext,0,0)
        self.motorposition_change_window.addWidget(self.motorxposition,0,1)
        self.motorposition_change_window.addLayout(self.motorxposition_change_window,1,0,1,2)
        self.motorposition_change_window.addWidget(self.motorypositiontext,2,0)
        self.motorposition_change_window.addWidget(self.motoryposition,2,1)
        self.motorposition_change_window.addLayout(self.motoryposition_change_window,3,0,1,2)
        self.motorposition_change_window.addWidget(self.motorzpositiontext,4,0)
        self.motorposition_change_window.addWidget(self.motorzposition,4,1)
        self.motorposition_change_window.addLayout(self.motorzposition_change_window,5,0,1,2)
        self.motorposition_change_window.addLayout(self.motorchange_inc_param,6,0,1,2)
        self.motorposition_change_window.addLayout(self.motorchange_speed_param,7,0,1,2)
        groupbox_motorpanel_window = QGroupBox('Manipulator')
        groupbox_motorpanel_window.setLayout(self.motorposition_change_window)

        #response monitor 
        self.response_monitorgrid= QVBoxLayout()
        self.response_monitor_window = QTextBrowser()
        self.response_monitor_window.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOn)
        self.response_monitorgrid.addWidget(self.response_monitor_window)
        groupbox_response_monitorgrid= QGroupBox('System Status')
        groupbox_response_monitorgrid.setLayout(self.response_monitorgrid)

        #organize main window
        self.mastergrid = QGridLayout()
        self.leftside=QVBoxLayout()
        self.leftside.addWidget(self.pip_cal_group)
        self.leftside.addWidget(groupbox_image_processing_windowmanual)
        self.leftside.addWidget(groupbox_misc)
        self.leftside.addWidget(self.data_gen_group)

        if self.restest == "On":
            #resolution test
            overrideymotorthetalab = QLabel("Theta Value")
            overridepixelsizelab = QLabel("Microns/Pixel")
            step1 = QLabel("Step 1")
            step2 = QLabel("Step 2")
            step3 = QLabel("Step 3")
            step4 = QLabel("Step 4")
            adjustpoint = QLabel("Adjust Point")
            self.overrideymotortheta = QLineEdit()
            self.overridepixelsize = QLineEdit()
            go_to_point = QPushButton("Go to res point")
            get_current_pos = QPushButton("Record point") #records targeted pos
            calculate_error= QPushButton("Calculate Error") #finds dist between current pos and targeted pos and outputs dist
            go_to_centerpoint = QPushButton("Go to center point")
            go_to_point.clicked.connect(self.go_to_point_func)
            get_current_pos.clicked.connect(self.get_current_pos_func)
            calculate_error.clicked.connect(self.calculate_error_func)
            go_to_centerpoint.clicked.connect(self.go_to_centerpoint_func)
            addpoint = QPushButton("+")
            addpoint.clicked.connect(self.add_restestpoint)
            subpoint = QPushButton("-")
            subpoint.clicked.connect(self.sub_restestpoint)

            restestlayout = QGridLayout()
            restestlayout.addWidget(step1,2,0)
            restestlayout.addWidget(go_to_point,2,1,1,2)
            restestlayout.addWidget(step2,3,0)
            restestlayout.addWidget(get_current_pos,3,1,1,2)
            restestlayout.addWidget(step3,4,0)
            restestlayout.addWidget(calculate_error,4,1,1,2)
            restestlayout.addWidget(step4,5,0)
            restestlayout.addWidget(go_to_centerpoint,5,1,1,2)
            restestlayout.addWidget(adjustpoint,6,0)
            restestlayout.addWidget(addpoint,6,1)
            restestlayout.addWidget(subpoint,6,2)

            overrideres = QGridLayout()
            overrideonlab = QLabel("Override?")
            overrideon = QComboBox(self)
            overrideon.addItem("No")
            overrideon.addItem("Yes")
            overrideon.textActivated[str].connect(self.updateoverride)
            overrideres.addWidget(overrideonlab,6,0)
            overrideres.addWidget(overrideon,6,1,1,2)
            overrideres.addWidget(overrideymotorthetalab, 7,0)
            overrideres.addWidget(self.overrideymotortheta, 7,2)
            overrideres.addWidget(overridepixelsizelab, 8, 0)
            overrideres.addWidget(self.overridepixelsize,8,2)
            groubox_override = QGroupBox("Override Calibrated Settings")
            groubox_override.setLayout(overrideres)

            restestlayout.addWidget(groubox_override,7,0,1,3)
            groupbox_restest = QGroupBox("Resolution Test")
            groupbox_restest.setLayout(restestlayout)
            self.leftside.addWidget(groupbox_restest)


        self.leftside.addStretch()
        self.rightside=QVBoxLayout()
        self.rightside.addWidget(groupbox_motorpanel_window)
        self.rightside.addWidget(self.zen_group)
        self.rightside.addWidget(groubox_trajectory)
        self.rightside.addStretch()

        #Main window details...
        self.setWindowTitle('Autoinjector')
        self.setGeometry(100,100,200,200)
        self._central_widget.setLayout(self.mastergrid)
        self.show()
        self.setWindowIcon(QIcon('favicon.png'))
        self.timer = QTimer()  
        self.mastergrid.addLayout(self.leftside,1,0,1,1)
        self.mastergrid.addWidget(groupbox_image_analysis_window,1,1,1,1)
        self.mastergrid.addLayout(self.rightside,1,3,1,1)
        self.mastergrid.addWidget(groupbox_response_monitorgrid, 2,0,1,4)
        self.mastergrid.setContentsMargins(5, 5, 5, 5)

        #print errors on response monitor if manipulator or arduino has an error
        if self.motorfound == False:
            self.response_monitor_window.append(">> Manipulators not detected. Wait 2 minutes then relaunch the app. If this does not work, replug manipulators into computer.")
        else:
            self.response_monitor_window.append(">> Manipulators detected and working.")
        if self.arduinofound == False:
            self.response_monitor_window.append(">> Arduino not detected, make sure you selected the correct com port, plug in, and try again")
        else:
            self.response_monitor_window.append(">> Arduino connected and working on port " + str(self.com))

        if self.vidctrl.camerafound == False:
            self.response_monitor_window.append(">> Camera not detected. Wait 2 minutes then relaunch app. Make sure proper camera settings are used, and camera is shown in windows device manager under USB or camera.")
        else:
            self.response_monitor_window.append(">> Camera detected and working.")

    def set_angle_mode(self):
        '''
        Sets the mode for querying the pipette angle.
        Called when pipette angle combo box changes.

        Activity
            if automatic:
                angle entry box to readonly and dark gray
                Disable set button
                query sensapex for angle, insert in combobox and set thetaz
            if manual:
                clear thetaz
                enable entry box and make white
                enable set button
            if load:
                raise error message box
                default to manual
        '''
        # Get the selected mode
        angle_mode = self.angle_mode_box.currentText()
        # Handle different angle mode selections
        if angle_mode == 'Automatic':
            # Read only and gray out angle entry box
            palette = QPalette()
            palette.setColor(QPalette.ColorRole.Base, QColor('lightGray'))
            self.angle_entry.setReadOnly(True)
            self.angle_entry.setPalette(palette)
            # Disable set button
            self.set_angle_button.setEnabled(False)
            # Get angle
            dev = SensapexDevice(1)
            ang = dev.get_axis_angle()
            # Display in box
            self.angle_entry.clear()
            self.angle_entry.insert(str(ang))
            # Set the angle
            self.set_pipette_angle()
        if angle_mode == 'Manual':
            # Enable angle entry and set to default white
            palette = QPalette()
            palette.setColor(QPalette.ColorRole.Base, QColor('white'))
            self.angle_entry.setReadOnly(False)
            self.angle_entry.setPalette(palette)
            # Enable set button
            self.set_angle_button.setEnabled(True)
            # Clear the box and reset thetaz
            self.angle_entry.clear()
            # FIXME poor practice (will be fixed when change trajecotry)
            if 'thetaz' in self.__dict__.keys():
                del self.thetaz
        if angle_mode == 'Load':
            try:
                raise NotImplementedError("The 'Load' pipette angle functionality hasn't been implimented yet")
            except NotImplementedError:
                self.logger.exception("Cant load pipette angle")
                self.error_msg.setText("'Load' pipette angle hasn't been implimented yet. Defaulting to 'Manual'.")
                self.error_msg.exec()
            finally:
                self.angle_mode_box.setCurrentText('Manual')
            
    def set_pipette_angle(self):
        ''' Sets pipette angle from entry box '''
        angle_str = self.angle_entry.text()
        if val.is_valid_number(angle_str) is False:
            self.logger.warning(f'Invalid angle: {angle_str}. Angle must be a valid number.')
            self.error_msg.setText(f'Invalid angle: {angle_str}. Angle must be a valid number.')
            self.error_msg.exec()
        else:
            self.thetaz = np.deg2rad(float(angle_str))
            self.response_monitor_window.append(f">> Pipette angle set as {angle_str}")

    def valuechange(self):
        self.pressureslidervalue= self.sl.value()
        self.displaypressure = int(self.pressureslidervalue/2.55)
        self.compensatpres.setText('         '+str(self.displaypressure)+'%')

    def exposurevaluechange(self):
        self.exposureslidervalue = self.exposureslider.value()/3.33
        self.displayexposure = float(self.exposureslidervalue)
        self.vidctrl.changeexposure(self.displayexposure)

    def updateoverride(self,text):
        self.overrideon = str(text)
        self.response_monitor_window.append(">> Override calibrated settings for resolution test set to ON.")
    
    """
    ------------------------ Data Generation Controls -------------------------
    Functions control saving images from data
    """
    def acquire_tip_data(self, tip_position):
        '''
        Queries whether checkbox selected to save tip data, and saves data if checked
        '''
        if self.save_tip_annot.isChecked():
            tip_dict = {'x':tip_position.x(), 'y':tip_position.y()}
            self.save_pip_cal_data(tip_dict)

    def acquire_tissue_data(self, raw_edge_coord, inter_edge_cood):
        '''
        Queries whether checkbox selected to save tissue data, and saves data if checked
        '''
        if self.save_tiss_annot.isChecked():
            raw = np.asarray(raw_edge_coord)
            inter = np.asarray(inter_edge_coord)
            self.save_tiss_anot_data(raw_annot=raw, interpolate_annot=inter)

    """
    ----------Calibration Controls -----------------------------------------------------------------
    These functions control the calibration of the manipulators to the camera axes
    """

    def add_cal_positions(self,x_click:float, y_click:float):
        ''' Adds clicked calibration positions to calibration data '''
        if self.conduct_calibration_but.isChecked():
            z_scope = self.zen_group.zen.get_focus_um()
            ex_pos = [x_click, y_click, z_scope]
            dev = SensapexDevice(1)
            man_pos = dev.get_pos()
            self.pip_cal.data.add_cal_position(ex=ex_pos, man=man_pos)
            print(self.pip_cal.data.data_df)
            if self.save_tip_annot.isChecked():
                self.tipposition1 = self.vidctrl.tipcircle
                tip_dict = {'x':self.tipposition1.x(), 'y':self.tipposition1.y()}
                self.save_pip_cal_data(tip_dict)

    def compute_calibration(self):
        ''' Computes calibration from calibration data '''
        if self.conduct_calibration_but.isChecked() is False:
            try:
                self.pip_cal.compute(z_polarity=-1, pip_angle=self.thetaz)
            except:
                print(traceback.format_exc())
            finally:
                self.logger.info('Calibration unchecked. Deleting calibration points.')
                self.pip_cal.data.rm_all()
    
    def display_calibration(self):
        if self.display_calibration_but.isChecked():
            if self.pip_cal.model.is_calibrated is True:
                if self.pip_disp_timer.isActive() is False:
                    self.pip_disp_timer.start(self.pip_disp_timeout)
                    self.vidctrl.display_tip_pos = True
                dev = SensapexDevice(1)
                pos = dev.get_pos()
                ex = self.pip_cal.model.forward(man=pos)
                self.vidctrl.show_tip_pos(ex[0], ex[1])
        else:
            self.vidctrl.display_tip_pos = False
            self.pip_disp_timer.stop()

    def update_calibration(self):
        ''' Updates calibration by setting new reference position '''
        pass

    def load_calibration(self):
        ''' Loads the most recent calibration '''
        pass
    
    def save_pip_cal_data(self, tip_dict:dict):
        '''
        Save image from camera and annotated coordinates of a mouse click

        Arguments:
            tip_dict: {'x':x, 'y':y} x and y coordinates of tip annotation on image.
        '''
        
        # Directories for data
        data_dir = self.cfg.cfg_gui.values['data directory'].replace('\\','/')
        pip_data_dir = f"{data_dir}/pipette/calibration_images"
        # Instance of object to save data
        pip_data_saver = PipTipData(pip_data_dir=pip_data_dir)
        image = np.copy(self.vidctrl.unmod_frame)
        pip_data_saver.save_data(image=image, tip_position=tip_dict)

    """
    ----------Desired Trajectory Control -----------------------------------------------------------------
    This ontrols the detection of the edge of the tissue, and the tip of the pipette
    """   
    def save_tiss_anot_data(self, raw_annot:np.ndarray, interpolate_annot:np.ndarray):
        '''
        Save image from camera and annotated coordinates of tissue trajectory

        Arguments:
            raw_annot (np.ndarray): nx2 array of drawn annotation coord. nth row = [x_n, y_n]
            interpolate_annot (np.ndarray): nx2 array of interpolated annotation coord. nth row = [x_n, y_n]
        '''
        
        # Directories for data
        data_dir = self.cfg.cfg_gui.values['data directory'].replace('\\','/')
        tis_data_dir = f"{data_dir}/tissue/annotation_images"
        # Instance of object to save data
        tis_data_saver = TissueEdgeData(tis_data_dir=tis_data_dir)
        image = np.copy(self.vidctrl.unmod_frame)
        tis_data_saver.save_data(image=image, raw_annot=raw_annot, interpolate_annot=interpolate_annot)


    def drawedge(self):
        
        try:
            self.vidctrl.showshapes()
            self.d = drawobj(self.vidctrl.frame)
            self.d.drawedgecoord1 = np.asarray(self.d.drawedgecoord1)
            self.vidctrl.edgearraypointer(self.d.drawedgecoord1)
            if self.save_tiss_annot.isChecked():
                raw = np.asarray(self.d.drawedgecoord)
                inter = np.asarray(self.d.drawedgecoord1)
                self.save_tiss_anot_data(raw_annot=raw, interpolate_annot=inter)
            
        except:
            self.error_msg.setText("CAM error, is camera plugged in? \nPython error = \n" + str(sys.exc_info()[1]))
            self.error_msg.exec()
            self.response_monitor_window.append(">> Camera error. \n>> Python error = " + str(sys.exc_info()))
        
    """
    ---------- Resolution Test Controls   -----------------------------------------------------------------
    These Functions control testing resolution error
    """
    def go_to_point_func(self):
        try:
            if self.overrideon == "Yes":
                self.ymotortheta = float(self.overrideymotortheta.text())
                self.pixelsize = float(self.overridepixelsize.text())

            #get current position of manipulator
            getmotorpos = delmotor('', '', 0, 1000,'getposition_m0',0)
            getmotorpos.start()
            
            time.sleep(0.2)
            print('m0 = ' + str(getmotorpos.m0))
            self.m0 = getmotorpos.m0
            self.stringm0 = str(self.m0)
            
            #get position of first point in restest grid
            c2x = self.vidctrl.points.drawpointsx
            c2y = self.vidctrl.points.drawpointsy
            self.c2 = (c2x[self.i],c2y[self.i])
            
            #generate commands to go from current point to restest point
            self.c0 = (self.vidctrl.height/2,self.vidctrl.width/2)
            getpos = GetPos(self.c0,self.c2,self.m0,1000,self.ymotortheta,self.thetaz,self.pixelsize)
            print('m1 instructed = ' +  str(getpos.futuremotor))
            self.m1 = getpos.futuremotor

            #use generated commands to move from current point to restest point
            move = delmotor('x', 'increase', getpos.futuremotor, 1000,'absolute',0)
            move.start()
        except:
            self.error_msg.setText("restest err. \nPython error = \n" + str(sys.exc_info()[1]))
            self.error_msg.exec()
            self.response_monitor_window.append(">> Python error = " + str(sys.exc_info()))
        
    def get_current_pos_func(self):
        try:
            #get c1, get m1
            getcurrentmotorm1 = delmotor('', '', 0, 1000,'getposition_m1',0)
            getcurrentmotorm1.start()
            time.sleep(0.4)
            self.m1 = getcurrentmotorm1.m1
            self.c1 = self.vidctrl.positionnow
            print('m1 real =' + str(self.m1))
        except:
            self.error_msg.setText("restest err. \nPython error = \n" + str(sys.exc_info()[1]))
            self.error_msg.exec()
            self.response_monitor_window.append(">> Python error = " + str(sys.exc_info()))

    def calculate_error_func(self):
        try:
            #move to absolute truth position
            getcurrentmotorm2 = delmotor('', '', 0, 1000,'getposition_m2',0)
            getcurrentmotorm2.start()
            time.sleep(0.4)
            self.m2 = getcurrentmotorm2.m2

            self.errorx = abs(int(self.m2[0])-int(self.m1[0]))
            self.errory = abs(int(self.m2[1])-int(self.m1[1]))
            self.errorz = abs(int(self.m2[2])-int(self.m1[2]))
            print('m2 = ' + str(self.m2))
            print('error x,y,z,d in nm = ')
            print(self.errorx)
            print(self.errory)
            print(self.errorz)

            if len(self.m2) == 4:
                self.errord = abs(int(self.m2[3])-int(self.m1[3]))
                print(self.errord)
        except:
            self.error_msg.setText("restest err. \nPython error = \n" + str(sys.exc_info()[1]))
            self.error_msg.exec()
            self.response_monitor_window.append(">> Python error = " + str(sys.exc_info()))

    def go_to_centerpoint_func(self):
        try:
            #go back to center point
            s1m0 = self.stringm0.replace(',','')
            s2m0 = s1m0.replace('[','')
            s3m0 = s2m0.replace(']','')
            if len(self.m0) == 3:
                s4m0 = s3m0.split(' ',2)
            elif len(self.m0) ==4:
                s4m0 = s3m0.split(' ',3)            
            m0 = [int(i) for i in s4m0]
            move = delmotor('x', 'increase', m0, 1000,'absolute',0)
            move.start()
        except:
            self.error_msg.setText("restest err. \nPython error = \n" + str(sys.exc_info()[1]))
            self.error_msg.exec()
            self.response_monitor_window.append(">> Python error = " + str(sys.exc_info()))

    def add_restestpoint(self):
        #changes selected point
        self.i = self.i + 1
        print('point vertex = ' + str(self.i))

    def sub_restestpoint(self):
        #changes selected point
        self.i = self.i - 1
        print('point vertex = ' + str(self.i))

    """
    ----------Motor Controls -----------------------------------------------------------------
    These functions control displaying motor position
    All functions call the class delmotor in the altermotorposition.py file
    """

    def showmotorposition(self, motorposition):
        try:
            self.motorposition = motorposition
            self.registerpositionx = int(self.motorposition[0])/1000
            self.registerpositiony = int(self.motorposition[1])/1000
            self.registerpositionz = int(self.motorposition[2])/1000
            self.motorxposition.setText(str(self.registerpositionx) +  self.mu +"m")
            self.motoryposition.setText(str(self.registerpositiony) +  self.mu +"m")
            self.motorzposition.setText(str(self.registerpositionz) +  self.mu +"m")
        except:
            self.error_msg.setText("Manipulator error. \nPython error = \n" + str(sys.exc_info()[1]))
            self.error_msg.exec()
            self.response_monitor_window.append(">> Python error = " + str(sys.exc_info()))

    def advancemotor(self, axis, direction):
        try:
            dist = float(float(self.motorchangeincrement.text())*1000)
            speed = float(self.motorchangespeed.text())
            move = delmotor(axis, direction, dist, speed,'relative',0)
            move.start()
        except:
            self.error_msg.setText("Please enter an increment and speed in the manipulator window. \nPython error = \n" + str(sys.exc_info()[1]))
            self.error_msg.exec()
            self.response_monitor_window.append(">> Python error = " + str(sys.exc_info()))


    """
    ----------Automated Microinjection Controls  -----------------------------------------------------------------
    These functions control the trajectory and pressure controls of the GUI
    """

    def setautomatedparameters(self):
        try:
            self.compensationpressureval = self.pressureslidervalue
            self.compensationpressureval =str(self.compensationpressureval)
            self.approachdist = self.trajectoryplan_approach.text()
            self.depthintissue = self.trajectoryplan_injectiondepth.text()
            self.stepsize = self.trajectoryplan_spacingbtwn.text()
            self.motorspeed = self.trajectoryplan_speed.text()
            self.injectpressurevoltage = self.compensationpressureval
            self.response_monitor_window.append(">> Values set")
            self.injector_compensate = injection(arduino,self.compensationpressureval, 0,self.injectpressurevoltage,0,'bp')
            self.injector_compensate.start()

        except:
            self.error_msg.setText("Error, did you enter all parameters? Is the arduino plugged in? \nPython error = \n" + str(sys.exc_info()[1]))
            self.error_msg.exec()
            self.response_monitor_window.append(">> Python error = " + str(sys.exc_info()))
        
    def run_3D_trajectory(self):
        try:
            #get values from GUI
            um2nm = 1000
            approach_nm = int(float(self.approachdist)*um2nm)
            depth_nm = int(float(self.depthintissue)*um2nm)
            spacing_nm = int(float(self.stepsize)*um2nm)
            speed_ums = int((self.motorspeed))
            dev = SensapexDevice(1)
            cal = self.pip_cal
            edge = self.d.drawedgecoord1
            z_scope = self.zen_group.zen.get_focus_um()
            edge_arr = np.array(edge)
            z_arr = z_scope*np.ones((edge_arr.shape[0],1))
            edge_3D = np.concatenate((edge_arr,z_arr),axis=1).tolist()
            self.inj_trajectory = SurfaceLineTrajectory3D(dev, cal, edge_3D, approach_nm, depth_nm, spacing_nm, speed_ums)
            self.inj_trajectory.start()
            self.inj_trajectory.finished.connect(self.show_n_injected)
        except:
            print(traceback.format_exc())
            self.error_msg.setText("Please complete calibration, enter all parameters, and select tip of pipette.\nPython error = \n" + str(sys.exc_info()[1]))
            self.error_msg.exec()
            self.response_monitor_window.append(">> Python error = " + str(sys.exc_info()))

    def show_n_injected(self):
        self.response_monitor_window.append(">> Number of injections =" + str(self.inj_trajectory.n_injected))

    def stoptrajectory(self):
        try:
            self.inj_trajectory.stop()
        except:
            self.error_msg.setText("You have to start the trajectory in order to be able to stop it...\nPython error = \n" + str(sys.exc_info()[1]))
            self.error_msg.exec()
            self.response_monitor_window.append(">> Python error = " + str(sys.exc_info()))

    def closeEvent(self, event):
        close_pressure = injection(arduino,0, 0,0,0,'bp')
        close_pressure.start()
        self.vidctrl.vid_stop()
        time.sleep(0.5)
        self.close()

if __name__ == "__main__":
    import sys
    app = QApplication(sys.argv)
    app.setApplicationName('MyWindow')
    main = ControlWindow('HamamatsuHam_DCAM', 'HamamatsuHam', 'HamamatsuHam_DCAM', '2x2', 180, 256, 1.3, 'Off', 'com3', 40000)
    main.show()
    sys.exit(app.exec())
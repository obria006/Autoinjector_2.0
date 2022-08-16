
# -*- coding: utf-8 -*-
import time
import os
import sys
import traceback
from datetime import datetime
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
from src.Qt_utils.gui_objects import QHLine
from src.miscellaneous.standard_logger import StandardLogger as logr
from src.miscellaneous import validify as val
from src.data_generation.data_generators import PipTipData, TissueEdgeData
from src.manipulator_control.calibration import Calibrator, AngleIO, CalibrationError, CalibrationDNEError, CalibrationFileError, CalibrationDataError, AngleFileError
from src.manipulator_control.sensapex_utils import SensapexDevice
from src.manipulator_control.injection_trajectory import SurfaceLineTrajectory3D
from src.manipulator_control.calibration_trajectory import SemiAutoCalibrationTrajectory
from src.ZEN_interface.ZEN_App import ZenGroup



class ControlWindow(QMainWindow):
    """ QWidget class to control video stream and capture
    This class controls the GUI of the autoinjector and all subsequent controls including:
    - motor controls and obtaining motor position information
    - injection protocols and triggering 
    - video streaming and acquisition 
    Calling this class will initiate all functions and also present user with GUI (hence bioler plate at bottom of file)
    """
    _scope_complete = pyqtSignal(bool)
    _angle_complete = pyqtSignal(bool)
    _calibration_complete = pyqtSignal(bool)
    _annotation_complete = pyqtSignal(bool)
    _parameter_complete = pyqtSignal(bool)
    cal_pos_added = pyqtSignal()
    def __init__(self,cam,brand,val,bins,rot,imagevals,scale,restest,com,fourtyxcalibdist, parent=None):
        super().__init__(parent)
        self.logger = logr(__name__)
        self._central_widget = QWidget(self)
        self.setCentralWidget(self._central_widget)
        QApplication.setStyle(QStyleFactory.create("Fusion"))
        self.error_msg = QMessageBox()
        self.error_msg.setIcon(QMessageBox.Icon.Critical)
        self.error_msg.setWindowTitle("Error")
        self.warn_msg = QMessageBox()
        self.warn_msg.setIcon(QMessageBox.Icon.Warning)
        self.warn_msg.setWindowTitle("Warning")
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
        self.define_dirs()
        self.pipette_calibrator_widgets()
        self.data_generator_widgets()
        self.injection_parameter_widgets()
        self.workflow_widgets()
        self.zen_group = ZenGroup()
        self.pip_cal = Calibrator(cal_data_dir=self.cal_data_dir)
        self.angle_io = AngleIO(ang_data_dir=self.cal_data_dir)
        self.zen_group.obj_changed.connect(self.obj_changed)
        self.zen_group.obj_changed.connect(self.calibration_inputs_changed)
        self.zen_group.opto_changed.connect(self.calibration_inputs_changed)
        self.zen_group.opto_changed.connect(self.opto_changed)
        self.zen_group.ref_changed.connect(self.ref_changed)
        self.GUIsetup()
        self.init_from_ZEN()
        self.vidctrl.clicked_pos.connect(self.add_cal_positions)
        self.workflow_connections()
        self.stateify_pipette_calibrator_widgets()
        self.stateify_data_generator_widgets()
        self.stateify_injection_parameter_widgets()

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
    
    def define_dirs(self):
        ''' Set dirs (and create if necessary) for GUI '''
        # General data directory
        self.data_dir = self.cfg.cfg_gui.values['data directory'].replace('\\','/')
        if os.path.isdir(self.data_dir) is False:
            os.makedirs(self.data_dir)
            self.logger.info(f'Created data directory: {self.data_dir}')
        # For saving pipette images
        self.pip_data_dir = f"{self.data_dir}/pipette/calibration_images"
        if os.path.isdir(self.pip_data_dir) is False:
            os.makedirs(self.pip_data_dir)
            self.logger.info(f'Created pipette image data directory: {self.pip_data_dir}')
        # For saving tissue images
        self.tis_data_dir = f"{self.data_dir}/tissue/annotation_images"
        if os.path.isdir(self.tis_data_dir) is False:
            os.makedirs(self.tis_data_dir)
            self.logger.info(f'Created tissue image data directory: {self.tis_data_dir}')
        # For saving calibration data 
        self.cal_data_dir = f"{self.data_dir}/calibration"
        if os.path.isdir(self.cal_data_dir) is False:
            os.makedirs(self.cal_data_dir)
            self.logger.info(f'Created calibration data directory: {self.cal_data_dir}')
        # For saving calibration arrays
        self.cal_arr_dir = f"{self.data_dir}/calibration/ndarrays"
        if os.path.isdir(self.cal_arr_dir) is False:
            os.makedirs(self.cal_arr_dir)
            self.logger.info(f'Created calibration array directory: {self.cal_arr_dir}')

    def pipette_calibrator_widgets(self):
        ''' Creates widgets for pipette calibration '''
        # Pipette angle
        angle_label = QLabel("Pipette \n Angle", parent=self)
        self.angle_mode_box = QComboBox(parent=self)
        self.angle_mode_box.setPlaceholderText('Angle Mode')
        self.angle_entry = QLineEdit(self)
        self.set_angle_button = QPushButton("Set", self)
        self.save_angle_button = QPushButton("Save Angle", self)
        self.load_angle_button = QPushButton("Load Angle", self)
        grid_layout1 = QGridLayout()
        grid_layout1.addWidget(angle_label, 0, 0, 2, 1)
        grid_layout1.addWidget(self.angle_mode_box, 0, 1, 1, 2)
        grid_layout1.addWidget(self.angle_entry, 1, 1, 1, 1)
        grid_layout1.addWidget(self.set_angle_button, 1, 2, 1, 1)
        h_layout_angio = QHBoxLayout()
        h_layout_angio.addWidget(self.save_angle_button)
        h_layout_angio.addWidget(self.load_angle_button)
        # Separator
        h_sep1 = QHLine()
        # Calibration mode
        cal_mode_label = QLabel("Calibration Mode:")
        self.cal_mode_box = QComboBox(parent=self)
        self.cal_mode_box.setPlaceholderText('Mode')
        new_cal_label = QLabel('New Calibration:')
        upd_cal_label = QLabel('Update Calibration:')
        show_cal_label = QLabel('Show Calibration:')
        self.conduct_calibration_but = QCheckBox("")
        self.update_calibration_but = QCheckBox("")
        self.save_calibration_but = QPushButton("Save Calibration", self)
        self.load_calibration_but = QPushButton("Load Calibration", self)
        self.display_calibration_but = QCheckBox("")
        form = QFormLayout()
        form.addRow(cal_mode_label,self.cal_mode_box)
        form.addRow(new_cal_label,self.conduct_calibration_but)
        form.addRow(upd_cal_label,self.update_calibration_but)
        form.addRow(show_cal_label,self.display_calibration_but)
        h_layout2 = QHBoxLayout()
        h_layout2.addWidget(self.save_calibration_but)
        h_layout2.addWidget(self.load_calibration_but)
        # Groupbox and master layout
        pip_cal_layout = QVBoxLayout()
        pip_cal_layout.addLayout(grid_layout1)
        pip_cal_layout.addLayout(h_layout_angio)
        pip_cal_layout.addWidget(h_sep1)
        pip_cal_layout.addLayout(form)
        pip_cal_layout.addLayout(h_layout2)
        self.pip_cal_group = QGroupBox('Pipette Calibration')
        self.pip_cal_group.setLayout(pip_cal_layout)
        # Set connections
        self.conduct_calibration_but.stateChanged.connect(self.conduct_calibration)
        self.update_calibration_but.stateChanged.connect(self.update_calibration)
        self.save_calibration_but.clicked.connect(self.save_calibration)
        self.load_calibration_but.clicked.connect(self.load_calibration)
        self.display_calibration_but.stateChanged.connect(self.display_calibration)
        self.angle_mode_box.currentTextChanged.connect(self.set_angle_mode)
        self.cal_mode_box.currentTextChanged.connect(self.set_cal_mode)
        self.set_angle_button.clicked.connect(self.set_pipette_angle)
        self.save_angle_button.clicked.connect(self.save_pipette_angle)
        self.load_angle_button.clicked.connect(self.load_pipette_angle)

    def stateify_pipette_calibrator_widgets(self):
        ''' Set initial states for the pipette calirbator widgets '''
        self.angle_mode_box.insertItems(0, ['Automatic','Manual'])
        self.angle_mode_box.setCurrentText('Automatic')
        self.cal_mode_box.insertItems(0,['Manual','Semi-Auto.','Automatic'])
        self.cal_mode_box.setCurrentText('Semi-Auto.')
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

    def workflow_widgets(self):
        '''
        Creates widgets for showing status of injection workflow
        '''
        angle = QLabel('Set pipette angle')
        self.angle_indicator = QLabel()
        self._set_incomplete(self.angle_indicator)
        calibration = QLabel('Conduct manipulator calibration')
        self.calibration_indicator = QLabel()
        self._set_incomplete(self.calibration_indicator)
        annotation = QLabel('Annotate injection target')
        self.annotation_indicator = QLabel()
        self._set_incomplete(self.annotation_indicator)
        parameter = QLabel('Set injection parameters')
        self.parameter_indicator = QLabel()
        self._set_incomplete(self.parameter_indicator)
        form = QFormLayout()
        form.addRow(self.angle_indicator,angle)
        form.addRow(self.calibration_indicator,calibration)
        form.addRow(self.annotation_indicator,annotation)
        form.addRow(self.parameter_indicator,parameter)
        self.workflow_group = QGroupBox('Pre-injection Workflow')
        self.workflow_group.setLayout(form)

    def workflow_connections(self):
        self._angle_complete.connect(self._set_angle_ind)
        self._calibration_complete.connect(self._set_calibration_ind)
        self._annotation_complete.connect(self._set_annotation_ind)
        self._parameter_complete.connect(self._set_parameter_ind)

    def injection_parameter_widgets(self):
        ''' Creates widgets for injection parameters '''
        mu = "µ"
        approach_label = QLabel("Approach Distance ("+ mu +"m)")
        depth_label = QLabel("Depth ("+ mu +"m)")
        spacing_label = QLabel("Spacing ("+ mu +"m)")
        speed_label = QLabel("Speed (%)")
        self.approach_entry = QLineEdit(self)
        self.depth_entry= QLineEdit(self)
        self.spacing_entry = QLineEdit(self)
        self.speed_entry = QLineEdit(self)
        self.run_button = QPushButton("Run Trajectory")
        self.run_button.clicked.connect(self.run_3D_trajectory)
        self.stop_button = QPushButton("Stop Process")
        self.stop_button.clicked.connect(self.stoptrajectory)
        self.pressure_slider = QSlider(Qt.Orientation.Horizontal)
        self.pressure_slider = QSlider(Qt.Orientation.Horizontal)
        self.pressure_slider.setMinimum(10)
        self.pressure_slider.setMaximum(255)
        self.pressure_slider.setTickPosition(QSlider.TickPosition.TicksBelow)
        self.pressure_slider.setTickInterval(30)
        self.pressure_slider.valueChanged.connect(self.valuechange)
        pressure_label = QLabel("Pressure")
        self.pressure_display = QLineEdit(self)
        self.set_values_button = QPushButton("Set Values")
        self.set_values_button.clicked.connect(self.setautomatedparameters)
        h_layout1 = QHBoxLayout()
        h_layout1.addWidget(approach_label)
        h_layout1.addWidget(self.approach_entry)
        h_layout2 = QHBoxLayout()
        h_layout2.addWidget(depth_label)
        h_layout2.addWidget(self.depth_entry)
        h_layout3 = QHBoxLayout()
        h_layout3.addWidget(spacing_label)
        h_layout3.addWidget(self.spacing_entry)
        h_layout4 = QHBoxLayout()       
        h_layout4.addWidget(speed_label)
        h_layout4.addWidget(self.speed_entry)
        v_layout1 = QVBoxLayout()
        v_layout1.addWidget(self.pressure_slider)
        v_layout2 = QVBoxLayout()
        v_layout2.addWidget(self.pressure_display)
        h_layout5 = QHBoxLayout()
        h_layout5.addLayout(v_layout1)
        h_layout5.addLayout(v_layout2)
        v_layout3 = QVBoxLayout()
        v_layout3.addWidget(pressure_label)
        v_layout3.addLayout(h_layout5)
        v_layout4 = QVBoxLayout()
        v_layout4.addWidget(self.set_values_button)
        v_layout4.addWidget(self.run_button)
        v_layout4.addWidget(self.stop_button)
        v_layout3.addLayout(v_layout4)
        inj_param_layout = QVBoxLayout()
        inj_param_layout.addLayout(h_layout1)
        inj_param_layout.addLayout(h_layout2)
        inj_param_layout.addLayout(h_layout3)
        inj_param_layout.addLayout(h_layout4)
        inj_param_layout.addLayout(v_layout3)
        self.inj_parameter_group = QGroupBox('Automated Microinjection Controls')
        self.inj_parameter_group.setLayout(inj_param_layout)
    
    def stateify_injection_parameter_widgets(self):
        ''' set initial states for injection parameters'''
        self.approach_entry.insert('100')
        self.depth_entry.insert('20')
        self.spacing_entry.insert('50')
        self.speed_entry.insert('1000')
        self.pressure_slider.setValue(20)


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
        
        #Manipulator Status 
        # -*- coding: utf-8 -*-
        self.mu = "µ"
        self.motorchangeincrementtext = QLabel("Increment (" + self.mu + "m)")
        self.motorchangespeedtext = QLabel("Speed (%)")
        self.motorchangeincrement = QLineEdit(self)
        self.motorchangespeed = QLineEdit(self)
        self.motorxpositiontext = QLabel("X Position")
        self.motorypositiontext = QLabel("Y Position")
        self.motorzpositiontext = QLabel("Z Position")
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
        self.leftside.addWidget(self.workflow_group)

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
        self.rightside.addWidget(self.inj_parameter_group)
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

    def valuechange(self):
        self.pressureslidervalue= self.pressure_slider.value()
        self.displaypressure = int(self.pressureslidervalue/2.55)
        self.pressure_display.setText(str(self.displaypressure)+'%')

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

    def ask_continue_change_angle(self):
        '''
        If the system is currently calibrated with an angle, ask user if want to continue
        which will invalidate the current calibraiton

        If user clicks no (or not calibrated), nothing happens
        If user clicks yes, resets calibration
        '''
        if self.pip_cal.model.is_calibrated is True:
            qm = QMessageBox()
            ret = qm.question(self,'New angle?','Calibration is complete, and changing the angle will invalidate the current calibration. Do you still want to change the angle ?')
            if ret == QMessageBox.StandardButton.No:
                return False
            else:
                self.pip_cal.model.reset_calibration()
                self._calibration_complete.emit(False)
                return True
        else:
            return True

    def set_angle_mode(self):
        '''
        Sets the mode for querying the pipette angle.
        Called when pipette angle combo box changes.

        Activity
            if automatic:
                angle entry box to readonly and dark gray
                Disable set button
                query sensapex for angle, insert in combobox and set pip_angle
            if manual:
                clear pip_angle
                enable entry box and make white
                enable set button
        '''
        # Ask user if wants to change angle mode since will effect calibration
        change_ang = self.ask_continue_change_angle()
        if change_ang is False:
            angle_mode = self.angle_mode_box.currentIndex()
            # Change back to other state if user doesn't want to effect calibration
            if angle_mode == 1:
                tmp_block = QSignalBlocker(self.angle_mode_box)
                self.angle_mode_box.setCurrentIndex(0)
                tmp_block.unblock()
            else:
                tmp_block = QSignalBlocker(self.angle_mode_box)
                self.angle_mode_box.setCurrentIndex(1)
                tmp_block.unblock()
            return None

        angle_mode = self.angle_mode_box.currentText()
        # Handle different angle mode selections
        if angle_mode == 'Automatic':
            # Read only and gray out angle entry box
            palette = QPalette()
            palette.setColor(QPalette.ColorRole.Base, QColor('lightGray'))
            self.angle_entry.setReadOnly(True)
            self.angle_entry.setPalette(palette)
            # Disable set, and load buttons
            self.set_angle_button.setEnabled(False)
            self.load_angle_button.setEnabled(False)
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
            self.load_angle_button.setEnabled(True)
            # Clear the box and reset pip_angle
            self.angle_entry.clear()
            # FIXME poor practice (will be fixed when change trajecotry)
            if 'pip_angle' in self.__dict__.keys():
                self._angle_complete.emit(False)
                del self.pip_angle

            
    def set_pipette_angle(self):
        ''' Sets pipette angle from entry box '''
        # Ask user if they want to change angle since will effect calibration.
        change_ang = self.ask_continue_change_angle()
        if change_ang is False:
            # Break function (dont set angle)
            return None

        angle_str = self.angle_entry.text()
        if val.is_valid_number(angle_str) is False:
            self.logger.warning(f'Invalid angle: {angle_str}. Angle must be a valid number.')
            self.error_msg.setText(f'Invalid angle: {angle_str}. Angle must be a valid number.')
            self.error_msg.exec()
        else:
            self.pip_angle = np.deg2rad(float(angle_str))
            self.response_monitor_window.append(f">> Pipette angle set as {angle_str}")
            self._angle_complete.emit(True)
            self.calibration_inputs_changed()

    def save_pipette_angle(self):
        ''' Saves pipette angle to file '''
        if 'pip_angle' not in self.__dict__.keys():
            self.logger.warning(f"Cannot save pipette angle because the angle is not set.")
            self.error_msg.setText(f"Cannot save pipette angle because the angle is not set.")
            self.error_msg.exec()
        else:
            try:
                self.angle_io.save(pip_angle_rad=self.pip_angle)
            except Exception as e:
                self.logger.exception("Error while saving pipette angle")
                self.error_msg.setText(f"Error: {e}\n\nSee logs for more information.")
                self.error_msg.exec()
            finally:
                self.logger.info(f"Saved angle of {round(np.rad2deg(self.pip_angle),1)} to file.")
                self.response_monitor_window.append(f">> Saved pipette angle of {round(np.rad2deg(self.pip_angle),1)} to file.")

    def load_pipette_angle(self):
        ''' Loads pipette angle from file '''
        # Ask user if they want to change angle since will effect calibration.
        change_ang = self.ask_continue_change_angle()
        if change_ang is False:
            # Break function (dont set angle)
            return None
        # Ask if want to change angle if already set
        if 'pip_angle' in self.__dict__.keys():
            qm = QMessageBox()
            ret = qm.question(self,'Load angle?','Angle is already set. Do you still want to load the most recently saved angle?')
            if ret == QMessageBox.StandardButton.No:
                return None
        # Load the angle
        try:
            ang_data = self.angle_io.load_latest()
            ang_rad = ang_data['Angle']
            ang_date = ang_data['Date']
            ang_time = ang_data['Time']
            ang_deg = round(np.rad2deg(ang_rad),1)
        except AngleFileError as e:
            self.logger.warning(str(e))
            self.warn_msg.setText(f"Error: {e}.\n\nYou must save the angle file before it can be loaded.")
            self.warn_msg.exec()
        except Exception as e:
            self.logger.exception('Error while loading pipette error')
            self.error_msg.setText(f"Error: {e}. Check logs for traceback.")
            self.error_msg.exec()
        else:
            self.logger.info(f"Pipette angle loaded as {round(ang_deg,1)} degrees from {ang_date} {ang_time}.")
            self.response_monitor_window.append(f">> Most recent pipette angle loaded as {round(ang_deg,1)} degrees from {ang_date} {ang_time}.")
            self.angle_entry.clear()
            self.angle_entry.insert(str(ang_deg))
            self.set_pipette_angle()
            
    def set_cal_mode(self):
        ''' Sets the mode for conducting calibration. Called when cal_mode_box
        changes. '''
        # Get selected mode
        cal_mode = self.cal_mode_box.currentText()
        info_str = f"Calibration mode selected as {cal_mode}"
        self.logger.info(info_str)
        self.response_monitor_window.append(f">> {info_str}")
        if cal_mode == "Automatic":
            self.cal_mode_box.setCurrentText('Semi-Auto.')
            self.logger.warning("Automatic calibration not yet implimented")
            self.warn_msg.setText(f"Automatic calibration not yet implimented. Defaulting to 'Semi-Auto.' calibration mode.")
            self.warn_msg.exec()
        if cal_mode == "Semi-Auto.":
            pass
        if cal_mode == "Manual":
            pass

    def add_cal_positions(self,x_click:float, y_click:float):
        ''' Adds clicked calibration positions to calibration data '''
        if self.conduct_calibration_but.isChecked():
            z_scope = self.zen_group.zen.get_focus_um()
            ex_pos = [x_click, y_click, z_scope]
            dev = SensapexDevice(1)
            man_pos = dev.get_pos()
            self.pip_cal.data.add_cal_position(ex=ex_pos, man=man_pos)
            self.cal_pos_added.emit()
            if self.save_tip_annot.isChecked():
                self.tipposition1 = self.vidctrl.tipcircle
                tip_dict = {'x':self.tipposition1.x(), 'y':self.tipposition1.y()}
                self.save_pip_cal_data(tip_dict)
        if self.update_calibration_but.isChecked():
            self.pip_cal.data.rm_all()
            z_scope = self.zen_group.zen.get_focus_um()
            ex_pos = [x_click, y_click, z_scope]
            dev = SensapexDevice(1)
            man_pos = dev.get_pos()
            self.pip_cal.data.add_cal_position(ex=ex_pos, man=man_pos)
            self.cal_pos_added.emit()

    def manual_calibration(self):
        ''' Conducts manual calibration process '''
        self.logger.debug('Doing manual calibration')

    def semi_auto_calibration(self):
        self.logger.debug('Doing Semi-Auto. calibration')
        # Initalize for the calibration trajectory
        dev = SensapexDevice(1)
        vid_width = self.vidctrl.width
        vid_height = self.vidctrl.height
        z_scope = self.zen_group.zen.get_focus_um()
        _, obj_mag = self.zen_group.parse_combobox('objective')
        _, opto_mag = self.zen_group.parse_combobox('optovar')
        self.cal_trajectory = SemiAutoCalibrationTrajectory(dev=dev, cal=self.pip_cal, img_w=vid_width, img_h=vid_height, ex_z=z_scope,z_polarity=-1,pip_angle=self.pip_angle, obj_mag=obj_mag, opto_mag=opto_mag)
        self.cal_pos_added.connect(self.cal_trajectory.next_cal_position)
        self.cal_trajectory.finished.connect(self.conduct_calibration_but.toggle)

    def auto_calibration(self):
        ''' Conducts automatic calibration process '''
        tmp_block = QSignalBlocker(self.conduct_calibration_but)
        self.conduct_calibration_but.setChecked(False)
        tmp_block.unblock()
        self.cal_mode_box.setCurrentText('Semi-Auto.')
        self.logger.warning("Automatic calibration not yet implimented")
        self.warn_msg.setText(f"Automatic calibration not yet implimented. Defaulting to 'Semi-Auto.' calibration mode.")
        self.warn_msg.exec()

    def conduct_calibration(self):
        ''' Conducts calibration process '''
        # If user activates checkbox, start calibration
        if self.conduct_calibration_but.isChecked() is True:
            if self.update_calibration_but.isChecked():
                tmp_block = QSignalBlocker(self.conduct_calibration_but)
                self.conduct_calibration_but.setChecked(False)
                tmp_block.unblock()
                self.logger.warning('Cannot conduct calibraiton while updating.')
                self.warn_msg.setText('Cannot conduct calibraiton while updating. Complete update calibration process (and uncheck the box) before conducting a new calibration.')
                self.warn_msg.exec()
                return None
            cal_mode = self.cal_mode_box.currentText()
            if cal_mode == "Automatic":
                self.auto_calibration()
            elif cal_mode == 'Manual':
                self.manual_calibration
            elif cal_mode == 'Semi-Auto.':
                self.semi_auto_calibration()
            else:
                self.logger.warning("No mode calibration mode selected")
                self.warn_msg.setText(f"Select calibration mode before starting calibration.")
                self.warn_msg.exec()

        # If user deactivates checkbox, finish and compute calibration
        if self.conduct_calibration_but.isChecked() is False:
            self.compute_calibration()

    def compute_calibration(self):
        ''' Computes calibration from calibration data '''
        if self.conduct_calibration_but.isChecked() is False:
            try:
                if "pip_angle" not in list(self.__dict__.keys()):
                    raise CalibrationError(f'Pipette angle not set. The pipette angle must be set before conducting a calibration.')
                _, obj_mag = self.zen_group.parse_combobox('objective')
                _, opto_mag = self.zen_group.parse_combobox('optovar')
                self.logger.info(f"Computing calibration with data\n{self.pip_cal.data.data_df}")
                self.pip_cal.compute(z_polarity=-1, pip_angle=self.pip_angle, obj_mag=obj_mag, opto_mag=opto_mag)
            except CalibrationDataError as e:
                self.logger.warning(e)
                self.warn_msg.setText(f"Calibration not completed. Error: {e}\n\nMake sure you click on the tip to register at least 3 points (that don't lie on a line) before unchecking 'Calibrate'.")
                self.warn_msg.exec()
            except CalibrationError as e:
                self.logger.warning(e)
                self.error_msg.setText(f"Calibration not completed. Error: {e}.")
                self.error_msg.exec()
            except Exception as e:
                self.logger.exception("Error while computing calibration")
                self.error_msg.setText(f"Error: {e}\n\nSee logs for more information.")
                self.error_msg.exec()
            else:
                self._calibration_complete.emit(True)
                self.logger.info(f"Existing models:\n{self.pip_cal.tmp_storage._existing_models.keys()}")
            finally:
                self.logger.info('Calibration unchecked. Deleting calibration points.')
                self.pip_cal.data.rm_all()
    
    def display_calibration(self):
        ''' Send computed tip positoin (in camera) to video to be displayed.
        Calls itself on timer to update tip position in video'''
        if self.display_calibration_but.isChecked():
            if self.pip_disp_timer.isActive() is False:
                self.pip_disp_timer.start(self.pip_disp_timeout)
            if self.pip_cal.model.is_calibrated is True:
                self.vidctrl.display_tip_pos = True
                dev = SensapexDevice(1)
                pos = dev.get_pos()
                ex = self.pip_cal.model.forward(man=pos)
                self.vidctrl.show_tip_pos(ex[0], ex[1])
            else:
                self.vidctrl.display_tip_pos = False
        else:
            self.vidctrl.display_tip_pos = False
            self.pip_disp_timer.stop()

    def update_calibration(self):
        ''' Updates calibration by setting new reference position '''
        if self.update_calibration_but.isChecked():
            if self.pip_cal.model.is_calibrated is False:
                tmp_block = QSignalBlocker(self.update_calibration_but)
                self.update_calibration_but.setChecked(False)
                tmp_block.unblock()
                e = "System is not calibrated. Can not update non-existent calibration."
                self.logger.warning(e)
                self.warn_msg.setText(f"Error: {e}\n\nConduct a new calibration or load an exsisting calibration before updating.")
                self.warn_msg.exec()
            if self.conduct_calibration_but.isChecked():
                tmp_block = QSignalBlocker(self.update_calibration_but)
                self.update_calibration_but.setChecked(False)
                tmp_block.unblock()
                self.logger.warning('Cannot update calibration while already conducting calibration.')
                self.warn_msg.setText('Cannot update calibration while already conducting calibration. Complete calibration process (and uncheck the box) before updating a calibration.')
                self.warn_msg.exec()
        else:
            try:
                _, obj_mag = self.zen_group.parse_combobox('objective')
                _, opto_mag = self.zen_group.parse_combobox('optovar')
                self.pip_cal.update(obj_mag=obj_mag, opto_mag=opto_mag, pip_angle=self.pip_angle)
            except CalibrationDataError as e:
                self.logger.warning(e)
                self.warn_msg.setText(f"Calibration not updated. Error: {e}\n\nMake sure you click on the tip to register the calibration point before unchecking 'Update'.")
                self.warn_msg.exec()
            except Exception as e:
                self.logger.exception("Error while updating calibration")
                self.error_msg.setText(f"Error: {e}\n\nSee logs for more information.")
                self.error_msg.exec()
            finally:
                self.logger.info('Calibration unchecked. Deleting calibration points.')
                self.pip_cal.data.rm_all()

    def save_calibration(self):
        ''' Saves calibration to a file '''
        if self.pip_cal.model.is_calibrated is False:
            e = "System is not calibrated. Can not save non-existent calibration."
            self.logger.warning(e)
            self.warn_msg.setText(f"Error: {e}\n\nConduct a new calibration or load and update an existing calibration before saving.")
            self.warn_msg.exec()
        else:
            try:
                _, obj_mag = self.zen_group.parse_combobox('objective')
                _, opto_mag = self.zen_group.parse_combobox('optovar')
                self.pip_cal.save_model(obj_mag=obj_mag, opto_mag=opto_mag, ang_deg=np.rad2deg(self.pip_angle))
            except Exception as e:
                self.logger.exception("Error while saving calibration")
                self.error_msg.setText(f"Error while saving calibration: {e}")
                self.error_msg.exec()

    def load_calibration(self):
        ''' Loads the most recent calibration '''
        # Ask if user wants to overwrite an existing calibration
        if self.pip_cal.model.is_calibrated is True:
            qm = QMessageBox()
            ret = qm.question(self,'Load calibration?','System is already calibrated. Do you still want to load the most recent calibration?')
            if ret == QMessageBox.StandardButton.No:
                return None
        try:
            _, obj_mag = self.zen_group.parse_combobox('objective')
            _, opto_mag = self.zen_group.parse_combobox('optovar')
            self.pip_cal.load_model(obj_mag=obj_mag, opto_mag=opto_mag, ang_deg=np.rad2deg(self.pip_angle))
        except CalibrationFileError as e:
            self.logger.warning(str(e))
            self.warn_msg.setText(f"Error: {e}.\n\nYou must save a calibration file before it can be loaded.")
            self.warn_msg.exec()
        except CalibrationDNEError as e:
            self.logger.warning(str(e))
            self.warn_msg.setText(f"Error: {e}.\n\nYou must save a calibration with this microscope configuration before it can be loaded.")
            self.warn_msg.exec()
        except Exception as e:
            self.logger.exception('Error while loading calibration')
            self.error_msg.setText(f"Error: {e}. Check logs for traceback.")
            self.error_msg.exec()    
        else:
            self._calibration_complete.emit(True)        

    def calibration_inputs_changed(self):
        ''' Changes the calibration if the microscope changes'''
        # Set new calibraiton model if it exists or warn user
        _, obj_mag = self.zen_group.parse_combobox('objective')
        _, opto_mag = self.zen_group.parse_combobox('optovar')
        # Use an existing calibration for this config or show warning if was already calibrated and now its not
        if self.pip_cal.model.is_calibrated is True:
            try:
                self.pip_cal.use_existing_model(obj_mag=obj_mag, opto_mag=opto_mag, ang_deg=np.rad2deg(self.pip_angle))
                self._calibration_complete.emit(True)
            except CalibrationDNEError as e:
                self.pip_cal.model.reset_calibration()
                self._calibration_complete.emit(False)
                self.logger.warning(str(e))
                self.warn_msg.setText(f"You switched the microscope configuration, but '{e}'.\n\nYour previous calibration is invalid. Either switch to the previous microscope configuration or conduct/load a calibration for this microscope configuration.")
                self.warn_msg.exec()
        # Load calibration or no warning if not exist
        else:
            try:
                self.pip_cal.use_existing_model(obj_mag=obj_mag, opto_mag=opto_mag, ang_deg=np.rad2deg(self.pip_angle))
                self._calibration_complete.emit(True)
            except CalibrationDNEError as e:
                pass


    
    def save_pip_cal_data(self, tip_dict:dict):
        '''
        Save image from camera and annotated coordinates of a mouse click

        Arguments:
            tip_dict: {'x':x, 'y':y} x and y coordinates of tip annotation on image.
        '''
        # Instance of object to save data
        pip_data_saver = PipTipData(pip_data_dir=self.pip_data_dir)
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
        # Instance of object to save data
        tis_data_saver = TissueEdgeData(tis_data_dir=self.tis_data_dir)
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
            self._annotation_complete.emit(False)
            self.error_msg.setText("CAM error, is camera plugged in? \nPython error = \n" + str(sys.exc_info()[1]))
            self.error_msg.exec()
            self.response_monitor_window.append(">> Camera error. \n>> Python error = " + str(sys.exc_info()))
        else:
            self._annotation_complete.emit(True)

    """
    -------------------------- Pre injection workflow controls   -----------------------------
    These Functions control testing resolution error
    """    
    def _set_angle_ind(self,state:bool):
        if state is True:
            self._set_complete(self.angle_indicator)
        else:
            self._set_incomplete(self.angle_indicator)

    def _set_calibration_ind(self,state:bool):
        if state is True:
            self._set_complete(self.calibration_indicator)
        else:
            self._set_incomplete(self.calibration_indicator)

    def _set_annotation_ind(self,state:bool):
        if state is True:
            self._set_complete(self.annotation_indicator)
        else:
            self._set_incomplete(self.annotation_indicator)

    def _set_parameter_ind(self,state:bool):
        if state is True:
            self._set_complete(self.parameter_indicator)
        else:
            self._set_incomplete(self.parameter_indicator)

        
    def _set_complete(self,label:QLabel):
        ''' Set the widget with a complete icon '''
        pixmapi = QStyle.StandardPixmap.SP_DialogYesButton
        icon = self.style().standardIcon(pixmapi)
        pixmap = icon.pixmap(QSize(18,18))
        label.setPixmap(pixmap)

    def _set_incomplete(self,label:QLabel):
        ''' Set the widget with a incomplete icon '''
        pixmapi = QStyle.StandardPixmap.SP_DialogNoButton
        icon = self.style().standardIcon(pixmapi)
        pixmap = icon.pixmap(QSize(18,18))
        label.setPixmap(pixmap)
        
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
            getpos = GetPos(self.c0,self.c2,self.m0,1000,self.ymotortheta,self.pip_angle,self.pixelsize)
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
            self.approachdist = self.approach_entry.text()
            self.depthintissue = self.depth_entry.text()
            self.stepsize = self.spacing_entry.text()
            self.motorspeed = self.speed_entry.text()
            self.injectpressurevoltage = self.compensationpressureval
            self.response_monitor_window.append(">> Values set")
            self.injector_compensate = injection(arduino,self.compensationpressureval, 0,self.injectpressurevoltage,0,'bp')
            self.injector_compensate.start()

        except:
            self.error_msg.setText("Error, did you enter all parameters? Is the arduino plugged in? \nPython error = \n" + str(sys.exc_info()[1]))
            self.error_msg.exec()
            self.response_monitor_window.append(">> Python error = " + str(sys.exc_info()))
        else:
            self._parameter_complete.emit(True)
        
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

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
from src.video_control.video_utils import interpolate, AnnotationError
from src.video_control.video import MMCamera, VideoDisplay
from src.motorcontrol.motorlocationThread import motorpositionThread
from src.pythonarduino.injectioncontrolmod import injection
from src.cfg_mgmt.cfg_mngr import CfgManager
from src.Qt_utils.gui_objects import QHLine
from src.miscellaneous.standard_logger import StandardLogger as logr
from src.miscellaneous import validify as val
from src.data_generation.data_generators import PipTipData, TissueEdgeData
from src.manipulator_control.calibration import Calibrator, AngleIO, CalibrationError, CalibrationDNEError, CalibrationFileError, CalibrationDataError, AngleFileError
from src.manipulator_control.sensapex_utils import SensapexDevice
from src.manipulator_control.injection_trajectory import SurfaceLineTrajectory3D
from src.manipulator_control.calibration_trajectory import SemiAutoCalibrationTrajectory
from src.ZEN_interface.ZEN_mvc import ModelZEN, ControllerZEN, ViewZENComplete, ViewZENFocus
from src.deep_learning.tissue_detection import ModelTissueDetection
from src.deep_learning.edge_utils.error_utils import EdgeNotFoundError



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

    def __init__(self,cam,brand,val,bins,rot,imagevals,scale,com, parent=None):
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

        # initiate thread to poll position of motors and report error if they are not found
        try:
            self.getposition = motorpositionThread()
            self.getposition.start()
            self.motorfound = True
        except:
            msg = "Manipulators not detected. Wait 2 minutes then relaunch the app. If this does not work, replug manipulators into computer."
            self.show_error_box(msg)
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
            msg = "Arduino not detected, make sure you selected the correct com port, plug in, and try again."
            self.show_error_box(msg)
            self.arduinofound = False

        #initiate video stream thread using camera settings
        self.cam_ = cam
        self.brand_ = brand
        self.val_ = val
        self.bins_ = bins
        self.rot_ = rot
        self.imagevals_ = imagevals
        self.file_selected = 0
        self.setup_gui()

        #initiate parameters for injection
        self.ninjection = 0 
        self.injectpressurevoltage = 0
        self.pulseduration = 0
        self.edgedetected = False
    
    """
    ============================================================================================

    GUI INITIALIZATION

    ============================================================================================
    """
    def setup_gui(self):
        ''' 
        Setup the gui by importing configuration, defining directires, instantiating imports,
        creating widgets, laying out the gui, setting widget connections, initializeing
        the widget states, and finally assessing startup errors.
        '''
        # Load the configuration values
        self.get_gui_cfg()

        # Define necessary directories
        self.define_dirs()

        # Instantiate imports
        self.pip_cal = Calibrator(cal_data_dir=self.cal_data_dir)
        self.angle_io = AngleIO(ang_data_dir=self.cal_data_dir)
        ckpt_path = "Autoinjector/src/deep_learning/weights/20220824_180000_Colab_gpu/best.pth"
        self.tissue_model = ModelTissueDetection(ckpt_path)

        # Instantiate the imported camera
        try:
            mm_path = self.cfg.cfg_gui.values['micromanager path'].replace('\\','/')
            self.cam_MM = MMCamera(mm_path, self.cam_, self.brand_, self.val_, self.bins_, self.rot_, self.imagevals_)
        except Exception as e:
            msg = f"Error while interfacing with camera: {e}.\n\nSee logs for more info."
            self.show_exception_box(e)

        # Instantiate imported widgets
        self.vid_display = VideoDisplay(self.cam_MM, height=900, fps=50)
        zen_model = ModelZEN()
        self.zen_controller = ControllerZEN(zen_model)
        self.full_zen_app = ViewZENComplete(self.zen_controller)
        self.focus_zen_app = ViewZENFocus(self.zen_controller)

        # Create main gui widgets
        self.make_widgets()
        
        # Layout the widget groups in the main GUI
        self.layout_gui()
        
        # Set the widget signal/slot connections
        self.set_connections()

        # Define the initial widget states
        self.stateify_widgets()
        
        # Assess any erros from startup
        self.assess_startup_errors()

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

    def make_widgets(self):
        """ Create the GUI's widgets """
        # Create all interactive widgets
        self.make_pipette_calibrator_widgets()
        self.make_data_generator_widgets()
        self.make_annotation_widgets()
        self.make_display_modification_widgets()
        self.make_video_display_widgets()
        self.make_injection_parameter_widgets()
        self.make_workflow_widgets()
        self.make_system_status_widgets()
        # Create pages for the stacked layout with different modes (like calibration
        # mode or anotation mode)
        self.make_default_left_page_widget()
        self.make_default_right_page_widget()
        self.make_annotation_mode_left_page_widget()
        self.make_annotation_mode_right_page_widget()

    def set_connections(self):
        """ Set the signal/slot connections for the GUI's widgets """
        self.set_pipette_calibrator_connections()
        self.set_annotation_connections()
        self.set_display_modification_connections()
        self.set_video_display_connections()
        self.set_inject_parameter_connections()
        self.set_workflow_connections()
        self.set_zeiss_connections()

    def stateify_widgets(self):
        """ Set the initial state for the GUI's widgets """
        # Initialize states for the various gui widgets
        self.stateify_pipette_calibrator_widgets()
        self.stateify_data_generator_widgets()
        self.stateify_annotation_widgets()
        self.stateify_display_modification_widgets()
        self.stateify_injection_parameter_widgets()
        # Show microscope configuration in response monitor
        self.print_inital_ZEN_states_to_monitor()

    def layout_gui(self):
        # Organize the GUI layout
        self.master_layout = QGridLayout()
        self.left_stacked_layout = QStackedLayout()
        self.right_stacked_layout = QStackedLayout()
        self.center_layout = QHBoxLayout()
        self.bottom_layout = QHBoxLayout()

        # Add the different pages to the stacked layout
        self.left_stacked_layout.addWidget(self.default_left_page)
        self.left_stacked_layout.addWidget(self.annotation_mode_left_page)
        self.right_stacked_layout.addWidget(self.default_right_page)
        self.right_stacked_layout.addWidget(self.annotation_mode_right_page)

        # Define the center layout widgets (places in `center_widget` so aligns with stacked pages)
        center_widget = QWidget()
        tmp_layout1 = QHBoxLayout()
        tmp_layout1.addWidget(self.video_display_group)
        center_widget.setLayout(tmp_layout1)
        self.center_layout.addWidget(center_widget)

        # Define the bottom layout widgets (places in `bottom_widget` so aligns with stacked pages)
        bottom_widget = QWidget()
        tmp_layout2 = QHBoxLayout()
        tmp_layout2.addWidget(self.system_status_group)
        bottom_widget.setLayout(tmp_layout2)
        self.bottom_layout.addWidget(bottom_widget)

        # Add layouts to master layout
        self.master_layout.addLayout(self.left_stacked_layout,0,0,1,1)
        self.master_layout.addLayout(self.center_layout,0,1,1,1)
        self.master_layout.addLayout(self.right_stacked_layout,0,2,1,1)
        self.master_layout.addLayout(self.bottom_layout,1,0,1,3)

        # Set main window details
        self.setWindowTitle('Autoinjector 2.0')
        self._central_widget.setLayout(self.master_layout)
        self.show()
        self.setWindowIcon(QIcon('favicon.png'))
        self.master_layout.setContentsMargins(5, 5, 5, 5)


    """
    Initialize pipette calibration widgets --------------------------------------------------------
    """
    def make_pipette_calibrator_widgets(self):
        ''' Creates widgets for pipette calibration and specifies layout '''
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
        form = QFormLayout()
        form.addRow(cal_mode_label,self.cal_mode_box)
        form.addRow(new_cal_label,self.conduct_calibration_but)
        form.addRow(upd_cal_label,self.update_calibration_but)
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
    
    def set_pipette_calibrator_connections(self):
        """ Set the signal/slot connections for the pipette calibrator widgets """
        self.conduct_calibration_but.clicked.connect(self.handle_calibration_button_clicked)
        self.update_calibration_but.clicked.connect(self.update_calibration)
        self.save_calibration_but.clicked.connect(self.save_calibration)
        self.load_calibration_but.clicked.connect(self.load_calibration)
        self.angle_mode_box.currentTextChanged.connect(self.set_angle_mode)
        self.cal_mode_box.currentTextChanged.connect(self.set_cal_mode)
        self.set_angle_button.clicked.connect(self.set_pipette_angle)
        self.save_angle_button.clicked.connect(self.save_pipette_angle)
        self.load_angle_button.clicked.connect(self.load_pipette_angle)

    def stateify_pipette_calibrator_widgets(self):
        ''' Set initial states for the pipette calibrator widgets '''
        self.angle_mode_box.insertItems(0, ['Automatic','Manual'])
        self.angle_mode_box.setCurrentText('Automatic')
        self.cal_mode_box.insertItems(0,['Manual','Semi-Auto.','Automatic'])
        self.cal_mode_box.setCurrentText('Semi-Auto.')


    """
    Initialize data generation widgets ------------------------------------------------------------
    """
    def make_data_generator_widgets(self):
        ''' Creates widgets for data generation and specifies layout '''
        save_tip_annot_label = QLabel('Save tip:')
        self.save_tip_annot_rbon = QRadioButton("On")
        rboff1 = QRadioButton("Off")
        save_tissue_annot_label = QLabel('Save tissue:')
        self.save_tiss_annot_rbon = QRadioButton("On")
        rboff2 = QRadioButton("Off")
        bg1 = QButtonGroup(self)
        bg1.addButton(self.save_tip_annot_rbon)
        bg1.addButton(rboff1)
        bg2 = QButtonGroup(self)
        bg2.addButton(self.save_tiss_annot_rbon)
        bg2.addButton(rboff2)
        rboff1.setChecked(True)
        rboff2.setChecked(True)
        hl1 = QHBoxLayout()
        hl1.addWidget(self.save_tip_annot_rbon)
        hl1.addWidget(rboff1)
        hl2 = QHBoxLayout()
        hl2.addWidget(self.save_tiss_annot_rbon)
        hl2.addWidget(rboff2)
        form = QFormLayout()
        form.addRow(save_tip_annot_label, hl1)
        form.addRow(save_tissue_annot_label, hl2)

        self.data_gen_group = QGroupBox('GUI Data Acquisition')
        self.data_gen_group.setLayout(form)

    def stateify_data_generator_widgets(self):
        ''' Set initial states for data generator widgets '''
        self.save_tip_annot_rbon.setChecked(True)
        self.save_tiss_annot_rbon.setChecked(False)


    """
    Initialize injection target annotation widgets ------------------------------------------------
    """
    def make_annotation_widgets(self):
        """ Creates widgets for injection target annotation """
        # Make the default annotation widgets
        mode_label1 = QLabel('Annotation Mode:')
        self.annotation_combo_box = QComboBox()
        self.annotation_combo_box.setPlaceholderText('Mode')
        edge_label1 = QLabel('Auto-detect Edge:')
        self.edge_type_combo_box = QComboBox()
        self.edge_type_combo_box.setPlaceholderText('Type')
        self.annotation_button = QPushButton("Annotate Target")
        self.default_annotation_group = QGroupBox("Trajectory Annotation")
        # Make alternate annotation widgets
        mode_label2 = QLabel(mode_label1.text())
        self.annotation_mode_display = QLabel('')
        edge_label2 = QLabel(edge_label1.text())
        self.edge_type_display = QLabel('')
        self.complete_annotation_button = QPushButton("Complete Annotation")
        self.exit_annotation_button = QPushButton("Exit Annotation")
        hline = QHLine()
        guidance_label = QLabel('Annotation Guidance')
        self.annotation_guidance = QLabel('')
        self.annotation_guidance.setWordWrap(True)
        self.alt_annotation_group = QGroupBox("Annotation Control")
        # Specify the default annotation layout and group
        default_layout = QVBoxLayout()
        form1 = QFormLayout()
        form1.addRow(mode_label1, self.annotation_combo_box)
        form1.addRow(edge_label1, self.edge_type_combo_box)
        default_layout.addLayout(form1)
        default_layout.addWidget(self.annotation_button)
        self.default_annotation_group.setLayout(default_layout)
        alt_layout = QVBoxLayout()
        form2 = QFormLayout()
        form2.addRow(mode_label2, self.annotation_mode_display)
        form2.addRow(edge_label2, self.edge_type_display)
        alt_layout.addLayout(form2)
        alt_layout.addWidget(self.complete_annotation_button)
        alt_layout.addWidget(self.exit_annotation_button)
        alt_layout.addWidget(hline)
        alt_layout.addWidget(guidance_label)
        alt_layout.addWidget(self.annotation_guidance)
        self.alt_annotation_group.setLayout(alt_layout)

    def set_annotation_connections(self):
        """ Set signal/slot connections for annotation widgets """
        self.annotation_button.clicked.connect(self.annotate_trajectory_pressed)
        self.complete_annotation_button.clicked.connect(self.annotation_complete_pressed)
        self.exit_annotation_button.clicked.connect(self.exit_annotation_mode)
        self.annotation_combo_box.currentTextChanged.connect(self.annotation_combo_changed)
        self.edge_type_combo_box.currentTextChanged.connect(self.edge_type_combo_changed)

    def stateify_annotation_widgets(self):
        self.annotation_combo_box.insertItems(0,['Manual','Automatic'])
        self.annotation_combo_box.setCurrentText('Manual')
        self.edge_type_combo_box.insertItems(0,['Apical','Basal'])
        self.edge_type_combo_box.setCurrentText('Apical')
        self.update_annotation_display_labels()

    """
    Initialize video display modification widgets -------------------------------------------------
    """
    def make_display_modification_widgets(self):
        """ Make widgets to modify the video display and specifies widget layout """
        layout = QVBoxLayout()
        exposure_label = QLabel("Camera Exposure")
        self.exposure_slider = QSlider(Qt.Orientation.Horizontal)
        self.exposure_slider.setMinimum(15)
        self.exposure_slider.setMaximum(100)
        self.exposure_slider.setTickPosition(QSlider.TickPosition.TicksBelow)
        self.exposure_slider.setTickInterval(0)
        display_tip_label = QLabel('Show tip:')
        self.display_tip_rbon = QRadioButton("On")
        rboff1 = QRadioButton("Off")
        display_annotation_label = QLabel('Show annotation:')
        self.display_annotation_rbon = QRadioButton("On")
        rboff2 = QRadioButton("Off")
        display_segmentation_label = QLabel('Show tissue:')
        self.display_segmentation_rbon = QRadioButton("On")
        rboff3 = QRadioButton("Off")
        display_edge_label = QLabel('Show edge:')
        self.display_edges_rbon = QRadioButton("On")
        rboff4 = QRadioButton("Off")
        bg1 = QButtonGroup(self)
        bg1.addButton(self.display_tip_rbon)
        bg1.addButton(rboff1)
        bg2 = QButtonGroup(self)
        bg2.addButton(self.display_annotation_rbon)
        bg2.addButton(rboff2)
        bg3 = QButtonGroup(self)
        bg3.addButton(self.display_segmentation_rbon)
        bg3.addButton(rboff3)
        bg4 = QButtonGroup(self)
        bg4.addButton(self.display_edges_rbon)
        bg4.addButton(rboff4)

        hl1 = QHBoxLayout()
        hl1.addWidget(self.display_tip_rbon)
        hl1.addWidget(rboff1)
        hl2 = QHBoxLayout()
        hl2.addWidget(self.display_annotation_rbon)
        hl2.addWidget(rboff2)
        hl3 = QHBoxLayout()
        hl3.addWidget(self.display_segmentation_rbon)
        hl3.addWidget(rboff3)
        hl4 = QHBoxLayout()
        hl4.addWidget(self.display_edges_rbon)
        hl4.addWidget(rboff4)
        form = QFormLayout()
        form.addRow(display_tip_label, hl1)
        form.addRow(display_annotation_label, hl2)
        form.addRow(display_segmentation_label, hl3)
        form.addRow(display_edge_label, hl4)
        layout.addWidget(exposure_label)
        layout.addWidget(self.exposure_slider)
        layout.addLayout(form)
        self.display_modification_group = QGroupBox('Display Settings')
        self.display_modification_group.setLayout(layout)
        
    def stateify_display_modification_widgets(self):
        """ Sets initial states for dispaly modification widgets """
        exposure = int(float(self.cam_MM.get_exposure())*10)
        self.exposure_slider.setValue(exposure)
        self.display_tip_rbon.setChecked(True)
        self.display_annotation_rbon.setChecked(True)
        self.display_segmentation_rbon.setChecked(True)
        self.display_edges_rbon.setChecked(True)

    def set_display_modification_connections(self):
        """ Sets signal slot connections for display modification widgets """
        self.exposure_slider.valueChanged.connect(self.exposure_value_change)
        self.display_tip_rbon.toggled.connect(self.display_calibration)
        self.display_annotation_rbon.toggled.connect(self.display_annotation)
        self.display_segmentation_rbon.toggled.connect(self.display_tissue_mask)
        self.display_edges_rbon.toggled.connect(self.display_edge_mask)
        self.pip_disp_timer = QTimer()
        self.pip_disp_timer.timeout.connect(self.display_calibration)
        self.pip_disp_timeout = 25


    """
    Initialize video display widgets --------------------------------------------------------------
    """
    def make_video_display_widgets(self):
        layout = QVBoxLayout()
        layout.addWidget(self.vid_display)
        self.video_display_group = QGroupBox('Microscope Video Stream')
        self.video_display_group.setLayout(layout)

    def set_video_display_connections(self):
        self.vid_display.clicked_camera_pixel.connect(self.add_cal_positions)
        self.vid_display.drawn_camera_pixels.connect(self.handle_drawn_edge)
    

    """
    Initialize injection parameter widgets ---------------------------------------------------------
    """
    def make_injection_parameter_widgets(self):
        ''' Creates widgets for injection parameters and specifies layout '''
        mu = "µ"
        approach_label = QLabel("Approach Distance ("+ mu +"m)")
        depth_label = QLabel("Depth ("+ mu +"m)")
        spacing_label = QLabel("Spacing ("+ mu +"m)")
        speed_label = QLabel(f"Speed ({mu}/s)")
        self.approach_entry = QLineEdit(self)
        self.depth_entry= QLineEdit(self)
        self.spacing_entry = QLineEdit(self)
        self.speed_entry = QLineEdit(self)
        self.pressure_slider = QSlider(Qt.Orientation.Horizontal)
        self.pressure_slider = QSlider(Qt.Orientation.Horizontal)
        self.pressure_slider.setMinimum(10)
        self.pressure_slider.setMaximum(255)
        self.pressure_slider.setTickPosition(QSlider.TickPosition.TicksBelow)
        self.pressure_slider.setTickInterval(30)
        pressure_label = QLabel("Pressure")
        self.pressure_display = QLineEdit(self)
        self.set_values_button = QPushButton("Set Values")
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
        v_layout3.addLayout(v_layout4)
        inj_param_layout = QVBoxLayout()
        inj_param_layout.addLayout(h_layout1)
        inj_param_layout.addLayout(h_layout2)
        inj_param_layout.addLayout(h_layout3)
        inj_param_layout.addLayout(h_layout4)
        inj_param_layout.addLayout(v_layout3)
        self.inj_parameter_group = QGroupBox('Automated Microinjection Controls')
        self.inj_parameter_group.setLayout(inj_param_layout)

    def set_inject_parameter_connections(self):
        """ Sets signal/slot connections for injection parameter widgets """
        self.pressure_slider.valueChanged.connect(self.valuechange)
        self.set_values_button.clicked.connect(self.setautomatedparameters)
    
    def stateify_injection_parameter_widgets(self):
        ''' set initial states for injection parameter widgets'''
        self.approach_entry.insert('100')
        self.depth_entry.insert('20')
        self.spacing_entry.insert('50')
        self.speed_entry.insert('1000')
        self.pressure_slider.setValue(20)


    """
    Initialize injection workflow widgets ---------------------------------------------------------
    """
    def make_workflow_widgets(self):
        ''' Creates widgets for showing status of injection workflow and specifies layout '''       
        angle = QLabel('Set pipette angle')
        self.angle_indicator = QLabel()
        calibration = QLabel('Conduct manipulator calibration')
        self.calibration_indicator = QLabel()
        annotation = QLabel('Annotate injection target')
        self.annotation_indicator = QLabel()
        parameter = QLabel('Set injection parameters')
        self.parameter_indicator = QLabel()
        hline = QHLine()
        self.run_button = QPushButton("Run Trajectory")
        self.stop_button = QPushButton("Stop Process")
        layout = QVBoxLayout()
        form = QFormLayout()
        form.addRow(self.angle_indicator,angle)
        form.addRow(self.calibration_indicator,calibration)
        form.addRow(self.annotation_indicator,annotation)
        form.addRow(self.parameter_indicator,parameter)
        vl = QVBoxLayout()
        vl.addWidget(hline)
        vl.addWidget(self.run_button)
        vl.addWidget(self.stop_button)
        layout.addLayout(form)
        layout.addLayout(vl)
        self.workflow_group = QGroupBox('Injection Workflow')
        self.workflow_group.setLayout(layout)
        # Set all indicators to be incomplete (red) on initialization
        self._set_incomplete(self.angle_indicator)
        self._set_incomplete(self.calibration_indicator)
        self._set_incomplete(self.annotation_indicator)
        self._set_incomplete(self.parameter_indicator)

    def set_workflow_connections(self):
        """ Sets signal/slot connections for workflow widgets """
        self.run_button.clicked.connect(self.run_3D_trajectory)
        self.stop_button.clicked.connect(self.stoptrajectory)
        self._angle_complete.connect(self._set_angle_ind)
        self._calibration_complete.connect(self._set_calibration_ind)
        self._annotation_complete.connect(self._set_annotation_ind)
        self._parameter_complete.connect(self._set_parameter_ind)


    """
    Initialize system status widgets --------------------------------------------------------------
    """
    def make_system_status_widgets(self):
        layout = QVBoxLayout()
        self.response_monitor_window = QTextBrowser()
        self.response_monitor_window.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOn)
        layout.addWidget(self.response_monitor_window)
        self.system_status_group = QGroupBox('System Status')
        self.system_status_group.setLayout(layout)


    """
    Initialize zeiss microscope widgets -----------------------------------------------------------
    """
    def set_zeiss_connections(self):
        self.zen_controller.obj_changed.connect(self.obj_changed)
        self.zen_controller.obj_changed.connect(self.calibration_inputs_changed)
        self.zen_controller.opto_changed.connect(self.calibration_inputs_changed)
        self.zen_controller.opto_changed.connect(self.opto_changed)
        self.zen_controller.ref_changed.connect(self.ref_changed)

    def print_inital_ZEN_states_to_monitor(self):
        # Set magnification of objective
        _, _, mag_level = self.zen_controller.get_current_objective()
        self.response_monitor_window.append(">> Magnification set to " +str(mag_level))
        # Set magnificaiton of optovar
        _, _, opto_mag_level = self.zen_controller.get_current_optovar()
        self.response_monitor_window.append(">> Optovar set to " +str(opto_mag_level))
        # Set name of reflector
        ref_name, _ = self.zen_controller.get_current_reflector()
        self.response_monitor_window.append(">> Reflector set to " +str(ref_name))


    """
    Initialize stacked layout widgets -------------------------------------------------------------
    """
    def make_default_left_page_widget(self):
        """ Define default left page for stacked layout """
        self.default_left_page = QWidget()
        default_left_layout = QVBoxLayout()
        default_left_layout.addWidget(self.pip_cal_group)
        default_left_layout.addWidget(self.default_annotation_group)
        default_left_layout.addWidget(self.display_modification_group)
        default_left_layout.addWidget(self.data_gen_group)
        default_left_layout.addStretch()
        self.default_left_page.setLayout(default_left_layout)
    
    def make_default_right_page_widget(self):
        """ Define default right page for stacked layout """
        self.default_right_page = QWidget()
        default_right_layout = QVBoxLayout()
        default_right_layout.addWidget(self.full_zen_app.zen_group)
        default_right_layout.addWidget(self.inj_parameter_group)
        default_right_layout.addWidget(self.workflow_group)
        default_right_layout.addStretch()
        self.default_right_page.setLayout(default_right_layout)

    def make_annotation_mode_left_page_widget(self):
        """ Define annotation mode left page for stacked layout """
        self.annotation_mode_left_page = QWidget()
        layout = QVBoxLayout()
        layout.addWidget(self.alt_annotation_group)
        layout.addStretch()
        self.annotation_mode_left_page.setLayout(layout)

    def make_annotation_mode_right_page_widget(self):
        """ Define annotation mode left page for stacked layout """
        self.annotation_mode_right_page = QWidget()
        layout = QVBoxLayout()
        layout.addWidget(self.focus_zen_app.zen_group)
        layout.addStretch()
        self.annotation_mode_right_page.setLayout(layout)


    """
    ============================================================================================

    AUXILLIARY GUI FUNCTIONS

    Miscellaneous functions for GUI operation.
    ============================================================================================
    """
    def show_warning_box(self, msg:str):
        """
        Show GUI warning box to user and write warning to logger.
        """
        self.logger.warning(str(msg).replace('\n',' '))
        self.warn_msg.setText(str(msg))
        self.warn_msg.exec()

    def show_error_box(self, msg:str):
        """
        Show GUI error box to user and write error to logger.
        """
        self.logger.error(str(msg).replace('\n',' '))
        self.error_msg.setText(str(msg))
        self.error_msg.exec()

    def show_exception_box(self, msg:str):
        """
        Show GUI error box to user and write exception to logger.
        This includes whole traceback in the logger for more detail.
        """
        self.logger.exception(str(msg).replace('\n',' '))
        self.error_msg.setText(str(msg))
        self.error_msg.exec()
        
    def assess_startup_errors(self):
        #print errors on response monitor if manipulator or arduino has an error
        if self.motorfound == False:
            self.response_monitor_window.append(">> Manipulators not detected. Wait 2 minutes then relaunch the app. If this does not work, replug manipulators into computer.")
        else:
            self.response_monitor_window.append(">> Manipulators detected and working.")
        if self.arduinofound == False:
            self.response_monitor_window.append(">> Arduino not detected, make sure you selected the correct com port, plug in, and try again")
        else:
            self.response_monitor_window.append(">> Arduino connected and working on port " + str(self.com))

    def closeEvent(self, event):
        """ Functions to call when gui is closed (`X` is clicked) """
        close_pressure = injection(arduino,0, 0,0,0,'bp')
        close_pressure.start()
        self.vid_display.stop()
        time.sleep(0.5)
        self.close()


    """
    ============================================================================================

    DATA GENERATION CONTROLS

    Functions to control the saving of images and text/numerical data from GUI operation.
    ============================================================================================
    """
    def acquire_tip_data(self, tip_position):
        '''
        Queries whether checkbox selected to save tip data, and saves data if checked
        '''
        if self.save_tip_annot_rbon.isChecked():
            tip_dict = {'x':tip_position.x(), 'y':tip_position.y()}
            self.save_pip_cal_data(tip_dict)

    def acquire_tissue_data(self, raw_edge_coord, inter_edge_cood):
        '''
        Queries whether checkbox selected to save tissue data, and saves data if checked
        '''
        if self.save_tiss_annot_rbon.isChecked():
            raw = np.asarray(raw_edge_coord)
            inter = np.asarray(inter_edge_coord)
            self.save_tiss_anot_data(raw_annot=raw, interpolate_annot=inter)


    """
    ============================================================================================

    CALIBRATION CONTROLS

    Functions to control the calibration of the manipulator to the camera/microscope axes.
    ============================================================================================
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
            msg = f'Invalid angle: {angle_str}. Angle must be a valid number.'
            self.show_error_box(msg)
        else:
            self.pip_angle = np.deg2rad(float(angle_str))
            self.response_monitor_window.append(f">> Pipette angle set as {angle_str}")
            self._angle_complete.emit(True)
            self.calibration_inputs_changed()

    def save_pipette_angle(self):
        ''' Saves pipette angle to file '''
        if 'pip_angle' not in self.__dict__.keys():
            msg = f"Cannot save pipette angle because the angle is not set."
            self.show_error_box(msg)
        else:
            try:
                self.angle_io.save(pip_angle_rad=self.pip_angle)
            except Exception as e:
                msg = f"Error while saving pipette angle: {e}\n\nSee logs for more information."
                self.show_exception_box(msg)
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
            msg = f"Error: {e}.\n\nYou must save the angle file before it can be loaded."
            self.show_warning_box(msg)
        except Exception as e:
            msg = f"Error while loading pipette angle: {e}\n\nSee logs for more information."
            self.show_exception_box(msg)
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
            msg = f"Automatic calibration not yet implimented. Defaulting to 'Semi-Auto.' calibration mode."
            self.show_warning_box(msg)
        if cal_mode == "Semi-Auto.":
            pass
        if cal_mode == "Manual":
            pass

    def add_cal_positions(self,pixel):
        ''' Adds clicked calibration positions to calibration data '''
        x_click, y_click = pixel
        if self.conduct_calibration_but.isChecked():
            z_scope = self.zen_controller.get_focus_um()
            ex_pos = [x_click, y_click, z_scope]
            dev = SensapexDevice(1)
            man_pos = dev.get_pos()
            self.pip_cal.data.add_cal_position(ex=ex_pos, man=man_pos)
            self.cal_pos_added.emit()
            if self.save_tip_annot_rbon.isChecked():
                tip_dict = {'x':x_click, 'y':y_click}
                self.save_pip_cal_data(tip_dict)
        if self.update_calibration_but.isChecked():
            self.pip_cal.data.rm_all()
            z_scope = self.zen_controller.get_focus_um()
            ex_pos = [x_click, y_click, z_scope]
            dev = SensapexDevice(1)
            man_pos = dev.get_pos()
            self.pip_cal.data.add_cal_position(ex=ex_pos, man=man_pos)
            self.cal_pos_added.emit()

    def conduct_manual_calibration(self):
        ''' Conducts manual calibration process '''
        self.logger.debug('Doing manual calibration')

    def conduct_semi_auto_calibration(self):
        self.logger.debug('Doing Semi-Auto. calibration')
        # Initalize for the calibration trajectory
        dev = SensapexDevice(1)
        img_width = self.cam_MM.width
        img_height = self.cam_MM.height
        z_scope = self.zen_controller.get_focus_um()
        _, _, obj_mag = self.zen_controller.get_current_objective()
        _, _, opto_mag = self.zen_controller.get_current_optovar()
        self.cal_trajectory = SemiAutoCalibrationTrajectory(dev=dev, cal=self.pip_cal, img_w=img_width, img_h=img_height, ex_z=z_scope,z_polarity=-1,pip_angle=self.pip_angle, obj_mag=obj_mag, opto_mag=opto_mag)
        self.cal_pos_added.connect(self.cal_trajectory.next_cal_position)
        self.cal_trajectory.finished.connect(self.conduct_calibration_but.toggle)
        self.cal_trajectory.finished.connect(self.compute_calibration)
        self.cal_trajectory.finished.connect(self.cal_trajectory.deleteLater)
        self.conduct_calibration_but.clicked.connect(self.cal_trajectory.deleteLater)

    def conduct_auto_calibration(self):
        ''' Conducts automatic calibration process '''
        self.conduct_calibration_but.setChecked(False)
        self.cal_mode_box.setCurrentText('Semi-Auto.')
        msg = "Automatic calibration not yet implimented. Defaulting to 'Semi-Auto.' calibration mode."
        self.show_warning_box(msg)

    def handle_calibration_button_clicked(self):
        """
        Handles when the state of the calibration button clicked. When checked, starts
        doing the calibration process. When unchecked, computes the calibration.
        """
        if self.conduct_calibration_but.isChecked() is True:
            self.start_calibration_process()
        else:
            self.compute_calibration()

    def start_calibration_process(self):
        """
        Starts the calibration process if proper conditions are met (update calibration 
        must be unchecked and a calibration mode must be selected).
        """
        # Untoggles calibration button if update calibration is selected
        if self.update_calibration_but.isChecked():
            self.conduct_calibration_but.setChecked(False)
            msg = 'Cannot conduct calibration while updating. Complete update calibration process (and uncheck the box) before conducting a new calibration.'
            self.show_warning_box(msg)
            return None
        # Asks user to if conditions valid for calibrating
        else:
            try:
                cal_mode = self.cal_mode_box.currentText()
                self._ask_to_calibrate(cal_mode=cal_mode)
            except ValueError as e:
                msg = str(e)
                self.show_error_box(msg)
            except Exception as e:
                msg = f"Error while calibrating: {str(e)}.\n\nSee logs for more info."
                self.show_exception_box(msg)

    def _ask_to_calibrate(self, cal_mode:str):
        """ Ask user if conditions for calibration are set before calibrating """
        # Validate calibration mode
        if cal_mode not in ["Automatic", "Manual", "Semi-Auto."]:
            raise ValueError(f'Invalid calibration mode: {cal_mode}.\n\nSelect valid calibration mode before calibrating.')

        # Query user for correct calibraiton positions
        if cal_mode == 'Manual':
            qm = QMessageBox()
            ret = qm.question(self,'Valid calibration conditions?','Confirm the following conditions:\n\n1. Is the pipette tip in-focus?')
            proceed = ret == QMessageBox.StandardButton.Yes
        if cal_mode == 'Semi-Auto.' or cal_mode == 'Automatic':
            qm = QMessageBox()
            ret = qm.question(self,'Valid calibration conditions?','Confirm the following conditions:\n\n1. Is the pipette tip in-focus?\n2. Is the pipette tip near the center of the FOV?\n3. The pipette will move to the edges of the FOV. Are the FOV edges clear of obstructions?')
            proceed = ret == QMessageBox.StandardButton.Yes

        # Conduct calibration 
        if proceed is False:
            self.conduct_calibration_but.setChecked(False)
        elif proceed is True and cal_mode == "Automatic":
            self.conduct_auto_calibration()
        elif proceed is True and cal_mode == 'Manual':
            self.conduct_manual_calibration()
        elif proceed is True and cal_mode == 'Semi-Auto.':
            self.conduct_semi_auto_calibration()

    def compute_calibration(self):
        ''' Computes calibration from calibration data '''
        try:
            if "pip_angle" not in list(self.__dict__.keys()):
                raise CalibrationError(f'Pipette angle not set. The pipette angle must be set before conducting a calibration.')
            _, _, obj_mag = self.zen_controller.get_current_objective()
            _, _, opto_mag = self.zen_controller.get_current_optovar()
            self.logger.info(f"Computing calibration with data\n{self.pip_cal.data.data_df}")
            self.pip_cal.compute(z_polarity=-1, pip_angle=self.pip_angle, obj_mag=obj_mag, opto_mag=opto_mag)
        except CalibrationDataError as e:
            msg = f"Calibration not completed. Error: {e}\n\nMake sure you click on the tip to register at least 3 points (that don't lie on a line) before unchecking 'Calibrate'."
            self.show_warning_box(msg)
        except CalibrationError as e:
            msg = f"Calibration not completed. Error: {e}."
            self.show_error_box(msg)
        except Exception as e:
            msg = f"Error: {e}\n\nSee logs for more information."
            self.show_exception_box(msg)
        else:
            self._calibration_complete.emit(True)
            self.logger.info(f"Existing models:\n{self.pip_cal.tmp_storage._existing_models.keys()}")
        finally:
            self.logger.info('Calibration unchecked. Deleting calibration points.')
            self.pip_cal.data.rm_all()
    

    def update_calibration(self):
        ''' Updates calibration by setting new reference position '''
        if self.update_calibration_but.isChecked():
            # Untoggles calibration button if no calibraiton exists
            if self.pip_cal.model.is_calibrated is False:
                self.update_calibration_but.setChecked(False)
                msg = "System is not calibrated. Can not update non-existent calibration.\n\nConduct a new calibration or load an exsisting calibration before updating."
                self.show_warning_box(msg)
            # Untoggles calibration button if calibraiton button is checked
            if self.conduct_calibration_but.isChecked():
                self.update_calibration_but.setChecked(False)
                msg = 'Cannot update calibration while already conducting calibration.\n\nComplete calibration process (and uncheck the box) before updating a calibration.'
                self.show_warning_box(msg)
        else:
            try:
                _, _, obj_mag = self.zen_controller.get_current_objective()
                _, _, opto_mag = self.zen_controller.get_current_optovar()
                self.pip_cal.update(obj_mag=obj_mag, opto_mag=opto_mag, pip_angle=self.pip_angle)
            except CalibrationDataError as e:
                msg = f"Calibration not updated. Error: {e}\n\nMake sure you click on the tip to register the calibration point before unchecking 'Update'."
                self.show_warning_box(msg)
            except Exception as e:
                msg = f"Error while updating calibration: {e}\n\nSee logs for more information."
                self.show_exception_box(msg)
            finally:
                self.logger.info('Calibration unchecked. Deleting calibration points.')
                self.pip_cal.data.rm_all()

    def save_calibration(self):
        ''' Saves calibration to a file '''
        if self.pip_cal.model.is_calibrated is False:
            msg = "System is not calibrated. Can not save non-existent calibration.\n\nConduct a new calibration or load and update an existing calibration before saving."
            self.show_warning_box(msg)
        else:
            try:
                _, _, obj_mag = self.zen_controller.get_current_objective()
                _, _, opto_mag = self.zen_controller.get_current_optovar()
                self.pip_cal.save_model(obj_mag=obj_mag, opto_mag=opto_mag, ang_deg=np.rad2deg(self.pip_angle))
            except Exception as e:
                msg = f"Error while saving calibration {e}\n\nSee logs for more information."
                self.show_exception_box(msg)

    def load_calibration(self):
        ''' Loads the most recent calibration '''
        # Ask if user wants to overwrite an existing calibration
        if self.pip_cal.model.is_calibrated is True:
            qm = QMessageBox()
            ret = qm.question(self,'Load calibration?','System is already calibrated. Do you still want to load the most recent calibration?')
            if ret == QMessageBox.StandardButton.No:
                return None
        try:
            _, _, obj_mag = self.zen_controller.get_current_objective()
            _, _, opto_mag = self.zen_controller.get_current_optovar()
            self.pip_cal.load_model(obj_mag=obj_mag, opto_mag=opto_mag, ang_deg=np.rad2deg(self.pip_angle))
        except CalibrationFileError as e:
            msg = f"Error: {e}.\n\nYou must save a calibration file before it can be loaded."
            self.show_warning_box(msg)
        except CalibrationDNEError as e:
            msg = f"Error: {e}.\n\nYou must save a calibration with this microscope configuration before it can be loaded."
            self.show_warning_box(msg)
        except Exception as e:
            msg = f"Error while loading calibration: {e}.\n\nSee logs for more information."
            self.show_exception_box(msg)
        else:
            self._calibration_complete.emit(True)        

    def calibration_inputs_changed(self):
        ''' Changes the calibration if the microscope changes'''
        # Set new calibraiton model if it exists or warn user
        _, _, obj_mag = self.zen_controller.get_current_objective()
        _, _, opto_mag = self.zen_controller.get_current_optovar()
        # Use an existing calibration for this config or show warning if was already calibrated and now its not
        if self.pip_cal.model.is_calibrated is True:
            try:
                self.pip_cal.use_existing_model(obj_mag=obj_mag, opto_mag=opto_mag, ang_deg=np.rad2deg(self.pip_angle))
                self._calibration_complete.emit(True)
            except CalibrationDNEError as e:
                self.pip_cal.model.reset_calibration()
                self._calibration_complete.emit(False)
                msg = f"You switched the microscope configuration, but '{e}'.\n\nYour previous calibration is invalid. Either switch to the previous microscope configuration or conduct/load a calibration for this microscope configuration."
                self.show_warning_box(msg)
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
        image = np.copy(self.vid_display.frame)
        pip_data_saver.save_data(image=image, tip_position=tip_dict)


    """
    ============================================================================================

    ANNOTATION CONTROLS

    Functions to control the annotation of injection targets on the GUI display.
    ============================================================================================
    """

    def make_edge_mask(self, edge_df:pd.DataFrame, edge_cc:np.ndarray, edge_type:str):
        """
        Create binary mask image of desired edge.

        Args:
            edge_df (pd.DataFrame): Dataframe containing edge numerical label
                and semantic labels.
            edge_cc (np.ndarray): Connected component labeled edge image.
            edge_type (str): type of mask to create like ['apical', 'basal']
        
        Returns:
            np.ndarray of binary edge mask
        """
        # Validate proper edge type
        if edge_type not in ['apical', 'basal']:
            raise ValueError(f"`edge_type` must be in ['apical','basal']")
        # Extract the numerical edge labels associated with desired semantic edge type
        edge_num_labels = edge_df.loc[edge_df['semantic']==edge_type,'edge'].to_numpy()
        # Boolean binary mask of edges matching the numerical (therefore semantic) label
        bin_mask = np.zeros(edge_cc.shape, dtype=np.uint8)
        for ind, label in enumerate(edge_num_labels):
            bin_mask = np.bitwise_or(bin_mask, edge_cc == label)
        # Dilate the single pixel width edge for increased visibility
        kernel = cv2.getStructuringElement(shape=cv2.MORPH_RECT, ksize=(11, 11))
        bin_mask = cv2.dilate(bin_mask, kernel)
        
        return bin_mask

    def automatic_tissue_annotation(self):
        annot_mode = self.annotation_combo_box.currentText()
        edge_type = self.edge_type_combo_box.currentText().lower()
        if annot_mode == "Automatic":
            try:
                # Make detection of tissue edge
                detection_dict = self.tissue_model.detect(image, edge_type)
                # Extract detection data
                df = detection_dict['detection_data']
                edge_cc = detection_dict['edge_image']
                edge_pixels = detection_dict['edge_coordinates']
                # Construct binary masks of tissue/edge detection for display
                tissue_mask = detection_dict['segmentation_image']>0
                basal_mask = self.make_edge_mask(edge_df=df, edge_cc=edge_cc, edge_type='basal')
                apical_mask = self.make_edge_mask(edge_df=df, edge_cc=edge_cc, edge_type='apical')
                self.vid_display.set_masks(tissue_mask=tissue_mask, apical_mask=apical_mask, basal_mask=basal_mask)
                # Raise error if no edge for desired edge is found
                if edge_pixels is None:
                    raise EdgeNotFoundError(f"Edge not found: {edge_type}.")
                # Set the anntoation for injection
                # self.interpolated_pixels = edge_pixels
                self.handle_drawn_edge(edge_pixels)
            except EdgeNotFoundError as e:
                msg = f"{e}\n\nManually annotate the edge."
                self.show_warning_box(msg)
            except:
                msg = "Error while detecting tissue.\n\nSee logs for more info."
                self.show_exception_box(msg)

    def annotation_combo_changed(self):
        """ Handle what to do when the annotation combobox state is changed """
        self.update_annotation_display_labels()

    def edge_type_combo_changed(self):
        """ Handle what to do when the edge type combobox state is changed """
        self.update_annotation_display_labels()

    def update_annotation_display_labels(self):
        annot_mode = self.annotation_combo_box.currentText()
        self.annotation_mode_display.setText(annot_mode)
        edge_type = self.edge_type_combo_box.currentText()
        self.edge_type_display.setText(edge_type)

    def annotation_complete_pressed(self):
        """ Handle what to do when user clicks annotation complete """
        # Stop showing the raw drawn edge
        self.vid_display.show_drawn_annotation(False)
        self.vid_display.reset_masks()
        # Switch GUI to default mode
        self.switch_to_default_mode()

    def annotate_trajectory_pressed(self):
        """ Handle what to do when user clicks annotate trajectory """
        # Allow video display to show the drawn and interpolated annotations
        self.vid_display.show_drawn_annotation(True)
        self.vid_display.show_interpolated_annotation(True)
        # Update the annotaiton guidance
        self.modify_annotation_guidance()
        # Switch gui to the annotaiton mode
        self.switch_to_annotation_mode()
        self.automatic_tissue_annotation()

    def exit_annotation_mode(self):
        """
        Exit the annotation mode by stop showing the drawn edge and change
        the gui to the default mode 
        """
        # Stop showing the raw drawn edge
        self.vid_display.show_drawn_annotation(False)
        # Nullify any interpolated edge coordinates if they exist
        self.interpolated_pixels = []
        self.vid_display.reset_interpolated_annotation()
        self.vid_display.reset_masks()
        self._annotation_complete.emit(False)
        # Switch GUI to default mode
        self.switch_to_default_mode()

    def switch_to_annotation_mode(self):
        """ Modify GUI to show annotation mode layout """
        self.left_stacked_layout.setCurrentWidget(self.annotation_mode_left_page)
        self.right_stacked_layout.setCurrentWidget(self.annotation_mode_right_page)

    def switch_to_default_mode(self):
        """ Modify GUI to show default application layout """
        self.left_stacked_layout.setCurrentWidget(self.default_left_page)
        self.right_stacked_layout.setCurrentWidget(self.default_right_page)

    def modify_annotation_guidance(self):
        """ Modify the annotaiton guidance text to inform user how to proceed """
        annot_mode = self.annotation_combo_box.currentText()
        annot_notes = ("Notes:\n"
            "1. Multiple annotations will overwrite each other.\n"
            "2. Selecting `Exit` will exit without the annotation.")
        if annot_mode == "Manual":
            msg = ("Process:\n1. In video display, click-and-drag mouse along tissue edge."
            f"\n2. Select `Complete Annotation` to exit to main GUI.\n\n{annot_notes}")
        elif annot_mode == "Automatic":
            msg = ("Process:\n1. Automatic annotation is shown. If satisfied, select `Complete "
            " Annotation`.\n2. If unstisfied, manually annotate by click-and-drag mouse along "
            f"tissue edge.\n3.Select `Complete Annotation` to exit to main GUI.\n\n{annot_notes}")
        else:
            err_msg = f"Invalid annotation mode: {annot_mode}. Must be in ['Manual', 'Auotmatic']"
            self.show_error_box(err_msg)
            raise NotImplementedError(err_msg)
        self.annotation_guidance.setText(msg)
        

    def save_tiss_anot_data(self, raw_annot:np.ndarray, interpolate_annot:np.ndarray):
        '''
        Save image from camera and annotated coordinates of tissue trajectory

        Arguments:
            raw_annot (np.ndarray): nx2 array of drawn annotation coord. nth row = [x_n, y_n]
            interpolate_annot (np.ndarray): nx2 array of interpolated annotation coord. nth row = [x_n, y_n]
        '''
        # Instance of object to save data
        tis_data_saver = TissueEdgeData(tis_data_dir=self.tis_data_dir)
        image = np.copy(self.vid_display.frame)
        tis_data_saver.save_data(image=image, raw_annot=raw_annot, interpolate_annot=interpolate_annot)


    def handle_drawn_edge(self, drawn_pixels:list):
        ''' Handles events when video display returns the drawn edge '''
        # Only handle the drawn edge when the annotaion mode is active
        if self.left_stacked_layout.currentWidget() == self.annotation_mode_left_page:
            # Try to interpolate the drawn edge coordinates
            try:
                self.interpolated_pixels = interpolate(drawn_pixels)
                self.vid_display.set_interpolated_annotation(self.interpolated_pixels)
                self._annotation_complete.emit(True)
            except AnnotationError as e:
                self.interpolated_pixels = []
                self.vid_display.reset_interpolated_annotation()
                msg = "Error while interpolating annotation. The annotation must go in one direction: either top-to-bottom or bottom-to-top.\n\nTry annotating again."
                self.show_warning_box(msg)
                self._annotation_complete.emit(False)
            except Exception as e:
                self.interpolated_pixels = []
                self.vid_display.reset_interpolated_annotation()
                msg = "Error while interpolating annotation.\n\nSee logs for more info."
                self.show_exception_box(msg)
                self._annotation_complete.emit(False)
            else:
                if self.save_tiss_annot_rbon.isChecked():
                    raw = np.asarray(drawn_pixels)
                    inter = np.asarray(self.interpolated_pixels)
                    self.save_tiss_anot_data(raw_annot=raw, interpolate_annot=inter)

    
    """
    ============================================================================================

    CAMERA/MICROSCOPE/ZEISS CONTROLS

    Additional functions to control operation of zeiss microscope and camera
    ============================================================================================
    """
    def obj_changed(self, mag_level:float):
        self.response_monitor_window.append(">> Magnification set to " +str(mag_level))

    def opto_changed(self, mag_level:float):
        self.response_monitor_window.append(">> Optovar set to " +str(mag_level))

    def ref_changed(self, ref_name:str):
        self.response_monitor_window.append(">> Reflector set to " +str(ref_name))

    def exposure_value_change(self):
        new_exposure_value = float(self.exposure_slider.value())/10
        self.cam_MM.set_exposure(new_exposure_value)

    
    """
    ============================================================================================

    DISPLAY MODIFICATION CONTROLS

    Functions for controlling the additional annotations/modificaions shown on video display
    ============================================================================================
    """
    def display_annotation(self):
        """ Tells video display whether to display annotated coordinates """
        if self.display_annotation_rbon.isChecked():
            self.vid_display.show_interpolated_annotation(True)
        else:
            self.vid_display.show_interpolated_annotation(False)

    def display_calibration(self):
        ''' Send computed tip positoin (in camera) to video to be displayed.
        Calls itself on timer to update tip position in video'''
        if self.display_tip_rbon.isChecked():
            if self.pip_disp_timer.isActive() is False:
                self.pip_disp_timer.start(self.pip_disp_timeout)
            if self.pip_cal.model.is_calibrated is True:
                self.vid_display.show_tip_position(True)
                dev = SensapexDevice(1)
                pos = dev.get_pos()
                ex = self.pip_cal.model.forward(man=pos)
                self.vid_display.set_tip_position(ex[0], ex[1])
            else:
                self.vid_display.show_tip_position(False)
        else:
            self.vid_display.show_tip_position(False)
            self.pip_disp_timer.stop()

    def display_tissue_mask(self):
        """ Tells video display whether to display annotated coordinates """
        if self.display_segmentation_rbon.isChecked():
            self.vid_display.show_tissue_mask(True)
        else:
            self.vid_display.show_tissue_mask(False)

    def display_edge_mask(self):
        """ Tells video display whether to display annotated coordinates """
        if self.display_edges_rbon.isChecked():
            self.vid_display.show_edge_mask(True)
        else:
            self.vid_display.show_edge_mask(False)


    """
    ============================================================================================

    INJECTION WORKFLOW CONTROLS

    Functions to show indication of successfully doing GUI workflow and readiness for injection.
    ============================================================================================
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
    ============================================================================================

    INJECTION PARAMETER CONTROLS

    Functions to control setting of injection parameters and running injections.
    ============================================================================================
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
            msg = "Error, did you enter all parameters? Is the arduino plugged in? \nPython error = \n" + str(sys.exc_info()[1])
            self.show_error_box(msg)
            self.response_monitor_window.append(">> Python error = " + str(sys.exc_info()))
        else:
            self._parameter_complete.emit(True)

    def valuechange(self):
        self.pressureslidervalue= self.pressure_slider.value()
        self.displaypressure = int(self.pressureslidervalue/2.55)
        self.pressure_display.setText(str(self.displaypressure)+'%')
        
    def run_3D_trajectory(self):
        if self.conduct_calibration_but.isChecked():
            msg = "Cannot start injections while calibration button is checked. Complete calibration before starting."
            self.show_warning_box(msg)
            return
        if self.update_calibration_but.isChecked():
            msg = "Cannot start injections while update calibration button is checked. Complete calibration update before starting."
            self.show_warning_box(msg)
            return
        try:
            #get values from GUI
            um2nm = 1000
            approach_nm = int(float(self.approachdist)*um2nm)
            depth_nm = int(float(self.depthintissue)*um2nm)
            spacing_nm = int(float(self.stepsize)*um2nm)
            speed_ums = int((self.motorspeed))
            dev = SensapexDevice(1)
            cal = self.pip_cal
            edge = self.interpolated_pixels
            z_scope = self.zen_controller.get_focus_um()
            edge_arr = np.array(edge)
            z_arr = z_scope*np.ones((edge_arr.shape[0],1))
            edge_3D = np.concatenate((edge_arr,z_arr),axis=1).tolist()
            self.inj_trajectory = SurfaceLineTrajectory3D(dev, cal, edge_3D, approach_nm, depth_nm, spacing_nm, speed_ums)
            self.inj_trajectory.start()
            self.inj_trajectory.finished.connect(self.show_n_injected)
        except:
            msg = "Please complete calibration, enter all parameters, and select tip of pipette.\nPython error = \n" + str(sys.exc_info()[1])
            self.show_error_box(msg)
            self.response_monitor_window.append(">> Python error = " + str(sys.exc_info()))

    def show_n_injected(self):
        self.response_monitor_window.append(">> Number of injections =" + str(self.inj_trajectory.n_injected))

    def stoptrajectory(self):
        try:
            self.inj_trajectory.stop()
        except:
            msg = "You have to start the trajectory in order to be able to stop it...\nPython error = \n" + str(sys.exc_info()[1])
            self.show_error_box(msg)
            self.response_monitor_window.append(">> Python error = " + str(sys.exc_info()))

    

if __name__ == "__main__":
    import sys
    app = QApplication(sys.argv)
    app.setApplicationName('MyWindow')
    main = ControlWindow('HamamatsuHam_DCAM', 'HamamatsuHam', 'HamamatsuHam_DCAM', '2x2', 180, 256, 1.3, 'com3')
    main.show()
    sys.exit(app.exec())
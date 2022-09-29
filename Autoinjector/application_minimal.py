
# -*- coding: utf-8 -*-
import time
import os
import sys
from datetime import datetime
from typing import Optional, Union, List, Tuple
import cv2
import serial
import numpy as np
from PyQt6.QtWidgets import *
from PyQt6.QtCore import *
from PyQt6.QtGui import QIcon, QPalette, QColor
import pandas as pd
from src.video_control.video_utils import interpolate, AnnotationError
from src.video_control.annotations import AnnotationManager
from src.video_control.camera import MMCamera
from src.video_control.video import VideoDisplay
from src.motorcontrol.motorlocationThread import motorpositionThread
from src.pythonarduino.injectioncontrolmod import injection
from src.cfg_mgmt.cfg_mngr import CfgManager
from src.GUI_utils.gui_objects import QHLine
import src.GUI_utils.display_modifier_mvc as disp_mod_mvc
from src.miscellaneous.standard_logger import StandardLogger as logr
from src.miscellaneous import validify as val
from src.miscellaneous.error_utils import InjectionParameterError
from src.data_generation.data_generators import PipTipData, TissueEdgeData, ImageStackData
from src.manipulator_control.calibration import Calibrator, AngleIO
from src.manipulator_control.error_utils import (CalibrationError,
                                                CalibrationDataError,
                                                CalibrationDNEError,
                                                CalibrationFileError,
                                                AngleFileError,
                                                TrajectoryError)
from src.manipulator_control.sensapex_utils import SensapexDevice
from src.manipulator_control.injection_trajectory import SurfaceLineTrajectory3D, TrajectoryManager
from src.manipulator_control.calibration_trajectory import SemiAutoCalibrationTrajectory
from src.manipulator_control.convenience_trajectories import ConvenienceTrajectories
from src.ZEN_interface.ZEN_mvc import ModelZEN, ControllerZEN, ViewZENComplete, ViewZENFocus
from src.ZEN_interface.z_stack import ZStackManager, ZStackDataWithAnnotations
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
    leaving_calibration = pyqtSignal()
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
            dev = SensapexDevice(1)
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
        # Initialize attributes
        self.initialize_attributes()

        # Load the configuration values
        self.get_gui_cfg()

        # Define necessary directories
        self.define_dirs()

        # Instantiate imports
        self.pip_cal = Calibrator(cal_data_dir=self.cal_data_dir)
        self.angle_io = AngleIO(ang_data_dir=self.cal_data_dir)
        ckpt_path = "Autoinjector/src/deep_learning/weights/20220824_180000_Colab_gpu/best.pth"
        self.tissue_model = ModelTissueDetection(ckpt_path)
        dev = SensapexDevice(1)
        self.convenience_trajectories = ConvenienceTrajectories(dev=dev)

        # Instantiate the imported camera
        try:
            mm_path = self.cfg.values.micromanager_path.replace('\\','/')
            self.cam_MM = MMCamera(mm_path, self.cam_, self.brand_, self.val_, self.bins_, self.rot_, self.imagevals_)
        except Exception as e:
            msg = f"Error while interfacing with camera: {e}.\n\nSee logs for more info."
            self.show_exception_box(e)

        # Instantiate imported widgets
        self.instantiate_mvcs()
        self.annot_mgr = AnnotationManager()
        self.vid_display = VideoDisplay(self.cam_MM, self.annot_mgr, height=900, fps=50)
        self.zstack = ZStackManager(self.zen_controller)

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

    def initialize_attributes(self):
        """ Initialize constants and variables used in GUI """
        # Booleans of current calibration process
        self.conducting_calibration = False
        self.updating_calibration = False
        # Booleans for injection workflow status
        self._is_angle_set = False
        self._is_calibration_set = False
        self._is_annotation_set = False
        self._is_parameters_set = False

    def get_gui_cfg(self):
        ''' Loads the configuration values for the GUI '''
        self.cfg = CfgManager()
        self.cfg.from_pointer_file()

    def define_dirs(self):
        ''' Set dirs (and create if necessary) for GUI '''
        # General data directory
        self.data_dir = self.cfg.values.data_directory.replace('\\','/')
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
            self.logger.info(f'Created tissue image and annotation data directory: {self.tis_data_dir}')
        # For saving z-stacks
        self.zstack_data_dir = f"{self.data_dir}/zstacks"
        if os.path.isdir(self.zstack_data_dir) is False:
            os.makedirs(self.zstack_data_dir)
            self.logger.info(f'Created Z-Stack image data directory: {self.zstack_data_dir}')
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

    def instantiate_mvcs(self):
        """ Instantiate imported model view controllers """
        # Zeiss zen controller
        zen_model = ModelZEN()
        self.zen_controller = ControllerZEN(zen_model)
        self.full_zen_app = ViewZENComplete(self.zen_controller)
        self.focus_zen_app1 = ViewZENFocus(self.zen_controller)
        self.focus_zen_app2 = ViewZENFocus(self.zen_controller)
        # Display modification mvc
        self.disp_mod_controller = disp_mod_mvc.Controller()
        self.disp_mod_view = disp_mod_mvc.View(self.disp_mod_controller)
        self.disp_mod_view_alt = disp_mod_mvc.View(self.disp_mod_controller)

    def make_widgets(self):
        """ Create the GUI's widgets """
        # Create all interactive widgets
        self.make_pipette_calibrator_widgets()
        self.make_data_generator_widgets()
        self.make_annotation_widgets()
        self.make_display_modification_widgets()
        self.make_video_display_widgets()
        self.make_injection_parameter_widgets()
        self.make_manipulator_control_widgets()
        self.make_workflow_widgets()
        self.make_system_status_widgets()
        # Create pages for the stacked layout with different modes (like calibration
        # mode or anotation mode)
        self.make_default_left_page_widget()
        self.make_default_right_page_widget()
        self.make_annotation_mode_left_page_widget()
        self.make_annotation_mode_right_page_widget()
        self.make_calibration_mode_left_page_widget()
        self.make_calibration_mode_right_page_widget()

    def set_connections(self):
        """ Set the signal/slot connections for the GUI's widgets """
        self.set_pipette_calibrator_connections()
        self.set_annotation_connections()
        self.set_display_modification_connections()
        self.set_video_display_connections()
        self.set_inject_parameter_connections()
        self.set_manipulator_control_connections()
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
        self.left_stacked_layout.addWidget(self.calibration_mode_left_page)
        self.right_stacked_layout.addWidget(self.default_right_page)
        self.right_stacked_layout.addWidget(self.annotation_mode_right_page)
        self.right_stacked_layout.addWidget(self.calibration_mode_right_page)

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
        self.conduct_calibration_but = QPushButton("Conduct Calibration")
        self.update_calibration_but = QPushButton("Update Calibration")
        self.save_calibration_but = QPushButton("Save Calibration", self)
        self.load_calibration_but = QPushButton("Load Calibration", self)
        form = QFormLayout()
        form.addRow(cal_mode_label,self.cal_mode_box)
        h_layout2 = QHBoxLayout()
        h_layout2.addWidget(self.save_calibration_but)
        h_layout2.addWidget(self.load_calibration_but)
        # Groupbox and master layout
        pip_cal_layout = QVBoxLayout()
        pip_cal_layout.addLayout(grid_layout1)
        pip_cal_layout.addLayout(h_layout_angio)
        pip_cal_layout.addWidget(h_sep1)
        pip_cal_layout.addLayout(form)
        pip_cal_layout.addWidget(self.conduct_calibration_but)
        pip_cal_layout.addWidget(self.update_calibration_but)
        pip_cal_layout.addLayout(h_layout2)
        self.pip_cal_group = QGroupBox('Pipette Calibration')
        self.pip_cal_group.setLayout(pip_cal_layout)

        # alternate calibration 
        cal_mode_label2 = QLabel("Calibration Mode:")
        self.cal_mode_display = QLabel("")
        cal_type_label = QLabel("Calibration Type:")
        self.cal_type_display = QLabel("")
        self.complete_calibration_but = QPushButton("Complete Calibration")
        self.save_complete_calibration_but = QPushButton("Save and Complete Calibration")
        self.exit_calibration_but = QPushButton("Exit")
        h_sep2 = QHLine()
        self.calibration_guidance = QLabel("")
        self.calibration_guidance.setWordWrap(True)
        alt_form = QFormLayout()
        alt_form.addRow(cal_mode_label2,self.cal_mode_display)
        alt_form.addRow(cal_type_label,self.cal_type_display)
        alt_layout = QVBoxLayout()
        alt_layout.addLayout(alt_form)
        alt_layout.addWidget(self.complete_calibration_but)
        alt_layout.addWidget(self.save_complete_calibration_but)
        alt_layout.addWidget(self.exit_calibration_but)
        alt_layout.addWidget(h_sep2)
        alt_layout.addWidget(self.calibration_guidance)
        self.alt_pip_cal_group = QGroupBox('Pipette Calibration')
        self.alt_pip_cal_group.setLayout(alt_layout)
    
    def set_pipette_calibrator_connections(self):
        """ Set the signal/slot connections for the pipette calibrator widgets """
        self.conduct_calibration_but.clicked.connect(self.conduct_calibration_pressed)
        self.update_calibration_but.clicked.connect(self.update_calibration_pressed)
        self.save_calibration_but.clicked.connect(self.save_calibration)
        self.load_calibration_but.clicked.connect(self.load_calibration)
        self.angle_mode_box.currentTextChanged.connect(self.set_angle_mode)
        self.cal_mode_box.currentTextChanged.connect(self.set_cal_mode)
        self.set_angle_button.clicked.connect(self.set_pipette_angle)
        self.save_angle_button.clicked.connect(self.save_pipette_angle)
        self.load_angle_button.clicked.connect(self.load_pipette_angle)
        # Alt connections
        self.complete_calibration_but.clicked.connect(self.complete_calibration_pressed)
        self.save_complete_calibration_but.clicked.connect(self.save_complete_calibration_pressed)
        self.exit_calibration_but.clicked.connect(self.exit_calibration_pressed)

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
        self.annotation_button = QPushButton("Annotate Target")
        self.rm_annotation_button = QPushButton("Remove All Annotations")
        self.default_annotation_group = QGroupBox("Trajectory Annotation")
        # Make alternate annotation widgets
        mode_label1 = QLabel('Annotation Mode:')
        self.annotation_combo_box = QComboBox()
        self.annotation_combo_box.setPlaceholderText('Mode')
        self.complete_annotation_button = QPushButton("Complete Annotation")
        self.exit_annotation_button = QPushButton("Exit Annotation")
        hline = QHLine()
        guidance_label = QLabel('Annotation Guidance')
        self.annotation_guidance = QLabel('')
        self.annotation_guidance.setWordWrap(True)
        self.alt_annotation_group = QGroupBox("Annotation Control")
        # Make automatic annotation and the z-stack annotation widgets
        edge_label1 = QLabel('Auto-detect Edge:')
        self.edge_type_combo_box = QComboBox()
        self.edge_type_combo_box.setPlaceholderText('Type')
        self.single_auto_annotation_button = QPushButton('Run Single Auto. Annotation')
        zstack_label = QLabel('Z-Stack Annotation')
        self.plane1_button = QPushButton('Set first focus')
        self.plane2_button = QPushButton('Set last focus')
        palette = QPalette()
        palette.setColor(QPalette.ColorRole.Base, QColor('lightGray'))
        self.plane1_entry = QLineEdit()
        self.plane1_entry.setAlignment(Qt.AlignmentFlag.AlignRight)
        self.plane1_entry.setReadOnly(True)
        self.plane1_entry.setPalette(palette)
        self.plane2_entry = QLineEdit()
        self.plane2_entry.setAlignment(Qt.AlignmentFlag.AlignRight)
        self.plane2_entry.setReadOnly(True)
        self.plane2_entry.setPalette(palette)
        num_slices_label = QLabel("Number of slices")
        self.slices_entry = QLineEdit()
        self.slices_entry.setAlignment(Qt.AlignmentFlag.AlignRight)
        self.zstack_button = QPushButton('Run Z-Stack Auto. Annotation')
        self.zstack_stop_button = QPushButton('Stop Z-Stack')
        progress_label = QLabel("Z-Stack Progress:")
        self.zstack_progress = QProgressBar()
        zstack_status_label = QLabel("Z-Stack Results:")
        self.zstack_status = QLabel('Not Applicable')
        self.auto_annotation_group = QGroupBox("Automatic Annotation Control")
        # Specify the default annotation layout and group
        default_layout = QVBoxLayout()
        default_layout.addWidget(self.annotation_button)
        default_layout.addWidget(self.rm_annotation_button)
        self.default_annotation_group.setLayout(default_layout)
        # Specify alt annotation layout and group
        alt_layout = QVBoxLayout()
        form1 = QFormLayout()
        form1.addRow(mode_label1, self.annotation_combo_box)
        alt_layout.addLayout(form1)
        alt_layout.addWidget(self.complete_annotation_button)
        alt_layout.addWidget(self.exit_annotation_button)
        alt_layout.addWidget(hline)
        alt_layout.addWidget(guidance_label)
        alt_layout.addWidget(self.annotation_guidance)
        self.alt_annotation_group.setLayout(alt_layout)
        # Specify z stack layout and group
        auto_annot_layout = QVBoxLayout()
        form2 = QFormLayout()
        form2.addRow(edge_label1, self.edge_type_combo_box)
        form3 = QFormLayout()
        form3.addRow(self.plane1_button, self.plane1_entry)
        form3.addRow(self.plane2_button, self.plane2_entry)
        form3.addRow(num_slices_label, self.slices_entry)
        form4 = QFormLayout()
        form4.addRow(progress_label, self.zstack_progress)
        form4.addRow(zstack_status_label, self.zstack_status)
        auto_annot_layout.addLayout(form2)
        auto_annot_layout.addWidget(QHLine())
        auto_annot_layout.addWidget(self.single_auto_annotation_button)
        auto_annot_layout.addWidget(QHLine())
        auto_annot_layout.addWidget(zstack_label)
        auto_annot_layout.addLayout(form3)
        auto_annot_layout.addWidget(self.zstack_button)
        auto_annot_layout.addWidget(self.zstack_stop_button)
        auto_annot_layout.addLayout(form4)
        self.auto_annotation_group.setLayout(auto_annot_layout)

    def set_annotation_connections(self):
        """ Set signal/slot connections for annotation widgets """
        self.annotation_button.clicked.connect(self.annotate_trajectory_pressed)
        self.rm_annotation_button.clicked.connect(self.rm_annotation_pressed)
        self.complete_annotation_button.clicked.connect(self.annotation_complete_pressed)
        self.exit_annotation_button.clicked.connect(self.annotation_exit_pressed)
        self.annotation_combo_box.currentTextChanged.connect(self.annotation_combo_changed)
        self.single_auto_annotation_button.clicked.connect(self.single_automatic_annotation)
        self.plane1_button.clicked.connect(self.set_zstack_plane1)
        self.plane2_button.clicked.connect(self.set_zstack_plane2)
        self.zstack_button.clicked.connect(self.run_zstack_annotation)
        self.zstack_stop_button.clicked.connect(self.zstack.stop)
        self.zstack.ask_for_data.connect(self.send_zstack_data)
        self.zstack.progress.connect(lambda percent: self.zstack_progress.setValue(int(percent)))
        self.zstack.errors.connect(self.show_recieved_exception)
        self.zstack.finished.connect(self.process_zstack_data)

    def stateify_annotation_widgets(self):
        self.annotation_combo_box.insertItems(0,['Manual','Automatic'])
        self.annotation_combo_box.setCurrentText('Automatic')
        self.edge_type_combo_box.insertItems(0,['Apical','Basal'])
        self.edge_type_combo_box.setCurrentText('Apical')

    """
    Initialize video display modification widgets -------------------------------------------------
    """
    def make_display_modification_widgets(self):
        """ Make widgets to modify the video display and specifies widget layout """
        # Make default widgets
        exposure_label = QLabel("Camera Exposure")
        self.exposure_slider = QSlider(Qt.Orientation.Horizontal)
        self.exposure_slider.setMinimum(15)
        self.exposure_slider.setMaximum(100)
        self.exposure_slider.setTickPosition(QSlider.TickPosition.TicksBelow)
        self.exposure_slider.setTickInterval(0)
        display_tip_label = QLabel('Show tip:')
        display_annotation_label = QLabel('Show annotation:')
        display_segmentation_label = QLabel('Show tissue:')
        display_edge_label = QLabel('Show edge:')
        # default layout
        layout = QVBoxLayout()
        hl1 = QHBoxLayout()
        hl1.addWidget(self.disp_mod_view.display_calibration_rbon)
        hl1.addWidget(self.disp_mod_view.display_calibration_rboff)
        hl2 = QHBoxLayout()
        hl2.addWidget(self.disp_mod_view.display_annotation_rbon)
        hl2.addWidget(self.disp_mod_view.display_annotation_rboff)
        hl3 = QHBoxLayout()
        hl3.addWidget(self.disp_mod_view.display_segmentation_rbon)
        hl3.addWidget(self.disp_mod_view.display_segmentation_rboff)
        hl4 = QHBoxLayout()
        hl4.addWidget(self.disp_mod_view.display_edges_rbon)
        hl4.addWidget(self.disp_mod_view.display_edges_rboff)
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
        # Make alt widgets
        display_tip_label_alt = QLabel('Show tip:')
        display_annotation_label_alt = QLabel('Show annotation:')
        display_segmentation_label_alt = QLabel('Show tissue:')
        display_edge_label_alt = QLabel('Show edge:')
        # default layout
        layout_alt = QVBoxLayout()
        hl1_alt = QHBoxLayout()
        hl1_alt.addWidget(self.disp_mod_view_alt.display_calibration_rbon)
        hl1_alt.addWidget(self.disp_mod_view_alt.display_calibration_rboff)
        hl2_alt = QHBoxLayout()
        hl2_alt.addWidget(self.disp_mod_view_alt.display_annotation_rbon)
        hl2_alt.addWidget(self.disp_mod_view_alt.display_annotation_rboff)
        hl3_alt = QHBoxLayout()
        hl3_alt.addWidget(self.disp_mod_view_alt.display_segmentation_rbon)
        hl3_alt.addWidget(self.disp_mod_view_alt.display_segmentation_rboff)
        hl4_alt = QHBoxLayout()
        hl4_alt.addWidget(self.disp_mod_view_alt.display_edges_rbon)
        hl4_alt.addWidget(self.disp_mod_view_alt.display_edges_rboff)
        form_alt = QFormLayout()
        form_alt.addRow(display_tip_label_alt, hl1_alt)
        form_alt.addRow(display_annotation_label_alt, hl2_alt)
        form_alt.addRow(display_segmentation_label_alt, hl3_alt)
        form_alt.addRow(display_edge_label_alt, hl4_alt)
        layout_alt.addLayout(form_alt)
        self.display_modification_group_alt = QGroupBox('Display Settings')
        self.display_modification_group_alt.setLayout(layout_alt)
        
        
    def stateify_display_modification_widgets(self):
        """ Sets initial states for dispaly modification widgets """
        # Default widget connections
        exposure = int(float(self.cam_MM.get_exposure())*10)
        self.exposure_slider.setValue(exposure)
        self.disp_mod_view.stateify_widgets()
        self.disp_mod_view_alt.stateify_widgets()

    def set_display_modification_connections(self):
        """ Sets signal slot connections for display modification widgets """
        self.exposure_slider.valueChanged.connect(self.exposure_value_change)
        self.disp_mod_controller.display_calibration_toggled.connect(self.display_calibration)
        self.disp_mod_controller.display_annotation_toggled.connect(self.display_annotation)
        self.disp_mod_controller.display_segmentation_toggled.connect(self.display_tissue_mask)
        self.disp_mod_controller.display_edges_toggled.connect(self.display_edge_mask)
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
        self.vid_display.drawn_edge_camera_pixels.connect(self.handle_drawn_edge)
        self.vid_display.ask_for_focus_z.connect(self.send_focus_z_to_display)
        self.annot_mgr.annotation_changed.connect(self.set_trajectory)
        self.annot_mgr.new_annotation.connect(self.acquire_tissue_data)
    

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
    Initialize manipulator widgets ---------------------------------------------------------
    """
    def make_manipulator_control_widgets(self):
        ''' Creates widgets for controlling manipulator '''
        self.unload_button = QPushButton("Go to 'unload'")
        self.displace_button = QPushButton("Go to 'right'")
        self.undo_button = QPushButton("Undo previous")
        # self.center_button = QPushButton("To center")
        layout = QGridLayout()
        layout.addWidget(self.unload_button,0,0,1,1)
        layout.addWidget(self.displace_button,0,1,1,1)
        # layout.addWidget(self.center_button,1,1,1,1)
        layout.addWidget(self.undo_button,1,0,1,2)
        self.manipulator_group = QGroupBox('Manipulator Control')
        self.manipulator_group.setLayout(layout)

    def set_manipulator_control_connections(self):
        '''Sets signal/slot connnections for manipulator control widgets '''
        self.unload_button.clicked.connect(self.moveto_unload_position)
        self.displace_button.clicked.connect(self.moveto_displace_position)
        # self.center_button.clicked.connect(self.moveto_center_position)
        self.undo_button.clicked.connect(self.moveto_undo_position)

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
        default_right_layout.addWidget(self.manipulator_group)
        default_right_layout.addWidget(self.inj_parameter_group)
        default_right_layout.addWidget(self.workflow_group)
        default_right_layout.addStretch()
        self.default_right_page.setLayout(default_right_layout)

    def make_annotation_mode_left_page_widget(self):
        """ Define annotation mode left page for stacked layout """
        self.annotation_mode_left_page = QWidget()
        layout = QVBoxLayout()
        layout.addWidget(self.alt_annotation_group)
        layout.addWidget(self.display_modification_group_alt)
        layout.addStretch()
        self.annotation_mode_left_page.setLayout(layout)

    def make_annotation_mode_right_page_widget(self):
        """ Define annotation mode left page for stacked layout """
        self.annotation_mode_right_page = QWidget()
        layout = QVBoxLayout()
        layout.addWidget(self.focus_zen_app1.zen_group)
        layout.addWidget(self.auto_annotation_group)
        layout.addStretch()
        self.annotation_mode_right_page.setLayout(layout)
    
    def make_calibration_mode_left_page_widget(self):
        """ Define calibration mode left page for stacked layout """
        self.calibration_mode_left_page = QWidget()
        layout = QVBoxLayout()
        layout.addWidget(self.alt_pip_cal_group)
        layout.addStretch()
        self.calibration_mode_left_page.setLayout(layout)

    def make_calibration_mode_right_page_widget(self):
        """ Define calibration mode left page for stacked layout """
        self.calibration_mode_right_page = QWidget()
        layout = QVBoxLayout()
        layout.addWidget(self.focus_zen_app2.zen_group)
        layout.addStretch()
        self.calibration_mode_right_page.setLayout(layout)


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

    def show_recieved_exception(self, err:Exception):
        """
        Show exception box for a given error
        """
        self.show_exception_box(err)

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

    def acquire_tissue_data(self):
        '''
        Queries whether checkbox selected to save tissue data, and saves data if checked.
        Data includes a the raw and interpolated coordinates (and a photo)
        '''
        if self.save_tiss_annot_rbon.isChecked():
            all_raw_xy = self.annot_mgr.get_annotations(type_='raw',coords='xy')
            all_inter_xy = self.annot_mgr.get_annotations(type_='interpolated',coords='xy')
            # Only return the most recent ones that were added
            raw = all_raw_xy[-1]
            inter = all_inter_xy[-1]
            raw = np.asarray(raw)
            inter = np.asarray(inter)
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
        if self.conducting_calibration is True:
            z_scope = self.zen_controller.get_focus_um()
            ex_pos = [x_click, y_click, z_scope]
            dev = SensapexDevice(1)
            man_pos = dev.get_pos()
            self.pip_cal.data.add_cal_position(ex=ex_pos, man=man_pos)
            self.cal_pos_added.emit()
            if self.save_tip_annot_rbon.isChecked():
                tip_dict = {'x':x_click, 'y':y_click}
                self.save_pip_cal_data(tip_dict)

        if self.updating_calibration is True:
            self.pip_cal.data.rm_all()
            z_scope = self.zen_controller.get_focus_um()
            ex_pos = [x_click, y_click, z_scope]
            dev = SensapexDevice(1)
            man_pos = dev.get_pos()
            self.pip_cal.data.add_cal_position(ex=ex_pos, man=man_pos)
            self.cal_pos_added.emit()

    def conduct_calibration_pressed(self):
        self.conducting_calibration = True
        # Update display labels
        cal_mode = self.cal_mode_box.currentText()
        self.cal_mode_display.setText(cal_mode)
        self.cal_type_display.setText("New")
        self.modify_calibration_guidance()
        # Switch to the calibraiton mode pane
        self.switch_to_calibration_mode()
        # Start the calibratino
        self.start_calibration_process()

    def update_calibration_pressed(self):
        # Doesn't permit calibrtion if non existent calibration
        if self.pip_cal.model.is_calibrated is False:
            msg = "System is not calibrated. Can not update non-existent calibration.\n\nConduct a new calibration or load an exsisting calibration before updating."
            self.show_warning_box(msg)
        else:
            self.updating_calibration = True
            # Update display labels
            self.cal_mode_display.setText("NA")
            self.cal_type_display.setText("Update")
            self.modify_calibration_guidance()
            # Switch to the calibraiton mode pane
            self.switch_to_calibration_mode()
            
    def complete_calibration_pressed(self):
        # Compute the calibration
        if self.conducting_calibration:
            self.compute_calibration()
        elif self.updating_calibration:
            self.update_calibration()
        # Switch to the default GUI
        self.leave_calibration_mode()

    def exit_calibration_pressed(self):
        # Switch to the default GUI
        self.leave_calibration_mode()

    def save_complete_calibration_pressed(self):
        # Compute the calibration
        if self.conducting_calibration:
            self.compute_calibration()
        elif self.updating_calibration:
            self.update_calibration()
        # Save the calibration
        self.save_calibration()
        # Switch to the default GUI
        self.leave_calibration_mode()

    def switch_to_calibration_mode(self):
        self.left_stacked_layout.setCurrentWidget(self.calibration_mode_left_page)
        self.right_stacked_layout.setCurrentWidget(self.calibration_mode_right_page)

    def leave_calibration_mode(self):
        self.leaving_calibration.emit()
        self.conducting_calibration = False
        self.updating_calibration = False
        self.switch_to_default_mode()

    def modify_calibration_guidance(self):
        cal_mode = self.cal_mode_box.currentText()
        cal_notes = ("Notes:\n"
            "1. 'Exit' will return to the main screen without computing the calibration")
        if self.updating_calibration is True:
            msg = ("Process:\n1. Using the focus knob/widgets or manipulator wheels, bring tip "
            "into focus\n2. Click on the in-focus tip to register the manipulator position. "
            "(Multiple clicks will overwrite the last click.)\n3. 'Complete Calibration' or "
            "'Save and Complete Calibration' to finish and return to the main screen.")
        else:
            if cal_mode.lower() == 'manual':
                msg = ("Process:\n1. Using the focus knob/widgets or manipulator wheels, bring "
                "tip into focus\n2. Click ONCE on the in-focus tip to register the manipulator "
                "position. (Do not click multiple times in the same spot.)\n3. Use the "
                "manipulator wheels to move the tip to a new position in the field-of-view.\n"
                "4. Repeat step 2-3 until you've registered at least 3 calibration positions. "
                "(The 3+ positions must not be co-linear.)\n5. 'Complete Calibration' or "
                "'Save and Complete Calibration' to finish and return to the main screen.")
            elif cal_mode.lower() == 'semi-auto.':
                msg = ("Process:\n1. Ensure the field-of-view is free of obstructions. (The "
                "pipette will automatically move to the edges of the field-of-view.)\n2. Using "
                "the focus knob/widgets or manipulator wheels, bring tip into focus near center.\n"
                "2. Click ONCE on the in-focus tip to register the manipulator position. (Do not "
                "click multiple times in the same spot.)\n3. The pipette will automatically move "
                "to a new position. Click ONCE on the tip after it stops moving.\n4. Repeat step "
                "3 as the pipette automatically moves to the four corners before returning to the "
                "center.\n5. 'Complete Calibration' or 'Save and Complete Calibration' to finish "
                "and return to the main screen.")
            elif cal_mode.lower() == 'automatic':
                msg = ("Prcess:\nNot implimented.")
        msg = f"{msg}\n\n{cal_notes}"
        self.calibration_guidance.setText(msg)


    def conduct_manual_calibration(self):
        ''' Conducts manual calibration process '''
        self.logger.debug('Doing manual calibration')

    def conduct_semi_auto_calibration(self):
        self.logger.debug('Doing Semi-Auto. calibration')
        # Initalize for the calibration trajectory
        dev = SensapexDevice(1)
        img_width = self.cam_MM.width
        img_height = self.cam_MM.height
        _, _, obj_mag = self.zen_controller.get_current_objective()
        _, _, opto_mag = self.zen_controller.get_current_optovar()
        z_polarity = self.cfg.values.z_polarity
        self.cal_trajectory = SemiAutoCalibrationTrajectory(dev=dev, cal=self.pip_cal, img_w=img_width, img_h=img_height, z_polarity=z_polarity,pip_angle=self.pip_angle, obj_mag=obj_mag, opto_mag=opto_mag)
        self.cal_pos_added.connect(self.cal_trajectory.next_cal_position)
        # delete calibraiton trajecotory otherwise it will conintue to handle stuff from
        # its connections even after it is finished
        self.cal_trajectory.finished.connect(self.cal_trajectory.deleteLater)
        self.leaving_calibration.connect(self.cal_trajectory.deleteLater)

    def conduct_auto_calibration(self):
        ''' Conducts automatic calibration process '''
        self.cal_mode_box.setCurrentText('Semi-Auto.')
        msg = "Automatic calibration not yet implimented. Defaulting to 'Semi-Auto.' calibration mode."
        self.show_warning_box(msg)


    def start_calibration_process(self):
        """
        Starts the calibration process if proper conditions are met (update calibration 
        must be unchecked and a calibration mode must be selected).
        """
        # Asks user to if conditions valid for calibrating
        try:
            cal_mode = self.cal_mode_box.currentText()
            if cal_mode == "Automatic":
                self.conduct_auto_calibration()
            elif cal_mode == 'Manual':
                self.conduct_manual_calibration()
            elif cal_mode == 'Semi-Auto.':
                self.conduct_semi_auto_calibration()
        except ValueError as e:
            msg = str(e)
            self.show_error_box(msg)
        except Exception as e:
            msg = f"Error while calibrating: {str(e)}.\n\nSee logs for more info."
            self.show_exception_box(msg)


    def compute_calibration(self):
        ''' Computes calibration from calibration data '''
        try:
            if "pip_angle" not in list(self.__dict__.keys()):
                raise CalibrationError(f'Pipette angle not set. The pipette angle must be set before conducting a calibration.')
            _, _, obj_mag = self.zen_controller.get_current_objective()
            _, _, opto_mag = self.zen_controller.get_current_optovar()
            self.logger.info(f"Computing calibration with data\n{self.pip_cal.data.data_df}")
            z_polarity = self.cfg.values.z_polarity
            self.pip_cal.compute(z_polarity=z_polarity, pip_angle=self.pip_angle, obj_mag=obj_mag, opto_mag=opto_mag)
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
        image = self.vid_display.get_frame()
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

    def automatic_tissue_annotation(self, image:np.ndarray, edge_type:str, z_scope: Optional[float]=None)->list[list[float, float, float]]:
        """
        Returns list of tissue edge coordinates as [[x, y, z], ...] as detected
        from tissue segmentation and edge classification.

        !! IMPORTANT: This should be encapsulated in try except because it will
        raise errors when it cannot detect edges or reachable edges !!
        
        Raises:
            EdgeNotFoundError: Occurs when it cannot detect an edge (or a valid edge)

        Args:
            image (np.ndarray): image to annotate
            edge_type (str): Type of edge to annotate like ['apical' 'basal']
            z_scope (float): Focus height for the image (if not passed then this
            function measures the height when it is called.)
        Returns:
            list of tissue edge coordinates as [[x, y, z], ...] 
        """
        # Make detection of tissue edge
        detection_dict = self.tissue_model.detect(image, edge_type)
        # Extract detection data
        df = detection_dict['detection_data']
        edge_cc = detection_dict['edge_image']
        # Construct binary masks of tissue/edge detection for display
        tissue_mask = detection_dict['segmentation_image']>0
        basal_mask = self.make_edge_mask(edge_df=df, edge_cc=edge_cc, edge_type='basal')
        apical_mask = self.make_edge_mask(edge_df=df, edge_cc=edge_cc, edge_type='apical')
        # Show the masks in the video display and add the edge to the annotations
        # SHould be done before processing the edge which can raise error if the correct edge
        # is not found. This way it will show the detection and the user can understand why
        # the edge wasn't found
        self.vid_display.set_masks(tissue_mask=tissue_mask, apical_mask=apical_mask, basal_mask=basal_mask)
        self.vid_display.display_masks()
        # Return the pixels of the detected edge
        if self.pip_cal.model.is_calibrated is True:
            # Because image coordinate system has y axis in opposite direction of
            # right hand coordinate system, negate the angle of the pipette rotation to
            # compute the angel of the pipette realtive to a RH coordinate system
            ang_RH = -self.pip_cal.model.compute_pipette_rotation_deg()
            edge_pixels = self.tissue_model.longest_reachable_edge_of_type(detection_dict=detection_dict, edge_type=edge_type, pip_orient_RH=ang_RH)
        else:
            edge_pixels = self.tissue_model.longest_edge_of_type(detection_dict=detection_dict, edge_type=edge_type)
        # Add annotation as 3d list of [[x1, y1, z], [x2, y2, z],...]
        if z_scope is None:
            z_scope = self.zen_controller.get_focus_um()
        annotated_3d_edge = [[ele for ele in coord] + [z_scope] for coord in edge_pixels]
        return annotated_3d_edge

    def single_automatic_annotation(self):
        """
        Attempts a tissue detection, set detected masks in video display, and
        adds the detected annotation coordinates to the annotations to be
        targeted.

        For performing a auto tissue annotation during a z-stack. Errors during the
        tissue detection are suppressed (from issues like not being able to detect
        an edge) to allow the zstack to finish without user intervention of closing
        error boxes. 

        Returns:
            image (np.ndarray): Raw image that was used for detection
            annotated_3d_edge (list): List of detected edge coords as [[x,y,z],...]
        """
        edge_type = self.edge_type_combo_box.currentText().lower()
        image = self.vid_display.get_frame()
        # image = np.copy(self.vid_display.test_img)
        try:
            # Make detection of tissue edge
            annotated_3d_edge = self.automatic_tissue_annotation(image=image, edge_type=edge_type, z_scope=None)
        except EdgeNotFoundError as e:
            # ONly show error boxes when not zstack. During zstack pass errors to log.
            msg = f"{e}\n\nManually annotate the edge."
            self.show_warning_box(msg)
        except:
            msg = "Error while detecting tissue.\n\nSee logs for more info."
            self.show_exception_box(msg)
        else:
            # Add the detected edge coordiantes to the list of targets to be injected
            # (which will emit a signal to set a new injection trajectory)
            self.annot_mgr.add_annotation(annotated_3d_edge)

    def zstack_automatic_annotation(self)->Tuple[np.ndarray, Union[list[list[float,float,float]],None]]:
        """
        Attempts a tissue detection, set detected masks in video display, and
        returns the image and annotated coordinates. This image/annotation can
        be passed to the z-stack `set_stack_data` method.

        For performing a auto tissue annotation during a z-stack. Errors during the
        tissue detection are suppressed (from issues like not being able to detect
        an edge) to allow the zstack to finish without user intervention of closing
        error boxes. 

        Returns:
            image (np.ndarray): Raw image that was used for detection
            annotated_3d_edge (list): List of detected edge coords as [[x,y,z],...]
        """
        edge_type = self.edge_type_combo_box.currentText().lower()
        image = self.vid_display.get_frame()
        # image = np.copy(self.vid_display.test_img)
        try:
            annotated_3d_edge = self.automatic_tissue_annotation(image=image, edge_type=edge_type, z_scope=None)
        except EdgeNotFoundError as e:
            # Dont show error boxes (like when it can't detect the edge) during the z-stack
            # so the zstack can complete. Send error to log
            self.response_monitor_window.append(f">> {e}")
            msg = f"Supressed during z-stack: {e}"
            self.logger.warning(msg)
            # Send a None annotaiton to the z-stack
            annotated_3d_edge = None
        except:
            # Dont show error boxes (like when it can't detect the edge) during the z-stack
            # so the zstack can complete. Send error to log
            self.response_monitor_window.append(f">> Error during z-stack. See logs for more info.")
            msg = "Error suppresed during z-stack"
            self.logger.exception(msg)
            # Send a None annotaiton to the z-stack
            annotated_3d_edge = None

        return image, annotated_3d_edge

    def annotation_combo_changed(self, annot_mode:str):
        """
        Handle what to do when the annotation combobox state is changed
        
        Args:
            annot_mode (str): Selected text in combo box """
        # Enable or disable ability to conduct automatic annotation
        if annot_mode == 'Automatic':
            self.auto_annotation_group.setEnabled(True)
        else:
            self.auto_annotation_group.setEnabled(False)
        # Update the annotaiton guidance
        self.modify_annotation_guidance()

    def annotation_complete_pressed(self):
        """ Handle what to do when user clicks annotation complete """
        # Leave annotation and switch to default mode
        self.leave_annotation_mode()

    def annotate_trajectory_pressed(self):
        """ Handle what to do when user clicks annotate trajectory """
        # Start fresh with new annotation to remove any existing ones
        self.clear_annotation()
        # Allow video display to show the drawn and interpolated annotations
        self.vid_display.show_drawn_annotation(True)
        self.vid_display.show_interpolated_annotation(True)
        # Switch gui to the annotaiton mode
        self.switch_to_annotation_mode()

    def rm_annotation_pressed(self):
        """ Handle what to do when user clicks remove annotation """
        self.clear_annotation()

    def annotation_exit_pressed(self):
        """ Handle what to do when user clicks exit trajectory"""
        # Nullify any interpolated edge coordinates if they exist
        self.clear_annotation()
        # Leave annotation and switch to default mode
        self.leave_annotation_mode()

    def leave_annotation_mode(self):
        """
        Leave the annotation mode by stop showing the drawn edge and change
        gui do the default mode.
        """
        # Disable tissue annotations
        self.vid_display.enable_annotations(False)
        # Stop showing the raw drawn edge
        self.vid_display.show_drawn_annotation(False)
        # Stop showing the automated edge detected masks
        self.vid_display.reset_masks()
        self.vid_display.hide_masks()
        # Switch GUI to default mode
        self.switch_to_default_mode()

    def switch_to_annotation_mode(self):
        """ Modify GUI to show annotation mode layout """
        self.left_stacked_layout.setCurrentWidget(self.annotation_mode_left_page)
        self.right_stacked_layout.setCurrentWidget(self.annotation_mode_right_page)
        self.vid_display.enable_annotations(True)
        self.zstack_status.setText("Not Applicable")
        self.zstack_progress.setValue(0)

    def switch_to_default_mode(self):
        """ Modify GUI to show default application layout """
        self.left_stacked_layout.setCurrentWidget(self.default_left_page)
        self.right_stacked_layout.setCurrentWidget(self.default_right_page)

    def modify_annotation_guidance(self):
        """ Modify the annotaiton guidance text to inform user how to proceed """
        annot_mode = self.annotation_combo_box.currentText()
        annot_notes = ("Notes:\n"
            "1. Each annotation attempt is retained, so middle-mouse-button click near an "
            "exisiting annotation will remove it.\n2. Selecting `Exit` will exit without any "
            "annotations.")
        if annot_mode == "Manual":
            msg = ("Process:\n1. In video display, left-click-and-drag mouse along tissue "
            f"edge.\n2. Select `Complete Annotation` to exit to main GUI.\n\n{annot_notes}")
        elif annot_mode == "Automatic":
            msg = ("Process:\n1. Adjust focus to tissue.\n2. Select the edge-type to annotate.\n3. "
            "Either `Run Single Auto. Annotation` or focus to desired z-stack positions and set "
            "the starting and ending heights, enter the number of slices, and `Run Z-Stack Auto. "
            "Annotation`.\n4. Add manual annotations with left-click-and-drag in video display."
            f"\n5. Select `Complete Annotation` to exit to main GUI.\n\n{annot_notes}")
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
        image = self.vid_display.get_frame()
        tis_data_saver.save_data(image=image, raw_annot=raw_annot, interpolate_annot=interpolate_annot)

    def clear_annotation(self):
        """ Remove annotation and show annotation complete as false """
        # Nullify any interpolated edge coordinates if they exist
        self.annot_mgr.reset_annotations()
        self.interpolated_pixels = []
        self.vid_display.reset_interpolated_annotation()
        self._annotation_complete.emit(False)

    def handle_drawn_edge(self, annotated_cam_edge:list):
        """
        Attempt to add the annotation to annotation manager

        Args:
            annotated_cam_edge (list): Pixels of annotated edge in camera
        """
        try:
            # Add annotation as 3d list of [[x1, y1, z], [x2, y2, z],...]
            z_scope = self.zen_controller.get_focus_um()
            annotated_3d_edge = [[ele for ele in coord] + [z_scope] for coord in annotated_cam_edge]
            self.annot_mgr.add_annotation(annotated_3d_edge)
        except AnnotationError as e:
            msg = f"Error while annotating: {e}"
            self.show_error_box(msg)
        except:
            msg = "Error while annotating target.\n\n See logs for more info"
            self.show_exception_box(msg)

    def set_trajectory(self, annotation:list):
        """
        Set the trajectory to the `annotation`.

        Args:
            annotation (list): Orderded list of lists of injection points
            [
            [[x11,y11,], [x12,y12],... ],
            [[x21, y21], [x22,y22],.. ],
            ...,
            ]
        """
        # Annotation may be empty (e.g. if user deletes annotation), so dont set trajectory
        if len(annotation) == 0:
            self.interpolated_pixels = []
            self._annotation_complete.emit(False)
            return
        try:
            self.interpolated_pixels = []
            for edge in annotation:
                self.interpolated_pixels.append(edge)
            self._annotation_complete.emit(True)
        except Exception as e:
            self.show_exception_box("Error while setting trajectory\n\nSee logs for more info.")
            self.interpolated_pixels = []

    def send_focus_z_to_display(self):
        """
        Queries the focus height and sends value to video display 
        """
        z = self.zen_controller.get_focus_um()
        self.vid_display.set_focus_z(z)

    def set_zstack_plane1(self):
        """
        Queries the focus controller for its height and sets the 
        plane1 entry box to the focus controller height.
        """
        z_height = round(self.zen_controller.get_focus_um(),2)
        self.plane1_entry.clear()
        self.plane1_entry.insert(str(z_height))

    def set_zstack_plane2(self):
        """
        Queries the focus controller for its height and sets the 
        plane1 entry box to the focus controller height.
        """
        z_height = round(self.zen_controller.get_focus_um(),2)
        self.plane2_entry.clear()
        self.plane2_entry.insert(str(z_height))

    def run_zstack_annotation(self):
        """ Conducts automatic annotaiton of the tissue via a z-stack """
        try:
            self.validate_zstack_parameters()
        except ValueError as e:
            self.show_error_box(e)
            return
        except:
            self.show_exception_box('Error while validating z-stack params')
            return

        start = float(self.plane1_entry.text().strip())
        stop = float(self.plane2_entry.text().strip())
        slices = int(self.slices_entry.text().strip())
        try:
            self.zstack.run(start, stop, slices)
        except Exception as e:
            self.show_error_box(e)

    def send_zstack_data(self):
        """
        Performs automatic tissue annotation on current video frame, and sends
        the image and annotation to the z-stack
        """
        image, annotation = self.zstack_automatic_annotation()
        if annotation is not None:
            self.annot_mgr.add_annotation(annotation)
        self.zstack.set_stack_data(image=image, annotation=annotation)

    def process_zstack_data(self, zstack_data:ZStackDataWithAnnotations):
        """
        Handle data returned by the z-stack. Data is returned as a dataclass
        with lists for images, z-heights, and annotations which can be None
        or its own list

        Args:
            zstack_data (dataclass): Dataclass from z-stack
        """
        self.zstack_to_annotation(zstack_data)
        self.save_zstack_data(zstack_data)

    def zstack_to_annotation(self, zstack_data:ZStackDataWithAnnotations):
        """
        Add z-stack data to annotations which will be injected. Also
        update the status/results of the zstack by displaying number
        of successful annotations

        Args:
            zstack_data (dataclass): Dataclass from z-stack
        """
        annot_counter = 0
        for annotation in zstack_data.get_annotations():
            if annotation is not None:
                # self.annot_mgr.add_annotation(annotation)
                annot_counter += 1
        self.zstack_status.setText(f"{annot_counter} of {len(zstack_data.annotations)} annotated")

    def save_zstack_data(self, zstack_data:ZStackDataWithAnnotations):
        """
        Save the zstack data and associated heights

        Args:
            zstack_data (dataclass): Dataclass from z-stack
        """
        if self.save_tiss_annot_rbon.isChecked():
            stack_saver = ImageStackData(data_dir=self.zstack_data_dir)
            img_stack = zstack_data.get_images_as_stack()
            z = zstack_data.get_z_heights()
            stack_saver.save_data(image_stack=img_stack, z_height=z)

    def validate_zstack_parameters(self):
        """
        Validates that all parameters are set and ready to perform a z-stack.
        Raises ValueError if parameters are not ready
        """
        plane1_str = self.plane1_entry.text().strip()
        plane2_str = self.plane2_entry.text().strip()
        slices_str = self.slices_entry.text().strip()
        if plane1_str == '':
            msg = f"Must set first focus before running z-stack.\n\nPlease click 'Set first focus' to register the starting z-stack height."
            raise ValueError(msg)
        elif val.is_valid_number(plane1_str) is False:
            msg = f'Invalid first focus: {plane1_str}. Must be a valid number.'
            raise ValueError(msg)
        if plane2_str == '':
            msg = f"Must set last focus before running z-stack.\n\nPlease click 'Set last focus' to register the ending z-stack height."
            raise ValueError(msg)
        elif val.is_valid_number(plane2_str) is False:
            msg = f'Invalid last focus: {plane2_str}. Must be a valid number.'
            raise ValueError(msg)
        if slices_str == '':
            msg = f"Must enter number of slices before running z-stack.\n\nPlease click enter the desired number of z-stack slices."
            raise ValueError(msg)
        elif val.is_valid_number(slices_str) is False:
            msg = f'Invalid slices number: {slices_str}. Must be a valid number.'
            raise ValueError(msg)

    
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
        if self.disp_mod_controller.display_annotation_bool is True:
            self.vid_display.show_interpolated_annotation(True)
        else:
            self.vid_display.show_interpolated_annotation(False)

    def display_calibration(self):
        ''' Send computed tip positoin (in camera) to video to be displayed.
        Calls itself on timer to update tip position in video'''
        if self.disp_mod_controller.display_calibration_bool is True:
            if self.pip_disp_timer.isActive() is False:
                self.pip_disp_timer.start(self.pip_disp_timeout)
            if self.pip_cal.model.is_calibrated is True:
                self.vid_display.show_tip_position(True)
                dev = SensapexDevice(1)
                pos = dev.get_pos()
                ex = self.pip_cal.model.forward(man=pos)
                z_scope = self.zen_controller.get_focus_um_approx()
                delta_z = ex[2] - z_scope
                self.vid_display.set_tip_position(ex[0], ex[1], delta_z)
            else:
                self.vid_display.show_tip_position(False)
        else:
            self.vid_display.show_tip_position(False)
            self.pip_disp_timer.stop()

    def display_tissue_mask(self):
        """ Tells video display whether to display annotated coordinates """
        if self.disp_mod_controller.display_segmentation_bool is True:
            self.vid_display.show_tissue_mask(True)
        else:
            self.vid_display.show_tissue_mask(False)

    def display_edge_mask(self):
        """ Tells video display whether to display annotated coordinates """
        if self.disp_mod_controller.display_edges_bool is True:
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
        self._is_angle_set = state
        if state is True:
            self._set_complete(self.angle_indicator)
        else:
            self._set_incomplete(self.angle_indicator)

    def _set_calibration_ind(self,state:bool):
        self._is_calibration_set = state
        if state is True:
            self._set_complete(self.calibration_indicator)
        else:
            self._set_incomplete(self.calibration_indicator)

    def _set_annotation_ind(self,state:bool):
        self._is_annotation_set = state
        if state is True:
            self._set_complete(self.annotation_indicator)
        else:
            self._set_incomplete(self.annotation_indicator)

    def _set_parameter_ind(self,state:bool):
        self._is_parameters_set = state
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

    def validate_ready_for_injection(self):
        """ Check that all conditions are met for injections to proceed """
        if self._is_angle_set is False:
            raise CalibrationError("Cannot conduct injection because pipette angle is not set.")
        if self._is_calibration_set is False:
            raise CalibrationError("Cannot conduct injection because pipette is not calibrated.")
        if self._is_annotation_set is False:
            raise AnnotationError("Cannot conduct injection because target annotation is incomplete.")
        if self._is_parameters_set is False:
            raise InjectionParameterError("Cannot conduct injection because injection parameters are not set.")
        
    def run_3D_trajectory(self):
        try:
            # Validate that everything is ready for injections
            self.validate_ready_for_injection()
            #get values from GUI
            um2nm = 1000
            approach_nm = int(float(self.approachdist)*um2nm)
            depth_nm = int(float(self.depthintissue)*um2nm)
            spacing_nm = int(float(self.stepsize)*um2nm)
            speed_ums = int((self.motorspeed))
            pullout_nm = self.cfg.values.pullout_nm
            dev = SensapexDevice(1)
            cal = self.pip_cal
            goto_z = self.zen_controller.goto_focus_absolute
            self.inj_trajectory = TrajectoryManager(goto_z)
            for ind, edge_3D in enumerate(self.interpolated_pixels):
                # Have trajectory end at the "finish" position if its the last trajectoyr
                end_at_fin_pos = ind == len(self.interpolated_pixels) - 1
                self.inj_trajectory.add_trajectory(SurfaceLineTrajectory3D(dev, cal, edge_3D, approach_nm, depth_nm, spacing_nm, speed_ums, pullout_nm, end_at_fin_pos))
            self.inj_trajectory.start()
            self.inj_trajectory.finished.connect(self.show_n_injected)
        except CalibrationError as e:
            self.show_error_box(f"Error: {e}\n\nPlease conduct calibration before injecting.")
        except AnnotationError as e:
            self.show_error_box(f"Error: {e}\n\nPlease annotate targets before injecting.")
        except InjectionParameterError as e:
            self.show_error_box(f"Error: {e}\n\nPlease set injection parameters before injecting.")
        except:
            msg = "Error occured while trying to start injections.\n\nSee logs for more info."
            self.show_exception_box(msg)

    def show_n_injected(self):
        self.response_monitor_window.append(">> Number of injections =" + str(self.inj_trajectory.n_injected))

    def stoptrajectory(self):
        try:
            self.inj_trajectory.stop()
        except:
            msg = "You have to start the trajectory in order to be able to stop it..."
            self.show_exception_box(msg)

    def moveto_unload_position(self):
        """ move pipette to unload position so user can change pipette"""
        try:
            self.convenience_trajectories.goto_unloaded()
        except TrajectoryError as e:
            self.show_error_box(e)
        except:
            self.show_exception_box("Error while unloading pipette.\n\nSee log for more info.")

    def moveto_displace_position(self):
        try:
            self.convenience_trajectories.goto_displaced()
        except TrajectoryError as e:
            self.show_error_box(e)
        except:
            self.show_exception_box("Error while moving away pipette.\n\nSee log for more info.")

    def moveto_center_position(self):
        if self.pip_cal.model.is_calibrated is False:
            msg = "Calibration incomplete. Cannot move to center without calibration."
            self.show_warning_box(msg)
            return
        else:
            try:
                z_scope = self.zen_controller.get_focus_um()
                center_pos = [int(self.vid_display.cam.width/2), int(self.vid_display.cam.height/2), z_scope]
                self.convenience_trajectories.goto_centered(self.pip_cal, center_pos)
            except TrajectoryError as e:
                self.show_error_box(e)
            except:
                self.show_exception_box("Error while centering pipette.\n\nSee log for more info.")

    def moveto_undo_position(self):
        try:
            self.convenience_trajectories.goto_undo()
        except TrajectoryError as e:
            self.show_error_box(e)
        except:
            self.show_exception_box("Error while moving away pipette.\n\nSee log for more info.")


    

if __name__ == "__main__":
    import sys
    app = QApplication(sys.argv)
    app.setApplicationName('MyWindow')
    main = ControlWindow('HamamatsuHam_DCAM', 'HamamatsuHam', 'HamamatsuHam_DCAM', '2x2', 180, 256, 1.3, 'com3')
    main.show()
    sys.exit(app.exec())
"""
Opens configuration application where user specifies parameters for the
Autoinjector application
"""

import os
import sys
from PyQt6.QtCore import pyqtSlot
from PyQt6 import QtGui
from PyQt6 import QtWidgets
from PyQt6.QtWidgets import (
    QApplication,
    QStatusBar,
    QStyleFactory,
    QStyle,
    QDialog,
    QFileDialog,
    QSizePolicy,
    QMessageBox,
    QComboBox,
    QPushButton,
    QGroupBox,
    QLineEdit,
    QFormLayout,
    QHBoxLayout,
    QVBoxLayout,
)
from src.cfg_mgmt.definitions import CONFIG_PATH
from src.cfg_mgmt.cfg_mngr import Configuration
from application_minimal import ControlWindow

class AutoinjectorConfigWindow(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent=parent)
        QApplication.setStyle(QStyleFactory.create("Fusion"))
        self.setWindowTitle("Autoinjector 2.0 Configuration")
        self._initialize_attributes()
        self._make_widgets()
        self._set_connections()
        self._stateify_widgets()
        self._layout_app()

    def _initialize_attributes(self):
        # Indicator for if parameters were saved
        self.is_saved = False

        # WhatsThis dialog for micromanager
        self.UMANAGER_WT = ("<i>Micro-Manager</i> is a open-source software "
        "for controlling various microscope hardware. The Autoinjector uses "
        "<i>Micro-Manager</i> to interface with some microscope cameras to "
        "stream live video in the Autoinjector software.<br><br>To use certain "
        "cameras, the <i>Micro-Manager</i> software must be installed on the "
        "computer. You must specify the path to the <i>Micro-Manager</i> "
        "installation directory here, so the Autoinjector can interface with "
        "the <i>Micro-Manager</i> software. (The default Windows installation "
        "directory is: `C:/Program Files/Micro-Manager-2.0`")

        # WhatsThis dialog for sensapex device
        self.SENSAPEX_WT = ("Select the <i>manipulator</i> device number of the "
        "manipulator to use for microinjections. It is likely that you only have "
        "one <i>manipulator</i> connected to the computer but it is possible "
        "that you have multiple <i>manipulators</i> connected, so you must select "
        "which device to use.<br><br>You can find the <i>manipulator</i> device "
        "number on the device's touch screen unit. If you only have one device "
        "connected, then the device number is likely to be '1'.")

        # WhatsThis dialog for autoinjector camera
        self.CAMERA_WT = ("A <i>camera</i> is used by the Autoinjector to "
        "recieve live video feed from the microscope and display this video "
        "feed in the Autoinjector software.<br><br>The computer may have "
        "multiple connected <i>cameras</i>, and certain <i>cameras</i> require "
        "specific initialization procedures to work properly with the "
        "Autoinjector. Hence, you must specify the Autoinjector <i>camera</i>, "
        "so the computer knows which camera to use and how to initilaize it.")

        # WhatsThis dialog for autoinjector camera
        self.COM_WT = ("An <i>Arduino</i> microcontroller is used by the "
        "Autoinjector to interface with the pressure control system for "
        "delivering the injection solution. The Autoinjector communicates with "
        "the <i>Arduino</i> through the <i>Arduino's</i> specific COM port "
        "address.<br><br>The computer may have multiple devices that each have "
        "unique COM port addresses. You must specify the COM port used by "
        "the pressure controller <i>Arduino</i>, so Autoinjector knows the "
        "correct communication address.")

        # WHatsThis dialog for z-polarity
        self.polarity_WT = ("The Autoinjector uses a calibration process to "
        "facilitate accurate 3D microinjection targetting. This process "
        "consists of constructing a spatial relationship between the microscope "
        "and the manipulator, so that the manipulator 'knows' how to go to any "
        "location in the microscope's FOV.<br><br>To simplify the calibration "
        "process for the user, the Autoinjector uses pre-specified constants to "
        "define the relationship between the microscope z-axis and the "
        "manipulator z-axis. The <i>polarity</i> defines the directionality between "
        "the different z-axes. The <i>polarity</i> = +1 if an equivalent change in "
        "position corresponds to an equivalent change in unit direction. For "
        "the Human Technopole Autoinjector, the polarity is -1.<br><br>Example: "
        "If you move the microscope z-axis and the manipulator z-axis in the same "
        "physical direction (i.e. towards the tissue sample), the <i>polarity</i> "
        "= +1 if their units move in the same direction (both positive or both "
        "negative) and the <i>polarity</i> = -1 if their units move in the "
        "opposite direction (one positive and one negative).")

        # WHatsThis dialog for z-scaling
        self.scaling_WT = ("The Autoinjector uses a calibration process to "
        "facilitate accurate 3D microinjection targetting. This process "
        "consists of constructing a spatial relationship between the microscope "
        "and the manipulator, so that the manipulator 'knows' how to go to any "
        "location in the microscope's FOV.<br><br>To simplify the calibration "
        "process for the user, the Autoinjector uses pre-specified constants to "
        "define the relationship between the microscope z-axis and the "
        "manipulator z-axis. The <i>scaling</i> defines the ratio of microscope "
        "z-axis movement to the manipulator z-axis movement. Compute the "
        "<i>scaling</i> dividing the change in the microscope z-position "
        "by an equivalent change in manipulator z-position.<br><br>Example: "
        "Starting with the microinjection needle in focus, you move the "
        "microscope focus by 1000um. Then you move the manipulator z-axis to bring "
        "the microinjection needle back into focus, and note that the "
        "manipulator z-position changed by 750um. Therefore, the scaling is "
        "750um / 1000um = 0.75. Note that ideally the scaling would be 1:1, but "
        "this may not be the case if you have refractive index mismatch.")

    def _make_widgets(self):
        """ Makes widgets for the application """

        # Entry for path to micromanager directory
        self.umanager_entry = QLineEdit(self)
        self.umanager_entry.setToolTip("Enter directory of installed Micro-Manager software")
        self.umanager_entry.setWhatsThis(self.UMANAGER_WT)

        # Directory browser for micromanager directory
        self.umanager_browser = QPushButton("")
        self.umanager_browser.setToolTip("Open file browser to select Micro-Manager directory")
        self.umanager_browser.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Preferred)
        # SHow browser button as a directory folder icon
        pixmapi = QStyle.StandardPixmap.SP_DirIcon
        icon = self.style().standardIcon(pixmapi)
        self.umanager_browser.setIcon(icon)
        
        # Help button for micromanager directory
        self.umanager_help = QPushButton("")
        self.umanager_help.setToolTip("Show more information about Micro-Manager directory")
        self.umanager_help.setWhatsThis(self.UMANAGER_WT)
        self.umanager_help.setSizePolicy(QSizePolicy.Policy.Maximum, QSizePolicy.Policy.Preferred)
        # Show the micromanager help button as a help icon
        pixmapi = QStyle.StandardPixmap.SP_TitleBarContextHelpButton
        icon = self.style().standardIcon(pixmapi)
        self.umanager_help.setIcon(icon)

        # Dropdown box for selecting sensapex id
        self.sensapex_combo = QComboBox()
        self.sensapex_combo.setToolTip("Select the Sensapex micromanipulator ID number")
        self.sensapex_combo.setWhatsThis(self.SENSAPEX_WT)
        self.sensapex_combo.setPlaceholderText("Select")

        # Help button for camera
        self.sensapex_help = QPushButton("")
        self.sensapex_help.setToolTip("Show more information about manipulator number")
        self.sensapex_help.setSizePolicy(QSizePolicy.Policy.Maximum, QSizePolicy.Policy.Preferred)
        # Show the sensapex help button as a help icon
        pixmapi = QStyle.StandardPixmap.SP_TitleBarContextHelpButton
        icon = self.style().standardIcon(pixmapi)
        self.sensapex_help.setIcon(icon)

        # Dropdown box for selecting camera
        self.camera_combo = QComboBox()
        self.camera_combo.setToolTip("Select the camera of the Autoinjector microscope")
        self.camera_combo.setWhatsThis(self.CAMERA_WT)
        self.camera_combo.setPlaceholderText("Select")
        
        # Help button for camera
        self.camera_help = QPushButton("")
        self.camera_help.setToolTip("Show more information about camera")
        self.camera_help.setSizePolicy(QSizePolicy.Policy.Maximum, QSizePolicy.Policy.Preferred)
        # Show the camera help button as a help icon
        pixmapi = QStyle.StandardPixmap.SP_TitleBarContextHelpButton
        icon = self.style().standardIcon(pixmapi)
        self.camera_help.setIcon(icon)

        # Dropdown box for selecting presure controller (arduino) com port
        self.com_combo = QComboBox()
        self.com_combo.setToolTip("Select the COM port of the pressure controller Arduino")
        self.com_combo.setWhatsThis(self.COM_WT)
        self.com_combo.setPlaceholderText("Select")
        
        # Help button for camera
        self.com_help = QPushButton("")
        self.com_help.setToolTip("Show more information about Arduino COM port")
        self.com_help.setSizePolicy(QSizePolicy.Policy.Maximum, QSizePolicy.Policy.Preferred)
        # Show the COM port help button as a help icon
        pixmapi = QStyle.StandardPixmap.SP_TitleBarContextHelpButton
        icon = self.style().standardIcon(pixmapi)
        self.com_help.setIcon(icon)

        # Dropdown box for selecting manipulator-microscope z-polarity
        self.polarity_combo = QComboBox()
        self.polarity_combo.setToolTip("Select the direction polarity between the microscope and manipulator")
        self.polarity_combo.setWhatsThis(self.polarity_WT)
        self.polarity_combo.setPlaceholderText("Select")

        # Help button for z polarity
        self.polarity_help = QPushButton("")
        self.polarity_help.setToolTip("Show more information about z-polarity")
        self.polarity_help.setSizePolicy(QSizePolicy.Policy.Maximum, QSizePolicy.Policy.Preferred)
        # Show the polarity help button as a help icon
        pixmapi = QStyle.StandardPixmap.SP_TitleBarContextHelpButton
        icon = self.style().standardIcon(pixmapi)
        self.polarity_help.setIcon(icon)

        # Z-scaling entry for manipulator-microscope z-scaling
        self.scaling_entry = QLineEdit()
        self.scaling_entry.setToolTip("Enter microscope-manipulator z-scaling ratio (microscope z / manipulator z)")
        self.scaling_entry.setWhatsThis(self.scaling_WT)

        # Help button for z polarity
        self.scaling_help = QPushButton("")
        self.scaling_help.setToolTip("Show more information about z-scaling")
        self.scaling_help.setSizePolicy(QSizePolicy.Policy.Maximum, QSizePolicy.Policy.Preferred)
        # Show the scaling help button as a help icon
        pixmapi = QStyle.StandardPixmap.SP_TitleBarContextHelpButton
        icon = self.style().standardIcon(pixmapi)
        self.scaling_help.setIcon(icon)

        # Buttons for saving configuration and exiting
        self.save_but = QPushButton("Save configuration")
        self.save_but.setToolTip("Save configuration parameters to a file that will be loaded by the Autoinjector")
        self.goto_autoinjector_but = QPushButton("Open Autoinjector")
        self.goto_autoinjector_but.setToolTip("Open Autoinjector application with saved configuration values")
        self.exit_but = QPushButton("Exit")
        self.exit_but.setToolTip("Exit the configuration application")

    def _layout_app(self):
        """ Place the widgets in layouts and organize layouts in app """

        # Directories group
        self.dir_group = QGroupBox("Software Directories")
        dir_layout = QVBoxLayout()
        dir_hl = QHBoxLayout()
        dir_hl.addWidget(self.umanager_browser)
        dir_hl.addWidget(self.umanager_help)
        dir_layout2 = QFormLayout()
        dir_layout2.addRow("µManager Directory", dir_hl)
        dir_layout.addLayout(dir_layout2)
        dir_layout.addWidget(self.umanager_entry)
        self.dir_group.setLayout(dir_layout)

        # Hardware devices group
        self.hw_group = QGroupBox("Hardware Devices")
        hw_layout = QFormLayout()
        sensapex_hl = QHBoxLayout()
        sensapex_hl.addWidget(self.sensapex_combo)
        sensapex_hl.addWidget(self.sensapex_help)
        camera_hl = QHBoxLayout()
        camera_hl.addWidget(self.camera_combo)
        camera_hl.addWidget(self.camera_help)
        com_hl = QHBoxLayout()
        com_hl.addWidget(self.com_combo)
        com_hl.addWidget(self.com_help)
        hw_layout.addRow("Manipulator #:", sensapex_hl)
        hw_layout.addRow("Camera:", camera_hl)
        hw_layout.addRow("Arduino:", com_hl)
        self.hw_group.setLayout(hw_layout)

        # Calibration settings group
        self.cal_group = QGroupBox("Calibration Settings")
        cal_layout = QFormLayout()
        polarity_hl = QHBoxLayout()
        polarity_hl.addWidget(self.polarity_combo)
        polarity_hl.addWidget(self.polarity_help)
        scaling_hl = QHBoxLayout()
        scaling_hl.addWidget(self.scaling_entry)
        scaling_hl.addWidget(self.scaling_help)
        cal_layout.addRow("Z-Polarity:", polarity_hl)
        cal_layout.addRow("Z-Scaling:", scaling_hl)
        self.cal_group.setLayout(cal_layout)

        # Master applicaiton layout
        layout = QVBoxLayout()
        layout.addWidget(self.dir_group)
        layout.addWidget(self.hw_group)
        layout.addWidget(self.cal_group)
        layout.addWidget(self.save_but)
        layout.addWidget(self.goto_autoinjector_but)
        self.setLayout(layout)

    def _set_connections(self):
        # On button click, open file browser and insert directory into entry
        self.umanager_browser.clicked.connect(lambda: self.select_dir_for_entry(self.umanager_entry))
        # Open WhatsThis dialog when help button clicked
        self.umanager_help.clicked.connect(lambda: self.show_WhatsThis(self.UMANAGER_WT))
        self.sensapex_help.clicked.connect(lambda: self.show_WhatsThis(self.SENSAPEX_WT))
        self.camera_help.clicked.connect(lambda: self.show_WhatsThis(self.CAMERA_WT))
        self.com_help.clicked.connect(lambda: self.show_WhatsThis(self.COM_WT))
        self.polarity_help.clicked.connect(lambda: self.show_WhatsThis(self.polarity_WT))
        self.scaling_help.clicked.connect(lambda: self.show_WhatsThis(self.scaling_WT))
        # Saving parameters and opening 
        self.save_but.clicked.connect(self.on_save)
        self.goto_autoinjector_but.clicked.connect(self.on_autoinjector)

    def _stateify_widgets(self):
        # Set path as the default at Human Technopole (which should be default install location)
        self.umanager_entry.setText("C:/Program Files/Micro-Manager-2.0")

        # Set the available manipulator options for user to select
        dev_list = [str(ind+1) for ind in range(10)]
        self.sensapex_combo.insertItems(0, dev_list)
        self.sensapex_combo.setCurrentIndex(0)

        # Set the available camera options for user to select
        cam_list = ["Hamamatsu Orca DCAM", "Zeiss Axiocam", "PVCam"]
        self.camera_combo.insertItems(0, cam_list)
        # Default camera to the Hamamatsu used at Human Technopole
        self.camera_combo.setCurrentText(cam_list[0])

        # Set available COM options for the user to select
        com_list = [f"com{num}" for num in range(11)]
        self.com_combo.insertItems(0, com_list)
        # Default COM port to the port used at Human Technopole
        self.com_combo.setCurrentText(com_list[3])

        # Set available polarity options for the user to select
        pol_list = ["+1", "-1"]
        self.polarity_combo.insertItems(0, pol_list)
        # Default polarity to the polarity at Human Technopole
        self.polarity_combo.setCurrentText(pol_list[1])

        # Set scaling to default at Human Technopole
        self.scaling_entry.setText("0.741")

    @pyqtSlot()
    def show_WhatsThis(self, text:str):
        """ Shows `text` in WhatsThis dialog at the current mouse position. """
        QtWidgets.QWhatsThis.showText(QtGui.QCursor.pos(), text)    

    @pyqtSlot()
    def select_dir_for_entry(self, entry:QLineEdit):
        """
        Open directory broswer and insert selected directory into the
        entry box.
        
        Args:
            entry (QLineEdit): Entry box to modify with selected directory
        """
        fdir = self.get_dir()
        if os.path.isdir(fdir):
            self.umanager_entry.setText(fdir)

    @pyqtSlot()
    def on_save(self):
        """
        Handle actions when user presses save configuration. Validates the
        parameters and then saves to file
        """
        try:
            params = self.validate_params()
        except Exception as e:
            self.error_msg = QMessageBox()
            self.error_msg.setIcon(QMessageBox.Icon.Critical)
            self.error_msg.setWindowTitle("Error")
            self.error_msg.setText(str(e))
            self.error_msg.exec()
        else:
            cfg = Configuration(**params)
            cfg.to_file()

    @pyqtSlot()
    def on_autoinjector(self):
        """ Handles what to do if open autoinjector is clicked """
        if not os.path.exists(CONFIG_PATH):
            self.error_msg = QMessageBox()
            self.error_msg.setIcon(QMessageBox.Icon.Critical)
            self.error_msg.setWindowTitle("Error")
            msg = f"Cannot open Autoinjector because no configuration file saved at {CONFIG_PATH}\n\nSave configuration before opening Autoinjector."
            self.error_msg.setText(msg)
            self.error_msg.exec()
        else:
            self.open_autoinjector()

    def open_autoinjector(self):
        """ Opens autoinjector app """
        open_app = True
        if self.is_saved is False:
            qm = QMessageBox()
            ret = qm.question(self,'Open without new configuration?','No new configuration was saved. Do you want to open with the previous configuration?')
            if ret == QMessageBox.StandardButton.No:
                open_app = False
        
        if open_app is True:
            self.close()
            self.main_app = ControlWindow()
            self.main_app.show()

    def get_dir(self):
        """ Open directory to browse for a directory. """
        fdir = QFileDialog.getExistingDirectory(self)
        return fdir

    def validate_params(self) -> dict:
        """
        Validates parameters, converts to expected datatypes, and returns
        dictionary of parameters.

        Raises:
            OSError if invalid micromanager directory
            ValueError if numeric parameters dont match expected values
        
        Returns:
            dictionary of configuration parameters
        """
        # Get config parameters from widgets
        micromanager_dir = self.umanager_entry.text()
        sensapex_id = self.sensapex_combo.currentText()
        camera = self.camera_combo.currentText()
        com = self.com_combo.currentText()
        polarity = self.polarity_combo.currentText()
        scaling = self.scaling_entry.text()

        # Confirm micromanager directory is valid and contains micromanager file
        if not os.path.isdir(micromanager_dir):
            raise OSError(f"Invalid Micro-Manager directory: {micromanager_dir}.\n\nDirectory doesn't exist.")
        # TODO add a check that confirms micromanager file is in directory

        # Confirm that sensapex_id is number
        try:
            sensapex_id_num = float(sensapex_id)
        except ValueError:
            raise ValueError(f"Invalid manipulator number: {sensapex_id}\n\nMust be a valid number.")
        else:
            sensapex_id_num = int(sensapex_id_num)

        # Confirm that polarity is valid number and is -1 or +1
        try:
            polarity_num = float(polarity)
        except ValueError:
            raise ValueError(f"Invalid polarity: {polarity}\n\nPolarity must be a valid number (-1 or 1).")
        if polarity_num in [-1, 1]:
            polarity_num = int(polarity_num)
        else:
            raise ValueError(f"Invalid polarity: {polarity}\n\nPolarity must be a valid number (-1 or 1).")

        # Confirm that scaling is a positive float
        try:
            scaling_num = float(scaling)
        except ValueError:
            raise ValueError(f"Invalid scaling: {scaling}\n\nScaling must be a valid positive number.")
        if scaling_num < 0:
            raise ValueError(f"Invalid scaling: {scaling}\n\nScaling must be a positive number. If you computed a negative scaling, then enter -1 for polarity and enter a postive scaling number.")
        if scaling_num == 0:
            raise ValueError(f"Invalid scaling: {scaling}\n\nScaling must be a positive number.")
        
        # Return dictionary of parameters
        params = {
            "micromanager_path": micromanager_dir,
            "sensapex_id": sensapex_id_num,
            "camera": camera,
            "com": com,
            "z_polarity": polarity_num,
            "z_scaling": scaling_num
        }
        return params

def main():
    app = QApplication([])
    window = AutoinjectorConfigWindow()
    window.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
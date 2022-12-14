""" MVC implimentation of display modification widgets """

import sys
from PyQt6.QtCore import pyqtSignal, QObject, QSignalBlocker
from PyQt6.QtWidgets import (QApplication,
                            QWidget,
                            QGroupBox,
                            QLabel,
                            QRadioButton,
                            QButtonGroup,
                            QFormLayout,
                            QHBoxLayout)

class Controller (QObject):

    display_calibration_toggled = pyqtSignal(bool)
    display_annotation_toggled = pyqtSignal(bool)
    display_segmentation_toggled = pyqtSignal(bool)
    display_edges_toggled = pyqtSignal(bool)

    def __init__(self):
        super().__init__()
        self.display_calibration_bool = True
        self.display_annotation_bool = True
        self.display_segmentation_bool = True
        self.display_edges_bool = True

    def on_display_calibration_toggled(self, bool_:bool):
        """
        Set `display_calibration` attribute to `bool_`

        Args:
            bool_: Boolean state to set attribute to
        """
        self.display_calibration_bool = bool_
        self.display_calibration_toggled.emit(bool_)
    
    def on_display_annotation_toggled(self, bool_:bool):
        """
        Set `display_annotation` attribute to `bool_`

        Args:
            bool_: Boolean state to set attribute to
        """
        self.display_annotation_bool = bool_
        self.display_annotation_toggled.emit(bool_)
    
    def on_display_segmentation_toggled(self, bool_:bool):
        """
        Set `display_segmentation` attribute to `bool_`

        Args:
            bool_: Boolean state to set attribute to
        """
        self.display_segmentation_bool = bool_
        self.display_segmentation_toggled.emit(bool_)
    
    def on_display_edges_toggled(self, bool_:bool):
        """
        Set `display_edges` attribute to `bool_`

        Args:
            bool_: Boolean state to set attribute to
        """
        self.display_edges_bool = bool_
        self.display_edges_toggled.emit(bool_)

class View(QWidget):
    """
    Widgets for modifying the GUI display
    """

    # Emitted when change in focus position requested in the view
    new_display_calibration_state = pyqtSignal(bool)
    new_display_annotation_state = pyqtSignal(bool)
    new_display_segmentation_state = pyqtSignal(bool)
    new_display_edges_state = pyqtSignal(bool)
    
    def __init__(self, controller:Controller):
        """
        Args:
            controller: display modification controller for mvc architecture
        """
        super().__init__()
        self._controller = controller
        
        self._make_widgets()
        self._set_connections()

    def _make_widgets(self):
        """ Makes widgets of display modificaiotn interface """
        self.display_calibration_rbon = QRadioButton("On")
        self.display_calibration_rbon.setToolTip("Show calibrated tip position overlay")
        self.display_calibration_rboff = QRadioButton("Off")
        self.display_calibration_rboff.setToolTip("Hide calibrated tip position overlay")
        self.display_annotation_rbon = QRadioButton("On")
        self.display_annotation_rbon.setToolTip("Show the drawn annotations")
        self.display_annotation_rboff = QRadioButton("Off")
        self.display_annotation_rboff.setToolTip("Hide the drawn annotations")
        self.display_segmentation_rbon = QRadioButton("On")
        self.display_segmentation_rbon.setToolTip("Show the detected tissue (during auomated annotations)")
        self.display_segmentation_rboff = QRadioButton("Off")
        self.display_segmentation_rboff.setToolTip("Hide the detected tissue (during auomated annotations)")
        self.display_edges_rbon = QRadioButton("On")
        self.display_edges_rbon.setToolTip("Show the detected edges (during auomated annotations)")
        self.display_edges_rboff = QRadioButton("Off")
        self.display_edges_rboff.setToolTip("Show the detected edges (during auomated annotations)")
        bg1 = QButtonGroup(self)
        bg1.setExclusive(True)
        bg1.addButton(self.display_calibration_rbon)
        bg1.addButton(self.display_calibration_rboff)
        bg2 = QButtonGroup(self)
        bg2.addButton(self.display_annotation_rbon)
        bg2.addButton(self.display_annotation_rboff)
        bg3 = QButtonGroup(self)
        bg3.addButton(self.display_segmentation_rbon)
        bg3.addButton(self.display_segmentation_rboff)
        bg4 = QButtonGroup(self)
        bg4.addButton(self.display_edges_rbon)
        bg4.addButton(self.display_edges_rboff)

    def _set_connections(self):
        """ Sets view-controller signal/slot connections """
        # Set view signal connection w/ controller slots
        self.display_calibration_rbon.toggled.connect(self._controller.on_display_calibration_toggled)
        self.display_annotation_rbon.toggled.connect(self._controller.on_display_annotation_toggled)
        self.display_segmentation_rbon.toggled.connect(self._controller.on_display_segmentation_toggled)
        self.display_edges_rbon.toggled.connect(self._controller.on_display_edges_toggled)
        # set controller signal with view slow (like when user change a different view)
        self._controller.display_calibration_toggled.connect(self.set_display_calibration)
        self._controller.display_annotation_toggled.connect(self.set_display_annotation)
        self._controller.display_segmentation_toggled.connect(self.set_display_segmentation)
        self._controller.display_edges_toggled.connect(self.set_display_edges)

    def stateify_widgets(self):
        """ Initializes widgets """
        self.display_calibration_rbon.setChecked(self._controller.display_calibration_bool)
        self.display_annotation_rbon.setChecked(self._controller.display_annotation_bool)
        self.display_segmentation_rbon.setChecked(self._controller.display_segmentation_bool)
        self.display_edges_rbon.setChecked(self._controller.display_edges_bool)

    def set_display_calibration(self, bool_:bool):
        """
        Set state of display calibration radio button to `bool_`

        Args:
            bool_ (bool): whether to turn button 'on'
        """
        tmp_block = QSignalBlocker(self.display_calibration_rbon)
        if bool_ is True: 
            self.display_calibration_rbon.setChecked(bool_)
        else:
            self.display_calibration_rboff.setChecked(True)
        tmp_block.unblock()

    def set_display_annotation(self, bool_:bool):
        """
        Set state of display annotation radio button to `bool_`

        Args:
            bool_ (bool): whether to turn button 'on'
        """
        tmp_block = QSignalBlocker(self.display_annotation_rbon)
        if bool_ is True:
            self.display_annotation_rbon.setChecked(bool_)
        else:
            self.display_annotation_rboff.setChecked(True)
        tmp_block.unblock()

    def set_display_segmentation(self, bool_:bool):
        """
        Set state of display segmentation radio button to `bool_`

        Args:
            bool_ (bool): whether to turn button 'on'
        """
        tmp_block = QSignalBlocker(self.display_segmentation_rbon)
        if bool_ is True:  
            self.display_segmentation_rbon.setChecked(bool_)
        else:
            self.display_segmentation_rboff.setChecked(True)
        tmp_block.unblock()

    def set_display_edges(self, bool_:bool):
        """
        Set state of display edges radio button to `bool_`

        Args:
            bool_ (bool): whether to turn button 'on'
        """
        tmp_block = QSignalBlocker(self.display_edges_rbon)
        if bool_ is True:    
            self.display_edges_rbon.setChecked(bool_)
        else:
            self.display_edges_rboff.setChecked(True)
        tmp_block.unblock()
    
class ViewApp(QGroupBox):
    """ Fully assembled user interface for display modifcation widgets """

    def __init__(self, controller:Controller):
        """
        Args:
            controller: display modification controller for mvc architecture
        """
        super().__init__("Display Modification")
        self._view = View(controller)
        self._view.stateify_widgets()
        self._make_application()
    
    def _make_application(self):
        """ Makes widgets from the View and layout in window. """
        display_calibration_label = QLabel('Show tip:')
        display_annotation_label = QLabel('Show annotation:')
        display_segmentation_label = QLabel('Show tissue:')
        display_edge_label = QLabel('Show edge:')
        hl1 = QHBoxLayout()
        hl1.addWidget(self._view.display_calibration_rbon)
        hl1.addWidget(self._view.display_calibration_rboff)
        hl2 = QHBoxLayout()
        hl2.addWidget(self._view.display_annotation_rbon)
        hl2.addWidget(self._view.display_annotation_rboff)
        hl3 = QHBoxLayout()
        hl3.addWidget(self._view.display_segmentation_rbon)
        hl3.addWidget(self._view.display_segmentation_rboff)
        hl4 = QHBoxLayout()
        hl4.addWidget(self._view.display_edges_rbon)
        hl4.addWidget(self._view.display_edges_rboff)
        form = QFormLayout()
        form.addRow(display_calibration_label, hl1)
        form.addRow(display_annotation_label, hl2)
        form.addRow(display_segmentation_label, hl3)
        form.addRow(display_edge_label, hl4)
        self.setLayout(form)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    controller = Controller()
    win = ViewApp(controller)
    win.show()
    sys.exit(app.exec())
""" MVC implimentation of display modification widgets """

import sys
from PyQt6.QtCore import pyqtSignal, QObject
from PyQt6.QtWidgets import (QApplication,
                            QWidget,
                            QGroupBox,
                            QLabel,
                            QRadioButton,
                            QButtonGroup,
                            QFormLayout,
                            QHBoxLayout)

class Controller (QObject):

    def __init__(self):
        super().__init__()
        self.display_calibration_bool = True
        self.display_annotation_bool = True
        self.display_segmentation_bool = True
        self.display_edges_bool = True

    def set_display_calibration(self, bool_:bool):
        """
        Set `display_calibration` attribute to `bool_`

        Args:
            bool_: Boolean state to set attribute to
        """
        self.display_calibration_bool = bool_
    
    def set_display_annotation(self, bool_:bool):
        """
        Set `display_annotation` attribute to `bool_`

        Args:
            bool_: Boolean state to set attribute to
        """
        self.display_annotation_bool = bool_
    
    def set_display_segmentation(self, bool_:bool):
        """
        Set `display_segmentation` attribute to `bool_`

        Args:
            bool_: Boolean state to set attribute to
        """
        self.display_segmentation_bool = bool_
    
    def set_display_edges(self, bool_:bool):
        """
        Set `display_edges` attribute to `bool_`

        Args:
            bool_: Boolean state to set attribute to
        """
        self.display_edges_bool = bool_

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
        # self.stateify_widgets()

    def _make_widgets(self):
        """ Makes widgets of display modificaiotn interface """
        self.display_calibration_rbon = QRadioButton("On")
        self.display_calibration_rboff = QRadioButton("Off")
        self.display_annotation_rbon = QRadioButton("On")
        self.display_annotation_rboff = QRadioButton("Off")
        self.display_segmentation_rbon = QRadioButton("On")
        self.display_segmentation_rboff = QRadioButton("Off")
        self.display_edges_rbon = QRadioButton("On")
        self.display_edges_rboff = QRadioButton("Off")
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
        # Set view signal connection w/ view slots
        self.display_calibration_rbon.toggled.connect(self._display_calibration)
        self.display_annotation_rbon.toggled.connect(self._display_annotation)
        self.display_segmentation_rbon.toggled.connect(self._display_segmentation)
        self.display_edges_rbon.toggled.connect(self._display_edges)
        # set view signal connection w/ controller slots
        self.new_display_calibration_state.connect(self._controller.set_display_calibration)
        self.new_display_annotation_state.connect(self._controller.set_display_annotation)
        self.new_display_segmentation_state.connect(self._controller.set_display_segmentation)
        self.new_display_edges_state.connect(self._controller.set_display_edges)

    def stateify_widgets(self):
        """ Initializes widgets """
        self.display_calibration_rbon.setChecked(self._controller.display_calibration_bool)
        self.display_annotation_rbon.setChecked(self._controller.display_annotation_bool)
        self.display_segmentation_rbon.setChecked(self._controller.display_segmentation_bool)
        self.display_edges_rbon.setChecked(self._controller.display_edges_bool)
    
    def _display_calibration(self):
        """ Emits signal of new radio button state for displaying calibration """
        state = self.display_calibration_rbon.isChecked()
        self.new_display_calibration_state.emit(state)

    def _display_annotation(self):
        """ Emits signal of new radio button state for displaying annotation """
        state = self.display_annotation_rbon.isChecked()
        self.new_display_annotation_state.emit(state)
    
    def _display_segmentation(self):
        """ Emits signal of new radio button state for displaying segmentation """
        state = self.display_segmentation_rbon.isChecked()
        self.new_display_segmentation_state.emit(state)

    def _display_edges(self):
        """ Emits signal of new radio button state for displaying edges """
        state = self.display_edges_rbon.isChecked()
        self.new_display_edges_state.emit(state)

    
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
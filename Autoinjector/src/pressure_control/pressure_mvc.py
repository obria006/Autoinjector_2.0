""" Handles pressure control in Model-View-Controller software pattern """
import sys
import serial
import time
from PyQt6.QtCore import Qt, pyqtSignal, QObject, pyqtSlot
from PyQt6.QtWidgets import QApplication, QSlider, QLineEdit, QPushButton, QGroupBox, QGridLayout
from PyQt6.QtGui import QPalette, QColor
from src.pythonarduino.injectioncontrolmod import injection
from src.GUI_utils.gui_objects import SmallQLineEdit
from src.miscellaneous import validify as val
from src.miscellaneous.standard_logger import StandardLogger

class PressureModel(QObject):
    """
    Model to handle pressure controller buisness logic. Communicates with an
    arduino to send pressure commands. Arduino expects 8-bit pressure commands.
    """

    def __init__(self, arduino):
        """
        Args:
            arduino: serial object for sending recieving commands
        """
        super().__init__()
        self._logger = StandardLogger(__name__)
        self.arduino = arduino

    def backpressure(self, pres:int):
        """
        Sends back pressure command of `pres` to the arduino. Aruduino expects
        8-bit value, so pressure is clipped 0-255 and cast to int.

        Args:
            pres (int): Pressure command to arduino in 0-255
        """
        # Clip to 0 - 255
        if pres < 0 or pres > 255:
            self._logger.warning(f"Clipping pressure command of {pres} to 0-255")
            pres = int(min(255, max(0, pres)))

        # Send the pressure command
        pressure_command = injection(self.arduino, pres, 0, pres, 0, 'bp')
        pressure_command.start()


class PressureController(QObject):
    """
    MVC controller to coordinate betweeen the model and views. Pressure values
    in model expect 8-bit pressure commands (0-255), but pressure values are on
    scale 0-100% (for user usability). So controller handles conversion between
    these scales, and coordinates between model and view. Attributes in
    `PressureController` assume the same scale as model, so all attributes are
    stored on 0-255 scale.
    """
    pressure_to_view = pyqtSignal(int)
    errors = pyqtSignal(str)


    def __init__(self, model: PressureModel):
        """
        Args:
            model (PressureModel): MVC model for pressure
        """
        super().__init__()
        self._logger = StandardLogger(__name__)
        self._model = model

        # Allowable ranges for model and view pressure values
        self._MODEL_RANGE = (0, 255)
        self._VIEW_RANGE = (0, 100)

        # Current pressure of the system, and pressure before it was turned off
        self._current_bp = 0
        self._prezero_bp = self._current_bp

    @pyqtSlot()
    def on_slider_change(self, pres_str:str):
        """
        Handles when view pressure slider changes. Sends a new pressure command
        to the model and sends the pressure value back to the view, so all
        views and view elements can be updated.

        Args:
            pres_str (str): Pressure value from the view
        """
        try:
            # Validate pressure values
            if not val.is_valid_number(pres_str):
                raise ValueError(f"Invalid pressure value: {pres_str}. Must be a valid number.")
            pres = float(pres_str)
            if (pres < self._VIEW_RANGE[0] or pres> self._VIEW_RANGE[1]):
                raise ValueError(f"Invalid pressure value: {pres_str}. Must be between {self._VIEW_RANGE}.")
            
            # Apply new backpressure from the slider value
            self.new_bp(pres, 'view')
        except Exception as e:
            msg = f"Errors while changing slider: {e}. See logs for more info."
            self._logger.exception(msg)
            self.errors.emit(msg)
        

    @pyqtSlot()
    def on_clear_button(self):
        """
        Handles when the pressure "clear" button is clicked
        """
        try:
            raise NotImplementedError("`Clear` button is not implimented.")
        except Exception as e:
            msg = f"Errors while clearing pressure: {e}. See logs for more info."
            self._logger.exception(msg)
            self.errors.emit(msg)

    def _convert_pressure(self, pres:float, src:str) -> int:
        """
        Converts pressures between model and view scales. `src` specifies
        the range the `pres` value was from (either "model" or "view"), and it
        converts the pressure value into the other scale.

        Args:
            pres (float): Pressure value to convert
            src (str): "model" or "view" specifying which scale to convert from
        
        Returns:
            new_pres (int): Pressure value converted to opposite scale
        """
        if not val.is_valid_number(pres):
            raise ValueError(f"Invalid pressure value: {pres}. Must be a valid number.")
        if src not in ['model', 'view']:
            raise ValueError(f'Invalid pressure conversion source: {src}. Must be "model" or "view".')
        
        # Linear interpolation of pressure value depending on the destination
        if src == 'view':
            if (pres < self._VIEW_RANGE[0] or pres> self._VIEW_RANGE[1]):
                raise ValueError(f"Invalid pressure conversion value for {src}: {pres}. Must be between {self._VIEW_RANGE}")
            rate = (self._MODEL_RANGE[1] - self._MODEL_RANGE[0]) / (self._VIEW_RANGE[1] - self._VIEW_RANGE[0])
            new_val = self._MODEL_RANGE[0] + rate*(pres - self._VIEW_RANGE[0])
        if src == 'model':
            if (pres < self._MODEL_RANGE[0] or pres> self._MODEL_RANGE[1]):
                raise ValueError(f"Invalid pressure conversion value for {src}: {pres}. Must be between {self._MODEL_RANGE}")
            rate = (self._VIEW_RANGE[1] - self._VIEW_RANGE[0]) / (self._MODEL_RANGE[1] - self._MODEL_RANGE[0])
            new_val = self._VIEW_RANGE[0] + rate*(pres - self._MODEL_RANGE[0])

        return int(round(new_val))

    def _apply_backpressure(self, model_pres:int):
        """
        Command model to apply pressure according the the `model_pres`.

        Args:
            model_pres (int): Pressure command for model on scale of (0-255)
        """
        if not val.is_valid_number(model_pres):
            raise ValueError(f"Invalid pressure value: {model_pres}. Must be a valid number.")
        if (model_pres < self._MODEL_RANGE[0] or model_pres> self._MODEL_RANGE[1]):
            raise ValueError(f"Invalid pressure: {model_pres}. Must be between {self._MODEL_RANGE}")
        
        self._model.backpressure(model_pres)

    def zero_bp(self):
        """
        Set backpressure to 0. Sets the `_prezero_bp` attribute which can be
        used when re-opening pressure to apply the same pressure as before
        closing.
        """
        pres = 0
        self._prezero_bp = self._current_bp
        self.new_bp(pres, 'model')
    
    def reload_bp(self):
        """
        Reloads the pressure by applying the pressure from before set to zero
        """
        pres = self._prezero_bp
        self.new_bp(pres, 'model')
    
    def new_bp(self, pres, src):
        """
        Main function for applying backpressure. Takes pressure value (and 
        converts to the model scale if necessary), sends the pressure command
        to the model, sends pressure to view so it can be updated, and updates
        the `_current_bp` attribute.

        Args:
            pres (float): Pressure value to apply
            src (str): "model" or "view" specifying the scale of `pres`
        """
        try:
            if not val.is_valid_number(pres):
                raise ValueError(f"Invalid pressure value: {pres}. Must be a valid number.")
            if src not in ['model', 'view']:
                raise ValueError(f'Invalid pressure value source: {src}. Must be "model" or "view".')

            if src == 'view':
                view_pres = pres
                model_pres = self._convert_pressure(pres, src)
            if src == "model":
                model_pres = pres
                view_pres = self._convert_pressure(pres, src)

            # Send command to arduino
            self._apply_backpressure(model_pres)

            # Emit signal of pressure changed so views can update to reflect change
            self.pressure_to_view.emit(int(round(view_pres)))

            # Update current pressure
            self._current_bp = model_pres
        except Exception as e:
            msg = f"Errors while applying pressure: {e}. See logs for more info."
            self._logger.exception(msg)
            self.errors.emit(msg)

    def get_current_bp(self)->int:
        """ Return current pressure in model scale """
        return int(self._current_bp)


class PressureView(QObject):
    """ 
    View for user interface with pressure controller. Relevant widgets use
    a scale of 0-100% for the pressure values.
    """

    def __init__(self, controller: PressureController):
        """
        Args:
            controller (PressureController): Controller in MVC
        """
        super().__init__()
        self._controller = controller
        self._make_widgets()
        self._set_connections()
        self._stateify_widgets()

    def _make_widgets(self):
        """ Make widgets for the view """
        self.pressure_slider = QSlider(Qt.Orientation.Horizontal)
        self.pressure_slider.setMinimum(0)
        self.pressure_slider.setMaximum(100)
        self.pressure_slider.setTickPosition(QSlider.TickPosition.TicksBelow)
        self.pressure_slider.setTickInterval(10)
        self.pressure_display = SmallQLineEdit()
        self.pressure_display.setReadOnly(True)
        palette = QPalette()
        palette.setColor(QPalette.ColorRole.Base, QColor('lightGray'))
        self.pressure_display.setPalette(palette)
        self.clear_button = QPushButton("Clear")

    def _set_connections(self):
        """ 
        Connect signals and slots for view-view, view-controller, and
        controller-view.
        """
        # View signal to view slots
        self.pressure_slider.valueChanged.connect(lambda pres_val: self._mimic_slider(pres_val))
        # View signal to controller slots
        self.pressure_slider.sliderReleased.connect(lambda: self._controller.on_slider_change(self.pressure_slider.value()))
        self.clear_button.clicked.connect(self._controller.on_clear_button)
        # Controller signal to view slots
        self._controller.pressure_to_view.connect(lambda pres_val: self._update_view(pres_val))

    def _stateify_widgets(self):
        """ Initialize states of widgets """
        self.pressure_slider.setValue(0)
        self.pressure_display.clear()
        self.pressure_display.insert(f"0%")
    
    @pyqtSlot()
    def _mimic_slider(self, pres_val:int):
        """
        Update the pressure display to show the slider value (while
        user is changing the slider). Makes box red to show value inactive.
        """
        palette = QPalette()
        palette.setColor(QPalette.ColorRole.Base, QColor.fromRgb(255,204,203))
        self.pressure_display.setPalette(palette)
        self.pressure_display.clear()
        self.pressure_display.insert(f"{pres_val}%")
    
    @pyqtSlot()
    def _update_view(self, pres_val:int):
        """
        Update the slider and dispaly to show `pres_val`
        """
        self.pressure_slider.setValue(int(pres_val))
        palette = QPalette()
        palette.setColor(QPalette.ColorRole.Base, QColor('lightGray'))
        self.pressure_display.setPalette(palette)
        self.pressure_display.clear()
        self.pressure_display.insert(f"{pres_val}%")

class PressureApp(PressureView):

    def __init__(self, controller: PressureController):
        """
        Args:
            controller (PressureController): Controller in MVC
        """
        super().__init__(controller)
        self.pressure_group = QGroupBox('Pressure')
        self._make_application()

    def _make_application(self):
        layout = QGridLayout()
        layout.addWidget(self.pressure_slider,0,0,1,1)
        layout.addWidget(self.pressure_display,0,1,1,1)
        layout.addWidget(self.clear_button,1,0,1,2)
        self.pressure_group.setLayout(layout)


if __name__ == "__main__":
    arduino = serial.Serial(str('com3'), 9600,timeout=5)
    app = QApplication(sys.argv)
    model = PressureModel(arduino)
    controller = PressureController(model)
    win = PressureApp(controller).pressure_group
    win.show()
    sys.exit(app.exec())



import sys
import win32com.client
from PyQt6.QtCore import Qt, QTimer, pyqtSignal
from PyQt6.QtWidgets import (QApplication,
                            QLabel,
                            QPushButton,
                            QVBoxLayout,
                            QHBoxLayout,
                            QGroupBox,
                            QComboBox,
                            QLineEdit,)
from PyQt6.QtGui import QPalette, QColor
from src.Qt_utils.gui_objects import QHLine
from src.ZEN_interface.ZENInterface import ZEN
from src.thread_manager.thread_manager import aQThreader, aQWorker
from src.miscellaneous.standard_logger import StandardLogger as logr
import src.miscellaneous.validify as val

class ZenGroup(QGroupBox):
    obj_changed = pyqtSignal([float])
    opto_changed = pyqtSignal([float])
    ref_changed = pyqtSignal([str])
    def __init__(self, parent=None):
        super().__init__('Zeiss Control',parent)
        self.logger = logr(__name__)
        self.timer = QTimer()
        self.timeout = 1000
        self.timer.timeout.connect(self.timer_position_updates)
        self.timer.start(self.timeout)
        self.zen = ZEN()
        self._make_widgets()
        self._init_states()

    def _make_widgets(self):
        # Focus widgets
        foc_disp_label = QLabel('Focus (µm):', parent=self)
        self.foc_disp = QLineEdit(parent=self)
        self.foc_disp.setAlignment(Qt.AlignmentFlag.AlignRight)
        self.foc_disp.setReadOnly(True)
        palette = QPalette()
        palette.setColor(QPalette.ColorRole.Base, QColor('lightGray'))
        self.foc_disp.setPalette(palette)
        foc_inc_label = QLabel('Increment (µm):', parent=self)
        self.foc_inc = QLineEdit(parent=self)
        btn_minus = QPushButton('-', parent=self)
        btn_minus.clicked.connect(self.minus_focus)
        btn_plus = QPushButton('+', parent=self)
        btn_plus.clicked.connect(self.plus_focus)
        h_layout1 = QHBoxLayout()
        h_layout2 = QHBoxLayout()
        h_layout3 = QHBoxLayout()
        v_layout = QVBoxLayout()
        h_layout1.addWidget(foc_disp_label)
        h_layout1.addWidget(self.foc_disp)
        h_layout2.addWidget(btn_minus)
        h_layout2.addWidget(btn_plus)
        h_layout3.addWidget(foc_inc_label)
        h_layout3.addWidget(self.foc_inc)
        v_layout.addLayout(h_layout1)
        v_layout.addLayout(h_layout2)
        v_layout.addLayout(h_layout3)

        # Separator between widgets
        h_separator = QHLine()
        v_layout.addWidget(h_separator)

        # Objective widgets
        obj_selector_label = QLabel('Objective:', parent=self)
        self.obj_selector = QComboBox(parent=self)
        self.obj_selector.setPlaceholderText('Magnification')
        self.obj_selector.currentTextChanged.connect(self.change_objective)
        h_layout4 = QHBoxLayout()
        h_layout4.addWidget(obj_selector_label)
        h_layout4.addWidget(self.obj_selector)
        v_layout.addLayout(h_layout4)

        # Optovar widgets
        opto_selector_label = QLabel('Optovar:', parent=self)
        self.opto_selector = QComboBox(parent=self)
        self.opto_selector.setPlaceholderText('Magnification')
        self.opto_selector.currentTextChanged.connect(self.change_optovar)
        h_layout5 = QHBoxLayout()
        h_layout5.addWidget(opto_selector_label)
        h_layout5.addWidget(self.opto_selector)
        v_layout.addLayout(h_layout5)

        # Reflector widgets
        ref_selector_label = QLabel('Reflector:', parent=self)
        self.ref_selector = QComboBox(parent=self)
        self.ref_selector.setPlaceholderText('Name')
        self.ref_selector.currentTextChanged.connect(self.change_reflector)
        h_layout6 = QHBoxLayout()
        h_layout6.addWidget(ref_selector_label)
        h_layout6.addWidget(self.ref_selector)
        v_layout.addLayout(h_layout6)
        self.setLayout(v_layout)
    
    def _init_states(self):
        self.foc_inc.insert(str(10))
        self._populate_obj_box()
        self._populate_opto_box()
        self._populate_ref_box()

    def timer_position_updates(self):
        ''' Updates the zeiss component positoins every timer increment'''
        self.display_focus()
        self.display_objective()
        self.display_optovar()
        self.display_reflector()

    def plus_focus(self):
        ''' Moves focus in positive direction '''
        try:
            val = self.increment_to_num()
        except ValueError as e:
            self.logger.exception('Request invalid focus increment')
        else:
            # Unthreaded
            try: 
                self.zen.goto_focus_rel_um(val)
                self.display_focus()
            except ValueError:
                self.logger.exception('Error while attempting to increment focus')
            # # Threaded but gets error likely from trying to thread COM
            # self.worker = aQWorker(self.zen.goto_focus_rel_um, val)
            # self.worker.finished.connect(self.display_focus)
            # self.threaded_func = aQThreader(self.worker)
            # self.threaded_func.start()

        
    def minus_focus(self):
        ''' Moves focus in negative direction '''
        try:
            val = -self.increment_to_num()
        except ValueError as e:
            self.logger.exception('Request invalid focus increment')
        else:
            # Unthreaded 
            try: 
                self.zen.goto_focus_rel_um(val)
                self.display_focus()
            except ValueError:
                self.logger.exception('Error while attempting to increment focus')
            # # Threaded but gets error likely from trying to thread COM
            # self.worker = aQWorker(self.zen.goto_focus_rel_um, val)
            # self.worker.finished.connect(self.display_focus)
            # self.threaded_func = aQThreader(self.worker)
            # self.threaded_func.start()

    def increment_to_num(self):
        ''' Converts string in increment line edit to a number '''
        inc_val = self.foc_inc.text()
        if val.is_valid_number(inc_val):
            return float(inc_val)
        else:
            raise ValueError(f'Focus increment must be a valid number. {inc_val} is invalid.')

    def parse_combobox(self, box_name:str):
        '''
        Parse the desired combo box to get its position and secondary info.

        Arugments:
            box_name (str): Name of combo box. Must be in ['objective',
                'optovar', 'reflector']

        Return:
            tuple of (position (int), secondary info). Secondary info is
                magnification (float) for 'objective' and 'optovar', and it is
                the name (str) for 'reflector'
        '''
        # Validate
        if box_name not in ['objective', 'optovar', 'reflector']:
            raise KeyError(f"Request combobox name to parse: {box_name}. Name must be in ['objective', 'optovar', 'reflector']")
        # Get info
        if box_name == 'objective':
            # Objective box is "{position}: {magnification}x", so split on ":" and strip "x"
            obj_combobox = self.obj_selector.currentText()
            pos = int(obj_combobox.split(':')[0])
            mag = float(obj_combobox.split(':')[1].strip().strip('x'))
            ret = (pos, mag)
        if box_name == 'optovar':
            # optovar box is "{position}: {magnification}x", so split on ":" and strip "x"
            opto_combobox = self.opto_selector.currentText()
            pos = int(opto_combobox.split(':')[0])
            mag = float(opto_combobox.split(':')[1].strip().strip('x'))
            ret = (pos, mag)
        if box_name == 'reflector':
            # reflector box is "{position}: {name}", so split on ":"
            ref_combobox = self.ref_selector.currentText()
            # Selected refelector string is position: name, so get pos by split on :
            pos = int(ref_combobox.split(':')[0])
            name = str(ref_combobox.split(':')[1].strip())
            ret = (pos, name)
        return ret

    def display_focus(self):
        ''' Updates the line edit with the focus height '''
        pos = self.zen.get_focus_um()
        foc_text = self.foc_disp.text()
        if str(pos) != foc_text:
            self.foc_disp.clear()
            self.foc_disp.insert(str(pos))

    def display_objective(self):
        ''' Updates the objective combobox with the current objective '''
        # Get objective pos
        obj_pos = self.zen.get_obj_info("position")
        # Get currently displayed position
        combo_pos, combo_mag = self.parse_combobox('objective')
        # Update position if changed
        if obj_pos != combo_pos:
            # Set new combobox value
            if obj_pos in list(self.zen.objectives['position']):
                obj_mag = self.zen.objectives.at[obj_pos, 'magnification']
                new_combo_str = f"{obj_pos}: {obj_mag}x"
                self.obj_selector.setCurrentText(new_combo_str)
            # Objective pos not listed in objectives so raise key erros
            else:
                self.logger.error(f"Objectives dataframe doesn't have position: {obj_pos}. Objectives {self.zen.objectives}")
                raise KeyError(f"Objectives dataframe doesn't have position: {obj_pos}")


    def display_optovar(self):
        ''' Updates the optovar combobox with the current optovar '''
        # Get optovar pos
        opto_pos = self.zen.get_opto_info("position")
        # Get currently displayed position
        combo_pos, combo_mag = self.parse_combobox('optovar')
        # Update position if changed
        if opto_pos != combo_pos:
            # Set new combobox value
            if opto_pos in list(self.zen.optovars['position']):
                opto_mag = self.zen.optovars.at[opto_pos, 'magnification']
                new_combo_str = f"{opto_pos}: {opto_mag}x"
                self.opto_selector.setCurrentText(new_combo_str)
            # Wasn't listed optovars so raise error
            else:
                self.logger.error(f"Optovars dataframe doesn't have position: {opto_pos}. Optovars {self.zen.optovars}")
                raise KeyError(f"Optovars dataframe doesn't have position: {opto_pos}")

    def display_reflector(self):
        ''' Updates the reflector combobox with the current reflector '''
        # Get reflector pos
        ref_pos = self.zen.get_ref_info("position")
        # Get currently displayed position
        combo_pos, combo_name = self.parse_combobox('reflector')
        # Update position if changed
        if ref_pos != combo_pos:
            # If already in the listed optovars (had a valid name) then don't ask zen for the mag (to prevent zen block eventloop)
            if ref_pos in list(self.zen.reflectors['position']):
                ref_name = self.zen.reflectors.at[ref_pos, 'name']
                new_combo_str = f"{ref_pos}: {ref_name}"
                self.ref_selector.setCurrentText(new_combo_str)
            # Wasn't listed in reflectors so raise error
            else:
                self.logger.error(f"Reflectors dataframe doesn't have position: {ref_pos}. reflectors {self.zen.reflectors}")
                raise KeyError(f"Reflectors dataframe doesn't have position: {ref_pos}")

    def change_objective(self):
        ''' Change the objective to the one listed in the combo box '''
        # Get position and magnification from combobox
        try:
            selected_pos, selected_mag = self.parse_combobox('objective')
            self.zen.goto_obj_pos(selected_pos)
            self.obj_changed.emit(selected_mag)
        except:
            self.logger.exception('Error encountered while changing objective position')
            raise
    
    def change_optovar(self):
        ''' Change the optovar to the one listed in the combo box '''
        # Get position and magnification from combobox
        try:
            selected_pos, selected_mag = self.parse_combobox('optovar')
            self.zen.goto_opto_pos(selected_pos)
            self.opto_changed.emit(selected_mag)
        except:
            self.logger.exception('Error encountered while changing optovar position')
            raise

    def change_reflector(self):
        ''' Change the reflector to the one listed in the combo box '''
        # Get position and name from combobox
        try:
            selected_pos, selected_name = self.parse_combobox('reflector')
            self.zen.goto_ref_pos(selected_pos)
            self.ref_changed.emit(selected_name)
        except:
            self.logger.exception('Error encountered while changing reflector position')
            raise

    def _populate_obj_box(self):
        """
        Populate the objective selector combo box with active objectives. Also,
        set the combo box to the current position if it is an active objective.
        """
        # List of values to set in combo box
        combo_vals = []
        # Add all active objectives
        positions = list(self.zen.objectives.index.values)
        mags = list(self.zen.objectives['magnification'])
        combo_vals = [f"{positions[k]}: {mags[k]}x" for k in range(len(positions))]
        self.obj_selector.insertItems(0, combo_vals)
        # Add current positoin (if not in active objective positoin)
        cur_pos = self.zen.get_obj_info('position')
        cur_mag = self.zen.get_obj_info('magnification')
        cur_combo_str = f"{cur_pos}: {cur_mag}x"
        self.obj_selector.setCurrentText(cur_combo_str)

    def _populate_opto_box(self):
        """
        Populate the optovar selector combo box with active optovars. Also,
        set the combo box to the current position if it is an active optovar.
        """
        # List of values to set in combo box
        combo_vals = []
        # Add all active optovars
        positions = list(self.zen.optovars.index.values)
        mags = list(self.zen.optovars['magnification'])
        combo_vals = [f"{positions[k]}: {mags[k]}x" for k in range(len(positions))]
        self.opto_selector.insertItems(0, combo_vals)
        # Add current positoin (if not in active optovar positoin)
        cur_pos = self.zen.get_opto_info('position')
        cur_mag = self.zen.get_opto_info('magnification')
        cur_combo_str = f"{cur_pos}: {cur_mag}x"
        self.opto_selector.setCurrentText(cur_combo_str)

    def _populate_ref_box(self):
        """
        Populate the reflector selector combo box with active reflectors. Also,
        set the combo box to the current position if it is an active reflector.
        """
        # List of values to set in combo box
        combo_vals = []
        # Add all active reflectors
        positions = list(self.zen.reflectors.index.values)
        names = list(self.zen.reflectors['name'])
        combo_vals = [f"{positions[k]}: {names[k]}" for k in range(len(positions))]
        self.ref_selector.insertItems(0, combo_vals)
        # Add current positoin (if not in active reflector positoin)
        cur_pos = self.zen.get_ref_info('position')
        cur_name = self.zen.get_ref_info('name')
        cur_combo_str = f"{cur_pos}: {cur_name}"
        self.ref_selector.setCurrentText(cur_combo_str)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = ZenGroup()
    win.show()
    sys.exit(app.exec())


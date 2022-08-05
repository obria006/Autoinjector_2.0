import sys
import time
import win32com.client
from functools import partial
from PyQt6.QtCore import (Qt,
                          QObject,
                          QThread,
                          pyqtSignal,
                          pyqtBoundSignal)
from PyQt6.QtWidgets import (QApplication,
                            QMainWindow,
                            QLabel,
                            QPushButton,
                            QVBoxLayout,
                            QHBoxLayout,
                            QWidget,
                            QGroupBox,
                            QComboBox,
                            QLineEdit)

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
        self.zen = ZEN()
        self._make_widgets()
        self._init_states()

    def _make_widgets(self):
        # Focus widgets
        foc_disp_label = QLabel('Focus (µm):', parent=self)
        self.foc_disp = QLineEdit(parent=self)
        self.foc_disp.setAlignment(Qt.AlignmentFlag.AlignRight)
        self.foc_disp.setReadOnly(True)
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

        # Objective widgets
        obj_selector_label = QLabel('Objective:', parent=self)
        self.obj_selector = QComboBox(parent=self)
        self.obj_selector.setPlaceholderText('Magnification')
        self.obj_selector.currentTextChanged.connect(self.change_objective)
        h_layout4 = QHBoxLayout()
        h_layout4.addWidget(obj_selector_label)
        h_layout4.addWidget(self.obj_selector)
        v_layout.addLayout(h_layout4)
        self.setLayout(v_layout)

        # Optovar widgets
        opto_selector_label = QLabel('Optovar:', parent=self)
        self.opto_selector = QComboBox(parent=self)
        self.opto_selector.setPlaceholderText('Magnification')
        self.opto_selector.currentTextChanged.connect(self.change_optovar)
        h_layout5 = QHBoxLayout()
        h_layout5.addWidget(opto_selector_label)
        h_layout5.addWidget(self.opto_selector)
        v_layout.addLayout(h_layout5)
        self.setLayout(v_layout)

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
        self.foc_inc.insert(str(1000))
        self.static_z_label_update()
        self._populate_obj_box()
        self._populate_opto_box()
        self._populate_ref_box()

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
                self.static_z_label_update()
            except ValueError:
                self.logger.exception('Error while attempting to increment focus')
            # # Threaded but gets error likely from trying to thread COM
            # self.worker = aQWorker(self.zen.goto_focus_rel_um, val)
            # self.worker.finished.connect(self.static_z_label_update)
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
                self.static_z_label_update()
            except ValueError:
                self.logger.exception('Error while attempting to increment focus')
            # # Threaded but gets error likely from trying to thread COM
            # self.worker = aQWorker(self.zen.goto_focus_rel_um, val)
            # self.worker.finished.connect(self.static_z_label_update)
            # self.threaded_func = aQThreader(self.worker)
            # self.threaded_func.start()

    def increment_to_num(self):
        ''' Converts string in increment line edit to a number '''
        inc_val = self.foc_inc.text()
        if val.is_valid_number(inc_val):
            return float(inc_val)
        else:
            raise ValueError(f'Focus increment must be a valid number. {inc_val} is invalid.')

    def static_z_label_update(self):
        ''' Updates the line edit with the focus height '''
        pos = self.zen.get_focus_um()
        self.foc_disp.clear()
        self.foc_disp.insert(str(pos))

    def change_objective(self):
        ''' Change the objective to the one listed in the combo box '''
        # Get string from combo box
        selected_obj = self.obj_selector.currentText()
        # Selected objective string is position: mag, so get pos by split on :
        selected_pos = int(selected_obj.split(':')[0])
        selected_mag = float(selected_obj.split(':')[1].strip().strip('x'))
        self.zen.goto_obj_pos(selected_pos)
        self.obj_changed.emit(selected_mag)
    
    def change_optovar(self):
        ''' Change the optovar to the one listed in the combo box '''
        # Get string from combo box
        selected_opto = self.opto_selector.currentText()
        # Selected optovar string is position: mag, so get pos by split on :
        selected_pos = int(selected_opto.split(':')[0])
        selected_mag = float(selected_opto.split(':')[1].strip().strip('x'))
        self.zen.goto_opto_pos(selected_pos)
        self.opto_changed.emit(selected_mag)

    def change_reflector(self):
        ''' Change the reflector to the one listed in the combo box '''
        # Get string from combo box
        selected_ref = self.ref_selector.currentText()
        # Selected refelector string is position: name, so get pos by split on :
        selected_pos = int(selected_ref.split(':')[0])
        selected_name = str(selected_ref.split(':')[1].strip())
        self.zen.goto_ref_pos(selected_pos)
        self.ref_changed.emit(selected_name)

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
        for k in range(len(positions)):
            combo_str = f"{positions[k]}: {mags[k]}x"
            combo_vals.append(combo_str)
        self.obj_selector.insertItems(0, combo_vals)
        # Add current positoin (if not in active objective positoin)
        cur_pos = self.zen.get_obj_info('position')
        cur_mag = self.zen.get_obj_info('magnification')
        cur_combo_str = f"{cur_pos}: {cur_mag}x"
        if cur_combo_str in combo_vals:
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
        for k in range(len(positions)):
            combo_str = f"{positions[k]}: {mags[k]}x"
            combo_vals.append(combo_str)
        self.opto_selector.insertItems(0, combo_vals)
        # Add current positoin (if not in active optovar positoin)
        cur_pos = self.zen.get_opto_info('position')
        cur_mag = self.zen.get_opto_info('magnification')
        cur_combo_str = f"{cur_pos}: {cur_mag}x"
        if cur_combo_str in combo_vals:
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
        for k in range(len(positions)):
            combo_vals.append(f"{positions[k]}: {names[k]}")
        self.ref_selector.insertItems(0, combo_vals)
        # Add current positoin (if not in active reflector positoin)
        cur_pos = self.zen.get_ref_info('position')
        cur_name = self.zen.get_ref_info('name')
        cur_combo_str = f"{cur_pos}: {cur_name}"
        if cur_combo_str in combo_vals:
            self.ref_selector.setCurrentText(cur_combo_str)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = ZenGroup()
    win.show()
    sys.exit(app.exec())


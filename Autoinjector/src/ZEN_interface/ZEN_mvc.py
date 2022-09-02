"""
Interface w/ Zeiss ZEN software using model-view-controller architecture
"""
import sys
import pandas as pd
import win32com.client
from PyQt6.QtCore import Qt, QTimer, pyqtSignal, QSignalBlocker, QObject
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
from src.miscellaneous.standard_logger import StandardLogger
from src.miscellaneous import validify as val

class ModelZEN:
    """
    Model for controlling Zeiss/ZEN microscope/software.
    """

    def __init__(self):
        try:
            self.zen = win32com.client.GetActiveObject("Zeiss.Micro.Scripting.ZenWrapperLM")
        except:
            raise
        # Max and min values of focus position in ZEN
        self.focus_max_um = 10000
        self.focus_min_um = 0
        self.objectives = self._connected_objectives()
        self.optovars = self._connected_optovars()
        self.reflectors = self._connected_reflectors()

    def get_focus_um(self)->float:
        '''Returns position of focus in um (consistent w/ ZEN software)'''
        foc_ZEN_um = self.zen.Devices.Focus.ActualPosition
        return foc_ZEN_um

    def goto_focus_abs_um(self,des_foc_um:float)->None:
        '''
        Go to absoloute position of focus in um (consistent w/ ZEN software)

        Arguments:
            des_foc_um (float): ZEN focus position in um

        Returns:
            None
        '''
        if des_foc_um > self.focus_max_um or des_foc_um < self.focus_min_um:
            raise ValueError(f'Invalid goto_focus_abs_um position: {des_foc_um}um. Must be {self.focus_min_um}um to {self.focus_max_um}um.')
        # TODO add threaded move function
        self.zen.Devices.Focus.MoveTo(des_foc_um)

    def goto_focus_rel_um(self, delta_foc_um:float)->None:
        '''
        Change focus position by specified amount in um.

        Arguments:
            delta_foc_um (float): Desired displacement of focus position in um.

        Returns:
            None
        '''
        cur_foc_um = self.get_focus_um()
        des_foc_um = cur_foc_um + delta_foc_um
        if des_foc_um > self.focus_max_um or des_foc_um < self.focus_min_um:
            raise ValueError(f'Invalid goto_focus_rel_um position: {delta_foc_um}um. Current + relative ({cur_foc_um} + {delta_foc_um} = {des_foc_um}um) must be {self.focus_min_um}um to {self.focus_max_um}um.')
        self.goto_focus_abs_um(des_foc_um)

    def _connected_objectives(self,start_pos:int=1, end_pos:int=6) -> pd.DataFrame:
        '''
        Returns dataframe of objective name and magnification indexed by position in
        objective positions ranging from start_pos to end_pos.

        Arguments:
            start_pos (int): Starting position in Zeiss software to start listing objectives
            end_pos (int): Ending positoin in Zeiss software to stop listing objectives

        DataFrame: index = "position", column1 = "name", column2 = 'position', column3 = "magnification"
        '''
        if not val.is_of_types(start_pos,[int,float]):
            raise ValueError(f"Objective start position must be a number type. {type(start_pos)} is invalid.")
        if not val.is_of_types(end_pos,[int,float]):
            raise ValueError(f"Objective start position must be a number type. {type(start_pos)} is invalid.")
        obj_dict = {}
        for pos in range(start_pos, end_pos+1):
            name = self.zen.Devices.ObjectiveChanger.GetNameByPosition(pos)
            mag = self.zen.Devices.ObjectiveChanger.GetMagnificationByPosition(pos)
            obj_dict[pos] = {'name':name, 'position':pos, 'magnification':mag}
        obj_df = pd.DataFrame(obj_dict).transpose()
        return obj_df

    def obj_pos_from_info(self, key:str, val) -> int:
        """
        Return an objective's position from its info (name or magnification)

        Arguments:
            key (str): The info from which to get the position. Key must be in
                ['name', 'magnification']
            val: Value of the key for which to find the positoin

        Usage
            obj_pos_from_info('magnification',20.0)
            # returns position of 20x objective
            obj_pos_from_info('name','EC Plan-Neofluar 10x/0.30 Ph 1')
            # Returns position of 'EC Plan-Neofluar 10x/0.30 Ph 1'
        """
        # Validate args
        if key not in ['name', 'magnification']:
            raise KeyError(f"Invalid objective info key: {key}. Key must be in ['name', 'magnification']")
        positions = list(self.objectives.index[self.objectives[key]==val].values)
        if positions == []:
            raise ValueError(f"Objective {key} has no value: {val}.")
        if len(positions) > 1:
            raise ValueError(f"Objective has multiple instances of {key} = {val}.")
        return positions[0]
        

    def get_obj_info(self, key:str):
        '''
        Returns info about current objective

        Arguments:
            key (str): Info to return. key must be in ['position', 'name', 'magnification']
        
        Returns queried objective info
        '''
        # Validate correct argument
        if key not in ['position', 'name', 'magnification']:
            raise KeyError(f"Invalid objective info key: {key}. Key must be in ['position', 'name', 'magnification']")
        # Return requested info
        if key == 'position':
            return self.zen.Devices.ObjectiveChanger.ActualPosition
        if key == 'name':
            return self.zen.Devices.ObjectiveChanger.ActualPositionName
        if key == 'magnification':
            return self.zen.Devices.ObjectiveChanger.Magnification

    def goto_obj_mag(self, mag:float) -> None:
        '''
        Changes objective to the specified magnification
        
        Arugments:
            mag (float): magnification of desried objective (must be
                in objectives 'magnification' column)
        '''
        if mag not in list(self.objectives['magnification']):
            raise KeyError(f'Desired magnification {mag} not in objective magnifications.')
        des_obj_pos = self.obj_pos_from_info('magnification', mag)
        self.goto_obj_pos(des_obj_pos)

    def goto_obj_name(self, name:str) -> None:
        '''
        Changes objective to the specified name
        
        Arugments:
            name (str): Name of desried objective (must be in objectives
                'name' column)
        '''
        if name not in list(self.objectives['name']):
            raise KeyError(f'Desired name {name} not in objective names.')
        des_obj_pos = self.obj_pos_from_info('name', name)
        self.goto_obj_pos(des_obj_pos)

    def goto_obj_pos(self, pos:int) -> None:
        '''
        Change objective to specified position

        Arguments:
            pos (int): Desired objective position
        '''
        if pos not in list(self.objectives.index.values):
            raise ValueError(f'Requested invalid objective position: {pos}. No objective in position: {pos}')
        # Goto new position
        self.zen.Devices.ObjectiveChanger.TargetPosition = pos
        self.zen.Devices.ObjectiveChanger.Apply()

    def _connected_optovars(self, start_pos:int=1, end_pos:int=3)->pd.DataFrame:
        '''
        Returns dataframe of optovar name and magnification indexed by position with
        positions ranging from start_pos to end pos.

        Arguments:
            start_pos (int): Starting position in Zeiss software to start listing optovars
            end_pos (int): Ending positoin in Zeiss software to stop listing optovars


        DataFrame: index = "position", column1 = "name", column2 = "position", column3 = "magnification"
        '''
        if not val.is_of_types(start_pos,[int,float]):
            raise ValueError(f"Optovar start position must be a number type. {type(start_pos)} is invalid.")
        if not val.is_of_types(end_pos,[int,float]):
            raise ValueError(f"Optovar start position must be a number type. {type(start_pos)} is invalid.")
        opto_dict = {}
        for pos in range(start_pos, end_pos+1):
            name = self.zen.Devices.Optovar.GetNameByPosition(pos)
            mag = self.zen.Devices.Optovar.GetMagnificationByPosition(pos)
            opto_dict[pos] = {'name':name, 'position':pos, 'magnification':mag}
        opto_df = pd.DataFrame(opto_dict).transpose()
        return opto_df

    def opto_pos_from_info(self, key:str, val) -> int:
        """
        Return an optovar's position from its info (name or magnification)

        Arguments:
            key (str): The info from which to get the position. Key must be in
                ['name', 'magnification']
            val: Value of the key for which to find the position

        Usage:
            opto_pos_from_info('magnification',1.25)
            # returns position of 1.25x optovar
            opto_pos_from_info('name', '1.6x Tubelens)
            # returns position of 1.6x optovar
        """
        # Validate args
        if key not in ['name', 'magnification']:
            raise KeyError(f"Invalid optovar info key: {key}. Key must be in ['name', 'magnification']")
        positions = list(self.optovars.index[self.optovars[key]==val].values)
        if positions == []:
            raise ValueError(f"Optovar {key} has no value: {val}.")
        if len(positions) > 1:
            raise ValueError(f"Optovar has multiple instances of {key} = {val}.")
        return positions[0]
        

    def get_opto_info(self, key:str):
        '''
        Returns info about current optovar

        Arguments:
            key (str): Info to return. key must be in ['position', 'name', 'magnification']
        
        Returns queried optovar info
        '''
        # Validate correct argument
        if key not in ['position', 'name', 'magnification']:
            raise KeyError(f"Invalid optovar info key: {key}. Key must be in ['position', 'name', 'magnification']")
        # Return requested info
        if key == 'position':
            return self.zen.Devices.Optovar.ActualPosition
        if key == 'name':
            return self.zen.Devices.Optovar.ActualPositionName
        if key == 'magnification':
            return self.zen.Devices.Optovar.Magnification

    def goto_opto_mag(self, mag:float) -> None:
        '''
        Changes optovar to the specified magnification
        
        Arugments:
            mag (float): Magnification of desried optovar (must be
                in optovars 'magnification' column)
        '''
        if mag not in list(self.optovars['magnification']):
            raise KeyError(f'Desired magnification {mag} not in optovar magnifications.')
        des_opto_pos = self.opto_pos_from_info('magnification', mag)
        self.goto_opto_pos(des_opto_pos)

    def goto_opto_name(self, name:str) -> None:
        '''
        Changes optovar to the specified name
        
        Arugments:
            name (str): Name of desried optovar (must be in optovars
                'name' column)
        '''
        if name not in list(self.optovars['name']):
            raise KeyError(f'Desired name {name} not in optovar names.')
        des_opto_pos = self.opto_pos_from_info('name', name)
        self.goto_opto_pos(des_opto_pos)

    def goto_opto_pos(self, pos:int) -> None:
        '''
        Change optovar to specified position

        Arguments:
            pos (int): Desired optovar position
        '''
        if pos not in list(self.optovars.index.values):
            raise ValueError(f'Requested invalid optovar position: {pos}. No optovar in position: {pos}')
        # Goto new position
        self.zen.Devices.Optovar.TargetPosition = pos
        self.zen.Devices.Optovar.Apply()

    def _connected_reflectors(self, start_pos:int=1, end_pos:int=6)->pd.DataFrame():
        '''
        Returns dataframe of reflector name and position with positions ranging
        from start_pos to end_pos.

        Arguments:
            start_pos (int): Starting position in Zeiss software to start listing reflectors
            end_pos (int): Ending positoin in Zeiss software to stop listing reflectors

        DataFrame: index = "position", column1 = "name", column2 = "position"
        '''
        if not val.is_of_types(start_pos,[int,float]):
            raise ValueError(f"Reflector start position must be a number type. {type(start_pos)} is invalid.")
        if not val.is_of_types(end_pos,[int,float]):
            raise ValueError(f"Reflector start position must be a number type. {type(start_pos)} is invalid.")
        ref_dict = {}
        for pos in range(start_pos, end_pos+1):
            name = self.zen.Devices.Reflector.GetNameByPosition(pos)
            if name is None:
                name = f"Pos. {round(pos)}"
            ref_dict[pos] = {'name':name, 'position':pos}
        ref_df = pd.DataFrame(ref_dict).transpose()
        return ref_df

    def ref_pos_from_name(self, val:str) -> int:
        """
        Return an reflector's position from its name

        Arguments:
            val (str): Reflector name
        """
        # Validate args
        positions = list(self.reflectors.index[self.reflectors['name']==val].values)
        if positions == []:
            raise ValueError(f"Relector 'name' has no value: {val}.")
        if len(positions) > 1:
            raise ValueError(f"Relector has multiple instances of 'name' = {val}.")
        return positions[0]
        

    def get_ref_info(self, key:str):
        '''
        Returns info about current reflector

        Arguments:
            key (str): Info to return. key must be in ['position', 'name']
        
        Returns queried reflector info
        '''
        # Validate correct argument
        if key not in ['position', 'name']:
            raise KeyError(f"Invalid reflector info key: {key}. Key must be in ['position', 'name']")
        # Return requested info
        if key == 'position':
            return self.zen.Devices.Reflector.ActualPosition
        if key == 'name':
            ref_name = self.zen.Devices.Reflector.ActualPositionName
            if ref_name is None or ref_name == '':
                pos = self.zen.Devices.Reflector.ActualPosition
                ref_name = f"Pos. {round(pos)}"
            return ref_name

    def goto_ref_name(self, name:str) -> None:
        '''
        Changes reflector to the specified name
        
        Arugments:
            name (str): Name of desried reflector (must be in reflectors
                'name' column)
        '''
        if name not in list(self.reflectors['name']):
            raise KeyError(f'Desired name {name} not in reflector names.')
        des_ref_pos = self.ref_pos_from_name(name)
        self.goto_ref_pos(des_ref_pos)

    def goto_ref_pos(self, pos:int) -> None:
        '''
        Change reflector to specified position

        Arguments:
            pos (int): Desired reflector position
        '''
        if pos not in list(self.reflectors.index.values):
            raise ValueError(f'Requested invalid reflector position: {pos}. No reflector in position: {pos}')
        # Goto new position
        self.zen.Devices.Reflector.TargetPosition = pos
        self.zen.Devices.Reflector.Apply()


class ControllerZEN(QObject):
    """
    Controller for zeiss/ZEN interface within the MVC architecture. Uses a
    timer to periodically sample the position of the microscope components
    and send signals to the view if these positions change. Recieves signals
    from the view and acts on them to change to postion of the microscope to
    reflect the user input.
    """
    
    ex_foc_position_changed = pyqtSignal(str)
    ex_obj_position_changed = pyqtSignal(str)
    ex_opto_position_changed = pyqtSignal(str)
    ex_ref_position_changed = pyqtSignal(str)

    focus_changed = pyqtSignal([str])
    obj_changed = pyqtSignal([float])
    opto_changed = pyqtSignal([float])
    ref_changed = pyqtSignal([str])

    def __init__(self, model:ModelZEN):
        """
        Args:
            model: Model for zeiss/zen microscope control in MVC architecture
        """
        super().__init__()
        self.logger = StandardLogger(__name__)
        self._model = model
        self._init_states()

        # Timer to periodically sample microscope positions
        self._timer = QTimer()
        TIMEOUT_MS = 1000
        self._timer.timeout.connect(self._timer_position_updates)
        self._timer.start(TIMEOUT_MS)

    def _init_states(self):
        """
        Initialize states of hardware positions. These states will be used to
        reference against the periodically sampeled hardware positions to
        determine if external position change occurred.
        """
        self._focus_position = str(self._model.get_focus_um())
        self._obj_position = self._model.get_obj_info("position")
        self._opto_position = self._model.get_opto_info("position")
        self._ref_position = self._model.get_ref_info("position")

    def _timer_position_updates(self):
        """
        Periodically samples the hardware position and emits signal if their
        positions change.
        """
        self.sample_focus_position()
        self.sample_objective_position()
        self.sample_optovar_position()
        self.sample_reflector_position()

    def objective_view_change(self, obj_combobox:str):
        """
        Handles when objective changes in the view. Commands change of
        objective position.

        Args:
            obj_combobox (str): String as '{pos}: {mag}x' denoting which
                objective position and magnitude to change to.
        """
        # Update internal attribute of objective positoin
        obj_pos, obj_mag = self.parse_combobox('objective', obj_combobox)
        self._obj_position = obj_pos
        # Make the commanded objective move
        self.change_objective(obj_pos, obj_mag)

    def change_objective(self, des_pos:int, des_mag:float):
        '''
        Change objective position to desired position. Emits change in
        objective magnification.
        
        Args:
            des_pos (int): desired objective position
            des_mag (float): magnification of objective at position
        '''
        # Get position and magnification from combobox
        try:
            self._model.goto_obj_pos(des_pos)
            self.obj_changed.emit(des_mag)
        except:
            self.logger.exception('Error encountered while changing objective position')
            raise

    def optovar_view_change(self, opto_combobox:str):
        """
        Handles when optovar changes in the view. Commands change of
        optovar position.

        Args:
            opto_combobox (str): String as '{pos}: {mag}x' denoting which
                optovar position and magnitude to change to.
        """
        # Update internal attribute of optovar position
        opto_pos, opto_mag = self.parse_combobox('optovar', opto_combobox)
        self._opto_position = opto_pos
        # Make the commanded optovar move
        self.change_optovar(opto_pos, opto_mag)
    
    def change_optovar(self, des_pos:int, des_mag:float):
        '''
        Change optovar position to desired position. Emits change in
        optovar magnification.
        
        Args:
            des_pos (int): desired optovar position
            des_mag (float): magnification of optovar at position
        '''
        # Get position and magnification from combobox
        try:
            self._model.goto_opto_pos(des_pos)
            self.opto_changed.emit(des_mag)
        except:
            self.logger.exception('Error encountered while changing optovar position')
            raise

    def reflector_view_change(self, ref_combobox:str):
        """
        Handles when relector changes in the view. Commands change of
        relector position.

        Args:
            ref_combobox (str): String as '{pos}: {mag}x' denoting which
                relector position and magnitude to change to.
        """
        # Update internal attribute of reflector position
        ref_pos, ref_name = self.parse_combobox('reflector', ref_combobox)
        self._ref_position = ref_pos
        # Make commanded reflector omve
        self.change_reflector(ref_pos, ref_name)

    def change_reflector(self, des_pos:int, des_name:str):
        '''
        Change reflector position to desired position. Emits change in
        reflector name.
        
        Args:
            des_pos (int): desired reflector position
            des_mag (float): name of reflector at position
        '''
        # Get position and name from combobox
        try:
            self._model.goto_ref_pos(des_pos)
            self.ref_changed.emit(des_name)
        except:
            self.logger.exception('Error encountered while changing reflector position')
            raise

    def focus_view_change(self, rel_focus:float):
        """
        Move focus position relative to current position.

        Args:
            rel_focus (float): Relative positoin change
        """
        self._model.goto_focus_rel_um(rel_focus)
        self.sample_focus_position()

    def get_focus_pos(self)->str:
        """
        Return current position of focus

        Returns:
            string of current focus position
        """
        pos = self._model.get_focus_um()
        return str(pos)
        
    def parse_combobox(self, box_name:str, box_val:str):
        '''
        Parse the desired combo box to get its position and secondary info.

        Arugments:
            box_name (str): Name of combo box. Must be in ['objective',
                'optovar', 'reflector']
            box_val (str): String to parse from combobox

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
            obj_combobox = box_val
            pos = int(obj_combobox.split(':')[0])
            mag = float(obj_combobox.split(':')[1].strip().strip('x'))
            ret = (pos, mag)
        if box_name == 'optovar':
            # optovar box is "{position}: {magnification}x", so split on ":" and strip "x"
            opto_combobox = box_val
            pos = int(opto_combobox.split(':')[0])
            mag = float(opto_combobox.split(':')[1].strip().strip('x'))
            ret = (pos, mag)
        if box_name == 'reflector':
            # reflector box is "{position}: {name}", so split on ":"
            ref_combobox = box_val
            # Selected refelector string is position: name, so get pos by split on :
            pos = int(ref_combobox.split(':')[0])
            name = str(ref_combobox.split(':')[1].strip())
            ret = (pos, name)
        return ret

    def sample_focus_position(self):
        """
        Compares microscope focus position against internal position and emits
        a signal if the don't match and updates internal position
        """
        pos = str(self._model.get_focus_um())
        if pos != self._focus_position:
            self._focus_position = pos
            self.ex_foc_position_changed.emit(pos)

    def sample_objective_position(self):
        """
        Compares microscope objective position against internal position and emits
        a signal if the don't match and updates internal position
        """
        # Get objective pos
        obj_pos = self._model.get_obj_info("position")
        # Update position if changed. Sometimes while switch says pos is 0 (impossible, so dont handle 0 pos)
        if obj_pos != self._obj_position and obj_pos != 0:
            # Set new combobox value
            if obj_pos in list(self._model.objectives['position']):
                self._obj_position = obj_pos
                obj_mag = self._model.objectives.at[obj_pos, 'magnification']
                new_combo_str = f"{obj_pos}: {obj_mag}x"
                self.ex_obj_position_changed.emit(new_combo_str)
            # Objective pos not listed in objectives so raise key erros
            else:
                self.logger.error(f"Objectives dataframe doesn't have position: {obj_pos}. Objectives {self._model.objectives}")
                raise KeyError(f"Objectives dataframe doesn't have position: {obj_pos}")

    def sample_optovar_position(self):
        """
        Compares microscope optovar position against internal position and emits
        a signal if the don't match and updates internal position
        """
        # Get optovar pos
        opto_pos = self._model.get_opto_info("position")
        # Update position if changed. Sometimes while switch says pos is 0 (impossible, so dont handle 0 pos)
        if opto_pos != self._opto_position and opto_pos != 0:
            # Set new combobox value
            if opto_pos in list(self._model.optovars['position']):
                self._opto_position = opto_pos
                opto_mag = self._model.optovars.at[opto_pos, 'magnification']
                new_combo_str = f"{opto_pos}: {opto_mag}x"
                self.ex_opto_position_changed.emit(new_combo_str)
            # Wasn't listed optovars so raise error
            else:
                self.logger.error(f"Optovars dataframe doesn't have position: {opto_pos}. Optovars {self._model.optovars}")
                raise KeyError(f"Optovars dataframe doesn't have position: {opto_pos}")

    def sample_reflector_position(self):
        """
        Compares microscope reflector position against internal position and emits
        a signal if the don't match and updates internal position
        """
        # Get reflector pos
        ref_pos = self._model.get_ref_info("position")
        # Update position if changed. Sometimes while switch says pos is 0 (impossible, so dont handle 0 pos)
        if ref_pos != self._ref_position and ref_pos != 0:
            # If already in the listed optovars (had a valid name) then don't ask zen for the mag (to prevent zen block eventloop)
            if ref_pos in list(self._model.reflectors['position']):
                self._ref_position = ref_pos
                ref_name = self._model.reflectors.at[ref_pos, 'name']
                new_combo_str = f"{ref_pos}: {ref_name}"
                self.ex_ref_position_changed.emit(new_combo_str)
            # Wasn't listed in reflectors so raise error
            else:
                self.logger.error(f"Reflectors dataframe doesn't have position: {ref_pos}. reflectors {self._model.reflectors}")
                raise KeyError(f"Reflectors dataframe doesn't have position: {ref_pos}")


    def objectives_information(self):
        """
        Return list of connected objectives and the current objective

        Returns:
            combo_vals (list): connected objectives as ['{pos1}: {mag1}x', ...]
            cur_combo_str (str): current objective as '{pos}: {mag}'
        """
        # List of values to set in combo box
        combo_vals = []
        # Add all active objectives
        positions = list(self._model.objectives.index.values)
        mags = list(self._model.objectives['magnification'])
        combo_vals = [f"{positions[k]}: {mags[k]}x" for k in range(len(positions))]
        # Add current positoin (if not in active objective positoin)
        cur_pos = self._model.get_obj_info('position')
        cur_mag = self._model.get_obj_info('magnification')
        cur_combo_str = f"{cur_pos}: {cur_mag}x"

        return combo_vals, cur_combo_str

    def optovars_information(self):
        """
        Return list of connected optovars and the current optovar

        Returns:
            combo_vals (list): connected optovars as ['{pos1}: {mag1}x', ...]
            cur_combo_str (str): current optovar as '{pos}: {mag}'
        """
        # List of values to set in combo box
        combo_vals = []
        # Add all active optovars
        positions = list(self._model.optovars.index.values)
        mags = list(self._model.optovars['magnification'])
        combo_vals = [f"{positions[k]}: {mags[k]}x" for k in range(len(positions))]
        # Add current positoin (if not in active optovar positoin)
        cur_pos = self._model.get_opto_info('position')
        cur_mag = self._model.get_opto_info('magnification')
        cur_combo_str = f"{cur_pos}: {cur_mag}x"

        return combo_vals, cur_combo_str

    def reflectors_information(self):
        """
        Return list of connected reflectors and the current reflector

        Returns:
            combo_vals (list): connected reflectors as ['{pos1}: {name1}', ...]
            cur_combo_str (str): current reflector as '{pos}: {name}'
        """
        # List of values to set in combo box
        combo_vals = []
        # Add all active reflectors
        positions = list(self._model.reflectors.index.values)
        names = list(self._model.reflectors['name'])
        combo_vals = [f"{positions[k]}: {names[k]}" for k in range(len(positions))]
        # Add current positoin (if not in active reflector positoin)
        cur_pos = self._model.get_ref_info('position')
        cur_name = self._model.get_ref_info('name')
        cur_combo_str = f"{cur_pos}: {cur_name}"

        return combo_vals, cur_combo_str

class ViewZEN(QObject):
    """
    Widgets for creating a Zeiss/ZEN GUI interface. Widgets are already
    connected to the various controller signals and slots. Therefore, multiple
    instances of this view can be instantiated and used to create different
    ZEN interfaces while sharing data (assuming the same controller is used
    with each instantiated view).

    Signals:
        obj_selector.currentTextChanged(str): Text from objective selector
            combobox when its text changes (user selects new objective)
        opto_selector.currentTextChanged(str): Text from optovar selector
            combobox when its text changes (user selects new optovar)
        ref_selector.currentTextChanged(str): Text from reflector selector
            combobox when its text changes (user selects new reflector)
        change_focus_request(float): Desired relative move of focus
            relative to current position
    """

    # Emitted when change in focus position requested in the view
    change_focus_request = pyqtSignal(float)
    
    def __init__(self, controller:ControllerZEN):
        """
        Args:
            controller: zeiss controller for mvc architecture
            parent: parent of object
        """
        super().__init__()
        self.logger = StandardLogger(__name__)
        self._controller = controller

        self._make_widgets()
        self._set_connections()
        self._stateify_widgets()

    def _make_widgets(self):
        """
        Create widgets to comprise the available user interface components for
        the zeiss/ZEN view.
        """
        # Display focus position
        self.foc_disp = QLineEdit()
        self.foc_disp.setAlignment(Qt.AlignmentFlag.AlignRight)
        self.foc_disp.setReadOnly(True)
        palette = QPalette()
        palette.setColor(QPalette.ColorRole.Base, QColor('lightGray'))
        self.foc_disp.setPalette(palette)

        # Change in focus increment
        self.foc_inc = QLineEdit()

        # Change focus buttons
        self.btn_minus = QPushButton('-')
        self.btn_plus = QPushButton('+')

        # Objective selector combobox
        self.obj_selector = QComboBox()
        self.obj_selector.setPlaceholderText('Magnification')

        # Optovar selector combobox
        self.opto_selector = QComboBox()
        self.opto_selector.setPlaceholderText('Magnification')

        # Reflector selector combobox
        self.ref_selector = QComboBox()
        self.ref_selector.setPlaceholderText('Name')
    
    def _stateify_widgets(self):
        """
        Set initial states for the widgets using info from the controller
        """
        # Initialize the display of the inital focus position
        pos = self._controller.get_focus_pos()
        self.update_focus_display(pos)

        # Set the inital um increment for changing focus position
        self.foc_inc.insert(str(10))

        # Initialize availble combobox entries and inital objective position
        obj_vals, cur_obj = self._controller.objectives_information()
        self.obj_selector.insertItems(0, obj_vals)
        self.update_combobox(self.obj_selector, cur_obj)

        # Initialize available combobox entries and intial optovar position
        opto_vals, cur_opto = self._controller.optovars_information()
        self.opto_selector.insertItems(0, opto_vals)
        self.update_combobox(self.opto_selector, cur_opto)

        # Initialize available combobox entries and intial reflector position
        ref_vals, cur_ref = self._controller.reflectors_information()
        self.ref_selector.insertItems(0, ref_vals)
        self.update_combobox(self.ref_selector, cur_ref)


    def _set_connections(self):
        """
        Set the various view-view and view-controller connections
        """
        # Set view's signals' connections with view slots
        self.btn_minus.clicked.connect(self.minus_focus)
        self.btn_plus.clicked.connect(self.plus_focus)
        # Set view's signals' connection with controller slots.
        self.obj_selector.currentTextChanged.connect(self._controller.objective_view_change)
        self.opto_selector.currentTextChanged.connect(self._controller.optovar_view_change)
        self.ref_selector.currentTextChanged.connect(self._controller.reflector_view_change)
        self.change_focus_request.connect(self._controller.focus_view_change)
        # Set controller's signals' connection with view slots (ie updates the view
        # when microscope config changes w/o action from the view.)
        self._controller.ex_foc_position_changed.connect(lambda foc_pos: self.update_focus_display(foc_pos))
        self._controller.ex_obj_position_changed.connect(lambda combo_str: self.update_combobox(self.obj_selector, combo_str))
        self._controller.ex_opto_position_changed.connect(lambda combo_str: self.update_combobox(self.opto_selector, combo_str))
        self._controller.ex_ref_position_changed.connect(lambda combo_str: self.update_combobox(self.ref_selector, combo_str))

    def update_combobox(self, combo_box:QComboBox, combo_str:str):
        """
        Sets the view's QComboBox text while blocking signals (so not signal
        emitted when text is changed)

        Args:
            combo_box (QComboBox): Combobox whose text to modify
            combo_str (str): Value to set combobox to
        """
        # Block signal while setting objective so not to call connection
        tmp_block = QSignalBlocker(combo_box)
        combo_box.setCurrentText(combo_str)
        tmp_block.unblock()

    def update_focus_display(self, pos:str):
        """
        Update the view's focus display line entry to show the focus positoin

        Args:
            pos (str): Position of the focus
        """
        self.foc_disp.clear()
        self.foc_disp.insert(str(pos))

    def plus_focus(self):
        '''
        Handles when user clicks `+` button. Emits signal of desired focus
        change using the view's focus increment box. Logs exception for invalid
        increment number 
        '''
        try:
            val = self._increment_to_num()
        except ValueError as e:
            self.logger.exception('Request invalid focus increment')
        else:
            # Unthreaded
            try:
                self.change_focus_request.emit(val) 
            except ValueError:
                self.logger.exception('Error while attempting to increment focus')

        
    def minus_focus(self):
        '''
        Handles when user clicks `-` button. Emits signal of desired focus
        change using the view's focus increment box. Logs exception for invalid
        increment number 
        '''
        try:
            val = -self._increment_to_num()
        except ValueError as e:
            self.logger.exception('Request invalid focus increment')
        else:
            # Unthreaded 
            try:
                self.change_focus_request.emit(val)  
            except ValueError:
                self.logger.exception('Error while attempting to increment focus')

    def _increment_to_num(self):
        '''
        Converts string in increment line edit to a number. Raises error for
        invalid increment number.
        '''
        inc_val = self.foc_inc.text()
        if val.is_valid_number(inc_val):
            return float(inc_val)
        else:
            raise ValueError(f'Focus increment must be a valid number. {inc_val} is invalid.')


class ViewZENComplete(QGroupBox):
    """
    A full blown application by using the `ViewZEN` widgets to make a user
    interface.
    """
    
    def __init__(self, view: ViewZEN, parent=None):
        """
        Args:
            view: ziess view object with pre-initialized/connected widgets
            parent: where to place the user interface
        """
        super().__init__("Zeiss Control")
        self._view = view
        self._make_application()


    def _make_application(self):
        """
        Make widgets to support the `ViewZEN` and layout the widgets in an
        window.
        """
        # Focus widgets
        foc_disp_label = QLabel('Focus (µm):')
        foc_inc_label = QLabel('Increment (µm):')
        h_layout1 = QHBoxLayout()
        h_layout2 = QHBoxLayout()
        h_layout3 = QHBoxLayout()
        v_layout = QVBoxLayout()
        h_layout1.addWidget(foc_disp_label)
        h_layout1.addWidget(self._view.foc_disp)
        h_layout2.addWidget(self._view.btn_minus)
        h_layout2.addWidget(self._view.btn_plus)
        h_layout3.addWidget(foc_inc_label)
        h_layout3.addWidget(self._view.foc_inc)
        v_layout.addLayout(h_layout1)
        v_layout.addLayout(h_layout2)
        v_layout.addLayout(h_layout3)

        # Separator between widgets
        h_separator = QHLine()
        v_layout.addWidget(h_separator)

        # Objective widgets
        obj_selector_label = QLabel('Objective:')        
        h_layout4 = QHBoxLayout()
        h_layout4.addWidget(obj_selector_label)
        h_layout4.addWidget(self._view.obj_selector)
        v_layout.addLayout(h_layout4)

        # Optovar widgets
        opto_selector_label = QLabel('Optovar:')       
        h_layout5 = QHBoxLayout()
        h_layout5.addWidget(opto_selector_label)
        h_layout5.addWidget(self._view.opto_selector)
        v_layout.addLayout(h_layout5)

        # Reflector widgets
        ref_selector_label = QLabel('Reflector:')        
        h_layout6 = QHBoxLayout()
        h_layout6.addWidget(ref_selector_label)
        h_layout6.addWidget(self._view.ref_selector)
        v_layout.addLayout(h_layout6)

        # Create groupbox
        self.setLayout(v_layout)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    model = ModelZEN()
    controller = ControllerZEN(model)
    view = ViewZEN(controller)
    win = ViewZENComplete(view)
    win.show()
    sys.exit(app.exec())
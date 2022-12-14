"""
Interface w/ Zeiss ZEN software using model-view-controller architecture
"""
import sys
import pandas as pd
import win32com.client
from PyQt6.QtCore import Qt, QTimer, pyqtSignal, QSignalBlocker, QObject, pyqtSlot
from PyQt6.QtWidgets import (QApplication,
                            QLabel,
                            QPushButton,
                            QVBoxLayout,
                            QHBoxLayout,
                            QGroupBox,
                            QComboBox,
                            QLineEdit,
                            QSlider,
                            QRadioButton,
                            QFormLayout)
from PyQt6.QtGui import QPalette, QColor
from src.GUI_utils.gui_objects import QHLine
from src.miscellaneous.standard_logger import StandardLogger
from src.miscellaneous import validify as val
from src.miscellaneous.utils import str_to_bool, df_compare_to_dict

class ModelZEN(QObject):
    """
    Model for controlling Zeiss/ZEN microscope/software. Uses a timer to
    periodically sample the position of the microscope components and send
    signals to the view if these positions change.

    Signals:
        foc_position_changed: emitted when periodically sampling reveals the
            focus position has changed.
        obj_position_changed: emitted when periodically sampling reveals the
            objective position has changed.
        opto_position_changed: emitted when periodically sampling reveals the
            optovar position has changed.
        ref_position_changed: emitted when periodically sampling reveals the
            reflector position has changed.
    """
    foc_position_changed = pyqtSignal(float)
    obj_position_changed = pyqtSignal()
    opto_position_changed = pyqtSignal()
    ref_position_changed = pyqtSignal()
    lamp_intensity_changed = pyqtSignal(float)
    lamp_shutter_changed = pyqtSignal(str)
    led_changed = pyqtSignal(dict)

    def __init__(self):
        super().__init__()
        try:
            self._zen = win32com.client.GetActiveObject("Zeiss.Micro.Scripting.ZenWrapperLM")
        except:
            raise
        # Max and min values of focus position in ZEN
        self.focus_max_um = 10000
        self.focus_min_um = 0
        # Dataframe of objectives, optovars, and reflectors of microscope
        self.objectives = self._connected_objectives()
        self.optovars = self._connected_optovars()
        self.reflectors = self._connected_reflectors()
        self.leds = self._connected_leds()
        # Initial starting positions for hardware
        self._focus_position = self.get_focus_um()
        self._objective_position = self.get_obj_info('position')
        self._optovar_position = self.get_opto_info('position')
        self._reflector_position = self.get_ref_info('position')
        self._lamp_intensity = self.get_lamp_intensity()
        self._shutter_position = self.get_shutter_position()
        # Timer to periodically sample microscope positions
        self._timer = QTimer()
        TIMEOUT_MS = 500
        self._timer.timeout.connect(self._sample_hardware)
        self._timer.start(TIMEOUT_MS)


    def _sample_hardware(self):
        """
        Periodically samples the hardware position and emits signal if their
        positions change.
        """
        self._sample_focus_position()
        self._sample_objective_position()
        self._sample_optovar_position()
        self._sample_reflector_position()
        self._sample_lamp_intensity()
        self._sample_hw_setting()

    def _sample_focus_position(self):
        """
        Compares microscope focus position against internal position and emits
        a signal if the don't match and updates internal position
        """
        pos = self.get_focus_um()
        if pos != self._focus_position:
            self._focus_position = pos
            self.foc_position_changed.emit(self._focus_position)

    def _sample_objective_position(self):
        """
        Compares microscope objective position against internal position and emits
        a signal if the don't match and updates internal position
        """
        # Get objective pos
        obj_pos = self.get_obj_info("position")
        # Update position if changed. Sometimes while switch says pos is 0 (impossible, so dont handle 0 pos)
        if obj_pos != self._objective_position and obj_pos != 0:
            # Set new combobox value
            if obj_pos in list(self.objectives['position']):
                self._objective_position = obj_pos
                self.obj_position_changed.emit()
            # Objective pos not listed in objectives so raise key erros
            else:
                self._logger.error(f"Objectives dataframe doesn't have position: {obj_pos}. Objectives {self._model.objectives}")
                raise KeyError(f"Objectives dataframe doesn't have position: {obj_pos}")

    def _sample_optovar_position(self):
        """
        Compares microscope optovar position against internal position and emits
        a signal if the don't match and updates internal position
        """
        # Get optovar pos
        opto_pos = self.get_opto_info("position")
        # Update position if changed. Sometimes while switch says pos is 0 (impossible, so dont handle 0 pos)
        if opto_pos != self._optovar_position and opto_pos != 0:
            # Set new combobox value
            if opto_pos in list(self.optovars['position']):
                self._optovar_position = opto_pos
                self.opto_position_changed.emit()
            # Wasn't listed optovars so raise error
            else:
                self._logger.error(f"Optovars dataframe doesn't have position: {opto_pos}. Optovars {self._model.optovars}")
                raise KeyError(f"Optovars dataframe doesn't have position: {opto_pos}")

    def _sample_reflector_position(self):
        """
        Compares microscope reflector position against internal position and emits
        a signal if the don't match and updates internal position
        """
        # Get reflector pos
        ref_pos = self.get_ref_info("position")
        # Update position if changed. Sometimes while switch says pos is 0 (impossible, so dont handle 0 pos)
        if ref_pos != self._reflector_position and ref_pos != 0:
            # If already in the listed optovars (had a valid name) then don't ask zen for the mag (to prevent zen block eventloop)
            if ref_pos in list(self.reflectors['position']):
                self._reflector_position = ref_pos
                self.ref_position_changed.emit()
            # Wasn't listed in reflectors so raise error
            else:
                self._logger.error(f"Reflectors dataframe doesn't have position: {ref_pos}. reflectors {self._model.reflectors}")
                raise KeyError(f"Reflectors dataframe doesn't have position: {ref_pos}")

    def _sample_lamp_intensity(self):
        """
        Compares microscope lamp intensity against internal attribute and emit
        signal if dont match and update intensity
        """
        inten = self.get_lamp_intensity()
        if inten != self._lamp_intensity:
            self._lamp_intensity = inten
            self.lamp_intensity_changed.emit(inten)
    
    def _sample_hw_setting(self):
        hw_set = self._zen.Devices.ReadHardwareSetting()
        self._sample_shutter_position(hw_set)
        self._sample_leds(hw_set)

    def _sample_shutter_position(self, hw_set=None):
        """
        Compares microscope shutter postion against internal attribute and emit
        signal if dont match

        If no `hw_set` provided, then reads the current hardware setting

        Args:
            hw_set: zeiss hardware setting to query for shutter status
        """
        shutter_pos = self.get_shutter_position(hw_set=hw_set)
        if shutter_pos != self._shutter_position:
            self._shutter_position = shutter_pos
            self.lamp_shutter_changed.emit(shutter_pos)

    def _sample_leds(self, hw_set=None):
        """
        Compares microscope led against internal attribute and emit
        signal if dont matchy

        If no `hw_set` provided, then reads the current hardware setting

        Args:
            hw_set: zeiss hardware setting to query for shutter status
        """
        leds = self._connected_leds()
        if not leds.equals(self.leds):
            diff_dict = df_compare_to_dict(self.leds, leds)
            self.leds = leds
            self.led_changed.emit(diff_dict)

    def get_focus_um(self)->float:
        '''Returns position of focus in um (consistent w/ ZEN software)'''
        foc_ZEN_um = self._zen.Devices.Focus.ActualPosition
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
        self._zen.Devices.Focus.MoveTo(des_foc_um)

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
            name = self._zen.Devices.ObjectiveChanger.GetNameByPosition(pos)
            mag = self._zen.Devices.ObjectiveChanger.GetMagnificationByPosition(pos)
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
            return self._zen.Devices.ObjectiveChanger.ActualPosition
        if key == 'name':
            return self._zen.Devices.ObjectiveChanger.ActualPositionName
        if key == 'magnification':
            return self._zen.Devices.ObjectiveChanger.Magnification

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
        self._zen.Devices.ObjectiveChanger.TargetPosition = pos
        self._zen.Devices.ObjectiveChanger.Apply()

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
            name = self._zen.Devices.Optovar.GetNameByPosition(pos)
            mag = self._zen.Devices.Optovar.GetMagnificationByPosition(pos)
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
            return self._zen.Devices.Optovar.ActualPosition
        if key == 'name':
            return self._zen.Devices.Optovar.ActualPositionName
        if key == 'magnification':
            return self._zen.Devices.Optovar.Magnification

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
        self._zen.Devices.Optovar.TargetPosition = pos
        self._zen.Devices.Optovar.Apply()

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
            name = self._zen.Devices.Reflector.GetNameByPosition(pos)
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
            return self._zen.Devices.Reflector.ActualPosition
        if key == 'name':
            ref_name = self._zen.Devices.Reflector.ActualPositionName
            if ref_name is None or ref_name == '':
                pos = self._zen.Devices.Reflector.ActualPosition
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
        self._zen.Devices.Reflector.TargetPosition = pos
        self._zen.Devices.Reflector.Apply()

    def get_lamp_intensity(self)->float:
        """ Returns lamp intensity as float between 0 - 100 """
        return round(self._zen.Devices.Lamp.ActualIntensity)

    def set_lamp_intensity(self, inten:float)->None:
        """
        Sets lamp intensity

        Arguments:
            inten (float): Float between 0-100 for lamp intensity
        """
        if inten <0 or inten> 100:
            raise ValueError(f"Invalid lamp intensity: {inten}. Must be 0-100.")
        self._zen.Devices.Lamp.TargetIntensity = inten
        self._zen.Devices.Lamp.Apply()

    def get_shutter_position(self, hw_set=None)->str:
        """
        Returns 'open' if shutter is open else 'closed'
        
        If no `hw_set` provided, then reads the current hardware setting

        Args:
            hw_set: zeiss hardware setting to query for shutter status
        """
        if hw_set is None:
            hw_set = self._zen.Devices.ReadHardwareSetting()
        is_closed = str_to_bool(self.get_hw_parameter(comp_id='MTBTLShutter', param='IsClosed', hw_set=hw_set))
        if is_closed:
            return 'closed'
        else:
            return 'open'

    def set_shutter_position(self, val:str):
        """
        Sets shutter position. If true, closes shutter else opens shutter.
        
        Args:
            val (str): 'open' or 'closed' position for shutter
        """
        if val not in ['open','closed']:
            raise ValueError(f"Invalid shutter close value: {val}. Must be ['open', 'closed']")
        hw_set = self._zen.Devices.ReadHardwareSetting()
        if val == 'open':
            is_closed = 'false'
        else:
            is_closed = 'true'
        self.set_hw_parameter(comp_id='MTBTLShutter', param='IsClosed', val=is_closed)

    def _connected_leds(self,start_pos:int=1, end_pos:int=6) -> pd.DataFrame:
        '''
        Returns dataframe of leds pos, intensity and active

        Arguments:
            start_pos (int): Starting position in Zeiss software to start listing leds
            end_pos (int): Ending positoin in Zeiss software to stop listing leds

        '''
        if not val.is_of_types(start_pos,[int,float]):
            raise ValueError(f"LED start position must be a number type. {type(start_pos)} is invalid.")
        if not val.is_of_types(end_pos,[int,float]):
            raise ValueError(f"LED end position must be a number type. {type(start_pos)} is invalid.")
        hw_set = self._zen.Devices.ReadHardwareSetting()
        led_dict = {}
        for pos in range(start_pos, end_pos+1):
            comp_id = f"MTBLED{pos}"
            inten = int(round(float(self.get_hw_parameter(comp_id=comp_id, param='Intensity',hw_set=hw_set))))
            enab = str_to_bool(self.get_hw_parameter(comp_id=comp_id, param='IsEnabled',hw_set=hw_set))
            led_dict[pos] = {'position':pos,'intensity':inten,'active':enab, 'comp_id':comp_id}
        led_df = pd.DataFrame(led_dict).transpose()
        return led_df

    def get_hw_parameter(self, comp_id:str, param:str, hw_set=None):
        """
        Return hardware setting parameter. If `hw_set` is None, reads a new
        hardware setting from ZEN.

        Args:
            comp_id (str): ZEN component id name
            param (str): ZEN parameter name
            hw_set: ZEN hardware setting

        Returns:
            parameter value
        """
        # Read hardware setting if none
        if hw_set is None:
            hw_set = self._zen.Devices.ReadHardwareSetting()
        # Validate
        if comp_id not in hw_set.GetAllComponentIds():
            raise KeyError(f"Invalid component ID: {comp_id}. Valid IDs: {hw_set.GetAllComponentIds()}")
        if param not in hw_set.GetAllParameterNames(comp_id):
            raise KeyError(f"Invalid parameter: {param}. Valid parameters: {hw_set.GetAllParameterNames(comp_id)}")
        # Get parameter from hardware setting
        val = hw_set.GetParameter(comp_id, param)
        return val

    def set_hw_parameter(self, comp_id:str, param:str, val:str):
        """
        Return hardware setting parameter. If `hw_set` is None, reads a new
        hardware setting from ZEN.

        Args:
            comp_id (str): ZEN component id name
            param (str): ZEN parameter name
            val (str): Value to set
        """
        hw_set = self._zen.Devices.ReadHardwareSetting()
        # Validate
        if comp_id not in hw_set.GetAllComponentIds():
            raise KeyError(f"Invalid component ID: {comp_id}. Valid IDs: {hw_set.GetAllComponentIds()}")
        if param not in hw_set.GetAllParameterNames(comp_id):
            raise KeyError(f"Invalid parameter: {param}. Valid parameters: {hw_set.GetAllParameterNames(comp_id)}")
        if not isinstance(val, str):
            raise TypeError(f'Invalid type of value: type={type(val)}, value={val}. Must be a string.')
        hw_set.SetParameter(comp_id, param, val)
        self._zen.Devices.ApplyHardwareSetting(hw_set)

    def get_led_info(self, led_pos:int, key:str):
        """
        Returns led info about led at position.

        Args:
            led_pos (int): Position of led that information is desired
            key (str): Type of information to return in ['intensity','active']

        Returns:
            queried information from led
        """
        # Validate args
        if led_pos not in list(self.leds['position']):
            raise KeyError(f"Invalid led position key: {led_pos}. Valid keys are {list(self.leds['position'])}.")
        if key not in ['intensity','active']:
            raise KeyError(f"Invalid led info key: {key}. Valid keys are ['intensity','active']")
        # Get queried info
        if key == 'intensity':
            comp_id = self.leds.loc[led_pos,'comp_id']
            return int(round(float(self.get_hw_parameter(comp_id,'Intensity'))))
        if key == 'active':
            comp_id = self.leds.loc[led_pos,'comp_id']
            return str_to_bool(self.get_hw_parameter(comp_id,'IsEnabled'))

    def set_led_intensity(self, led_pos:int, inten:float):
        """
        Sets led intensity for led at position

        Args:
            led_pos (int): Position of led to set intensity
            inten (float): 0-100 value of intensity
        """
        comp_id = self.leds.loc[led_pos,'comp_id']
        self.set_hw_parameter(comp_id,'Intensity', str(round(inten)))
    
    def set_led_active(self, led_pos:int, active:bool):
        """
        Sets led active for led at position

        Args:
            led_pos (int): Position of led to set intensity
            active (bool): True to turn led on else false
        """
        comp_id = self.leds.loc[led_pos,'comp_id']
        self.set_hw_parameter(comp_id,'IsEnabled', str(active).lower())


class ControllerZEN(QObject):
    """
    Controller for zeiss/ZEN interface within the MVC architecture. Recieves
    signals from model when hardware positoin has changed and sends these to
    view. Alro recieves signals from the view and acts on them to change to
    postion of the microscope to reflect the user input.
    """
    
    ex_foc_position_changed = pyqtSignal(str)
    ex_obj_position_changed = pyqtSignal(str)
    ex_opto_position_changed = pyqtSignal(str)
    ex_ref_position_changed = pyqtSignal(str)
    ex_lamp_intensity_changed = pyqtSignal(float)
    ex_shutter_position_opened = pyqtSignal(bool)
    ex_led_intensity_changed = pyqtSignal(dict)
    ex_led_active_changed = pyqtSignal(dict)

    focus_changed = pyqtSignal([str])
    inc_changed = pyqtSignal([str])
    obj_changed = pyqtSignal([float])
    opto_changed = pyqtSignal([float])
    ref_changed = pyqtSignal([str])

    def __init__(self, model:ModelZEN):
        """
        Args:
            model: Model for zeiss/zen microscope control in MVC architecture
        """
        super().__init__()
        self._logger = StandardLogger(__name__)
        self._model = model
        self._set_model_connections()

    def _set_model_connections(self):
        """
        Set signal/slot connections between model-controller
        """
        # Update controller when the model changed externally
        self._model.foc_position_changed.connect(self.focus_model_change)
        self._model.obj_position_changed.connect(self.objective_model_change)
        self._model.opto_position_changed.connect(self.optovar_model_change)
        self._model.ref_position_changed.connect(self.reflector_model_change)
        self._model.lamp_intensity_changed.connect(self.lamp_model_change)
        self._model.lamp_shutter_changed.connect(self.lamp_shutter_model_change)
        self._model.led_changed.connect(self.led_model_change)

    @pyqtSlot()
    def change_increment(self, inc_val:str):
        """
        Change the internal attribute corresponding to the focus increment value.
        Outputs a signal that can be recieved by all connected "views" so that the
        focus increment is consistenet across "views".

        Args:
            inc_val (str): Value of the focus increment
        """
        self.inc_changed.emit(inc_val)

    def objective_model_change(self):
        """
        Handles when objective changes in the model (like the user changed it
        in the ZEN software). Emits signal of the hardware change so view can
        reflect the change.
        """
        obj_combo_str = self.parse_to_combobox('objective')
        self.ex_obj_position_changed.emit(obj_combo_str)


    def objective_view_change(self, obj_combobox:str):
        """
        Handles when objective changes in the view. Commands change of
        objective position.

        Args:
            obj_combobox (str): String as '{pos}: {mag}x' denoting which
                objective position and magnitude to change to.
        """
        # Update internal attribute of objective positoin
        obj_pos, obj_mag = self.parse_from_combobox('objective', obj_combobox)
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
            self._logger.exception('Error encountered while changing objective position')
            raise

    def optovar_model_change(self):
        """
        Handles when optovar changes in the model (like the user changed it
        in the ZEN software). Emits signal of the hardware change so view can
        reflect the change.
        """
        opto_combo_str = self.parse_to_combobox('optovar')
        self.ex_opto_position_changed.emit(opto_combo_str)

    def optovar_view_change(self, opto_combobox:str):
        """
        Handles when optovar changes in the view. Commands change of
        optovar position.

        Args:
            opto_combobox (str): String as '{pos}: {mag}x' denoting which
                optovar position and magnitude to change to.
        """
        # Update internal attribute of optovar position
        opto_pos, opto_mag = self.parse_from_combobox('optovar', opto_combobox)
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
            self._logger.exception('Error encountered while changing optovar position')
            raise

    def reflector_model_change(self):
        """
        Handles when reflector changes in the model (like the user changed it
        in the ZEN software). Emits signal of the hardware change so view can
        reflect the change.
        """
        ref_combo_str = self.parse_to_combobox('reflector')
        self.ex_ref_position_changed.emit(ref_combo_str)

    def reflector_view_change(self, ref_combobox:str):
        """
        Handles when relector changes in the view. Commands change of
        relector position.

        Args:
            ref_combobox (str): String as '{pos}: {mag}x' denoting which
                relector position and magnitude to change to.
        """
        # Update internal attribute of reflector position
        ref_pos, ref_name = self.parse_from_combobox('reflector', ref_combobox)
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
            self._logger.exception('Error encountered while changing reflector position')
            raise

    def focus_model_change(self, foc_pos:float):
        """
        Handles when focus changes in the model (like the user changed it
        in the ZEN software). Emits signal of the hardware change so view can
        reflect the change.
        """
        position = str(foc_pos)
        self.ex_foc_position_changed.emit(position)

    def goto_focus_absolute(self, abs_focus:float):
        """
        Move to absolute focus position

        Args:
            abs_focus (float): Desired focus position
        """
        self._model.goto_focus_abs_um(abs_focus)

    def goto_focus_relative(self, rel_focus:float):
        """
        Move focus position relative to current position.

        Args:
            rel_focus (float): Relative positoin change
        """
        self._model.goto_focus_rel_um(rel_focus)

    def get_focus_um(self)->float:
        """
        Return current position of focus

        Returns:
            current focus position
        """
        pos = self._model.get_focus_um()
        return pos

    def get_focus_um_approx(self)->float:
        '''
        Returns the position of the focus as saved from sampling the focus height.
        It could be delayed from the actual focus level
        '''
        pos = self._model._focus_position
        return pos

    def lamp_model_change(self, inten:float):
        """
        Handles when lamp intensity changes in model (like user changed it
        in ZEN software). Emits signal of change so view can reflect change

        Args:
            inten (float): Lamp intensity value
        """
        self.ex_lamp_intensity_changed.emit(inten)

    def get_lamp_intensity(self)->float:
        """ Returns lamp intesnity on 0-100 """
        return self._model.get_lamp_intensity()

    def set_lamp_intensity(self, inten:float)->None:
        """
        Set the lamp intensity
        
        Args:
            inten (float): 0-100 value of intensity
        """
        self._model.set_lamp_intensity(inten)

    def lamp_shutter_model_change(self, position:str):
        """
        Handles when lamp shutter changes in model (like user changed it
        in ZEN software). Emits signal of change so view can reflect change

        Args:
            position (str): 'open' if shutter opened else 'closed'
        """
        if position == 'open':
            opened = True
        else:
            opened = False
        self.ex_shutter_position_opened.emit(opened)

    def get_lamp_on(self)->bool:
        """ Return true if shutter is open else false """
        position = self._model.get_shutter_position()
        if position == 'open':
            return True
        else:
            return False

    def set_lamp_on(self,on:bool)->None:
        """
        Sets whether lamp is on. True for on else false
        
        Args:
            on (bool): True for on else false
        """
        if on is True:
            position = 'open'
        else:
            position = 'closed'
        self._model.set_shutter_position(position)

    def led_model_change(self, change_dict:dict):
        """
        Handles when a led status chagnes in model (liek user changed it in
        the ZEN software). Emits signal of change so view can reflect change.

        Args:
            change_dict (dict): dictionary of change values as {pos: {chagnes}}
        """
        intensity_changes = {}
        active_changes = {}
        for key, sub_dict in change_dict.items():
            for param, val in sub_dict.items():
                if param == 'intensity':
                    intensity_changes[key] = val
                elif param == 'active':
                    active_changes[key] = val
        if intensity_changes != {}:
            self.ex_led_intensity_changed.emit(intensity_changes)
        if active_changes != {}:
            self.ex_led_active_changed.emit(active_changes)

    def get_led_on(self, led_pos:int)->bool:
        """ Return true if led at `led_pos` is on """
        return self._model.get_led_info(led_pos=led_pos, key='active')

    def set_led_on(self, led_pos:int, on:bool)->bool:
        """
        Sets led at `led_pos` to on if `on` is true else turns off

        Args:
            led_pos (int): Led to turn off/on
            on (bool): whether to turn led on or off
        """
        self._model.set_led_active(led_pos=led_pos, active=on)

    def get_led_intensity(self, led_pos:int)->float:
        """ Return intensity as float for led at `led_pos` """
        return self._model.get_led_info(led_pos=led_pos, key='intensity')

    def set_led_intensity(self, led_pos:int, inten:float):
        """
        Sets led intensity for led at position

        Args:
            led_pos (int): Position of led to set intensity
            inten (float): 0-100 value of intensity
        """
        self._model.set_led_intensity(led_pos=led_pos, inten=inten)
    
    def parse_to_combobox(self, box_name:str) -> str:
        """
        Creates a combobox string from the hardware's info associated with
        `box name`. Queries the model for the hardware info before creating the
        string to return

        Args:
            box_name (str): Name of combo box. Must be in ['objective',
                'optovar', 'reflector']

        Returns:
            string to set in combobox as '{pos}: {secondary info}'
        """
        # Validate
        if box_name not in ['objective', 'optovar', 'reflector']:
            raise KeyError(f"Request combobox name to parse: {box_name}. Name must be in ['objective', 'optovar', 'reflector']")
        if box_name == "objective":
            cur_pos = self._model.get_obj_info('position')
            cur_mag = self._model.get_obj_info('magnification')
            cur_combo_str = f"{cur_pos}: {cur_mag}x"
            return cur_combo_str
        if box_name == "optovar":
            cur_pos = self._model.get_opto_info('position')
            cur_mag = self._model.get_opto_info('magnification')
            cur_combo_str = f"{cur_pos}: {cur_mag}x"
            return cur_combo_str
        if box_name == "reflector":
            cur_pos = self._model.get_ref_info('position')
            cur_name = self._model.get_ref_info('name')
            cur_combo_str = f"{cur_pos}: {cur_name}"
            return cur_combo_str
        
    def parse_from_combobox(self, box_name:str, box_val:str):
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
        cur_combo_str = self.parse_to_combobox('objective')

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
        cur_combo_str = self.parse_to_combobox('optovar')

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
        cur_combo_str = self.parse_to_combobox('reflector')

        return combo_vals, cur_combo_str

    def get_current_objective(self):
        """
        Return tuple of objective information

        Returns:
            name (str): current name
            pos (int): current position
            mag (float): current magnification
        """
        pos = self._model.get_obj_info('position')
        mag = self._model.get_obj_info('magnification')
        name = self._model.get_obj_info('name')
        return name, pos, mag

    def get_current_optovar(self):
        """
        Return tuple of optovar information

        Returns:
            pos (int): current position
            mag (float): current magnification
            name (str): current name
        """
        pos = self._model.get_opto_info('position')
        mag = self._model.get_opto_info('magnification')
        name = self._model.get_opto_info('name')
        return name, pos, mag

    def get_current_reflector(self):
        """
        Return tuple of reflector information

        Returns:
            pos (int): current position
            name (str): current name
        """
        pos = self._model.get_ref_info('position')
        name = self._model.get_ref_info('name')
        return name, pos


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
        """
        super().__init__()
        self._logger = StandardLogger(__name__)
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
        self.foc_disp.setToolTip("Current microscope focus position")
        palette = QPalette()
        palette.setColor(QPalette.ColorRole.Base, QColor('lightGray'))
        self.foc_disp.setPalette(palette)

        # Change in focus increment
        self.foc_inc = QLineEdit()
        self.foc_inc.setToolTip("Enter movement increment for '+'/'-' buttons")

        # Change focus buttons
        self.btn_minus = QPushButton('-')
        self.btn_minus.setToolTip("Decrease focus position")
        self.btn_plus = QPushButton('+')
        self.btn_plus.setToolTip("Increase focus position")

        # Objective selector combobox
        self.obj_selector = QComboBox()
        self.obj_selector.setPlaceholderText('Magnification')
        self.obj_selector.setToolTip("Choose microscope objective")

        # Optovar selector combobox
        self.opto_selector = QComboBox()
        self.opto_selector.setPlaceholderText('Magnification')
        self.opto_selector.setToolTip("Choose microscope optovar")

        # Reflector selector combobox
        self.ref_selector = QComboBox()
        self.ref_selector.setPlaceholderText('Name')
        self.ref_selector.setToolTip("Choose microscope reflector")

        # LED values for each position (copied from ZEN software)
        self.led_wavelengths = {1: 385, 
                                2: 430, 
                                3: 475,
                                4: 511, 
                                5: 555, 
                                6: 630}

        # Light source buttons
        self.lamp_button = QRadioButton('Lamp')
        self.lamp_button.setAutoExclusive(False)
        self.lamp_button.setToolTip("Turn on/off transmitted light source")
        self.led1_button = QRadioButton(str(self.led_wavelengths[1]))
        self.led1_button.setAutoExclusive(False)
        self.led1_button.setToolTip("Turn on/off LED source")
        self.led2_button = QRadioButton(str(self.led_wavelengths[2]))
        self.led2_button.setAutoExclusive(False)
        self.led2_button.setToolTip("Turn on/off LED source")
        self.led3_button = QRadioButton(str(self.led_wavelengths[3]))
        self.led3_button.setAutoExclusive(False)
        self.led3_button.setToolTip("Turn on/off LED source")
        self.led4_button = QRadioButton(str(self.led_wavelengths[4]))
        self.led4_button.setAutoExclusive(False)
        self.led4_button.setToolTip("Turn on/off LED source")
        self.led5_button = QRadioButton(str(self.led_wavelengths[5]))
        self.led5_button.setAutoExclusive(False)
        self.led5_button.setToolTip("Turn on/off LED source")
        self.led6_button = QRadioButton(str(self.led_wavelengths[6]))
        self.led6_button.setAutoExclusive(False)
        self.led6_button.setToolTip("Turn on/off LED source")
        self.led_button_dict = {1: self.led1_button,
                                2: self.led2_button,
                                3: self.led3_button,
                                4: self.led4_button,
                                5: self.led5_button,
                                6: self.led6_button,}
        self.inv_led_button_dict = {v:k for k, v in self.led_button_dict.items()}
        
        # Lamp intensity slider
        self.lamp_slider = QSlider(Qt.Orientation.Horizontal)
        self.lamp_slider.setMinimum(0)
        self.lamp_slider.setMaximum(100)
        self.lamp_slider.setTickPosition(QSlider.TickPosition.TicksBelow)
        self.lamp_slider.setTickInterval(10)
        self.lamp_slider.setToolTip("Adjust transmitted light source power")

        # LED intensity combobox and slider
        self.led_selector = QComboBox()
        self.led_selector.setPlaceholderText('LED')
        self.led_selector.setToolTip("Select LED wavelength power to adjust")
        self.led_slider = QSlider(Qt.Orientation.Horizontal)
        self.led_slider.setMinimum(0)
        self.led_slider.setMaximum(100)
        self.led_slider.setTickPosition(QSlider.TickPosition.TicksBelow)
        self.led_slider.setTickInterval(10)
        self.led_slider.setToolTip("Adjust currently selected LED power")
    
    def _stateify_widgets(self):
        """
        Set initial states for the widgets using info from the controller
        """
        # Initialize the display of the inital focus position
        pos = str(self._controller.get_focus_um())
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

        # Initialize lamp slider
        lamp_val = self._controller.get_lamp_intensity()
        self.lamp_slider.setValue(lamp_val)

        # Intialize lamp shutter
        is_on = self._controller.get_lamp_on()
        self.lamp_button.setChecked(is_on)

        # Initialize led buttons
        led1_val = self._controller.get_led_on(self.inv_led_button_dict[self.led1_button])
        self.led1_button.setChecked(led1_val)
        led2_val = self._controller.get_led_on(self.inv_led_button_dict[self.led2_button])
        self.led2_button.setChecked(led2_val)
        led3_val = self._controller.get_led_on(self.inv_led_button_dict[self.led3_button])
        self.led3_button.setChecked(led3_val)
        led4_val = self._controller.get_led_on(self.inv_led_button_dict[self.led4_button])
        self.led4_button.setChecked(led4_val)
        led5_val = self._controller.get_led_on(self.inv_led_button_dict[self.led5_button])
        self.led5_button.setChecked(led5_val)
        led6_val = self._controller.get_led_on(self.inv_led_button_dict[self.led6_button])
        self.led6_button.setChecked(led6_val)

        # Initilaize led combobox
        combovals = [f"{key}: {wav}" for key, wav in self.led_wavelengths.items()]
        self.led_selector.insertItems(0,combovals)

        # # Initialize led slider
        # lamp_val = self._controller.get_led_intensity()
        # self.led_slider.setValue(lamp_val)

    def _set_connections(self):
        """
        Set the various view-view and view-controller connections
        """
        # Set view's signals' connections with view slots
        self.btn_minus.clicked.connect(self.minus_focus)
        self.btn_plus.clicked.connect(self.plus_focus)
        self.lamp_slider.sliderReleased.connect(self.change_lamp_intensity)
        self.led_slider.sliderReleased.connect(self.change_led_intensity)
        self.lamp_button.clicked.connect(lambda lamp_on: self.change_lamp_active(lamp_on))
        self.led1_button.clicked.connect(lambda but_on: self.change_led_active(but_on, self.led1_button))
        self.led2_button.clicked.connect(lambda but_on: self.change_led_active(but_on, self.led2_button))
        self.led3_button.clicked.connect(lambda but_on: self.change_led_active(but_on, self.led3_button))
        self.led4_button.clicked.connect(lambda but_on: self.change_led_active(but_on, self.led4_button))
        self.led5_button.clicked.connect(lambda but_on: self.change_led_active(but_on, self.led5_button))
        self.led6_button.clicked.connect(lambda but_on: self.change_led_active(but_on, self.led6_button))
        self.led_selector.currentTextChanged.connect(self.led_combobox_change)
        # Set view's signals' connection with controller slots.
        self.obj_selector.currentTextChanged.connect(self._controller.objective_view_change)
        self.opto_selector.currentTextChanged.connect(self._controller.optovar_view_change)
        self.ref_selector.currentTextChanged.connect(self._controller.reflector_view_change)
        self.change_focus_request.connect(self._controller.goto_focus_relative)
        # Set controller's signals' connection with view slots (ie updates the view
        # when microscope config changes w/o action from the view.)
        self._controller.ex_foc_position_changed.connect(lambda foc_pos: self.update_focus_display(foc_pos))
        self._controller.ex_obj_position_changed.connect(lambda combo_str: self.update_combobox(self.obj_selector, combo_str))
        self._controller.ex_opto_position_changed.connect(lambda combo_str: self.update_combobox(self.opto_selector, combo_str))
        self._controller.ex_ref_position_changed.connect(lambda combo_str: self.update_combobox(self.ref_selector, combo_str))
        self._controller.ex_lamp_intensity_changed.connect(lambda inten: self.update_lamp_slider(inten))
        self._controller.ex_shutter_position_opened.connect(lambda opened: self.update_lamp_button(opened))
        self._controller.ex_led_intensity_changed.connect(lambda inten: self.update_led_slider(inten))
        self._controller.ex_led_active_changed.connect(lambda led_active_dict: self.update_led_buttons(led_active_dict))
        # The controller can be connected to multiple views. For all views to display the
        # same focus increment, send the a signal to the controller about the increment
        # changing then have the controller set the text of the view (and all views)
        self.foc_inc.textEdited.connect(lambda inc_text: self._controller.change_increment(inc_text))
        self._controller.inc_changed.connect(lambda inc_text: self.foc_inc.setText(inc_text))

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

    def update_lamp_slider(self, inten:float):
        """
        Updates the view's lamp intensity slider to show the intensity

        Args:
            inten (float): Lamp intensity
        """
        self.lamp_slider.setValue(int(round(inten)))

    def update_led_slider(self, led_dict:float):
        """
        Updates the view's led intensity slider to show the intensity

        Args:
            led_dict (dict): dictionary of {led_num: intensity, ...}
        """
        for led_num, inten in led_dict.items():
            selector_text = self.led_selector.currentText()
            selector_led_num, _ = self._parse_from_led_combobox(selector_text)
            if led_num == selector_led_num:
                self.led_slider.setValue(int(round(inten)))

    def update_lamp_button(self, is_opened:bool):
        """
        Updates the view's lamp button to show whether on or off

        Args:
            is_opened (bool): if shutter is opened
        """
        self.lamp_button.setChecked(is_opened)

    def update_led_buttons(self, led_dict:dict):
        """
        Updates the view's led button to show whether on or off

        Args:
            led_dict (dict): dictionary of {led_num: is_on, ...}
        """
        for led_num, val in led_dict.items():
            self.led_button_dict[led_num].setChecked(val)

    def change_lamp_intensity(self):
        """
        Sets new lamp inensity when the lamp slider is changed
        """
        inten = self.lamp_slider.value()
        self._controller.set_lamp_intensity(inten)

    def change_led_intensity(self):
        """
        Sets new led inensity when the led slider is changed
        """
        selector_text = self.led_selector.currentText()
        led_num, led_wave = self._parse_from_led_combobox(selector_text)
        if led_num is not None:
            inten = self.led_slider.value()
            self._controller.set_led_intensity(led_num,inten)

    def change_lamp_active(self, lamp_on:bool):
        """
        Sets new shutter position when lamp button clicked

        Args:
            lamp_on (bool): state of the lamp button (true for checked)
        """
        self._controller.set_lamp_on(lamp_on)

    def change_led_active(self, led_on:bool, led_but:QRadioButton):
        """
        Turns led on/off when led button clicked

        Args:
            led_on (bool): state of the led button (true for checked)
            led_but (QRadioButton): the led button that was clicked
        """
        led_pos = int(self.inv_led_button_dict[led_but])
        if led_on is True:
            combostr = self._parse_to_led_combobox(led_pos)
            self.led_selector.setCurrentText(combostr)
        self._controller.set_led_on(led_pos, led_on)

    def led_combobox_change(self, led_combo:str):
        """
        Handles when the text of the led selector changes

        Args:
            led_combo (str): text from led combobox
        """
        led_num, led_wav = self._parse_from_led_combobox(led_combo)
        if led_num is not None:
            inten = self._controller.get_led_intensity(led_num)
            self.led_slider.setValue(inten)

    def _parse_to_led_combobox(self, led_pos:int)->str:
        """
        Makes led combobox string from led position

        Args:
            led_pos (int): led positoin

        Returns:
            string as f"{led_pos}: {wavelength}"
        """
        combo_str = f"{led_pos}: {self.led_wavelengths[led_pos]}"
        return combo_str

    def _parse_from_led_combobox(self, combo_str:str):
        """
        Parses led combobox to led positoin and wavelength. Returns None
        if combo_str is emtpy

        Args:
            combo_str (str): string form combobox as f"{led_pos}: {wavelength}"

        Returns:
            (int) led positon
            (int) led wavelength
        """
        if combo_str == '':
            return None, None
        else:
            led_pos = int(combo_str.split(":")[0])
            led_wav = int(combo_str.split(":")[0].strip())
            return led_pos, led_wav

    def plus_focus(self):
        '''
        Handles when user clicks `+` button. Emits signal of desired focus
        change using the view's focus increment box. Logs exception for invalid
        increment number 
        '''
        try:
            val = self._increment_to_num()
        except ValueError as e:
            self._logger.exception('Request invalid focus increment')
        else:
            # Unthreaded
            try:
                self.change_focus_request.emit(val) 
            except ValueError:
                self._logger.exception('Error while attempting to increment focus')

        
    def minus_focus(self):
        '''
        Handles when user clicks `-` button. Emits signal of desired focus
        change using the view's focus increment box. Logs exception for invalid
        increment number 
        '''
        try:
            val = -self._increment_to_num()
        except ValueError as e:
            self._logger.exception('Request invalid focus increment')
        else:
            # Unthreaded 
            try:
                self.change_focus_request.emit(val)  
            except ValueError:
                self._logger.exception('Error while attempting to increment focus')

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


class ViewZENComplete(ViewZEN):
    """
    A full blown application by using the `ViewZEN` widgets to make a user
    interface.
    """
    
    def __init__(self, controller:ControllerZEN):
        """
        Args:
            controller: zeiss controller for mvc architecture
        """
        super().__init__(controller)
        self.zen_group = QGroupBox("Zeiss Control")
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
        h_layout1.addWidget(self.foc_disp)
        h_layout2.addWidget(self.btn_minus)
        h_layout2.addWidget(self.btn_plus)
        h_layout3.addWidget(foc_inc_label)
        h_layout3.addWidget(self.foc_inc)
        v_layout.addLayout(h_layout1)
        v_layout.addLayout(h_layout2)
        v_layout.addLayout(h_layout3)

        # Separator between widgets
        h_separator = QHLine()
        v_layout.addWidget(h_separator)

        # Objective, optovar, and reflector widgets
        obj_selector_label = QLabel('Objective:')        
        opto_selector_label = QLabel('Optovar:')       
        ref_selector_label = QLabel('Reflector:')        
        form1 = QFormLayout()
        form1.addRow(obj_selector_label, self.obj_selector)
        form1.addRow(opto_selector_label, self.opto_selector)
        form1.addRow(ref_selector_label, self.ref_selector)
        v_layout.addLayout(form1)

        # Lightsource widgets
        lamp_slider_label = QLabel("Lamp:")
        source_label = QLabel("Light\nSources:")
        h_layout7 = QHBoxLayout()
        h_layout8 = QHBoxLayout()
        h_layout9 = QHBoxLayout()
        h_layout7.addWidget(self.led1_button)
        h_layout7.addWidget(self.led2_button)
        h_layout7.addWidget(self.led3_button)
        h_layout8.addWidget(self.led4_button)
        h_layout8.addWidget(self.led5_button)
        h_layout8.addWidget(self.led6_button)
        v_layout2 = QVBoxLayout()
        v_layout2.addWidget(self.lamp_button)
        v_layout2.addLayout(h_layout7)
        v_layout2.addLayout(h_layout8)
        v_layout2.setAlignment(self.lamp_button,Qt.AlignmentFlag.AlignHCenter)
        h_layout9.addWidget(source_label)
        h_layout9.addLayout(v_layout2)
        form = QFormLayout()
        form.addRow(lamp_slider_label, self.lamp_slider)
        form.addRow(self.led_selector, self.led_slider)
        v_layout.addWidget(QHLine())
        v_layout.addLayout(h_layout9)
        v_layout.addLayout(form)
        # Create groupbox
        self.zen_group.setLayout(v_layout)

class ViewZENFocus(ViewZEN):
    """
    A full blown application by using the `ViewZEN` widgets to make a user
    interface.
    """
    
    def __init__(self, controller:ControllerZEN):
        """
        Args:
            controller: zeiss controller for mvc architecture
        """
        super().__init__(controller)
        self.zen_group = QGroupBox("Zeiss Control")
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
        h_layout1.addWidget(self.foc_disp)
        h_layout2.addWidget(self.btn_minus)
        h_layout2.addWidget(self.btn_plus)
        h_layout3.addWidget(foc_inc_label)
        h_layout3.addWidget(self.foc_inc)
        v_layout.addLayout(h_layout1)
        v_layout.addLayout(h_layout2)
        v_layout.addLayout(h_layout3)

        # Create groupbox
        self.zen_group.setLayout(v_layout)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    model = ModelZEN()
    controller = ControllerZEN(model)
    win = ViewZENComplete(controller).zen_group
    # win = ViewZENFocus(controller).zen_group
    win.show()
    sys.exit(app.exec())
"""Scripts for interfacing with ZEN to automate microscope movemetns"""
import threading
import time
import numpy as np
import pandas as pd
import win32com.client
from src.miscellaneous import numerify

class ZEN():
    """Interface for ZEN application"""
    
    def __init__(self):
        try:
            self.zen = win32com.client.GetActiveObject("Zeiss.Micro.Scripting.ZenWrapperLM")
        except:
            raise
        # Max and min values of focus position in ZEN
        self.focus_max_um = 10000
        self.focus_min_um = 0
        self.objectives = self._connected_objectives()

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

    def _connected_objectives(self) -> pd.DataFrame:
        '''
        Returns dataframe of objective name and magnification indexed by position


        DataFrame: index = "position", column1 = "name", column2 = "magnification"
        '''
        obj_dict = {}
        for pos in range(10):
            name = self.zen.Devices.ObjectiveChanger.GetNameByPosition(pos)
            mag = self.zen.Devices.ObjectiveChanger.GetMagnificationByPosition(pos)
            if name is not None:
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
            mag (float): Magnificaiton of desried objective (must be
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

class _ZEN_focus_move_request():
    ''' Handles move requests for ZEN focus '''
    def __init__(self, zen:ZEN, des_foc_um:float):
        # if des_foc_um > self.focus_max_um or des_foc_um < self.focus_min_um:
        #     raise ValueError(f'Invalid goto_focus_rel_um position: {delta_foc_um}um. Current + relative ({cur_foc_um} + {delta_foc_um} = {des_foc_um}um) must be {self.focus_min_um}um to {self.focus_max_um}um.')
        self.zen = zen
        self.des_foc_um = des_foc_um
        self.moving = False
        self._lock = threading.Lock()
        self._move_thread = threading.Thread(target=self._make_move)
        self._move_thread.start()
        # print(self._move_thread.is_alive())
        # time.sleep(1)
        # print('here')
        # time.sleep(4)
        # print(self._move_thread.is_alive())

    def _make_move(self):
        p = self.zen.get_focus_um()
        print('pos:',p)
        # print('_make_move')
        # self.zen.Devices.Focus.MoveTo(self.des_foc_um)
        # with self._lock:
        #     # time.sleep(2)
        #     # print('in lock')
        #     self.zen.Devices.Focus.MoveTo(self.des_foc_um)
        # # self._move_thread.join()
        # # print(self._move_thread.is_alive())

def focus_test(z):
    pos = z.get_focus_um()
    if pos < 5000:
        des_pos = 8000
    else:
        des_pos = 2000

    # Unthreaded move
    t0 = time.time()
    print('making unthreaded move...')
    z.goto_focus_abs_um(des_pos)
    t1 = time.time()
    print('Unthreaded dur: ',t1-t0)

    time.sleep(0.25)

    # Treaded move
    t0 = time.time()
    # Try to make threaded move
    t1 = time.time()
    print('Threaded dur: ',t1-t0)

def objective_test(z):
    print(z.objectives)
    cur_mag = z.get_obj_info('magnification')
    print(f'Current mag: {cur_mag}')
    if cur_mag <= 10.0:
        des_mag = 20.0
    else:
        des_mag = 10.0
    z.goto_obj_mag(des_mag)
    new_mag = z.get_obj_info('magnification')
    print(f'New mag: {new_mag}')

if __name__ == "__main__":
    z = ZEN()
    # focus_test(z)
    objective_test(z)
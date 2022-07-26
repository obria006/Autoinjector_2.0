"""Scripts for interfacing with ZEN to automate microscope movemetns"""
import threading
import time
import numpy as np
import win32com.client


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
        self.focus_params()

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
        # self.zen.Devices.Focus.MoveTo(des_foc_um)
        _ZEN_focus_move_request(self.zen, des_foc_um)

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

    def focus_params(self):
        # top and bottom position as displayed on scope
        top_disp_mm = 5
        bot_disp_mm = -4.5
        # Max and min position from Zen.Devices.Focus.ActualPosition
        top_ZEN_raw = 10000
        bot_ZEN_raw = 0
        # Scaling and offset to convert from ActualPosition to display value
        scale = (top_disp_mm - bot_disp_mm)/(top_ZEN_raw - bot_ZEN_raw)
        offset = bot_disp_mm
        # Transformation matrix
        self.ZEN_to_disp = np.array([[scale, offset],[0,1]])
        self.disp_to_ZEN = np.linalg.inv(self.ZEN_to_disp)

if __name__ == "__main__":
    z = ZEN()
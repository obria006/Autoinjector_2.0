"""Scripts for interfacing with ZEN to automate microscope movemetns"""
import threading
import time
import numpy as np
import win32com.client


class ZEN():
    """Interface for ZEN application"""
    #TODO thread get_position and gotopositoin (at least goto so it doesn't block during exec)
    #TODO catch error when ZEN not open
    #TODO warning when request large move
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
        
    def T_z(self, foc_raw:float)->float:
        '''
        Tranform focus controller coordinates. (raw -> mm)
        
        Arguments:
            foc_raw (float): Focus position in raw coordinates.

        Returns
            float of focus position in mm coordinates. 
        '''
        foc_raw_vec = np.array([foc_raw, 1]).reshape(2,1)
        foc_mm_vec = np.matmul(self.ZEN_to_disp,foc_raw_vec)
        foc_mm = foc_mm_vec[0,0]
        return foc_mm

    def Ti_z(self, foc_mm:float)->float:
        '''
        Inverse transform focus controller coordinates. (mm -> raw)
        
        Arguments:
            foc_mm (float): Focus position in mm coordinates.

        Returns
            float of focus position in raw coordinates. 
        '''
        foc_mm_vec = np.array([foc_mm,1]).reshape(2,1)
        foc_raw_vec = np.matmul(self.disp_to_ZEN,foc_mm_vec)
        foc_raw = foc_raw_vec[0,0]
        return foc_raw

    # def get_focus_mm(self)->float:
    #     '''
    #     Get focus position in mm.

    #     Arguments:
    #         None

    #     Returns:
    #         float of focus position in mm coordinates
    #     '''
    #     foc_raw = self.zen.Devices.Focus.ActualPosition
    #     print(foc_raw)
    #     foc_mm = self.T_z(foc_raw)
    #     return foc_mm

    # def goto_focus_abs(self, des_foc_mm:float)->None:
    #     '''
    #     Go to absolute position of focus controller.

    #     Arguments:
    #         des_foc_mm (float): Desired focus position in mm.

    #     Returns:
    #         None
    #     '''
    #     des_foc_raw = self.Ti_z(des_foc_mm)
    #     self.zen.Devices.Focus.MoveTo(des_foc_raw)

    # def goto_focus_rel(self, delta_foc_mm):
    #     '''
    #     Go to relative position of focus controller.

    #     Arguments:
    #         delta_foc_mm (float): Desired displacement of focus position in mm.

    #     Returns:
    #         None
    #     '''
    #     cur_foc_mm = self.get_focus_mm()
    #     des_foc_mm = cur_foc_mm + delta_foc_mm
    #     self.goto_focus_abs(des_foc_mm)

class _ZEN_focus_move_request():
    ''' Handles move requests for ZEN focus '''
    def __init__(self, zen:ZEN, des_foc_um:float):
        # if des_foc_um > self.focus_max_um or des_foc_um < self.focus_min_um:
        #     raise ValueError(f'Invalid goto_focus_rel_um position: {delta_foc_um}um. Current + relative ({cur_foc_um} + {delta_foc_um} = {des_foc_um}um) must be {self.focus_min_um}um to {self.focus_max_um}um.')
        self.zen = zen
        self.des_foc_um = des_foc_um
        self.zen.Devices.Focus.MoveTo(des_foc_um)
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
        self.zen.Devices.Focus.MoveTo(self.des_foc_um)
        # with self._lock:
        #     # time.sleep(2)
        #     # print('in lock')
        #     self.zen.Devices.Focus.MoveTo(self.des_foc_um)
        # # self._move_thread.join()
        # # print(self._move_thread.is_alive())




if __name__ == "__main__":
    z = ZEN()
    pos = z.get_focus_um()
    print(pos)
    t0 = time.time()
    z.goto_focus_abs_um(1000)
    # t1 = time.time()
    # print(f'Move 1:{t1-t0}')
    # z.goto_focus_rel_um(8000)
    # t2 = time.time()
    # print(f'Move 1:{t2-t1}')
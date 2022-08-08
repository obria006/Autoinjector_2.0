''' Handles control of the sensapex micromanipulator '''
from src.manipulator_control.sensapex_utils import SensapexDevice, UMP
from src.miscellaneous import validify as val


class Manipulator():
    ''' Controls sensapex micromanipulator '''

    def __init__(self, dev_id):
        # Get ids of all connected sensapex devices
        self.ump = UMP.get_ump()
        self.dev_ids = self.ump.list_devices()
        # Validate desired device to interface with
        if dev_id not in self.dev_ids:
            raise ValueError(f'Invalid sensapex device number: {dev_ids}. Connected sensapex device numbers include {self.dev_ids}')
        # Interface with device
        self.dev_id = dev_id
        self.dev = SensapexDevice(dev_id)
        # Dictionary assoviate axis with its index in Sensapex software
        self.axis_dict = {'x':0, 'y':1, 'z':2, 'd':3}
        # Max and min position bounds for all axes
        self.mic2nm = 1000 # convert micron to nanometer
        self.pos_min = 0 * self.mic2nm
        self.pos_max = 20000 * self.mic2nm

    def pos(self)->list:
        '''
        Returns manipulator position as list like [x, y, z, d]
        '''
        pos = self.dev.get_pos()
        return pos

    def move(self, pos:list, speed:float, mode:str):
        '''
        Commands manipulator to make move. 'mode' dictates whether move is to
        be relative or absolute.

        Arugments:
            pos (list): Desired actual position (mode='absolute') or
                displacement (mode='relative'). List as [x, y, z, d].
            speed (float): Desired speed of move in um/s.
            mode (str): Type of move to make. Must be in ['absolute', 'relative']
        
        Returns
            Sensapex move request
        '''
        # Validate args
        if not val.is_of_types(pos, [list, tuple]):
            raise TypeError('Movement positoin/displacement must be a list or tuple.')
        if len(pos) < 4:
            raise ValueError(f'Movement position/displacement ({pos}) must have 4 entries. ')
        if speed < 10:
            raise ValueError(f'Requested speed ({speed}) must be somewhat greater than 0')
        if mode not in ['absolute', 'relative']:
            raise KeyError(f"Movement mode ({mode}) must be 'absolute' or 'relative'.")
        # Compute desired position
        if mode == 'relative':
            cur_pos = self.dev.get_pos()
            des_pos  = (np.array(pos) + np.array(cur_pos)).tolist()
        else:
            des_pos = list(pos)
        # Validate position
        if np.any(np.array(des_pos) < self.pos_min) or np.any(np.array(des_pos) > self.pos_max):
            raise ValueError(f'Requested ({mode}) move to final position ({des_pos}) exceeds boundary extremes ({self.pos_min} -> {self.pos_max}).')
        # Make move
        move_req = self.dev.goto_pos(des_pos, speed)
        return move_req

        
        




        
        


''' Scripts for handling manipulator to coordinate system calibration '''
import numpy as np
import pandas as pd
import traceback
from src.miscellaneous.standard_logger import StandardLogger
from src.miscellaneous import validify as val

class Calibrator():
    ''' Performs manipulator to exernal CSYS calibration '''

    def __init__(self):
        '''
        Attributes:
            data (CalibrationData): CalibrationData object for holding data
            man_0 (list): Initial manipulator position [x, y, z, d] assoc. with ex_0
            ex_0 (list): Initial external CSYS position [x, y, z] assoc. with man_0
            is_calibrated (bool): Inidicator for whehter or not calibrtion exists
        '''
        self.data = CalibrationData()
        self.man_0 = None
        self.ex_0 = None
        self.is_calibrated = False

class CalibrationTransform():
    '''
    Fits calibration matrices to data and computes forward (manipulator -> external)
    and inverse (external -> manipulator) transformations
    '''
    def __init__(self, data:CalibrationData):
        '''
        Assumes linear system of
        x = A*x + B*u
        y = C*x + D*u
        where x is state, u is input, and y is output

        Arguments:
            data

        Attributes:
            A (np.ndarray): State to state matrix
            B (np.ndarray): Input to state matrix
            C (np.ndarray): State to output matrix
            D (np.ndarray): Input to output matrix
            man_0 (list): Initial manipulator position [x, y, z, d] assoc. with ex_0
            ex_0 (list): Initial external CSYS position [x, y, z] assoc. with man_0
            is_calibrated (bool): Inidicator for whehter or not calibrtion exists
        '''
        self._logr = StandardLogger()
        self.man_0 = None
        self.ex_0 = None
        self.is_calibrated = False

    def compute_transform_simplest(self, data, z_polarity:int, pip_angle:float):
        '''
        Computes the transformation matrices according to simplest relationship between
        and manipulator and external coordinate system.

        Assumptions:
            manipulator x, y orthogonal to external z
            manipulator d orthgonal to manipulator y
            manipulator z equivalent units to external z
            ex_1 = ex_0 + T_(m_ex)^(m_xyz) * T_(xyz)^(xyzd) * delta_(m_xyzd)

        Based on these assumptions, we only need to compute how the manipulator x and y
        relate to the external x and why (aka compute how to rotate, scale, and translate
        the manipulator x and y axes to align with the external x and y axes)

        M = manipulator coordinates, and E are external coordinates
        |Ex|        |T11    T12     dx|     |Mx|
        |Ey|    =   |T21    T22     dy| *   |My|
        |1 |        | 0      0       1|     |1 |

        Want to determine transformation matrix entries so put in form of y = Hx
            y        =        H       *      x
        |Px1 Py1|        |Mx1 My1 1|     |T11 T21|
        |Px2 Py2|    =   |Mx2 My2 1|  *  |T12 T22|
        | .   . |        | .   .  .|     |dx   dy|
        |Pxn Pyn|        |Mxn Myn 1|

        Solve with x = (H'H)^-1 H' y (regular least squares)



        Arguments:
            data (CalibrationData): Must contain at least 3 distinct x,y locations and the
                z and d positions should not change.
            z_polarity (int): Directionality between manipulator z and external z.
                Positive = same direction. Negative = opposite direction
            pip_angle (float): Angle of pipette below horizontal axis (radians)
        '''
        # Parse data into n x ? numpy arrays
        man_xy = data[['m_x','m_y']].to_numpy().reshape(-1,2)
        man_z = data['m_z'].to_numpy().reshape(-1,1)
        man_d = data['m_d'].to_numpy().reshape(-1,1)
        ex_xy = data[['ex_x','ex_y']].to_numpy().reshape(-1,2)
        ex_z = data['ex_z'].to_numpy().reshape(-1,1)

        # Compute range of values along man z, d and ex z axes (because they shouldn't change)
        range_mz = np.ptp(man_z)
        range_md = np.ptp(man_d)
        range_exz = np.ptp(ex_z)

        # Validate that positions of z and d don't change drastically
        nm2um = 1/1000 # Nanometer to micron
        man_thresh_um = 2 # 2 micron range
        ex_thresh_um = 2 # 2 micron range for z
        if range_mz*nm2um > man_thresh_um:
            raise ValueError(f'Range of manipulator z positions ({range_mz} exceeds ({man_thresh_um}) limit for simplest calibration.')
        if range_md*nm2um > man_thresh_um:
            raise ValueError(f'Range of manipulator d positions ({range_md} exceeds ({man_thresh_um}) limit for simplest calibration.')
        if range_exz > ex_thresh_um:
            raise ValueError(f'Range of external z positions ({range_exz} exceeds ({ex_thresh_um}) limit for simplest calibration.')

        # Least squares regression for x, y
        H = np.concatenate([man_xy, np.ones(man_xy.shape[0],1)], axis=1)
        y = np.copy(ex_xy)
        H_l = np.dot( np.linalg.inv( np.dot(H.T, H) ), H.T)
        x_hat1 = np.dot(H_l, y)
        x_hat2 = np.linalg.lstsq(H,y)
        if x_hat1 == x_hat2:
            self._logr.debug('Same least squares solution')
        else:
            self._logr.debug(f'Different LS soln.\nfSoln1: {x_hat1}\nSoln2: {x_hat2}')
        # Transpose back to desired orientaiton
        T_xy = x_hat.T

        # Transformation for man_z to ex_z (assume same units so just need direction)
        if z_polarity == 0:
            raise ValueError('Z polarity must be positive of negative number. Positive = same direction. Negative = opposite direction')
        elif z_polarity >0:
            T_z = 1
        else:
            T_z = -1

        # Transformation from manipulator x, y, z, d to manipulator x, y, z
        T_mxyzd_to_mxyz = np.array([[1, 0, 0, np.cos(pip_angle)],
                                    [0, 1, 0, 0],
                                    [0, 0, 1, np.sin(pip_angle)]])
        T_mxyz_to_exxyz = np.array([[T_xy[0,0], T_xy[0,1], 0],
                                   [T_xy[1,0], T_xy[1,1], 0],
                                   [0, 0, T_z]])

        # Construct system matrices
        # FIXME include deltas?
        n_ex = 3 # number of external states [x, y, z]
        n_m = 4 # number of manipulator states [x, y, z, d]
        n_h = 1 # number of homo
        A11 = np.eye(n_ex)
        A12 = np.zeros((n_ex,n_m))
        A21 = np.zeros((n_m,n_ex))
        A22 = np.eye(n_m)
        self.A = np.eye(n_ex+n_m)
        B11 = np.matmul(T_mxyz_to_exxyz,T_mxyzd_to_mxyz)
        B21 = np.eye(n_m)
        self.B = np.concatenate([B11,B12],axis=0)
        self.C = np.eye(n_ex+n_m)
        self.D = np.zeros((n_ex+n_m, n_ex+n_m))
        
        # Set initial state as last calibration position
        self.x_0 = data['ex_x',
                        'ex_y',
                        'ex_z',
                        'm_x',
                        'm_y',
                        'm_z',
                        'm_d'].to_numpy().T[:,-1].reshape(-1,1)

    def forward(self, man:list)->list:
        '''
        Computes the forward transformation from external to manipulator

        Arguments:
            man (list): Manipulator position as [x,y,z,d]
        
        Returns:
            list of computed external position as [x,y,z]
        '''
        _val_positions(man=man)
        man_1 = np.array(man).reshape(4,1)
        man_0 = self.x_0[3:]
        u = man_1 - man_0
        x_1 = np.dot(self.A, self.x_0) + np.dot(self.B, u)
        y_1 = np.dot(self.C, x_1) + np.dot(self.D, u)
        ex = y_1[:3]
        return ex

    def inverse(self, ex:list, man_axis_const:str)->list:
        '''
        Computes the inverse transformation from external to manipulator

        Arguments:
            ex (list): External position as [x,y,z]
            man_axis_const: Manipulator axis to hold constant in order to compute inverse
        
        Returns:
            list of computed manipulator position as [x,y,z,d]
        '''
        _val_positions(ex=ex)


class CalibrationData():
    ''' Class for storing/adding data for calibration '''

    def __init__(self):
        '''
        Attributes:
            data_df: DataFrame of [m_x, m_y, m_z, m_d, ex_x, ex_y, ex_z]
                m_x = manipulator x coordinate
                m_y = manipulator y coordinate
                m_z = manipulator z coordinate
                m_d = manipulator d coordinate
                ex_x = external CSYS x coordinate
                ex_y = external CSYS y coordinate
                ex_z = external CSYS z coordinate
        '''
        self._logr = StandardLogger(__name__)
        self._column_names = ['m_x','m_y','m_z','m_d','ex_x','ex_y','ex_z']
        self.data_df = pd.DataFrame(columns = self._column_names)

    def add_cal_position(self,man:list,ex:list)->None:
        '''
        Adds/appends manipulator and associated external CSYS positoin into DataFrame

        Arugments:
            man (list): List of manipulator positions [x, y, z, d]
            ex (list): List of external CSYS positions [x, y, z]
        '''
        # Validate argumetns
        _val_positions(man=man, ex=ex)
        # Create df to append
        concat_dict = {'m_x':man[0],
                       'm_y':man[1],
                       'm_z':man[2],
                       'm_d':man[3],
                       'ex_x':ex[0],
                       'ex_y':ex[1],
                       'ex_z':ex[2]}
        concat_df = pd.DataFrame(concat_dict)
        # Add to dataframe
        self.data_df = pd.concat([self.data_df, concat_df], axis=0, ignore_index=False)

    def rm_all(self):
        ''' Removes all data from the calibration data '''
        self.data_df = pd.DataFrame(columns = self._column_names)

    def rm_cal_ind(self, ind:int):
        '''
        Remove calibration position by index. Supports negative indexing. 
    
        Arguments:
            ind (int): Calibration position index to remove
        '''
        # Convert negative index to a postive index for dataframe
        if ind < 0:
            ind = self.df.shape[0] + ind
        # Validate argmetns
        cur_inds = list(self.data_df.index.values)
        if ind not in cur_inds:
            raise KeyError(f"Calibration index to remove ({ind}) not in data indices ({cur_inds})")
        # Remove data row
        try:
            self.df.drop(ind, axis = 0)
            self._logr.info(f'Removed calibration data at index {ind}')
        except KeyError:
            self._logr.exception('Failed to remove calibration data')
            raise

    def rm_cal_pos(self, man:list=None, ex:list=None):
        '''
        Remove calibration position by manipulator and/or external positoin. WARNING: multiple rows
        will be removed of the the argument exists for multiple rows.

        Usage:
            rm_cal_pos(man=some_pos)
            # Removes all rows where manipulator = some_pos
            rm_cal_pos(ex=other_pos)
            # Removes all rows where external = other_pos
            rm_cal_pos(man=some_pos, ex=other_pos)
            # Removes all rows where manipulator = some_pos AND external = other_pos

        Arguments:
            man (list): Manipulator position to remove [x, y, z, d]
            ex (list): List of external CSYS positions [x, y, z]
        '''
        # Validate argumetns
        _val_positions(man=man, ex=ex)
        ind_list = []
        # get index of manipulator positions
        if man is not None:
            x_match = self.data_df['m_x'] == man[0]
            y_match = self.data_df['m_y'] == man[1]
            z_match = self.data_df['m_z'] == man[2]
            d_match = self.data_df['m_d'] == man[3]
            all_match = x_match & y_match & z_match & d_match
            inds = list(all_match[all_match].index.values)
            ind_list.append(inds)
        if ex is not None:
            x_match = self.data_df['ex_x'] == ex[0]
            y_match = self.data_df['ex_y'] == ex[1]
            z_match = self.data_df['ex_z'] == ex[2]
            all_match = x_match & y_match & z_match & d_match
            inds = list(all_match[all_match].index.values)
            ind_list.append(inds)
        # Intersection of man and ex positoins if both args passed
        if len(ind_list) == 2:
            rm_inds = np.intersect1d(np.array(ind_list[0]),np.array(ind_list[0])).tolist()
        else:
            rm_inds = ind_list[0]
        # Remove rows
        try:
            df.drop(rm_inds)
            self._logr.info(f'Removed calibration data at indices {inds}')
        except KeyError:
            self._logr.exception(f'Failed to remove calibration data at man={man} and ex={ex}')
            raise

def _val_positions(man:list=None, ex:list=None):
    '''
    Validates the manipulator and external coordinates

    Arugments:
        man (list): List of manipulator positions [x, y, z, d]
        ex (list): List of external CSYS positions [x, y, z]
    '''
    if man is not None:
        if not val.is_of_types(man, [list, tuple]):
            raise TypeError(f'Manipulator position ({man}) must be list or tuple')
        if len(man) != 4:
            raise ValueError(f'Manipulator position ({man}) must have 4 entries [x,y,z,d]')
    if ex is not None:
        if not val.is_of_types(ex, [list, tuple]):
            raise TypeError(f'External position must ({ex}) be list or tuple')
        if len(ex) != 3:
            raise ValueError(f'External position ({ex}) must have 3 entries [x,y,z]')
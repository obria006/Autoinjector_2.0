''' Scripts for handling manipulator to coordinate system calibration '''
import time
import numpy as np
import pandas as pd
import traceback
from src.miscellaneous.standard_logger import StandardLogger
from src.miscellaneous import validify as val

class Calibrator():
    '''
    Controls manipulator <-> external CSYS calibration processes
    
    Attributes:
        data_path (str): Path to file containing calibration to load
        data (CalibrationData): Instance of CalibrationData class for storing data during
            calibration process.
        model (CalibrationModel): Instance of CalibrationModel class for computing transformation
            matrices and computing forward/inverse kinematics.
        '''

    def __init__(self, data_path:str=None):
        '''
        Arguments:
            data_path (str): Path to file containing calibration to load
        '''
        self.data_path = data_path  
        self.data = CalibrationData()
        self.model = CalibrationModel()

    def compute(self, z_polarity=-1, pip_angle=np.deg2rad(45)):
        '''
        Use CalibrationModel to compute the transformation matrices
        
        Arguments:
            z_polarity (int): Indicator of directionality between external z and manipulator z.
                Positive = same direction. Negative = opposite direction.
            pip_angle (float): Angle of pipette relative to horizontal axis. Co-linear with
                horizontal = 0. Pointing straight down = +pi/2 (90 degrees)
        '''
        if self.data.data_df.shape[0] < 3:
            raise CalibrationError('Calibration data must have at least 3 non-singular entries')
        else:
            data = self.data.data_df
            self.model.compute_transform_simplest(data, z_polarity, pip_angle)

    def update(self, ex:list, man:list):
        '''
        Update the calibration by setting a new reference position

        Arguments:
            ex (list): External position [x, y, z] associated with man
            man (list): Manipulator positoin [x, y, z, d] associated with ez
        '''
        _val_positions(ex=ex, man=man)
        new_x_0 = ex+man
        self.model.set_x_0(x_0=new_x_0)

    def load(self):
        '''
        Load the calibraiton from a file
        '''
        pass

class CalibrationData():
    '''
    Dataset for calibration. Has methods to add and remove datapoints to the dataset.

    Attributes:
        _logr (StandardLogger): Logger for logging errors/warnings
        _column_names (list): List of column names in DataFrame
        data_df (pd.DataFrame): DataFrame for storing external and manipulator positions.
            DataFrame has form of [ex_x, ex_y, ex_z, m_x, m_y, m_z, m_d] where:
                ex_x = external CSYS x coordinate
                ex_y = external CSYS y coordinate
                ex_z = external CSYS z coordinate
                m_x = manipulator x coordinate
                m_y = manipulator y coordinate
                m_z = manipulator z coordinate
                m_d = manipulator d coordinate
    '''

    def __init__(self):
        self._logr = StandardLogger(__name__)
        self._column_names = ['ex_x','ex_y','ex_z','m_x','m_y','m_z','m_d']
        self.data_df = pd.DataFrame(columns = self._column_names)

    def add_cal_position(self,ex:list,man:list)->None:
        '''
        Adds/appends manipulator and associated external CSYS position into DataFrame

        Arugments:
            man (list): List of manipulator positions [x, y, z, d]
            ex (list): List of external CSYS positions [x, y, z]
        '''
        # Validate argumetns
        _val_positions(man=man, ex=ex)
        # Create df to append
        concat_dict = {'ex_x':[ex[0]],
                       'ex_y':[ex[1]],
                       'ex_z':[ex[2]],
                       'm_x':[man[0]],
                       'm_y':[man[1]],
                       'm_z':[man[2]],
                       'm_d':[man[3]],
                       }
        concat_df = pd.DataFrame(concat_dict)
        # Add to dataframe
        self.data_df = pd.concat([self.data_df, concat_df], axis=0, ignore_index=True)

    def rm_all(self):
        ''' Removes all data from the calibration data '''
        self.data_df = pd.DataFrame(columns = self._column_names)

    def rm_cal_ind(self, ind:int):
        '''
        Remove calibration position by index. Supports negative indexing. 
    
        Arguments:
            ind (int): Calibration position index to remove. The index is the index value in
                the DataFrame (not necessarily its position in the DataFrame because rows can
                be deleted, but indices don't change)
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

    def rm_cal_pos(self, ex:list=None, man:list=None):
        '''
        Remove calibration position by manipulator and/or external positoin. WARNING: multiple rows
        will be removed of the the argument exists for multiple rows.

        Usage:
            # Remove all rows where manipulator = some_pos
            rm_cal_pos(man=some_pos)
            # Remove all rows where external = other_pos
            rm_cal_pos(ex=other_pos)
            # Remove all rows where manipulator = some_pos AND external = other_pos
            rm_cal_pos(man=some_pos, ex=other_pos)
            
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

class CalibrationModel():
    '''
    Uses data to fit calibration matrices and computes forward (manipulator -> external) and
    inverse (external -> manipulator) transformations.

    Attributes:
        A (np.ndarray): State to state matrix
        B (np.ndarray): Input to state matrix
        C (np.ndarray): State to output matrix
        D (np.ndarray): Input to output matrix
        T_mxyzd_to_mxyz (np.ndarray): delta 4-axis man. pos. to delta 3-axis man. pos. matrix
        T_mxyz_to_exxyz (np.ndarray): delta 3-axis man. pos. to delta 3-axis ext. pos matrix
        x_0 (np.ndarray): Reference state matrix
        is_calibrated (bool): Inidicator for whehter or not calibrtion exists

    The model assumes a simple linear model for the system. Where the systems position (X) at some
    instance is its intial position plus the change in position. Formally: 

    X(k+1) = X(k) + delta_X(k)                                          (1)

    For the manipulator-microscope system, we set the states/positions as the column vector:

    X = (ex_x, ex_y, ex_z, m_x, m_y, m_z, m_d)^T                        (2)

    Where ex=external/microscope position of the pipette tip and m=manipulator position of the
    pipette tip. _x, _y, _z, _d denote the axis where _d is the manipulator's diagnol axis.

    In the manipulator-microscope system, we always have access to the initial position (because 
    the user will define the starting position) and we have constant access to the manipulator's
    positions. Therefore, we want to redefine delta_X(k) only in terms of the manipulator position,
    X_m.

    X(k+1) = X(k) + T_hat*delta_X_m(k)                                  (3)

    T_hat = T_(ex<-m_3)*T_(m_3<-m_4)                                    (4)

    T_hat is a transformation matrix that transforms a change in manipulator coordinates to a
    change in external coordinates. It is a composite of two transformations. The first
    transformation, T_(m_3<-m_4), transforms the manipulators 4-axis positions [x, y, z, d], to a
    psuedo-3 axis position (aka the position of the manipulator if it didn't have a diagnol axis).
    The second transformation, T_(ex<-m_3), transforms a psuedo-3 axis manipulator position to a
    3-axis external position. This transfomration handles the rotations and scalings between the
    camera x, y, microscope z coordinates and the manipulator's x, y, z coordinates.

    Ultimately, the general form for the system in canonical linear system form is:

    X(k+1) = A*X(k) + B*U(k)                                            (5)
    Y(k+1) = C*X(k) + D*U(k)                                            (6)

    where
    X is defined in (2)
    U = delta_X_m(k) (change in manipulator position)
    Y = external and manipulator positoins (aka Y = X)
    A = Identity (eye(7))
    B = T_(ex<-m_3)*T_(m_3<-m_4)
    C = Idendity (eye(7))
    D = zeros (zeros(3,4))

    Observing this model, the only thing that will change between calibrations is the B matrix.
    Aka T_(ex<-m_3)*T_(m_3<-m_4). So calibration is focused on defining these matrices. The
    copmutation of the forward and inverse kinematics are therefore independent of the type of
    calibration (probably, but the inverse might have some problems if B_prime ends up being 
    singular). The X(k) is the reference state from which future positions are computed. The
    reference state is taken to be the final calibration positoin 
    '''
    def __init__(self):
        self._logr = StandardLogger(__name__)
        self.reset_calibration()

    def reset_calibration(self):
        '''
        Resests the calibration attributes (A, B, C, D, x_0) to None and is_calibrated to False
        '''
        self.A = None
        self.B = None
        self.C = None
        self.D = None
        self.x_0 = None
        self.T_mxyzd_to_mxyz = None
        self.T_mxyz_to_exxyz = None
        self.pixel_size_nm = None
        self.is_calibrated = False
        
    def compute_transform_simplest(self, data:CalibrationData, z_polarity:int, pip_angle:float):
        '''
        Computes the transformation matrices according to simplest relationship between
        and manipulator and external coordinate system.

        Arguments:
            data (CalibrationData): Must contain at least 3 distinct x,y locations and the
                z and d positions should not change.
            z_polarity (int): Directionality between manipulator z and external z.
                Positive = same direction. Negative = opposite direction
            pip_angle (float): Angle of pipette below horizontal axis (radians)

        Assumptions:
            (1) manipulator d orthgonal to manipulator y
            (2) manipulator x, y orthogonal to manipulator z
            (3) manipulator z is parallel to external z
            (4) manipulator z and external z use real-world units
                (manipulator uses nm and external z uses um)

        With these assumptions we can define our transformation matrices T_(ex<-m_3), T_(m_3<-m_4) 

        Assumption 1 and 2 yield T_(m_3<-m_4) where column vectors are deltas (position changes):

            | m_x_prime |       | 1  0  0  cos(theta) | | m_x |
            | m_y_prime |   =   | 0  1  0  0          | | m_y |
            | m_y_prime |       | 0  0  1  sin(theta) | | m_z |
                                                        | m_d |

        Assumption 3 and 4 yield T_(ex<-m_3) where column vectors are deltas (position changes):

            | ex_x |       | a  b  0            | | m_x_prime |
            | ex_y |   =   | c  d  0            | | m_y_prime |
            | ex_y |       | 0  0  (+/- um2nm)  | | m_y_prime |

        Based on these assumptions and the defined matrices , we only need to compute how the
        changes in manipulator x and y relate to changes in the external x and y (aka compute how
        to rotate and scale displacements of the manipulator to achieve displacements of the
        external position. Therefore we can use the following form.

        |ex_x|          |T11    T12     dx|     |m_x|
        |ex_y|    =     |T21    T22     dy| *   |m_y|
        | 1  |          | 0      0       1|     | 1 |

        Where the T.. indicate the rotation and scaling and the d. indicate displacement between
        coordinate system origins (which isn't used since we use a reference position X(k))

        Want to determine transformation matrix entries so put in form of y = Hx
            y        =        H       *      x
        |Ex1 Ey1|        |Mx1 My1 1|     |T11 T21|
        |Ex2 Ey2|    =   |Mx2 My2 1|  *  |T12 T22|
        | .   . |        | .   .  .|     |dx   dy|
        |Exn Eyn|        |Mxn Myn 1|

        Solve with x = (H'H)^-1 H' y (regular least squares)
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
        H = np.concatenate([man_xy, np.ones((man_xy.shape[0],1))], axis=1).astype(np.float64)
        y = np.copy(ex_xy).astype(np.float64)
        x_hat, res_, rank_, sing_ = np.linalg.lstsq(H,y,rcond=None)
        # Transpose back to desired orientaiton
        T_xy = x_hat.T

        # Transformation for man_z to ex_z (assume same units so just need direction)
        if z_polarity == 0:
            raise ValueError('Z polarity must be positive of negative number. Positive = same direction. Negative = opposite direction')
        elif z_polarity >0:
            T_z = 1*nm2um
        else:
            T_z = -1*nm2um

        # Transformation from manipulator x, y, z, d to manipulator x, y, z
        self.T_mxyzd_to_mxyz = np.array([[1, 0, 0, np.cos(pip_angle)],
                                    [0, 1, 0, 0],
                                    [0, 0, 1, np.sin(pip_angle)]])
        self.T_mxyz_to_exxyz = np.array([[T_xy[0,0], T_xy[0,1], 0],
                                   [T_xy[1,0], T_xy[1,1], 0],
                                   [0, 0, T_z]])

        # Construct system matrices
        n_ex = 3 # number of external states [x, y, z]
        n_m = 4 # number of manipulator states [x, y, z, d]
        n_h = 1 # number of homo
        A11 = np.eye(n_ex)
        A12 = np.zeros((n_ex,n_m))
        A21 = np.zeros((n_m,n_ex))
        A22 = np.eye(n_m)
        self.A = np.eye(n_ex+n_m)
        B11 = np.matmul(self.T_mxyz_to_exxyz,self.T_mxyzd_to_mxyz)
        B21 = np.eye(n_m)
        self.B = np.concatenate([B11,B21],axis=0)
        self.C = np.eye(n_ex+n_m)
        self.D = np.zeros((n_ex+n_m, n_m))

        # Compute pixel size
        self.pixel_size_nm = self._compute_pixel_size(self.B[0,0],
                                                      self.B[0,1],
                                                      self.B[1,0],
                                                      self.B[1,1])
        
        # Set initial state as last calibration position
        self.x_0 = data[['ex_x',
                        'ex_y',
                        'ex_z',
                        'm_x',
                        'm_y',
                        'm_z',
                        'm_d']].to_numpy().T[:,-1].reshape(-1,1)

        # Set calibration boolean
        self.is_calibrated = True

    def _compute_pixel_size(self,B_xx, B_xy, B_yx, B_yy):
        '''
        Computes pixel size from B matrix assuming man x-y parallel to ex x-y

        delta(ex_x) = B_xx*delta(m_x) + B_xy*delta(m_y)
        delta(ex_y) = B_yx*delta(m_x) + B_yy*delta(m_y)

        In simplified form this means
        change in pixels = scaling * change in manipulator (nm)

        so we compute scaling = pixel size
        '''
        x_pixels_per_nm = np.linalg.norm(np.array([B_xx, B_xy]))
        y_pixels_per_nm = np.linalg.norm(np.array([B_yx, B_yy]))
        pix_per_nm = np.mean(np.array([x_pixels_per_nm, y_pixels_per_nm]))
        pix_size_nm = 1/pix_per_nm
        return pix_size_nm

    def set_x_0(self, x_0:list):
        '''
        Sets the reference state, x_0, from which new positions are referenced.

        Arguments:
            x_0 (list): New reference state in form of 
                ['ex_x', 'ex_y', 'ex_z', 'm_x', 'm_y', 'm_z', 'm_d']. Where 'ex'
                is external and 'm' is manipulator position
        '''
        if not val.is_of_types(x_0, ['list', 'tuple']):
            raise TypeError(f'Desired updated reference state {x_0} must be a list or tuple')
        if len(x_0) != 7:
            raise ValueError(f"Desired updated reference state {x_0} must have 7 entries according to ['ex_x', 'ex_y', 'ex_z', 'm_x', 'm_y', 'm_z', 'm_d']")
        self.x_0 = np.array(x_0).reshape(7,1)

    def forward(self, man:list)->list:
        '''
        Computes the forward transformation from external to manipulator

        Arguments:
            man (list): Manipulator position as [x,y,z,d]
        
        Returns:
            list of computed external position as [x,y,z]
        '''
        if self.is_calibrated:
            _val_positions(man=man)
            # Compute change in manipulator positoin
            man_1 = np.array(man).reshape(4,1)
            man_0 = self.x_0[3:]
            u = man_1 - man_0
            # Compute states and ouputs
            x_1 = np.dot(self.A, self.x_0) + np.dot(self.B, u)
            y_1 = np.dot(self.C, x_1) + np.dot(self.D, u)
            # Only return the external position (the first the entries in our state/output matrix)
            ex = y_1[:3]
            return ex.reshape(-1).tolist()
        else:
            raise CalibrationError('Cannot compute forward kinematics because calibration is incomplete')

    def forward_mxyzd_to_mxyz(self, del_man_4:list)->list:
        '''
        Computes forward kinematics of a change in 4-axis manipulator position
        to 3axis manipulator position

        Arguments:
            del_man_4 (list): Change in manipulator coordinates [x, y, z, d]
        
        Returns
            Change in 3-axis manipulator coordinates [x, y, z]
        ''' 
        if self.is_calibrated:
            _val_positions(man=del_man_4)
            del_man_4_arr = np.array(del_man_4).reshape(4,1)
            del_man_3_arr = np.dot(self.T_mxyzd_to_mxyz, del_man_4_arr)
            return del_man_3_arr.reshape(-1).tolist()
        else:
            raise CalibrationError('Cannot compute forward kinematics because calibration is incomplete')

    def inverse_mxyz_to_mxyzd(self, del_man_3:list)->list:
        '''
        Computes inverse kinematics of a change in 3-axis manipulator position
        to 4-axis manipulator positoin.

        Arguments:
            del_man_3 (list): Change in manipulator coordinates [x, y, z]
        
        Returns
            Change in 3-axis manipulator coordinates [x, y, z]
        ''' 
        if self.is_calibrated:
            if not val.is_of_types(del_man_3, [list, tuple]):
                raise TypeError(f'Delta-3 manipulator position ({del_man_3}) must be list or tuple')
            if len(del_man_3) != 3:
                raise ValueError(f'Delta-3 manipulator position ({del_man_3}) must have 3 entries [x,y,z]')
            T_mxyz_to_mxyzd = np.reciprocal((self.T_mxyzd_to_mxyz.T).astype(np.float64), where = self.T_mxyzd_to_mxyz.T>0)
            del_man_3_arr = np.array(del_man_3).reshape(3,1)
            del_man_4_arr = np.dot(T_mxyz_to_mxyzd, del_man_3_arr)
            return del_man_4_arr.reshape(-1).tolist()
        else:
            raise CalibrationError('Cannot compute forward kinematics because calibration is incomplete')


    def inverse(self, ex:list, man_axis_const:str)->list:
        '''
        Computes the inverse transformation from external to manipulator

        Arguments:
            ex (list): External position as [x,y,z]
            man_axis_const: Manipulator axis to hold constant in order to compute inverse
        
        Returns:
            list of computed manipulator position as [x,y,z,d]

        The B matrix transforms a change in manipulator position to a change in external position.
        Therefore we can use the 'inverse' B matrix to try and compute a change in manipulator
        position for a change in external postion. However, it can't be completely determined
        because B isn't invertible. But if we hold one of the manipulator axes constant, then we can
        modify the B matrix.

        For example our B matrix is generically (where column vectors are changes in position)

        | ex_x |  = [a b c d]   [m_x]
        | ex_y |  = [e f g h]   [m_y]
        | ex_y |  = [i j k l]   [m_z]
                                [m_d]

        B isn't square so it cant be inverted, but if we know that d is constant (for example)

        | ex_x |  = [a b c]   [m_x]
        | ex_y |  = [e f g]   [m_y]
        | ex_y |  = [i j k]   [m_z]

        and now we can solve for m_.
        '''
        if self.is_calibrated:
            _val_positions(ex=ex)
            if man_axis_const not in ['x', 'y', 'z', 'd']:
                raise ValueError(f"Constant manipulator axis {man_axis_const} must be in ['x', 'y', 'z', 'd']")
            # Compute change in external positoin
            ex_0 = self.x_0[:3]
            ex_1 = np.array(ex).reshape(3,1)
            delta_ex = (ex_1 - ex_0).astype(np.float64)
            # Remove the column in the B matrix that has the constant manipulator axis position
            axis_dict = {'x':0, 'y':1, 'z':2, 'd':3}
            B11 = self.B[:3,:]
            B_prime = np.delete(B11, axis_dict[man_axis_const], axis = 1)
            # Solve for the change in the manipulator position
            delta_man_prime = np.linalg.solve(B_prime, delta_ex)
            delta_man = np.insert(delta_man_prime, axis_dict[man_axis_const], 0).reshape(4,1)
            # Compute the manipulator position
            man_0 = self.x_0[3:]
            man_1 = man_0 + delta_man
            return man_1.reshape(-1).tolist()
        else:
            raise CalibrationError('Cannot compute forward kinematics because calibration is incomplete')


class CalibrationError(Exception):
    '''
    Exception raised for errors with calibration process

    Attributes:
        msg (str): Explanation of errror
    '''
    def __init__(self, msg="Calibration error occured"):
        self.msg = msg
        super().__init__(self.msg)


def _val_positions(ex:list=None, man:list=None):
    '''
    Validates the manipulator and external coordinates

    Arugments:
        man (list): List of manipulator positions [x, y, z, d]
        ex (list): List of external CSYS positions [x, y, z]
    '''
    if ex is not None:
        if not val.is_of_types(ex, [list, tuple]):
            raise TypeError(f'External position must ({ex}) be list or tuple')
        if len(ex) != 3:
            raise ValueError(f'External position ({ex}) must have 3 entries [x,y,z]')
    if man is not None:
        if not val.is_of_types(man, [list, tuple]):
            raise TypeError(f'Manipulator position ({man}) must be list or tuple')
        if len(man) != 4:
            raise ValueError(f'Manipulator position ({man}) must have 4 entries [x,y,z,d]')

if __name__ == "__main__":
    cal = Calibrator()
    cal.data.add_cal_position([788.0, 711.0, 98.4], [13673885, 7824242, 16301830, 14980624])
    cal.data.add_cal_position([780.0, 267.0, 98.4], [13673866, 8394526, 16301826, 14980624])
    cal.data.add_cal_position([294.0, 272.0, 98.4], [14291597, 8394509, 16301837, 14980624])
    cal.compute(z_polarity=-1, pip_angle=np.deg2rad(44.9))
    # print('A\n',cal.model.A,'\n')
    print('B\n',cal.model.B[:3,:],'\n')
    # print('C\n',cal.model.C,'\n')
    # print('D\n',cal.model.D,'\n')
    print('x\n',cal.model.x_0,'\n')
    # ex_test = [454.0, 533.0, 98.4]
    # man_test = [14092375, 8057759, 16301828, 14980624]
    # ex_fwd = cal.model.forward(man_test)
    # ex_fwd = [round(pos,1) for pos in ex_fwd]
    # print('EX REF', ex_test)
    # print('EX FWD', ex_fwd)
    # man_inv = cal.model.inverse(ex_test,'d')
    # man_inv = [int(pos) for pos in man_inv]
    # print('MAN REF', man_test)
    # print('MAN FWD', man_inv)


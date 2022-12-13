"""
WORK IN PROGRESS calibration functions for the `CalibrationModel` class in
calibration.py. These functions enable computing 3D transformation/calibration
fuctions (rather than just 2D [x, y] and assuming constant scalings ).

These functions would be used with- or replace the `compute_transform_simplest`
function in `calibration.py`.
"""

def compute_transform_z(self, data:CalibrationData):
    """
    Computes the Z-transformation.
    """
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
    if range_md*nm2um > man_thresh_um:
        raise ValueError(f'Range of manipulator d positions ({range_md} exceeds ({man_thresh_um}) limit for simplest calibration.')       

    # Least squares regression for z
    H = np.concatenate([man_z, np.ones((man_xy.shape[0],1))], axis=1).astype(np.float64)
    y = np.copy(ex_z).astype(np.float64)
    x_hat, res_, rank_, sing_ = np.linalg.lstsq(H,y,rcond=None)
    # Transpose back to desired orientaiton
    T_z = x_hat.T
    # Just the scaling not the translation
    T_z = T_z[0,0].astype(float)

    # Inser the z-scaling in the matrix
    self.T_mxyz_to_exxyz[2,2] = T_z

    # Define initial (reference) state as last calibration position
    x_0 = data[['ex_x',
                'ex_y',
                'ex_z',
                'm_x',
                'm_y',
                'm_z',
                'm_d']].to_numpy().T[:,-1].reshape(-1,1).astype(np.float64)

    # Set the system model
    self.set_model(self.T_mxyzd_to_mxyz, self.T_mxyz_to_exxyz, x_0)

def compute_transform_harder(self, data:CalibrationData, z_polarity:int, pip_angle:float):
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
    NM_TO_UM = 1000
    # Parse data into n x ? numpy arrays
    man_xy = data[['m_x','m_y']].to_numpy().reshape(-1,2)
    man_z = data['m_z'].to_numpy().reshape(-1,1)
    man_d = data['m_d'].to_numpy().reshape(-1,1)
    ex_xy = data[['ex_x','ex_y']].to_numpy().reshape(-1,2)
    ex_z = data['ex_z'].to_numpy().reshape(-1,1)

    sorted_man_z_inds = np.argsort(man_z.reshape(-1))
    sorted_man_z = man_z[sorted_man_z_inds].reshape(-1)
    sorted_ex_z = ex_z[sorted_man_z_inds].reshape(-1)
    delta_man_z = np.diff(sorted_man_z)
    large_diffs = delta_man_z > (5*NM_TO_UM)
    large_diffs = np.insert(large_diffs,0,False)
    split_sort_inds = np.where(large_diffs)[0]
    man_z_groups = np.array_split(sorted_man_z,split_sort_inds)
    avg_man_z = np.asarray([np.mean(group) for group in man_z_groups]).reshape(-1,1)
    ex_z_groups = np.array_split(sorted_ex_z,split_sort_inds)
    avg_ex_z = np.asarray([np.mean(group) for group in ex_z_groups]).reshape(-1,1)

    # Compute range of values along man z, d and ex z axes (because they shouldn't change)
    range_mz = np.ptp(man_z)
    range_md = np.ptp(man_d)
    range_exz = np.ptp(ex_z)

    # Validate that positions of z and d don't change drastically
    nm2um = 1/1000 # Nanometer to micron
    man_thresh_um = 2 # 2 micron range
    ex_thresh_um = 2 # 2 micron range for z
    if range_md*nm2um > man_thresh_um:
        raise ValueError(f'Range of manipulator d positions ({range_md} exceeds ({man_thresh_um}) limit for simplest calibration.')
    

    # Least squares regression for x, y
    H = np.concatenate([man_xy, np.ones((man_xy.shape[0],1))], axis=1).astype(np.float64)
    y = np.copy(ex_xy).astype(np.float64)
    x_hat, res_, rank_, sing_ = np.linalg.lstsq(H,y,rcond=None)
    # Transpose back to desired orientaiton
    T_xy = x_hat.T

    # Least squares regression for z
    H = np.concatenate([avg_man_z, np.ones((avg_man_z.shape[0],1))], axis=1).astype(np.float64)
    y = avg_ex_z.astype(np.float64)
    z_hat, res_, rank_, sing_ = np.linalg.lstsq(H,y,rcond=None)
    # Transpose back to desired orientaiton
    T_z = z_hat.T
    print(avg_man_z.astype(np.float64))
    print(avg_ex_z.astype(np.float64))
    # print(T_z)

    # # Transformation for man_z to ex_z (assume same units so just need direction)
    # if z_polarity == 0:
    #     raise ValueError('Z polarity must be positive of negative number. Positive = same direction. Negative = opposite direction')
    # elif z_polarity >0:
    #     T_z = 1*nm2um
    # else:
    #     T_z = -1*nm2um
    T_ex = np.array([[T_xy[0,0], T_xy[0,1], 0],
                    [T_xy[1,0], T_xy[1,1], 0],
                    [0, 0, T_z[0,0]]]).astype(np.float64)

    # Define initial (reference) state as last calibration position
    x_0 = data[['ex_x',
                'ex_y',
                'ex_z',
                'm_x',
                'm_y',
                'm_z',
                'm_d']].to_numpy().T[:,-1].reshape(-1,1).astype(np.float64)
    print(T_ex)
    print(x_0)
    print()

    # # Transformation from manipulator x, y, z, d to manipulator x, y, z
    # self.T_mxyzd_to_mxyz = np.array([[1, 0, 0, np.cos(pip_angle)],
    #                                  [0, 1, 0, 0],
    #                                  [0, 0, 1, np.sin(pip_angle)]]).astype(np.float64)
    # self.T_mxyz_to_exxyz = np.array([[T_xy[0,0], T_xy[0,1], 0],
    #                                  [T_xy[1,0], T_xy[1,1], 0],
    #                                  [0, 0, T_z]]).astype(np.float64)

    # # Define initial (reference) state as last calibration position
    # x_0 = data[['ex_x',
    #             'ex_y',
    #             'ex_z',
    #             'm_x',
    #             'm_y',
    #             'm_z',
    #             'm_d']].to_numpy().T[:,-1].reshape(-1,1).astype(np.float64)

    # # Set the system model
    # self.set_model(self.T_mxyzd_to_mxyz, self.T_mxyz_to_exxyz, x_0)

import time
from math import ceil
import traceback
import numpy as np
import pandas as pd
from PyQt6.QtCore import QObject, QThread, pyqtSignal
from src.manipulator_control.calibration import Calibrator
from src.manipulator_control.sensapex_utils import SensapexDevice, UMP
from src.miscellaneous.standard_logger import StandardLogger 

class InjectionConductor3D(QObject):
    """
    Controls the 4-axis sensapex manipulator to conduct prototypical injections into tissue.

    Attributes:
        dev (SensapexDevice): Object that manipulates the sensapex manipulator
        axes (dict): Relates sensapex axis label with its axis number. {'x':0,'y':1,'z':2,'d':3}
    """

    finished = pyqtSignal()
    def __init__(self, dev:SensapexDevice):
        super().__init__()
        self._logr = StandardLogger(__name__)
        self.dev = dev
        self.axes = {'x':0,'y':1,'z':2,'d':3}

    def injection_moves(self, inj_pos:list, approach_d_nm:int, depth_d_nm:int, speed_ums:int):
        '''
        Perform manipulator movements associated with the injection process. This function
        assumes the micromanipulator is starting at a non-injection position (micropipette
        is out of the tissue).

        Arguments:
            inj_pos (list): Manipulator coordinates of the injection location [x, y, z, d].
                This is the position the manipulator will be at during the injection.
            approach_d_nm (int): Approach distance in nm along manipulator injection axis (d axis)
                This should be the d-axis displacement from the edge/point annotation of the
                injection position that the manipulator should start approaching for injections.
            depth_d_nm (int): Depth distance in nm along manipulator injection axis (d axis).
                This should correspond to the d-axis displacement past the edge/point annotation
                of the injection position that the manipulator will achieve during injections.
            speed_ums (int): Speed of manipulator movement in um/s.

        Assuming the manipulator is at a non-injection position, there are 3 prototypical
        positions along the injection trajectory:
            1. Preinjection position:
                Injection (d) axis  aligned with the injection position with the tip located
                "approach distance" + "injection depth" away injection position.
            2. Injection position:
                pipette tip in focus at the desired injection position. This position is acheived
                by advancing the injection-axis by "approach distance" + "injection distance".
            3. Retraction position:
                injection (d) axis is retraceted by "approach distance" + "injection distance"
                to the preinjection position.

        Movement trajectory for injection position (Pinject):
            0. Currently at non-injection position (P0)
            1. Retract d axis to position defined as:
                P1 = [P0_x, P0_y, P0_z, Pinject_d - approach - depth]
            2. Move x,y,z axes to align injection axis with injection position:
                P2 = [Pinject_x, Pinject_y, Pinject_z, Pinject_d - approach - depth]
            3. Advance injection axis to injection position
                P3 = [Pinject_x, Pinject_y, Pinject_z, Pinject_d]
            4. Retract injection axes:
                P4 = P2 = [Pinject_x, Pinject_y, Pinject_z, Pinject_d - approach - depth]
        '''
        # Current position
        cur_pos = self.dev.get_pos()

        # Move to pre-injection position by first retract d then move xyz
        pre_inj_pos = inj_pos[:]
        pre_inj_pos[self.axes['d']] -= (approach_d_nm+depth_d_nm)
        ret_d_pos = cur_pos[:]
        ret_d_pos[self.axes['d']] = pre_inj_pos[self.axes['d']]
        move_req = self.dev.goto_pos(ret_d_pos, speed=speed_ums)
        while move_req.finished is False:
            time.sleep(0.1)
        move_req = self.dev.goto_pos(pre_inj_pos, speed=speed_ums)
        while move_req.finished is False:
            time.sleep(0.1)

        # Injection postition
        move_req = self.dev.goto_pos(inj_pos, speed=speed_ums)
        while move_req.finished is False:
            time.sleep(0.2)

        # Retraction pos
        ret_pos = pre_inj_pos[:]
        move_req = self.dev.goto_pos(ret_pos, speed=speed_ums)
        while move_req.finished is False:
            time.sleep(0.1)

        # Emit signal that injection is finished
        self.finished.emit()

    def finishing_moves(self, fin_pos:list, speed_ums:int):
        '''
        Move to the final position at the end of an injection trial. Assumes that the
        manipulator is at a retraction position (injection axis aligned with tissue, but
        retracted and outside of tissue).

        Arguments:
            fin_pos (list): Manipulator coordinates of the finish location [x, y, z, d].
                Likely will be the coordinates of last injection annotation with a displaced
                x-axis.
            speed_ums (int): Speed of manipulator movement in um/s.

        Movement trajectory for finish position (Pfin):
            0. Currently at retracted position (P0)
            1. Retract x axis to position defined as:
                P1 = [Pfin_x, P0_y, P0_z, P0]
            2. Move to finish position
                P2 = [Pfin_x, Pfin_y, Pfin_z, Pfin_d]

        At a retraction position, the d-axis will be retracted from the d-position defined by
        fin position. So if d was advanced first then it could collide with tissue. Thus, retract
        x first to move away from tissue then advance d.
        '''
        # Current position
        cur_pos = self.dev.get_pos()

        # Move to finish location by retract x first then advance d
        ret_x_pos = cur_pos[:]
        ret_x_pos[self.axes['x']] = fin_pos[self.axes['x']]
        move_req = self.dev.goto_pos(ret_x_pos, speed=speed_ums)
        while move_req.finished is False:
            time.sleep(0.1)
        move_req = self.dev.goto_pos(fin_pos, speed=speed_ums)
        while move_req.finished is False:
            time.sleep(0.1)
        self._logr.info('Moved to finishing position.')

    def estop_moves(self, estop_pos:list, estop_d_nm:int, speed_ums:int):
        '''
        Move to conduct when user clicks stop the injection. Pipette may be in tissue, 
        so retract d, retract x, then advace d

        Arguments:
            estop_pos (list): Manipulator coordinates of the estop position location [x, y, z, d].
            estop_d_nm (int): Displacement of d axis to retract and then advance.
            speed_ums (int): Speed of manipulator movement in um/s.
        '''
        # Current position
        cur_pos = self.dev.get_pos()

        # Move to estop, by retract d, retract x then advace d
        ret_d_pos = cur_pos[:]
        ret_d_pos[self.axes['d']] -= estop_d_nm
        move_req = self.dev.goto_pos(ret_d_pos, speed=speed_ums)
        while move_req.finished is False:
            time.sleep(0.1)

        ret_x_pos = ret_d_pos[:]
        ret_x_pos[self.axes['x']] = estop_pos[self.axes['x']]
        move_req = self.dev.goto_pos(ret_x_pos, speed=speed_ums)
        while move_req.finished is False:
            time.sleep(0.1)

        move_req = self.dev.goto_pos(estop_pos, speed=speed_ums)
        while move_req.finished is False:
            time.sleep(0.1)
        self._logr.warning('Moved to emergency stop position.')


class SurfacePointTrajectory3D(QThread):
    """
    Handles trajectory along set of points located at the tissue apical/basal surfaces
    
    Attributes:
        dev (SensapexDevice): Object that manipulates the sensapex manipulator
        cal (Calibrator): Object of micromanipulator <-> external calibraiton
        ex_points_3D (list): Ordered list of desired injection position in external CSYS
        approach_nm (int): Approach distance (in nm) along image x-axis that manipulator will
            begin injection trajectory
        depth_nm (int): Distance (in nm) along image x-axis that manipulator will surpass annotated
            injection position during injection trajectory
        speed_ums (int): Speed of manipulator movement in um/s.
        pullout_nm (int): Distance along (in nm) x-axis to retract manipulator
        _stop_pressed (bool): Indicator that stop button is pressed
        _move_complete (bool): Inidcator that movement is complete and can go to next position.
        n_injected (int): Number of injections completed
    """

    def __init__(self, dev:SensapexDevice, cal:Calibrator, ex_points_3D:list, approach_nm:int, depth_nm:int, speed_ums:int, pullout_nm:int):
        """
        Arguments:
            dev (SensapexDevice): Object that manipulates the sensapex manipulator
            cal (Calibrator): Object of micromanipulator <-> external calibraiton
            ex_points_3D (list): Ordered list of desired injection position in external CSYS
                list of [[x,y,z], ..., [x,y,z]]
            approach_nm (int): Approach distance (in nm) along image x-axis that manipulator will
                begin injection trajectory
            depth_nm (int): Distance (in nm) along image x-axis that manipulator will surpass annotated
                injection position during injection trajectory
            speed_ums (int): Speed of manipulator movement in um/s.
            pullout_nm (int): Distance along (in nm) x-axis to retract manipulator
        """
        super().__init__()
        self._logr = StandardLogger(__name__)
        self.dev = dev
        self.cal = cal
        self.ex_points_3D = ex_points_3D
        self.approach_nm = approach_nm
        self.depth_nm = depth_nm
        self.speed_ums = speed_ums
        self.pullout_nm = pullout_nm
        self._stop_pressed = False
        self._move_complete = True
        self.n_injected = 0

    def __del__(self):
        self.wait()

    def run(self):
        self.implement_trajectory()

    def _compute_inj_position(self, des_pos_ex:list) -> list:
        '''
        Copmute manipulator position of injection position in external CSYS using the calibration
        and injection depth
        
        Arguments:
            des_pos_ex (list): Desired injection position in external CSYS [x, y, z]

        Returns
            list of computed manipulator position as [x, y, z, d]
        '''
        # Compute the position of the manipulator at this position without chanigng d axis
        des_pos_m = self.cal.model.inverse(des_pos_ex, man_axis_const='d')
        des_pos_m[0] += self.depth_nm
        return des_pos_m

    def del_3axis_to_d(self, del_3axis:list)->int:
        del_4axis = self.cal.model.inverse_mxyz_to_mxyzd(del_3axis)
        del_d = del_4axis[3]
        return del_d

    def _compute_fin_position(self, des_pos_ex:list)->list:
        '''
        Copmute manipulator position of pullout position in external CSYS using the calibration
        and pullout displacement
        
        Arguments:
            des_pos_ex (list): Desired injection position in external CSYS [x, y, z]

        Returns
            list of computed manipulator position as [x, y, z, d]
        '''
        # Compute the position of the manipulator at this position without chanigng d axis
        des_pos_m = self.cal.model.inverse(des_pos_ex, man_axis_const='d')
        des_pos_m[0] -= self.pullout_nm
        return des_pos_m

    def _compute_estop_pos(self)->list:
        '''
        Copmute manipulator position of emergency stop position in external CSYS using the
        calibration and pullout displacement and current position

        Returns
            list of computed manipulator position as [x, y, z, d]
        '''
        cur_pos_m = self.dev.get_pos()
        des_pos_m = cur_pos_m[:]
        des_pos_m[0] -= self.pullout_nm
        self._logr.debug(f"Computed e-stop position: {des_pos_m}")
        return des_pos_m
        
    def implement_trajectory(self):
        """
        Function to thread that coordinates manipulator movments to injections
        """
        try:
            self.conduct_injection = InjectionConductor3D(self.dev)
            self.conduct_injection.finished.connect(self._next_move)
            for i in range(len(self.ex_points_3D)):
                self._move_complete = False
                self._logr.info(f"Injecting position {i}...")

                # Stop button presed
                if self._stop_pressed == True:
                    fin_pos_ex = self.ex_points_3D[i]
                    fin_pos_m = self._compute_fin_position(fin_pos_ex)
                    self.conduct_injection.finishing_moves(fin_pos_m, self.speed_ums)
                    break

                # Get the position of the injection in external CSYS
                des_inj_pos = self.ex_points_3D[i]

                # Get positoin in man CSYS and approach and depth distances along d
                des_pos_m = self._compute_inj_position(des_pos_ex=des_inj_pos)

                # Get approach distance along d from x
                del_3axis_approach_nm = [self.approach_nm, 0, 0]
                approach_d_nm = self.del_3axis_to_d(del_3axis_approach_nm)

                # Get depth deistance along d from x
                del_3axis_depth_nm = [self.depth_nm, 0, 0]
                depth_d_nm = self.del_3axis_to_d(del_3axis_depth_nm)

                # Run trajecotory
                self.conduct_injection.injection_moves(des_pos_m, approach_d_nm, depth_d_nm, self.speed_ums)

                # Keep waiting until injected, but then move to next position when current position injected
                while self._move_complete == False:
                    time.sleep(0.1)

                self.n_injected = i+1

                if i == len(self.ex_points_3D) - 1:
                    self._logr.info(f"Completed {i+1} injections")
                    #pull out last injection command...
                    fin_pos_ex = self.ex_points_3D[-1]
                    fin_pos_m = self._compute_fin_position(fin_pos_ex)
                    self.conduct_injection.finishing_moves(fin_pos_m, self.speed_ums)
        except:
            print(traceback.format_exc())
            print('except')

    def _next_move(self):
        """ Sets boolean to true so manipulator moves to next position """
        self._move_complete = True

    def stop(self):
        """ Sets boolean to true to stop injections """
        self._stop_pressed = True


class SurfaceLineTrajectory3D(SurfacePointTrajectory3D):
    """
    Handles trajectory along line defining apical/basal surfaces
    
    Attributes:
        dev (SensapexDevice): Object that manipulates the sensapex manipulator
        cal (Calibrator): Object of micromanipulator <-> external calibraiton
        edge_3D (list): List of interpolated edge coordinates in 3D in external CSYS.
            list of [[x,y,z], ..., [x,y,z]]
        ex_points_3D (list): Ordered list of desired injection position in external CSYS
        approach_nm (int): Approach distance (in nm) along image x-axis that manipulator will
            begin injection trajectory
        depth_nm (int): Distance (in nm) along image x-axis that manipulator will surpass annotated
            injection position during injection trajectory
        speed_ums (int): Speed of manipulator movement in um/s.
        pullout_nm (int): Distance along (in nm) x-axis to retract manipulator
        _stop_pressed (bool): Indicator that stop button is pressed
        _move_complete (bool): Inidcator that movement is complete and can go to next position.
        n_injected (int): Number of injections completed
    """

    def __init__(self, dev:SensapexDevice, cal:Calibrator, edge_3D:list, approach_nm:int, depth_nm:int, spacing_nm:int, speed_ums:int, pullout_nm:int):
        """
        Arguments:
            dev (SensapexDevice): Object that manipulates the sensapex manipulator
            cal (Calibrator): Object of micromanipulator <-> external calibraiton
            edge_3D (list): List of interpolated edge coordinates in 3D in external CSYS.
                list of [[x,y,z], ..., [x,y,z]]
            approach_nm (int): Approach distance (in nm) along image x-axis that manipulator will
                begin injection trajectory
            depth_nm (int): Distance (in nm) along image x-axis that manipulator will surpass annotated
                injection position during injection trajectory
            spacing_nm (int): Distance between injection positions along line
            speed_ums (int): Speed of manipulator movement in um/s.
            pullout_nm (int): Distance along (in nm) x-axis to retract manipulator
        """
        self._logr = StandardLogger(__name__)
        self.edge_3D = edge_3D
        ex_points_3D = self._line_to_points(cal, spacing_nm)
        super().__init__(dev, cal, ex_points_3D, approach_nm, depth_nm, speed_ums, pullout_nm)

    def _line_to_points(self, cal:Calibrator, spacing_nm:int)->list:
        """
        Convert trajectory line into distinct points for injection
        
        Arguments:
            cal (Calibrator): Object of micromanipulator <-> external calibraiton
            spacing_nm (int): Distance between injection positions along line

        Returns:
            list of injection points along line in external csys
        """
        # Get the pixel size from the calibration
        pix_size_nm = cal.model.pixel_size_nm
        # Assume each entry in edge coords is 1 pixel
        n_pix = len(self.edge_3D)
        n_inj = int(ceil((n_pix*pix_size_nm) / spacing_nm))
        pixel_spacing = spacing_nm / pix_size_nm
        self._logr.debug(f'For edge of {n_pix} pixels @ {round(pix_size_nm)}nm large with {spacing_nm}nm spacing, doing {n_inj} injections located {pixel_spacing} pixels apart')
        inj_points = []
        for i in range(n_inj):
            try:
                inj_points.append(self.edge_3D[int(i*pixel_spacing)])
            except IndexError:
                pass
        return inj_points
from PyQt6.QtWidgets import *
from PyQt6.QtCore import *
import os, sys, time
from src.sensapex_utils.sensapex_utils import SensapexDevice, UMP
from src.motorcontrol.injectioncontrolmod import injection 
import numpy as np

class motorcontroller_improved(QThread):
    """
    QThread class to control 4-axis sensapex manipulators and perform a
    prototypical injection into the tissue. This was designed for a 4-axis
    manipulator but includes code for 3-axis. It was tested in 3-axis mode
    by only letting the manipulator use the y, z, and d axes. It appeared to
    work as expected. Use the orignal "motorcontroller" if you encounter issues
    while using this class for 3-axis manipulator control.
    
    This class assumes you want to move from a micropipette tip located on the
    edge of a tissue to perform injections at the next location. There are 4
    stereo typed positions along an injection trajectory
        1. Preinjection position:
                pipette injection-axis (x-axis for 3 axes manipulators and
                d-axis for 4 axes manipulators) aligned with the injection
                position with the tip located "approach distance" away from the
                tissue edge (along the injection-axis).
        2. Injection position:
                pipette tip in focus at the desired injection position. This
                position is acheived by advancing the injection-axis by
                sum("approach distance" + "injection distance")
        3. Retracted position:
                pipette tip is out-of-focus at the edge of the tissue. This
                position is acheived by retracting the injection axis by the
                "injection distance"
        4. Pullout positon:
                pipette tip further retracts from the edge of the tissue by
                retracting injection axis by "approach distance". THis is so
                the pipette doesn't collide with the tissue when displacing
                laterally to the next preinjection positoin

    This class is designed to move between "retracted positions" corresponding
    to different injection coordinates.
    
    Arguments:
        inj_ind: Index of injection (starting at 1)
        dx: (nm) required manipulator displacement along a "pure" x-axis to get
            from the current "retracted position" to the next "retracted
            position". "Pure" meaning that the displacement occurs in the image
            plane (not a mixture of moving in the image plane and along the focal
            axis).
        dy: (nm) required manipulator displacement along its y-axis to get from
            the current "retracted position" to the next "retracted position".
        dx_rate: Amount of "pure x-axis" displacement that occurs for a
            1 unit move along the injection axis.
        dz_rate: Amount of z-axis/focal-axis displacement that occurs for a
            1 unit move along the injection axis.
        approachdist: (nm) Distance from which to start approaching tissue for
            injection.
        injectiondepth: (nm) Distance to injecte below annotated tissue surface
        speed: (um/s) Speed of injections
    """

    signal = pyqtSignal(str)

    def __init__(self, inj_ind, dx, dy, dx_rate, dz_rate, approachdist, injectiondepth, speed):
        QThread.__init__(self)
        self.ump = UMP.get_ump()
        self.devids = self.ump.list_devices()
        self.devs = {i:SensapexDevice(i) for i in self.devids}
        self.inj_ind = inj_ind
        self.stepsizex = dx
        self.stepsizey = dy
        self.dx_rate = dx_rate
        self.dz_rate = dz_rate
        self.approachdist = approachdist
        self.injectiondepth = injectiondepth
        self.speed = speed
        if len(self.devs[1].get_pos()) == 3:
            self.n_axes = 3
            self.axes = {'x':0,'y':1,'z':2,'d':0}
        elif len(self.devs[1].get_pos()) == 4:
            self.n_axes = 4
            self.axes = {'x':0,'y':1,'z':2,'d':3}

    def __del__(self):
        self.wait()

    def run(self):
        self.run_trajectory()

    def run_trajectory(self):
        '''
        Trajectory assumes your starting at a retraction position. Trajectory
        path:

        1. If you're doing the first injection (tip in inj-plane not at tissue)
                a. Move z axis away from tissue by inj. depth projected on z
                   axis
        2. Go to "Pullout position" by retracting injection axis by the
            "approach distance".
        3. Go to "Preinjection position" by displacing x-,y- axes by the
            specified/computed displacements.
        4. Go to "Injection position" by advancing injection axis by the
            sum("approach distance" + "injection depth")
        5. Go to "Retraction position" by retracing injection axis by
            "injection depth".
        6. Repeat 1-6
        7. If you're not doing any more injections
                a. Move z-axis towards from tissue by inj. depth projected on z
                   axis (to bring back in focus).
                b. Retract x-axis by 40 um to move away from tissue.
        '''
        position0 = self.devs[1].get_pos() #current position
        print("position0 = " +str(position0))
        
        #sometimes there is an error with the sensapex manipulator and it says its position is zero, this ruins the trajectory
        if position0[0] < 100:
            print('sensapex error, wait')
            time.sleep(0.3)
            position0 = self.devs[1].get_pos() #the delay should fix it
        
        # If first injection, then displace z axis so the injection positiongs are in focus
        start_pos = position0[:]
        if self.inj_ind == 1:
            start_pos[self.axes['z']] -= int(self.injectiondepth*self.dz_rate)
            move_req=self.devs[1].goto_pos(start_pos, self.speed)
            print(f'Starting position:\t{start_pos}',end='... ')
            while move_req.finished is False:
                time.sleep(0.1)
            print('position reached')
        
        # Retract injection axis (by approach distance)
        pullout_pos = start_pos[:]
        pullout_pos[self.axes['d']] -= int(self.approachdist)
        move_req = self.devs[1].goto_pos(pullout_pos, self.speed)
        print(f'Pull out position:\t{pullout_pos}',end='... ')
        while move_req.finished is False:
            time.sleep(0.1)
        print('position reached')

        # Move laterally along x and y to align with next injection postion
        preinject_pos = pullout_pos[:]
        preinject_pos[self.axes['y']] += int(self.stepsizey)
        # When moving x-axis for 3axis manip your actually moving "down" and "forward"
        if self.n_axes == 3:
            delta_inj_axis = self.stepsizex / self.dx_rate
            preinject_pos[self.axes['x']] += int(delta_inj_axis)
            preinject_pos[self.axes['z']] -= int(self.dz_rate*delta_inj_axis)
        elif self.n_axes == 4:
            preinject_pos[self.axes['x']] += int(self.stepsizex)
        move_req = self.devs[1].goto_pos(preinject_pos, self.speed)
        print(f'Preinject position:\t{preinject_pos}',end='... ')
        while move_req.finished is False:
            time.sleep(0.1)
        print('position reached')

        # go into tissue and inject
        inject_pos = preinject_pos[:]
        inject_pos[self.axes['d']] += int((self.injectiondepth + self.approachdist))
        move_req = self.devs[1].goto_pos(inject_pos, self.speed)
        print(f'Injection position:\t{inject_pos}',end='... ')
        while move_req.finished is False:
            time.sleep(0.1)
        print('position reached')
        time.sleep(0.1)

        # retract to tissue edge
        retract_pos = inject_pos[:]
        retract_pos[self.axes['d']] -= int(self.injectiondepth)
        move_req = self.devs[1].goto_pos(retract_pos, self.speed)
        print(f'Retraction position:\t{retract_pos}',end='... ')
        while move_req.finished is False:
            time.sleep(0.01)
        print('position reached')
               

    def finalpullout(self, dist, zdist):
        current_pos = self.devs[1].get_pos()

        #in some cases, sensapex does not report a correct xyz pos, thus we wait
        while len(current_pos) < 3:
            current_pos = self.devs[1].get_pos()
            time.sleep(0.01)      
            print("current pos =" + str(current_pos))

        #in some cases, sensapex says xaxis = 2, but this is just a bug so we wait
        while current_pos[self.axes['x']] == 2:
            current_pos = self.devs[1].get_pos()
            time.sleep(0.01)
            print("error, new current pos =" + str(current_pos))

        # Displace tissue away from edge and down into focal plane
        end_pos = current_pos[:]
        end_pos[self.axes['x']] -= dist
        end_pos[self.axes['z']] += self.injectiondepth*self.dz_rate
        if self.n_axes == 3:
            end_pos[2] += dist*self.dz_rate
        print("end pos =" + str(end_pos))
        self.devs[1].goto_pos(end_pos, 1000)


#---------------------Thread Class for motor position----------------------------
class motorpositionThread(QThread):
    """ 
    Qthread class. This class handles getting current position of motors and sending
    out motor information to GUI, no input required.
    """
    motorpos = pyqtSignal(list)
    motorposnum = []

    def __init__(self):
        QThread.__init__(self)
        ump = UMP.get_ump()
        self.devids = ump.list_devices()
        self.devs = {i:SensapexDevice(i) for i in self.devids}

    def __del__(self):
        self.wait()

    def _get_position(self):
        """
        asks motors for position, return [x,y,z] in uM 
        """ 
        self.print_pos()
        self.current_motor = self.devs[1].get_pos()
        print(self.current_motor)
        try:
            self.motorposition = self.pos_numerical
            return self.motorposition
        except:
            print('Manipulator error')

    def print_pos(self,timeout=None):
        line = ""
        for i in self.devids:
            self.dev = self.devs[i]
            try:
                pos = str(self.dev.get_pos(timeout=timeout))
                pos_numerical = self.dev.get_pos(timeout=timeout)
                self.pos_numerical = pos_numerical
            except Exception as err:
                pos = str(err.args[0])
                self.pos_numerical = [0,0,0]
            self.pos = pos + " " * (30 - len(pos))

    def run(self):
        """ 
        what actually calls _get_position and emits the signal
        """

        while True:
            motorposition = self._get_position()
            try:
                self.motorpos.emit(motorposition)
            except:
                print('manipulator error')
            self.sleep(1)
            
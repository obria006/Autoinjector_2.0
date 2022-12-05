"""
Functions/classes for implimenting object tracking to track injection targets
during microinjection process. Microinjecion needles probing the tissue may
cause tissue displacement, so tracking can be used to improve targetting
accuracy on displaced/displacing tissue
"""

import os
import cv2
import math
import numpy as np
import pandas as pd
from PyQt6.QtCore import (pyqtSlot,
                          pyqtSignal,
                          QObject,
                          QTimer,
                         )
from src.tracking.kabsch import kabsch


class TrackingManager(QObject):
    """
    Coordinates instantiation and initialization of object trackers and modifes
    the annotation and injection trajectories based on the result of the
    tracking. (AKA it modifies the injection trajectory and annotations as it
    tracks the features)

    !! You must make a connection with  the `update_frame()` slot in order for
    tracking to occur. The `update_frame()` emits a signal (internally to the 
    instance of this class) that a new frame is available and the tracker acts
    on this `new_frame_available` signal to track the updated frame. Therefore,
    the connection to `update_frame()` should recieve a frame everytime a new
    frame is available from the camera !!

    Signals:
        new_frame_available: emitted when recieves a new frame
        errors (Exception): emitted when error occurs during tracking

    Slots:
        update_frame (np.ndarray): pass new video frames here for tracking
        on_tracked_points (list): Takes tracking result and modifies annotation
            and trajectory.
    """

    new_frame_available = pyqtSignal()
    errors = pyqtSignal(Exception)

    def __init__(self, annot_mgr, inj_trajectory):
        super().__init__()
        self.annot_mgr = annot_mgr
        self.inj_trajectory = inj_trajectory
        self.frame = None
        self._is_tracking = True
        self.R_TOL = 0.035 # rotation of 2 degrees (arcsin(0.035) = 2 degree)
        self.T_TOL = 3 # pixel translation

        # Everytime new trajectory is started, init and start new tracking
        # Needs a new tracker for each new trajectory because the annotation
        # or tracking points may not be visible until the trajectory is
        # started. Aka the next annotation/tracking points may be out of focus
        # while the manipulator is injecting in a different focal plane. Hence,
        # the median flow tracker cannot track points it cannot see.
        self.inj_trajectory.trajectory_ready.connect(lambda: QTimer.singleShot(250,self._start_new_tracking))

    @pyqtSlot()
    def update_frame(self, frame:np.ndarray):
        """
        Conduct tracking in the new video frame.

        This emits a signal of a new frame which can be connected to the
        trackers `track()` method. This method isn't directly called here
        because the tracker may be being created or not exist when a frame
        is recieved. Therefore, with the signal, the tracker only handles
        new frames when it is able.
        
        Args:
            frame (np.ndarray): Video frame for next tracking computation
        """
        # Set frame attribute and emit signal to do track on new frame
        if self._is_tracking is True:
            self.frame = np.copy(frame)
            self.new_frame_available.emit()

    @pyqtSlot()
    def on_tracked_points(self, new_points:np.ndarray):
        """
        Handles what to do with the newly tracked points.

        Computes rotation/translation from intial point positions to positions
        of newly tracked points. Updates the annotation and injection positions
        if the rotation/translation exceed a threshold.

        Args:
            new_points (np.ndarray): Coordinates of tracked points
        """
        try:
            if self._is_tracking is True:
                # Copmute rotation/tranlation of tracked points since start tracking
                A = np.copy(new_points)
                B = self.p0[:,0,0:2]
                R,t,H = kabsch(A, B)
                # Evaluate how rotation and translation matrix has changed since tracking
                R_delta = np.abs(np.eye(R.shape[0]) - R)
                t_delta = np.abs(t)
                # Update the points if it exceeds the threshold
                if np.any(R_delta > self.R_TOL) or np.any(t_delta > self.T_TOL):
                    for ind, a in enumerate(self.annot0):
                        # Set new annotation
                        A = np.array(a)
                        arr2 = (np.matmul(R, A[:,0:2].T) + t).T
                        arr2 = np.concatenate((arr2.astype(int), A[:,2].reshape((-1,1))), axis = 1)
                        self.annot_mgr.set_annotation_by_ind(arr2.tolist(), ind)
                        # Set new injection position
                        A = np.array(self.inj0[ind])
                        arr2 = (np.matmul(R, A[:,0:2].T) + t).T
                        arr2 = np.concatenate((arr2.astype(int), A[:,2].reshape((-1,1))), axis = 1)
                        self.inj_trajectory.set_positions(arr2.tolist(), ind)
        except Exception as e:
            self.stop()
            self.errors.emit(e)

    def stop(self):
        """ Stop tracking """
        self._is_tracking = False
        if hasattr(self, "tracker"):
            self.tracker.stop()
            del self.tracker

    def _start_new_tracking(self):
        """
        Create a new tracker and start tracking.
        
        This is meant to be called everytime a new injection trajectory is
        started (aka after the microscope as focussed to the desired focal
        plane, and the manipulator is starting injections).
        """
        try:
            if self._is_tracking is True:
                # The initial coordinates of the annotation and injections before tracking
                self.annot0 = list(self.annot_mgr.get_annotations(type_='raw', coords='xyz'))
                self.inj0 = [self.inj_trajectory.positions(ind) for ind in range(len(self.inj_trajectory._trajectories))]
                # Sample points to track from interpolated annotation
                points = self._points_from_annotation(self.inj_trajectory.trajectory_index)
                self.p0 = np.copy(points)
                # Intialize median flow tracker
                self.tracker = TrackerMFLK(self.frame, points)
                # Handle what to do when tracker returns newly tracked points
                self.tracker.new_points.connect(lambda p: self.on_tracked_points(p))
                # When a new video frame is available, send to tracker for compute new points
                self.new_frame_available.connect(lambda: self.tracker.track(self.frame))
                # Stop tracking when the trajecotyr is done
                self.inj_trajectory.trajectory_finished.connect(self.tracker.stop)
                # Begin tracking
                self.tracker.start()
        except Exception as e:
            self.stop()
            self.errors.emit(e)

    def _points_from_annotation(self, ind:int) -> np.ndarray:
        """
        Sample points from the interpolated annotation to generate a sparse set
        of points for tracking.
        
        Args:
            ind (int): Index of annotation from which to sample points
            
        Returns:
            np.ndarray with shape (n,1,d) of points to track
        """
        # Get the xyz interpolated annotation and index `ind`
        annots = self.annot_mgr.get_annotations(type_='interpolated', coords='xyz')
        annot = annots[ind]
        # Total points in annotation and min points in sampled annotation
        length = len(annot)
        N_MIN = 10
        if length <= N_MIN:
            raise ValueError(f"Annotation is too short (length={length}) to track. Must be greater than {N_MIN}")        
        # Period of sampling frequency (period = number of points between samples)
        NOMINAL_PERIOD = 15
        if length / NOMINAL_PERIOD < 10:
            period = math.floor(length / N_MIN)
        else:
            period = NOMINAL_PERIOD
        # Sample points from annotation with computed period
        points = annot[::int(period)]
        points = np.array(points)
        points = points[:,np.newaxis,:].astype(np.float32)
        return points


class TrackerMFLK(QObject):
    """
    Object tracker using Median flow solved with Lucas-Kanade algo.
    
    Signals:
        new_points (list): Updates/measured points computed from LK algo.
        new_state (pd.Dataframe): Dataframe of tracked points and status
        
    Slots:
        track: Pass image frame to conduct object tracking
    """
    new_points = pyqtSignal(list)
    new_state = pyqtSignal(pd.DataFrame)

    def __init__(self, frame:np.ndarray, points:list):
        """
        Args:
            frame (np.ndarray): Initial image frame from camera
            points (list): Coordinates points in image frame to track
            dt (int): Time between tracking events in ms
        """
        super().__init__()
        assert len(frame.shape) == 2, "Must be a grayscale frame"
        self._validate_points(points)
        # Boolean to conduct tracking
        self._is_active = False
        # Parameters for Lucas-Kanade optical flow algorithim
        self.lk_params = dict(
            winSize = (31,31), # kernel size to track around each point
            maxLevel = 3, # number of levels in pyramid
            criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03)
            )
        # Set frame for image gradient
        self.prev_frame = np.copy(frame)
        # The cv2 optical flow tracker expects features/points as an np.ndarray
        # with shape = (n,1,2), so recast `points` to match
        points = self._recast_points(points)
        self._state = pd.DataFrame({
            'x0':points[:,0,0],
            'y0':points[:,0,1],
            'z0':points[:,0,2],
            'x':points[:,0,0],
            'y':points[:,0,1],
            'z':points[:,0,2],
            'status':[1]*len(points),
        })
        # Set features
        self.prev_points = points[:,:,0:2]
        self._x0 = self.prev_points[:,0,:]

    @pyqtSlot()
    def track(self, frame:np.ndarray):
        """
        Compute optical flow tracking for new image `frame`.

        Args:
            frame (np.ndarray): Initial image frame from camera
        """
        if self._is_active:
            frame = np.copy(frame)
            # LK algo on new image frame
            points, status, errors = cv2.calcOpticalFlowPyrLK(self.prev_frame, frame, self.prev_points, None, **self.lk_params)
            # get the 'good' points
            if points is not None:
                # self._update_state(points, status)
                # good_points = points
                good_points = points[status==1]
                # Emit signal of points
                self._send_tracked_points(good_points)
                # Set previous timestep for next iteration
                self.prev_frame = frame
                self.prev_points = np.copy(good_points)
                if len(self.prev_points.shape) ==2:
                    self.prev_points = self.prev_points[:,np.newaxis,:]

    def start(self):
        self._is_active = True

    def stop(self):
        self._is_active = False
        self.deleteLater()

    def is_tracking(self) -> bool:
        """ Returns boolean of whether actively tracking """
        return self._is_active

    def _validate_points(self, points):
        """ Validate that points are x,y or xyz style list or array """
        if isinstance(points, list):
            points = np.array(points)
        if len(points) == 0:
            raise ValueError("Cannot track empty points.")
        valid_super_shapes = [(1,2), (1,3), (2,), (3,)]
        if points[0].shape not in valid_super_shapes:
            raise ValueError(f"Invalid shape of points {points}. Final dimensions must be in {valid_super_shapes}")

    def _recast_points(self, points):
        """ The cv2 optical flow tracker expects features/points as an np.ndarray
        with shape = (n,1,2), so recast `points` to match
        """
        if isinstance(points, list):
            points = np.array(points)
        if points[0].shape not in [(1,2), (1,3)]:
            points = points[:,np.newaxis,:]
        return points.astype(np.float32)

    def _send_tracked_points(self, points:np.ndarray):
        """
        Emit list of 'points`.
        
        Args:
            points (np.ndarray): Array of tracked points from LK algo
        """
        # Convert points to correctly sized list
        points = np.copy(points).astype(int)
        if len(points.shape) == 2:
            points_list = points.tolist()
        elif len(points.shape) == 3:
            assert points.shape[1] == 1, f"Unexpected shape for tracked points: {points.shape}"
            points_list = points[:,0,:].tolist()
        else:
            raise ValueError(f"Unexpected shape for tracked points: {points.shape}. Expects (n,1,2) or (n,2)")
        self.new_points.emit(points_list)

    # def _update_state(self, points:np.ndarray, status:np.ndarray):
    #     self._state.loc[:,'x'] = self.prev_points[:,0,0]
    #     self._state.loc[:,'y'] = self.prev_points[:,0,1]
    #     self._state.loc[:,'status'] = status[:]
    #     self.new_state.emit(self._state)
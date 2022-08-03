'''
Scripts for handling threading of functions and classes.

Worker classes to be threaded should follow the template below.

class Worker(QObject)

    # pyqtSignal must be class attribute
    finished = pyqtSignal()
    def __init__(self, *args, **kwargs):
        super().__init__()

    # Function called in threading manager
    def run(self):
        self.long_running_func()

    # The task to be threaded
    def long_running_func(self):
        # Do stuff
        self.finished.emit()
'''
from PyQt6.QtCore import QObject, QThread, pyqtSignal, pyqtBoundSignal

class aQThreader():
    ''' 
    Manages threads for PyQT. Inspired from RealPython QThread article. 
    
    Attributes:
        thread (QThread): QThread object to handle worker thread
        worker (QObject): QObject with function to be threaded. Worker must
            have "finished" class attribute as pyQtSignal, "run" method, and
            emit the "finished" signal when "run" is complete.

    Usage:
        # Prototypical usage in GUI main event loop
        worker = QWorker(foo, args, kwargs)
        threaded_func = aQThreader(worker)
        # Other connections to worker.finished
        threaded_func.start()
    '''
    def __init__(self, worker:QObject):
        '''
        Instantate aQThreader object by validate correct worker object and creating a
        thread.

        Arguments:
            worker (QObject): QObject with function to be threaded. Worker must
                have "finished" class attribute as pyQtSignal, "run" method, and
                emit the "finished" signal when "run" is complete.
        '''
        # Validate worker is a QObject with finished attribute and run method
        if not isinstance(worker, QObject):
            raise TypeError('worker must be a QObject')
        if not hasattr(worker, 'finished'):
            raise AttributeError('worker must have a "finished" attribute')
        if type(worker.finished) != type(pyqtSignal()) and type(worker.finished) != type(pyqtBoundSignal()):
            raise TypeError(f'worker "finished" attiribute must be a pyqtSignal. {type(worker.finished)} is an invalid type for "finished".')
        if not hasattr(worker,'run'):
            raise AttributeError('worker must have "run" method')
        elif callable(worker.run) is False:
            raise AttributeError('worker has "run" attribute but it is not callable.')
        self.thread = QThread()
        self.worker = worker

    def start(self):
        self.worker.moveToThread(self.thread)
        self.thread.started.connect(self.worker.run)
        self.worker.finished.connect(self.thread.quit)
        self.worker.finished.connect(self.worker.deleteLater)
        self.thread.finished.connect(self.thread.deleteLater)
        self.thread.start()

class aQWorker(QObject):
    '''
    Worker QObject to be threaded by QThread.

    User passes a function (and its arguments) during instantiation. When
    used as input to aQthreader, the passed function is threaded to 
    improve GUI responsiveness (when the fcn normally takes long to run).

    Class Attributes:
        finished: pyqtSignal to be emitted when function is complete
    
    Instance Attributes:
        func_to_thread: the "long running" function to be threaded
        *args: arguments for func_to_thread
        **kwargs: keyword arguments for func_to_thread

    '''

    finished = pyqtSignal()
    def __init__(self, func_to_thread, *args, **kwargs):
        '''
        Creates worker object to be threaded.

        Arguments:
            func_to_thread: Function from ZEN class to be threaded
        '''
        super().__init__()
        self.func_to_thread = func_to_thread
        self.args = args
        self.kwargs = kwargs

    def run(self):
        '''
        Runs the pass function to be threaded. Threading manager requires
        worker to have run method.
        '''
        self._run_function()

    def _run_function(self):
        '''
        Execture function and emit finsihed signal when complete
        '''
        self.func_to_thread(*self.args, **self.kwargs)
        self.finished.emit()

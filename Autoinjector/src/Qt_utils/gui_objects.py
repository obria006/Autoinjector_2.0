''' Homebrewed objects for PyQT GUI '''
from PyQt6 import QtWidgets as QWid

class QHLine(QWid.QFrame):
    ''' Horizontal separator line widget. Don't set alignment when calling '''
    def __init__(self):
        super().__init__()
        self.setMinimumWidth(1)
        self.setFixedHeight(20)
        self.setFrameShape(QWid.QFrame.Shape.HLine)
        self.setFrameShadow(QWid.QFrame.Shadow.Sunken)
        self.setSizePolicy(QWid.QSizePolicy.Policy.Preferred, QWid.QSizePolicy.Policy.Minimum)

class QVLine(QWid.QFrame):
    ''' Vertical separator line widget. Don't set alignment when calling '''
    def __init__(self):
        super().__init__()
        self.setMinimumWidth(20)
        self.setFixedHeight(1)
        self.setFrameShape(QWid.QFrame.Shape.VLine)
        self.setFrameShadow(QWid.QFrame.Shadow.Sunken)
        self.setSizePolicy(QWid.QSizePolicy.Policy.Preferred, QWid.QSizePolicy.Policy.Minimum)
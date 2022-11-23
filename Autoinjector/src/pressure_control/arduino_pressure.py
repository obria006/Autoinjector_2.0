""" Interfaces with Arduino for pressure control """
import serial
from PyQt6.QtCore import QObject, QTimer

class ArduinoPressure(QObject):
    """
    Starts serial connection with Arduino and provides fucntions for sending
    pressure commands to the Arduino. 
    
    Arduino expects string commands with specific formats.
        backpressure: used to apply constant pressure to pipette for
        unclogging. Arduino expects commands like 'bc__C'. 'b' in the first
        character slot indicates back pressure and values contatined between
        'c' and 'C' indicate the digital pressure value to send to the
        electronic pressure controller.
        
        purge: Used to apply brief (500 ms) pulse of high pressure to unclog
        an already clogged pipette. Arduino expects 'x' in the first character
        slot.
    """

    def __init__(self, com:str, baud:int=9600, timeout:int=5):
        """
        Args:
            com (str): COM port of pressure control Arduino. Like 'com3'.
            baud (int): Baud rate of Arduino.
            timeout (int): Timeout of Arduino buffer.
        """
        super().__init__()
        if "com" not in com:
            raise ValueError(f"Invalid Arudino COM port name: {com}. Must include 'com'. Ex: 'com3' is valid.")
        if baud not in [4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 500000]:
            raise ValueError(f"Invalid Arduino baud rate: {baud}.")
        self.arduino = serial.Serial(com, baud, timeout=timeout)

    def backpressure(self, pres: int):
        """ Send backpressure command to arduino (to tell pressure controller 
        what pressure to apply and open the solenoid valve).
        
        Args:
            pres (int): Value for pressure controller as 0 (min) to 255 (max).
        """
        pres_command = "b" +"c" + str(pres) +"C"
        self.sendcommand(pres_command)

    def purge(self):
        """
        Purge pressure command to attempt to unclog pipette. Closes low
        pressure valve, open and close high pressure valve, and reopen low
        pressure valve.
        """
        pres_command = 'x'
        self.sendcommand(pres_command)

    def sendcommand(self, send:str):
        """
        Send command to Arduino.
        
        Args:
            send (str): Command to send to Arduino.
        """
        self.arduino.flush()
        self.arduino.write(send.encode('utf8'))
        # Recieve message from Arduino after 100ms
        QTimer.singleShot(100, self.listen)

    def listen(self):
        """ Recieve message from Arduino """
        response = self.arduino.read(self.arduino.inWaiting())
        print(response)

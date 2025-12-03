from pySerialTransfer import pySerialTransfer as txfer
from pySerialTransfer.pySerialTransfer import InvalidSerialPort
import time
import numpy as np


class ArduinoHandler:
    """
    Handles connections and messaging to an Arduino.

    Attributes:
        conn:   PySerialTransfer connection; has None value when no successsful
                connection has been made
        port:   name of connection port currently being used; has None value when
                no successful port has been used
    """

    def __init__(self, printer, port):
        self.conn = None
        self.port = port
        self.printer = printer

        

    def connect(self):
        """
        Initializes a connection to an arduino at a specified port. If successful,
        the conn and port attributes are updated
        """
        if self.conn is None:
            try:
                self.conn = txfer.SerialTransfer(self.port)
                self.port = self.port
                self.conn.open()
                time.sleep(2)
                self.printer(f"Arduino Connection initialized using port {self.port}")
            except InvalidSerialPort:
                self.printer("Could not connect to arduino, disabling")
                self.conn = None
                self.port = None
        else:
            self.printer(f"Connection already initialized at port {self.port}, new port {self.port} ignored")
   
   
    def send(self, Bx, By, Bz, alpha, gamma, freq, psi, gradient_status, equal_field_status, acoustic_freq) -> None:
        """
        sends action commands to arduino

         Args:
            actions = [Bx, By, Bz, alpha, gamma, freq]
        """
        
        alpha = round(alpha,3)
        gamma = round(gamma,3)
        psi = round(psi,3)
        freq = round(freq,3)
        Bx = round(Bx,3)
        By = round(By,3)
        Bz = round(Bz,3)

        data = [float(Bx), float(By), float(Bz), float(alpha), float(gamma), float(freq),float(psi), float(acoustic_freq), float(gradient_status), float(equal_field_status), 0.0]
        if self.conn is None:
            #self.printer("Connection not initialized..."+ str(data))  
            #self.printer("No Connection:  "+ "Bx: {},    By: {},    Bz: {},    alpha: {},    gamma: {},    freq: {},    psi: {}".format(Bx,By,Bz,alpha,gamma,freq,psi)) 
            self.printer("No Arduino Connection")
            #pass
        else:
            message = self.conn.tx_obj(data)
            self.conn.send(message)
            self.printer("Data Sent:  "+ "[Bx, By, Bz, alpha, gamma, freq, psi, acoust_freq, gradient, e_field] = "+str(data)) 







    def close(self) -> None:
        """
        Closes the current connection, if applicable

        Args:
        
        struct My_Sensor {
        
            None
        Returns:
            None
        """
        if self.conn is not None:
         
            self.send(0,0,0,0,0,0,0,0,0,0)
            self.conn.close()

            self.printer(f"Closing connection at port {self.port}")
   
            


if __name__ == "__main__":

    def tbprint(text):
        #print to textbox
        print(text)


    PORT = "COM3"
    arduino = ArduinoHandler(tbprint, PORT)
    arduino.connect()
    time.sleep(1)
    
    
    #send
    Bx = 0
    By = 0.0
    Bz = 0.0
    alpha = np.pi/2
    gamma = np.pi/2
    rolling_frequency = 1
    psi = 0
    gradient_status = 0.0
    equal_field_status = 0.0
    acoustic_frequency = 10000.0
    arduino.send(Bx, By, Bz, alpha, gamma, psi, rolling_frequency, gradient_status, equal_field_status, acoustic_frequency)
    print("sending")
    
    time.sleep(5)
    
    
    print("zeroing")
    arduino.close()
    
    

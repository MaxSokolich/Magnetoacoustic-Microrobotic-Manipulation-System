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

    def __init__(self, printer):
        self.conn = None
        self.port = None
        self.printer = printer

        

    def connect(self, port: str) -> None:
        """
        Initializes a connection to an arduino at a specified port. If successful,
        the conn and port attributes are updated
        """
        if self.conn is None:
            try:
                self.conn = txfer.SerialTransfer(port)
                self.port = port
                self.conn.open()
                time.sleep(1)
                self.printer(f"Arduino Connection initialized using port {port}")
            except InvalidSerialPort:
                self.printer("Could not connect to arduino, disabling")
                self.conn = None
                self.port = None
        else:
            self.printer(f"Connection already initialized at port {self.port}, new port {port} ignored")
   
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

        data = [float(Bx), float(By), float(Bz), float(alpha), float(gamma), float(freq),float(psi), float(acoustic_freq), float(gradient_status), float(equal_field_status), ]
        if self.conn is None:
            #self.printer("Connection not initialized..."+ str(data))  
            #self.printer("No Connection:  "+ "Bx: {},    By: {},    Bz: {},    alpha: {},    gamma: {},    freq: {},    psi: {}".format(Bx,By,Bz,alpha,gamma,freq,psi)) 
            self.printer("No Connection:  "+ "[Bx, By, Bz, alpha, gamma, freq, psi, acoust_freq, gradient, e_field] = "+str(data)) 
            #pass
        else:

            
            message = self.conn.tx_obj(data)
            self.conn.send(message)
            #self.printer("Data sent:"+ str(data))
            #self.printer("Data sent:  "+ "Bx: {},    By: {},    Bz: {},    alpha: {},    gamma: {},    freq: {},    psi: {}".format(Bx,By,Bz,alpha,gamma,freq,psi))
            self.printer("Data Sent:  "+ "[Bx, By, Bz, alpha, gamma, freq, psi, acoust_freq, gradient, e_field] = "+str(data)) 



    def receive(self):
        """
        receive information from arduino and store it in self.recieve class 
        return the values in the class self.receive
        """
        #recv
        if self.conn.available():
        
            recSize = 0
            Bx_sensor = self.conn.rx_obj(obj_type='f',start_pos=recSize)
            recSize += txfer.STRUCT_FORMAT_LENGTHS['f']

            By_sensor = self.conn.rx_obj(obj_type='f',start_pos=recSize)
            recSize += txfer.STRUCT_FORMAT_LENGTHS['f']

            Bz_sensor = self.conn.rx_obj(obj_type='f',start_pos=recSize)
            recSize += txfer.STRUCT_FORMAT_LENGTHS['f']
            

            
            


            return [Bx_sensor,By_sensor,Bz_sensor]


        else:
            return [0,0,0]


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


    PORT = "/dev/cu.usbmodem21101"
    arduino = ArduinoHandler(tbprint)
    arduino.connect(PORT)
    time.sleep(1)
    
    
    #send
    Bx = 0.0
    By = 0.0
    Bz = 0.0
    alpha = np.pi/2
    gamma = 0.0
    rolling_frequency = 0
    psi = 0
    gradient_status = 0.0
    equal_field_status = 0.0
    acoustic_frequency = 10000.0
    arduino.send(Bx, By, Bz, alpha, gamma, psi, rolling_frequency, gradient_status, equal_field_status, acoustic_frequency)
    print("sending")
    
    time.sleep(5)
    
    
    print("zeroing")
    arduino.close()
    
    

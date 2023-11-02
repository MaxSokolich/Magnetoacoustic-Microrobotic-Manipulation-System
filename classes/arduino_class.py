from pySerialTransfer import pySerialTransfer as txfer
from pySerialTransfer.pySerialTransfer import InvalidSerialPort
import time

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
                #time.sleep(5)
                self.printer(f"Arduino Connection initialized using port {port}")
            except InvalidSerialPort:
                self.printer("Could not connect to arduino, disabling")
                self.conn = None
                self.port = None
        else:
            self.printer(f"Connection already initialized at port {self.port}, new port {port} ignored")

    def send(self, Bx, By, Bz, alpha, gamma, freq, psi, acoustic_freq) -> None:
        """
        sends action commands to arduino

         Args:
            actions = [Bx, By, Bz, alpha, gamma, freq]
        """
        alpha = round(alpha,3)
        gamma = round(gamma,3)
        psi = round(psi,3)
        Bx = round(Bx,3)
        By = round(By,3)
        Bz = round(Bz,3)

        data = [float(Bx), float(By), float(Bz), float(alpha), float(gamma), float(freq),float(psi), float(acoustic_freq)]
        if self.conn is None:
            #self.printer("Connection not initialized..."+ str(data))  
            #self.printer("No Connection:  "+ "Bx: {},    By: {},    Bz: {},    alpha: {},    gamma: {},    freq: {},    psi: {}".format(Bx,By,Bz,alpha,gamma,freq,psi)) 
            self.printer("No Connection:  "+ "[Bx, By, Bz, alpha, gamma, freq, psi, acoustic_freq] = "+str(data)) 
            #pass
        else:
            #Bx = round(Bx,3)
            message = self.conn.tx_obj(data)
            self.conn.send(message)
            #self.printer("Data sent:"+ str(data))
            #self.printer("Data sent:  "+ "Bx: {},    By: {},    Bz: {},    alpha: {},    gamma: {},    freq: {},    psi: {}".format(Bx,By,Bz,alpha,gamma,freq,psi))
            self.printer("Data Sent:  "+ "[Bx, By, Bz, alpha, gamma, freq, psi, acoustic_freq] = "+str(data)) 
    
    
    def close(self) -> None:
        """
        Closes the current connection, if applicable

        Args:
            None
        Returns:
            None
        """
        if self.conn is not None:
         
            self.printer(f"Closing connection at port {self.port}")
            self.send(0,0,0,0,0,0,0,0)
            self.conn.close()
   
            


"""if __name__ == "__main__":
    PORT = "/dev/ttyACM0"
    arduino = MyArduino()
    arduino.connect(PORT)

    arduino.send(0,0,0,3.1,90,10)
    print("sending")
    time.sleep(5)
    arduino.send(0,0,0,0,0,0)
    print("zeroing")
    arduino.close()"""
    
    

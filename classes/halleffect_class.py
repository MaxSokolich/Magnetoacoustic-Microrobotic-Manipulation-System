"""
Note: Both HallEffect.py and AcousticHandler.py must be have the same pinmode 
configuration. i.e. GPIO.BOARD, BCM, TEGRA_SOC, or CVM. 

What I found is compatible with adafruit is BOARD. User must change line 8 
in python3.8/site-packages/adafruit_blinka/microcontroller/tegra/t194/pin.py
to:

Jetson.GPIO.setmode(GPIO.BOARD)
"""
from PyQt5.QtCore import pyqtSignal, pyqtSlot, Qt, QThread,QTimer
import time as time
try:
    import board
    import busio
    import numpy as np
    import matplotlib.pyplot as plt
    import adafruit_ads1x15.ads1115 as ADS
    from adafruit_ads1x15.analog_in import AnalogIn
    from scipy.interpolate import interp1d
   
    
    import time
    
    class HallEffect(QThread):
        """
        Class for managing the Hall Effect sensors via i2c
        Args:
            None
        """

        sensor_signal = pyqtSignal(list)

        def __init__(self, parent):
            super().__init__(parent=parent)
            self.parent = parent
            #set up sensor I2C
            #GPIO.setmode(GPIO.BOARD)
            self.i2c = busio.I2C(board.SCL, board.SDA)
            self.ads = ADS.ADS1115(self.i2c)
            
            self.senseBx = AnalogIn(self.ads, ADS.P0)
            self.senseBy = AnalogIn(self.ads, ADS.P1)  
            self.senseBz = AnalogIn(self.ads, ADS.P3)  #good


            self.run_flag = True

   

        def createBounds(self):
            """
            creates initial bounds to be updated when field is being read
            Args:
                none
            Returns:
                list of min max bounds
            """
            #call this in readField maybe
            neg_max = float(100000002)
            pos_max = -float(100000000)
            return [neg_max, pos_max]
        
        def readFIELD(self, channel,bound):
            """
            reads hall effect sensor field data given an analog obejct
            Args:
                channel:   AnalogIN channel object 1-4
                bound: class object that stores the min and max bounds generated from sensor
            Returns:
                mapped_field: scaled field value -100 to 100
            """             
            VAL = channel.value

            if VAL < bound[0]:
                bound[0] = VAL
            elif VAL > bound[1]:
                bound[1] = VAL
               
            m = interp1d([bound[0],bound[1]],[-80,80])
            mapped_field = int(m(VAL))
            return mapped_field
        
        def run(self):
              
            Bx_bounds = self.createBounds() #create bounds for positive Y EM sensor
            By_bounds = self.createBounds() #create bounds for positive X EM sensor
            Bz_bounds = self.createBounds() #create bounds for negative Y EM sensor
            
            while self.run_flag:
                
                bx = self.readFIELD(self.senseBx, Bx_bounds)/10
                by = self.readFIELD(self.senseBy, By_bounds)/10
                bz = self.readFIELD(self.senseBz, Bz_bounds)/10
                
               
                
               
             

                self.sensor_signal.emit([bx,by,bz])
                time.sleep(.1)
            time.sleep(.1)
            print(" -- Sensor Process Terminated -- ")


        def stop(self):
            self.run_flag = False
            self.wait()
            
except Exception:
    class HallEffect(QThread):
        sensor_signal = pyqtSignal(list)
        def __init__(self, parent):
            super().__init__(parent=parent)
            self.parent = parent
            self.run_flag = True
            
        def createBounds(self):
            pass
        def readFIELD(self, channel, bound):
            pass
        def showFIELD(self):
            pass
        def run(self):
            while self.run_flag:
                
                bx = 1
                by = 2
                bz = 3


                self.sensor_signal.emit([bx,by,bz])
                time.sleep(.1)
        def stop(self):
            self.run_flag = False
            self.wait()
        




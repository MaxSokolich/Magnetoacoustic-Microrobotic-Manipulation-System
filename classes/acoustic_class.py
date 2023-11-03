"""
Module containing the DigitalPot class

@authors: Max Sokolich
"""
import time 
try:
    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BOARD)
    
    class AcousticClass:
        def __init__(self):
            '''
            Acosutic Handler Class that enables use of the AD9850 0-40 MHz DDS signal generator module
            and  X9C104 100K Pot Module to vary the amplitude of the signal
    
            Args:
                None
            '''
            
            ##DEFINE DIGITAL POTENTIAMETER PINS
            self.dpCS = 11 #11
            self.dpUD = 13  #13
            self.dpINC = 15  #15
            GPIO.setup(self.dpCS, GPIO.OUT) # C/S
            GPIO.setup(self.dpUD, GPIO.OUT) # U/D
            GPIO.setup(self.dpINC, GPIO.OUT) # dpINC
            GPIO.output(self.dpCS, GPIO.HIGH)
            GPIO.output(self.dpINC, GPIO.HIGH)
            GPIO.output(self.dpUD, GPIO.LOW)
    
    
            ##DEFINE ACOUSTIC DDS PINS
            self.W_CLK = 21
            self.FQ_UD = 22
            self.DATA = 23
            self.RESET = 24
            GPIO.setup(self.W_CLK, GPIO.OUT)
            GPIO.setup(self.FQ_UD, GPIO.OUT)
            GPIO.setup(self.DATA, GPIO.OUT)
            GPIO.setup(self.RESET, GPIO.OUT)
            GPIO.output(self.W_CLK, GPIO.LOW)
            GPIO.output(self.FQ_UD, GPIO.LOW)
            GPIO.output(self.DATA, GPIO.LOW)
            GPIO.output(self.RESET, GPIO.HIGH)
    
            #keep track of total amplitude
            self.count = 0 
        
        # Function to activate digital pot
        def dp_activate(self):
            GPIO.output(self.dpCS, GPIO.LOW)
            time.sleep(0.000001)
    
        # Function to set incremiment increase or decrease
        def dp_set(self,flag):
            """
            sets the direction of increment: 
            1: inrease
            2: decrease
            """
            if flag == 1:
                GPIO.output(self.dpUD, GPIO.HIGH)
            elif flag == 0:
                GPIO.output(self.dpUD, GPIO.LOW)
            time.sleep(0.000005)
    
        # Function to incriment digital pot step amount
        def dp_move(self,flag,step):
            """
            outputs the correct signals to either increase or decrease the module
            """
            self.dp_set(flag)
            for i in range(step):
                GPIO.output(self.dpINC, GPIO.LOW)
                time.sleep(0.00001)
                GPIO.output(self.dpINC, GPIO.HIGH)
                time.sleep(0.000002)
    
        # Function to apply defined resistance value 
        def dp_apply(self, amplitude):
            """
            directly sets the resitance value 0-30
            """
            if amplitude >= self.count:
                actual_amp = amplitude - self.count
                self.dp_move(1,actual_amp)
                self.count = amplitude
            elif amplitude < self.count:
                actual_amp = self.count - amplitude
                self.dp_move(0, actual_amp)
                self.count = amplitude
        
        
        # Function to send a pulse to GPIO pin
        def am_pulseHigh(self,pin):
            GPIO.output(pin, True)
            GPIO.output(pin, True)
            GPIO.output(pin, False)
            
    
        # Function to send a byte to AD9850 module
        def am_tfr_byte(self,data):
            for i in range (0,8):
                GPIO.output(self.DATA, data & 0x01)
                self.am_pulseHigh(self.W_CLK)
                data=data>>1
            
        # Function to send frequency (assumes 125MHz xtal) to AD9850 module
        def am_sendFrequency(self,frequency):
            freq=int(frequency*4294967296/125000000)
            for b in range (0,4):
                self.am_tfr_byte(freq & 0xFF)
                freq=freq>>8
            self.am_tfr_byte(0x00)
            self.am_pulseHigh(self.FQ_UD)
            
    
        # start the DDS module
        def start(self,frequency, amplitude):
            self.am_pulseHigh(self.RESET)
            self.am_pulseHigh(self.W_CLK)
            self.am_pulseHigh(self.FQ_UD)
            self.am_sendFrequency(frequency)
            
            #set amplitude by adjusting digital pot
            self.dp_apply(amplitude) #0-30
                                                
        # resets acoustic module to 0
        def stop(self):
            self.am_pulseHigh(self.RESET)
        
        # resets digital pot to 0
        def close(self):
            self.dp_move(1, 99)
            self.dp_move(0,99)
            print("-- closed acoutic module --")
    
        # exits and cleansup GPIO pins
        def exit(self):
            GPIO.output(self.dpINC, GPIO.HIGH)
            GPIO.output(self.dpCS, GPIO.HIGH)
            GPIO.cleanup()

            
except Exception:
    class AcousticClass:
        def __init__(self):
            pass
        def dp_activate(self):
            pass
        def dp_set(self,flag):
            pass
        def dp_move(self,flag,step):
            pass
        def dp_apply(self, amplitude):
            pass
        def am_pulseHigh(self,pin):
            pass
        def am_tfr_byte(self,data):
            pass
        def am_sendFrequency(self,frequency):
            pass
        def start(self,frequency, amplitude):
            print(frequency,amplitude)
        def stop(self):
            print("stop")
        def close(self):
            pass
        def exit(self):
            pass
        
        

"""
max step = 30
we want a reading from 0 V to voltage maximum
~0 Volts is max on resistance

map(low_resistance, high resistance, high voltage, low voltage)
"""
"""
if __name__ == "__main__":

    AcousticMod = AcousticClass()
    AcousticMod.dp_activate()
    print("starting waveform...")
    freqinput = 1000000
    amplitude =0
    AcousticMod.start(freqinput,amplitude)
    time.sleep(4)
    AcousticMod.stop()
    print("stopped waveform")
    AcousticMod.close()
    AcousticMod.exit()"""
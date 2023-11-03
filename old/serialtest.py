from time import sleep
from pySerialTransfer import pySerialTransfer as txfer
import time as time
import numpy as np
import math
PORT = "/dev/cu.usbmodem11401"
link = txfer.SerialTransfer(PORT)
link.open()
sleep(1)

data = [float(0), float(0), float(0), float(0), float(0), float(0),float(0), float(10000)]

sendSize = link.tx_obj(data)
link.send(sendSize)
print("sent", data)
sleep(3)


data = [float(.5), float(.1), float(0), float(0), float(0), float(0),float(0), float(11000)]

sendSize = link.tx_obj(data)
link.send(sendSize)
print("sent", data)
sleep(2)

data = [float(0), float(0), float(0), float(0), float(0), float(0),float(0), float(0)]

sendSize = link.tx_obj(data)
link.send(sendSize)
print("sent", data)
sleep(2)
    
    
link.close()
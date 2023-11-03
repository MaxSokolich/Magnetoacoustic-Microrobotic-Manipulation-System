
from PyQt5.QtCore import pyqtSignal, pyqtSlot, Qt, QThread,QTimer
import numpy as np
import cv2
import matplotlib.pyplot as plt


class MaskThread(QThread):
    mask_signal = pyqtSignal(np.ndarray)
   



    def __init__(self, parent):
        super().__init__(parent=parent)
        self.parent = parent
         
        self.run_flag = True

    def run(self):
        while self.run_flag == True:
            mask = np.array([1,2])
            self.mask_signal.emit(mask)
    
    def stop(self):
        self.run_flag = False

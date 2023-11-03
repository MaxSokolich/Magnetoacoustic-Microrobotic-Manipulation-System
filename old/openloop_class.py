

from PyQt5.QtCore import pyqtSignal, pyqtSlot, Qt, QThread,QTimer
import time


class OpenLoop(QThread):
    openloopactions = pyqtSignal(list)


    def __init__(self, parent):
        super().__init__(parent=parent)
        self.runflag = True

    def run(self):
        while self.runflag == True:
            self.openloopactions.emit([2,3])
            time.sleep(.1)
    
    def stop(self):
        self.runflag = False
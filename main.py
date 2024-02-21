
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QApplication
import sys
from classes.gui_functions import MainWindow


# Convert blur into z position
# put RRT trajectory in seperate thead
# add joystick when camera is off. see above
# calibrate x and y z coils by adding a calbration value
# add a track all feature

 


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec_())

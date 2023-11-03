
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QApplication
import sys
from gui_functionstest import MainWindow

#test mask speed increase and Linux
#Remake ui for Linux machine ( make it so it can resize)
#Convert blur into z position
#add slots for field values
# add exposure spinbox
# add fps spin box? 
# add objective spin box
# still try and activily adjust window size
# maybe add robot list

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec_())

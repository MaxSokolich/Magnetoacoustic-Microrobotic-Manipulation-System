from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QApplication
import sys
from PyQt5.QtGui import QWheelEvent
from PyQt5 import QtGui
from PyQt5.QtWidgets import QWidget
from PyQt5.QtGui import QPixmap,QIcon
from PyQt5.QtCore import Qt, QTimer, PYQT_VERSION_STR
from PyQt5 import QtWidgets, QtGui, QtCore
import cv2
import os
from os.path import expanduser

import pandas as pd
from datetime import datetime
import sys
from PyQt5.QtWidgets import QApplication
import numpy as np
import cv2
import matplotlib.pyplot as plt 
import time
import platform

from tracker_class import VideoThread


from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QApplication
import sys

#import EasyPySpin



from gui_widgets_linux import Ui_MainWindow



class MainWindow(QtWidgets.QMainWindow):
    positionChanged = QtCore.pyqtSignal(QtCore.QPoint)

    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent=parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        screen  = QtWidgets.QDesktopWidget().screenGeometry(-1)
        self.window_width = screen.width()
        self.window_height = screen.height()
        self.resize(self.window_width, self.window_height)

        #self.display_width = #self.ui.VideoFeedLabel.frameGeometry().width()# self.ui.frameGeometry().width()
        #self.display_height = # self.ui.VideoFeedLabel.frameGeometry().height() #keep this fixed, changed the width dpending on the aspect ratio
        
     

        
        #self.ui.VideoFeedLabel.setGeometry(QtCore.QRect(10,  5,                       self.display_width,     self.display_height))
        #self.ui.frameslider.setGeometry(QtCore.QRect(10,    self.display_height+12,   self.display_width,     20))
        #self.ui.plainTextEdit.setGeometry(QtCore.QRect(10,  self.display_height+40,   self.display_width-400,     91))
        
        self.newx,self.newy =0,0
        self.result = None
        self.rec_start_time = time.time()
        self.videopath = 0
        self.cap = None
        self.drawing = False
        self.acoustic_frequency = 0
        self.magnetic_field_list = []
        self.actions = [0,0,0,0,0,0,0,0,0]

        self.ui.trackbutton.clicked.connect(self.track)
        self.ui.VideoFeedLabel.installEventFilter(self)
        
        self.aspect = 4/3
        self.windowaspect = 16/9


        self.display_width = self.ui.VideoFeedLabel.frameGeometry().width()
        self.display_height = int(self.display_width * self.aspect)
        
        self.video_width  = self.display_width
        self.video_height = self.display_height

        self.tar = [500, 500]
    
    
    

    def resizeEvent(self, event):
 
        if self.videopath is not None:

            
            displaysize = self.ui.VideoFeedLabel.size()
            self.aspect = self.video_width/self.video_height
            #displaysize.setWidth(int(displaysize.height()*self.aspect))
            #self.ui.VideoFeedLabel.resize(displaysize)W
            self.display_width = int(displaysize.height()*self.aspect)


            windowsize = event.size()
            #self.aspect = self.v_width/self.video_height
            windowsize.setWidth(int(windowsize.height()*self.windowaspect))
            self.resize(windowsize)
            #self.display_height = windowsize.height()-500

            #self.ui.VideoFeedLabel.setFixedHeight(windowsize.height()-500)
           
        
        return super().resizeEvent(event)
    

    def track(self):
        if self.videopath is not None:
            if self.ui.trackbutton.isChecked():
                self.cap  = cv2.VideoCapture(0)
                
                        
                #self.video_width  = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                #self.video_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                self.videofps = int(self.cap.get(cv2.CAP_PROP_FPS))
                

                #print(self.video_width,self.video_height,self.videofps)
                #self.display_width = self.ui.VideoFeedLabel.frameGeometry().width()# self.ui.frameGeometry().width()
                #self.display_height = self.ui.VideoFeedLabel.frameGeometry().height()
                


                self.video_width  = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                self.video_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                self.aspect = self.video_width/self.video_height

                self.display_height = self.video_height
                self.display_width = int(self.display_height * self.aspect)
                
                
                


                windowsize = self.size()
                windowsize.setWidth(int(windowsize.height()*self.windowaspect))
                self.resize(windowsize)

                self.ui.VideoFeedLabel.setMinimumWidth(self.video_width//5)
                self.ui.VideoFeedLabel.setMinimumHeight(self.video_height//5)
                #self.ui.VideoFeedLabel.setMaximumHeight(self.display_height//5)



           
             
                #self.ui.VideoFeedLabel.setFixedWidth = self.video_width
                #self.ui.VideoFeedLabel.setFixedHeight = self.video_height
                
                
                
                #self.display_width = int(self.display_height * (self.video_width / self.video_height))
                #self.ui.VideoFeedLabel.setGeometry(QtCore.QRect(10, 5, self.display_width,self.display_height))
                #self.ui.VideoFeedLabel.setStyleSheet("border:2px solid rgb(0, 255, 0); ")
                #self.ui.CroppedVideoFeedLabel.setStyleSheet("border:2px solid rgb(0, 255, 0); ")
                
                
                #self.window_width = self.display_width+265
                #self.resize(self.window_width, self.window_height)
                
                self.tracker = VideoThread(self)
                self.tracker.change_pixmap_signal.connect(self.update_image)
            
                self.tracker.start()
            else:
                self.tracker.stop()
        
    def update_image(self, frame):
        """Updates the image_label with a new opencv image"""
        print(self.ui.VideoFeedLabel.size(), self.size(), self.video_width, self.video_height)
        cv2.circle(frame,(int(self.tar[0]), int(self.tar[1])),6,(0,255,0), -1,)
        rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        
        bytes_per_line = ch * w
        convert_to_Qt_format = QtGui.QImage(rgb_image.data, w, h, bytes_per_line, QtGui.QImage.Format_RGB888)
        p = convert_to_Qt_format.scaled(self.display_width, self.display_height, Qt.KeepAspectRatio)
        qt_img = QPixmap.fromImage(p)

        self.ui.VideoFeedLabel.setPixmap(qt_img)

             
    def eventFilter(self, object, event):
    
        if object is self.ui.VideoFeedLabel: 
            
            if self.cap is not None:
                
                if event.type() == QtCore.QEvent.MouseButtonPress:   
                    if event.buttons() == QtCore.Qt.LeftButton:
                        newx, newy = self.convert_coords(event.pos())
                        self.tar = [newx, newy]
                        #generate original bounding box

                    
                        
                    
        return super().eventFilter(object, event)
    
    def convert_coords(self,pos):
        #need a way to convert the video position of mouse to the actually coordinate in the window
        newx = int(pos.x() * (self.video_width / self.display_width)) 
        newy = int(pos.y() * (self.video_height / self.display_height))
        return newx, newy

    
    


    def closeEvent(self, event):
        """
        called when x button is pressed
        """
        if self.cap is not None:
            self.tracker.stop()



if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec_())
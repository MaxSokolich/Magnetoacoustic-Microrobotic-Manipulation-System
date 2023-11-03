from PyQt5 import QtGui
from PyQt5.QtWidgets import QWidget
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import Qt
from PyQt5 import QtWidgets, QtGui, QtCore
import cv2
import os
from os.path import expanduser
import pandas as pd
from datetime import datetime
from typing import List, Tuple, Union
import sys
from PyQt5.QtWidgets import QApplication


from PyQt5.QtCore import pyqtSignal, pyqtSlot, Qt, QThread,QTimer
import numpy as np
import cv2
import matplotlib.pyplot as plt

from scipy import ndimage 
import time
    
#blank videl label on stop button
#add velocity and area
#add invert checkbox

class App(QWidget):
    positionChanged = QtCore.pyqtSignal(QtCore.QPoint)

    def __init__(self):
        super().__init__()
        
        
        self.setWindowTitle("Tracker")
    

        self.display_width = 1330
        self.display_height = 700 #keep this fixed, changed the width dpending on the aspect ratio

        self.window_width = 1600
        self.window_height = 780
        self.resize(self.window_width, self.window_height)
        
       
        self.videopath = 0
        self.cap = None
        self.qt_img = None


        self.tabWidget = QtWidgets.QTabWidget(self)
        
        self.tabWidget.setGeometry(QtCore.QRect(5, 0, 210, self.window_height))
        self.tabWidget.setObjectName("tabWidget")
    


        self.tracker_tab = QtWidgets.QWidget()
        self.tracker_tab.setObjectName("tracker_tab")
        self.tracker_tab.setStyleSheet("QTabBar {font: bold 9pt;}")
        self.tabWidget.setStyleSheet('font-size: 12pt; font-family: Courier;')
                                              
                                            
                                
        self.tabWidget.addTab(self.tracker_tab, "Tracking")



        self.control_tab = QtWidgets.QWidget()
        self.control_tab.setObjectName("control_tab")
        self.tabWidget.addTab(self.control_tab, "Control")

 
 
        




        self.choosevideobutton = QtWidgets.QPushButton(self.tracker_tab)
        self.choosevideobutton.setGeometry(QtCore.QRect(20, 0, 160, 25))
        self.choosevideobutton.setObjectName("choosevideobutton")
        self.choosevideobutton.setText("Choose Video")
        self.choosevideobutton.clicked.connect(self.selectFile)
        self.choosevideobutton.setStyleSheet('''
            QPushButton {
                color: rgb(255, 255, 255);
                background-color: rgb(0, 0, 0);
                border-style: outset;
                border-width: 2px;
                border-radius: 10px;
                border-color: rgb(0, 0, 0);
                font: bold 12px;
                min-width: 1em;
                padding: 1px;
            }
            QPushButton:hover {
                background-color: rgb(100, 100, 100);
                color: rgb(255, 255, 255);
                border-color: rgb(100, 100, 100);
            
            
            }
          
        ''')



        self.trackbutton = QtWidgets.QPushButton("Toggle",self.tracker_tab)
        self.trackbutton.setGeometry(QtCore.QRect(20, 30, 160, 35))
        self.trackbutton.setObjectName("trackbutton")
        self.trackbutton.setText("Track")
        self.trackbutton.setCheckable(True)
        self.trackbutton.clicked.connect(self.track)
        self.trackbutton.setStyleSheet('''
            QPushButton {
                color: rgb(255, 255, 255);
                background-color: rgb(0, 0, 255);
                border-style: outset;
                border-width: 3px;
                border-radius: 10px;
                border-color: rgb(0, 0, 255);
                font: bold 16px;
                min-width: 1em;
                padding: 6px;
            }
      
            QPushButton:checked {
                color: rgb(255, 255, 255);
                background-color: rgb(255, 0, 0);
                border-style: inset;
                border-width: 3px;
                border-radius: 10px;
                border-color: rgb(255, 0, 0);
                font: bold 16px;
                min-width: 1em;
               
            }
            QPushButton:hover {
                background-color: rgb(100, 100, 100);
                color: rgb(255, 255, 255);
                border-color: rgb(100, 100, 100);
                padding-left: 5px;
                padding-top: 5px;
            }
         
            
            
        ''')


        self.pausebutton = QtWidgets.QPushButton("Toggle", self.tracker_tab)
        self.pausebutton.setGeometry(QtCore.QRect(20, 70, 160, 20))
        self.pausebutton.setObjectName("pausebutton")
        self.pausebutton.setText("Pause")
        self.pausebutton.setCheckable(True)
        self.pausebutton.clicked.connect(self.pause)
        self.pausebutton.hide()
        self.pausebutton.setStyleSheet('''
            QPushButton {
                color: rgb(255, 255, 255);
                background-color: rgb(150, 0, 0);
                border-style: outset;
                border-width: 3px;
                border-radius: 10px;
                border-color: rgb(150, 0, 0);
                font: bold 12px;
                min-width: 1em;
                padding: 1px;
            }
            QPushButton:checked {
                color: rgb(255, 255, 255);
                background-color: rgb(0, 0, 150);
                border-style: inset;
                border-width: 3px;
                border-radius: 10px;
                border-color: rgb(0, 0, 150);
                font: bold 12px;
                min-width: 1em;
                padding: 1px;
            }
            QPushButton:hover {
                background-color: rgb(100, 100, 100);
                color: rgb(255, 255, 255);
                border-color: rgb(100, 100, 100);
                padding-left: 2px;
                padding-top: 2px;
            }
            
            
        ''')


        self.rightbutton = QtWidgets.QToolButton(self.tracker_tab)
        self.rightbutton.setGeometry(QtCore.QRect(105, 100, 60, 20))
        self.rightbutton.setArrowType(Qt.RightArrow)
        self.rightbutton.setAutoRepeat(True)
        self.rightbutton.setAutoRepeatDelay(200)
        self.rightbutton.setAutoRepeatInterval(50)
        self.rightbutton.clicked.connect(self.frameright)
        self.rightbutton.hide()
        self.rightbutton.setStyleSheet('''
            QToolButton {
                color: rgb(255, 255, 255);
                background-color: rgb(0, 0, 0);
                border-style: outset;
                border-width: px;
                border-radius: 2px;
                border-color: rgb(0, 0, 0);
                font: bold 16px;
                min-width: 1em;
                padding: 1px;
            }
            QToolButton:hover {
                background-color: rgb(100, 100, 100);
                color: rgb(255, 255, 255);
                border-color: rgb(100, 100, 100);
            }
            QToolButton:pressed {
                background-color: rgb(100, 100, 100);
                border: 2px solid rgb(100, 100, 100);
                border-style: inset;
                padding-left: 5px;
                padding-top: 5px;
            }
          
            
        ''')


        self.leftbutton = QtWidgets.QToolButton(self.tracker_tab)
        self.leftbutton.setGeometry(QtCore.QRect(35, 100, 60, 20))
        self.leftbutton.setArrowType(Qt.LeftArrow)
        self.leftbutton.setAutoRepeat(True)
        self.leftbutton.setAutoRepeatDelay(200)
        self.leftbutton.setAutoRepeatInterval(50)
        self.leftbutton.clicked.connect(self.frameleft)
        self.leftbutton.hide()
        self.leftbutton.setStyleSheet('''
            QToolButton {
                color: rgb(255, 255, 255);
                background-color: rgb(0, 0, 0);
                border-style: outset;
                border-width: px;
                border-radius: 2px;
                border-color: rgb(0, 0, 0);
                font: bold 16px;
                min-width: 1em;
                padding: 1px;
            }
            QToolButton:hover {
                background-color: rgb(100, 100, 100);
                color: rgb(255, 255, 255);
                border-color: rgb(100, 100, 100);
            }
            QToolButton:pressed {
                background-color: rgb(100, 100, 100);
                border: 2px solid rgb(100, 100, 100);
                border-style: inset;
                padding-left: 5px;
                padding-top: 5px;
            }
          
            
        ''')
        

     

        self.maskbutton = QtWidgets.QPushButton("Toggle",self.tracker_tab)
        self.maskbutton.setGeometry(QtCore.QRect(20, 130, 160, 20))
        self.maskbutton.setObjectName("maskbutton")
        self.maskbutton.setText("Mask")
        self.maskbutton.setCheckable(True)
        self.maskbutton.clicked.connect(self.showmask)
        self.maskbutton.hide()
        self.maskbutton.setStyleSheet('''
            QPushButton {
                color: rgb(255, 255, 255);
                background-color: rgb(0, 0, 0);
                border-style: outset;
                border-width: 2px;
                border-radius: 10px;
                border-color: rgb(0, 0, 0);
                font: bold 10px;
                min-width: 1em;
                padding: 1px;
            }
            QPushButton:hover {
                background-color: rgb(100, 100, 100);
                color: rgb(0, 0, 0);
                border-style: inset;
            }
   
       
          
        ''')
        


        self.maskinvert_checkBox = QtWidgets.QCheckBox(self.tracker_tab)
        self.maskinvert_checkBox.setGeometry(QtCore.QRect(20, 160, 25, 25))
        self.maskinvert_checkBox.setObjectName("maskinvert_checkBox")
        self.maskinvert_checkBox.setChecked(True)
        self.maskinvert_checkBox.toggled.connect(self.invertmaskcommand)
      
    
        self.maskinvert_label = QtWidgets.QLabel(self.tracker_tab)
        self.maskinvert_label.setGeometry(QtCore.QRect(50, 160, 160, 20))
        self.maskinvert_label.setObjectName("maskinvert_label")
        self.maskinvert_label.setText("Invert Mask: False")
        self.maskinvert_label.setStyleSheet("color: rgb(0, 0, 0);"
                                        #"background-color: rgb(0, 255, 0);"
                                        #"border-style: outset;"
                                        #"border-width: 2px;"
                                        #"border-radius: 6px;"
                                        #"border-color: rgb(0, 255, 100);"
                                        "font: bold 12px;"
                                        "min-width: 1em;"
                                        "padding: 1px;")





        self.masksigmalabel = QtWidgets.QLabel(self.tracker_tab)
        self.masksigmalabel.setGeometry(QtCore.QRect(20, 185, 160, 20))
        self.masksigmalabel.setObjectName("masksigmalabel")
        self.masksigmalabel.setText("Mask Sigma: 7")
        self.masksigmalabel.setStyleSheet("color: rgb(0, 0, 0);"
                                        #"background-color: rgb(0, 255, 0);"
                                        #"border-style: outset;"
                                        #"border-width: 2px;"
                                        #"border-radius: 6px;"
                                        #"border-color: rgb(0, 255, 100);"
                                        "font: bold 12px;"
                                        "min-width: 1em;"
                                        "padding: 1px;")

        self.masksigmaslider = QtWidgets.QSlider(self.tracker_tab)
        self.masksigmaslider.setGeometry(QtCore.QRect(20, 205, 160, 30))
        self.masksigmaslider.setMinimum(0)
        self.masksigmaslider.setMaximum(30)
        self.masksigmaslider.setSliderPosition(7)
        self.masksigmaslider.setOrientation(QtCore.Qt.Horizontal)
        self.masksigmaslider.setObjectName("masksigmaslider")
        self.masksigmaslider.valueChanged.connect(self.get_masksigma)
        

        self.croplengthlabel = QtWidgets.QLabel(self.tracker_tab)
        self.croplengthlabel.setGeometry(QtCore.QRect(20, 235, 160, 20))
        self.croplengthlabel.setObjectName("croplengthlabel")
        self.croplengthlabel.setText("Crop Length: 40")
        self.croplengthlabel.setStyleSheet("color: rgb(0, 0, 0);"
                                        #"background-color: rgb(0, 255, 0);"
                                        #"border-style: outset;"
                                        #"border-width: 2px;"
                                        #"border-radius: 6px;"
                                        #"border-color: rgb(0, 255, 100);"
                                        "font: bold 12px;"
                                        "min-width: 1em;"
                                        "padding: 1px;")

        self.croplengthslider = QtWidgets.QSlider(self.tracker_tab)
        self.croplengthslider.setGeometry(QtCore.QRect(20, 255, 160, 30))
        self.croplengthslider.setMinimum(5)
        self.croplengthslider.setMaximum(200)
        self.croplengthslider.setSliderPosition(40)
        self.croplengthslider.setOrientation(QtCore.Qt.Horizontal)
        self.croplengthslider.setObjectName("croplengthslider")
        self.croplengthslider.valueChanged.connect(self.get_croplength)

        self.savedatabutton = QtWidgets.QPushButton("Toggle",self.tracker_tab)
        self.savedatabutton.setGeometry(QtCore.QRect(20, 290, 160, 30))
        self.savedatabutton.setObjectName("savedatabutton")
        self.savedatabutton.setText("Save Data")
        self.savedatabutton.setCheckable(True)
        self.savedatabutton.clicked.connect(self.savedata)
        self.savedatabutton.setStyleSheet('''
            QPushButton {
                color: rgb(0, 0, 0);
                background-color: rgb(255, 255, 0);
                border-style: outset;
                border-width: 3px;
                border-radius: 6px;
                border-color: rgb(255, 255, 100);
                font: bold 12px;
                min-width: 1em;
                padding: 1px;
            }
            QPushButton:hover {
                background-color: rgb(255, 255, 200);
                color: rgb(0, 0, 0);
            }
            QPushButton:pressed {
                background-color: red;
                border: 2px solid red;
                padding-left: 5px;
                padding-top: 5px;
                border-style: inset;
                }
                        
        ''')


        self.robotarealabel = QtWidgets.QLabel(self.tracker_tab)
        self.robotarealabel.setGeometry(QtCore.QRect(20, 320, 160, 20))
        self.robotarealabel.setObjectName("robotarealabel")
        self.robotarealabel.setText("Area:")
        self.robotarealabel.setStyleSheet("color: rgb(0, 0, 0);"
                                        #"background-color: rgb(0, 255, 0);"
                                        #"border-style: outset;"
                                        #"border-width: 2px;"
                                        #"border-radius: 6px;"
                                        #"border-color: rgb(0, 255, 100);"
                                        "font: bold 12px;"
                                        "min-width: 1em;"
                                        "padding: 1px;")

        self.robotvelocitylabel = QtWidgets.QLabel(self.tracker_tab)
        self.robotvelocitylabel.setGeometry(QtCore.QRect(20, 340, 160, 20))
        self.robotvelocitylabel.setObjectName("robotvelocitylabel")
        self.robotvelocitylabel.setText("Velocity:")
        self.robotvelocitylabel.setStyleSheet("color: rgb(0, 0, 0);"
                                        #"background-color: rgb(0, 255, 0);"
                                        #"border-style: outset;"
                                        #"border-width: 2px;"
                                        #"border-radius: 6px;"
                                        #"border-color: rgb(0, 255, 100);"
                                        "font: bold 12px;"
                                        "min-width: 1em;"
                                        "padding: 1px;")

        self.robotblurlabel = QtWidgets.QLabel(self.tracker_tab)
        self.robotblurlabel.setGeometry(QtCore.QRect(20, 360, 160, 20))
        self.robotblurlabel.setObjectName("robotblurlabel")
        self.robotblurlabel.setText("Blur:")
        self.robotblurlabel.setStyleSheet("color: rgb(0, 0, 0);"
                                        #"background-color: rgb(0, 255, 0);"
                                        #"border-style: outset;"
                                        #"border-width: 2px;"
                                        #"border-radius: 6px;"
                                        #"border-color: rgb(0, 255, 100);"
                                        "font: bold 12px;"
                                        "min-width: 1em;"
                                        "padding: 1px;")


        self.CroppedVideoFeedLabel = QtWidgets.QLabel(self.tracker_tab)
        self.CroppedVideoFeedLabel.setGeometry(QtCore.QRect(0, 380, 200, 200))
        self.CroppedVideoFeedLabel.setAutoFillBackground(True)
        self.CroppedVideoFeedLabel.setFrameShadow(QtWidgets.QFrame.Raised)
        self.CroppedVideoFeedLabel.setText("")
        self.CroppedVideoFeedLabel.setObjectName("CroppedVideoFeedLabel")
        self.CroppedVideoFeedLabel.setStyleSheet("background-color: rgb(0,0,0); border:2px solid rgb(255, 0, 0); ")



        self.frameslider = QtWidgets.QSlider(self)
        self.frameslider.setGeometry(QtCore.QRect(220, 20, self.display_width, 30))
        self.frameslider.setMinimum(0)
        self.frameslider.setMaximum(100)
        self.frameslider.setSliderPosition(0)
        self.frameslider.setOrientation(QtCore.Qt.Horizontal)
        self.frameslider.setObjectName("frameslider")
        self.frameslider.tracking = True
        self.frameslider.valueChanged.connect(self.adjustframe)
       

        self.framelabel = QtWidgets.QLabel(self)
        self.framelabel.setGeometry(QtCore.QRect(220, 5, self.display_width, 20))
        self.framelabel.setObjectName("framelabel")
        self.framelabel.setText("Frame:")
        self.framelabel.setStyleSheet("color: rgb(0, 0, 0);"
                                        "font-family: Courier;"
                                        #"background-color: rgb(0, 255, 0);"
                                        #"border-style: outset;"
                                        #"border-width: 2px;"
                                        #"border-radius: 6px;"
                                        #"border-color: rgb(0, 255, 100);"
                                        "font: bold 12px;"
                                        "min-width: 1em;"
                                        "padding: 1px;")
   


        self.VideoFeedLabel = QtWidgets.QLabel(self)
        self.VideoFeedLabel.setGeometry(QtCore.QRect(220, 60, self.display_width, self.display_height))
        self.VideoFeedLabel.setAutoFillBackground(True)
        self.VideoFeedLabel.setFrameShadow(QtWidgets.QFrame.Raised)
        self.VideoFeedLabel.setText("")
        self.VideoFeedLabel.setObjectName("VideoFeedLabel")
        self.VideoFeedLabel.setStyleSheet("background-color: rgb(0,0,0); border:2px solid rgb(255, 0, 0); ")
        self.VideoFeedLabel.setMouseTracking(True)
        self.VideoFeedLabel.installEventFilter(self)






    def eventFilter(self, object, event):
        if object is self.VideoFeedLabel: 
            #if event.type() == QtCore.QEvent.MouseMove:
            #    self.positionChanged.emit(event.pos())
            #    pos = event.pos()
            #    print(pos)
                #Robot["pos"] = [pos.x(),pos.y()]
            if self.cap is not None:
                if event.type() == QtCore.QEvent.MouseButtonPress:
                    if event.buttons() == QtCore.Qt.LeftButton:
                    
                        self.positionChanged.emit(event.pos())
                        pos = event.pos()

                        #need a way to convert the video position of mouse to the actually coordinate in the window
                        newx = int(pos.x() * (self.video_width / self.display_width)) 
                        newy = int(pos.y() * (self.video_height / self.display_height))

                        #generate original bounding box
                        x_1 = int(newx - self.tracker.crop_length  / 2)
                        y_1 = int(newy - self.tracker.crop_length  / 2)
                        w = self.tracker.crop_length
                        h = self.tracker.crop_length

                        robot = Robot()  # create robot instance
                        
                        robot.add_frame(self.tracker.framenum)
                        robot.add_time(0)
                        robot.add_position([newx,newy])
                        robot.add_velocity([0,0])
                        robot.add_crop([x_1, y_1, w, h])
                        robot.add_area(0)
                        robot.add_blur(0)
                        
                        
                        self.tracker.robot_list.append(robot)
               
                    if event.buttons() == QtCore.Qt.RightButton:
                    
                        #self.positionChanged.emit(event.pos())
                        #pos = event.pos()
                        del self.tracker.robot_list[:]
                        
            
                
        return super().eventFilter(object, event)

    def selectFile(self):
        options = QtWidgets.QFileDialog.Options()
        options |= QtWidgets.QFileDialog.ReadOnly
        file_path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Open File", "", "All Files (*);;Text Files (*.txt);;Python Files (*.py)", options=options)

        if file_path:
            self.videopath = file_path
            file_info = QtCore.QFileInfo(file_path)
            file_name = file_info.fileName()
            print(file_path, file_name)
            self.choosevideobutton.setText(file_name)
        else:
            self.videopath = 0
            self.choosevideobutton.setText(str(0))
        
        


        
    def update_image(self, frame):
        """Updates the image_label with a new opencv image"""
        
        rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        
        bytes_per_line = ch * w
        convert_to_Qt_format = QtGui.QImage(rgb_image.data, w, h, bytes_per_line, QtGui.QImage.Format_RGB888)
        p = convert_to_Qt_format.scaled(self.display_width, self.display_height, Qt.KeepAspectRatio)
        self.qt_img = QPixmap.fromImage(p)
        
        
        
        #update frame slider too
        self.framelabel.setText("Frame:"+str(self.tracker.framenum))
        self.frameslider.setSliderPosition(self.tracker.framenum)

        #also update robot info
        if len(self.tracker.robot_list) > 0:
            self.robotarealabel.setText("Area: {0:.2f} px^2".format(self.tracker.robot_list[-1].avg_area))
            self.robotvelocitylabel.setText("Vx: {0:.2f}, Vy: {0:.2f} px/f".format(self.tracker.robot_list[-1].velocity_list[-1][0], self.tracker.robot_list[-1].velocity_list[-1][1]  )  )
            self.robotblurlabel.setText("Blur: {0:.2f} units".format(self.tracker.robot_list[-1].blur_list[-1]))
        
  
        self.VideoFeedLabel.setPixmap(self.qt_img)
        

    
    def update_croppedimage(self, frame):
        """Updates the cropped image_label with a new cropped opencv image"""
        
        rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        
        bytes_per_line = ch * w
        convert_to_Qt_format = QtGui.QImage(rgb_image.data, w, h, bytes_per_line, QtGui.QImage.Format_RGB888)
        p = convert_to_Qt_format.scaled(200, 200, Qt.KeepAspectRatio)
        qt_cimg = QPixmap.fromImage(p)
        self.CroppedVideoFeedLabel.setPixmap(qt_cimg)
        


    
    def track(self):
        if self.videopath is not None:
            if self.trackbutton.isChecked():
                

                #start video thread
                self.pausebutton.show()
                self.leftbutton.show()
                self.rightbutton.show()
                self.maskbutton.show()

                #need to resize the window in order to maintain proper aspect ratios
                self.cap = cv2.VideoCapture(self.videopath)
                self.video_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                self.video_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                self.totalnumframes = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))

                self.display_width = int(self.display_height * (self.video_width / self.video_height))
                self.VideoFeedLabel.setGeometry(QtCore.QRect(220, 60, self.display_width, self.display_height))
                self.VideoFeedLabel.setStyleSheet("border:2px solid rgb(0, 255, 0); ")
                self.CroppedVideoFeedLabel.setStyleSheet("border:2px solid rgb(0, 255, 0); ")
                


                self.frameslider.setGeometry(QtCore.QRect(220, 20, self.display_width, 30))
                self.frameslider.setMaximum(self.totalnumframes)
            

                self.tracker = VideoThread(self.cap)
                self.tracker.change_pixmap_signal.connect(self.update_image)
                self.tracker.cropped_frame_signal.connect(self.update_croppedimage)
                self.tracker.start()
                
                self.trackbutton.setText("Stop")
                
                if self.videopath == 0:
                    self.pausebutton.hide()
                    self.leftbutton.hide()
                    self.rightbutton.hide()
         
                
        
            else:
                self.VideoFeedLabel.setStyleSheet("background-color: rgb(0,0,0); border:2px solid rgb(255, 0, 0); ")
                self.CroppedVideoFeedLabel.setStyleSheet("background-color: rgb(0,0,0); border:2px solid rgb(255, 0, 0); ")
         
                
                if self.cap is not None:
                    self.trackbutton.setText("Track")
                    self.tracker.stop()
                    
                   
                    #also reset pause button
                    self.pausebutton.setChecked(False)
                    self.pausebutton.setText("Pause")

                    self.pausebutton.hide()
                    self.leftbutton.hide()
                    self.rightbutton.hide()
                    self.maskbutton.hide()
                    

                

        else:
            self.trackbutton.setText("No Video")
            
    def showmask(self):
        if self.cap is not None:
            if self.maskbutton.isChecked():
                self.maskbutton.setText("Original")
                self.tracker.mask_flag = True
                
            else:
                self.maskbutton.setText("Mask")
                self.tracker.mask_flag = False
                
           
    def invertmaskcommand(self):
        if self.cap is not None:
            self.maskinvert_checkBox.setText("Invert Mask: " + str(self.maskinvert_checkBox.isChecked()))
            self.tracker.maskinvert = self.maskinvert_checkBox.isChecked()

    def pause(self):
        if self.videopath != 0:
            if self.pausebutton.isChecked():
                self.tracker._play_flag = False
                self.pausebutton.setText("Play")
              
            else:#play
                self.tracker._play_flag = True
                self.pausebutton.setText("Pause")
                
               
    
    def adjustframe(self):
        if self.videopath != 0:
            self.tracker.framenum = self.frameslider.value()
            self.framelabel.setText("Frame:"+str(self.tracker.framenum))
        
            


    def frameright(self):
        if self.videopath != 0:
            self.tracker.framenum+=1
            self.frameslider.setSliderPosition(self.tracker.framenum)
            self.framelabel.setText("Frame:"+str(self.tracker.framenum))

    def frameleft(self):
        if self.videopath != 0:
            self.tracker.framenum-=1
            self.frameslider.setSliderPosition(self.tracker.framenum)
            self.framelabel.setText("Frame:"+str(self.tracker.framenum))
    
    
    def get_masksigma(self):
        sigma = self.masksigmaslider.value() /10
        self.masksigmalabel.setText("Mask Sigma: {}".format(sigma) )
        if self.cap is not None:        
            self.tracker.mask_sigma = sigma
        
    def get_croplength(self): 
        crop_length = self.croplengthslider.value()
        if crop_length %2 ==0:
            self.croplengthlabel.setText("Crop Length: {}".format(crop_length) )
            if self.cap is not None:
                self.tracker.crop_length = crop_length

    def savedata(self):
        #create folder in homerdiractory of user
        home_dir = expanduser("~")
        new_dir_name = "Tracking Data"
        desktop_path = os.path.join(home_dir, "Desktop")
        new_dir_path = os.path.join(desktop_path, new_dir_name)
        if not os.path.exists(new_dir_path):
            os.makedirs(new_dir_path)
        
        
        if self.cap is not None and len(self.tracker.robot_list)>0:      
            robot_dictionary = []
            for bot in self.tracker.robot_list:
                robot_dictionary.append(bot.as_dict())
            #create dictionarys from the robot class
            file_path  = os.path.join(new_dir_path, str(datetime.now())+".xlsx")

            
            with pd.ExcelWriter(file_path) as writer:
                for idx, mydict in enumerate(robot_dictionary, start=0):
                    # Create a DataFrame from the dictionary
                    df = pd.DataFrame()

                    for key, value in mydict.items():
                        if key == "Frame":
                            df[f"{key}"] = pd.Series(value, dtype='float64')
                    
                        elif key == "Times":
                            df[f"{key}"] = pd.Series(value, dtype='float64')
                    
                        elif key == "Position":
                            if len(value) > 0:
                                x_coords, y_coords = zip(*value)
                                df[f"{key}_X"] = pd.Series(x_coords, dtype='float64')
                                df[f"{key}_Y"] = pd.Series(y_coords, dtype='float64')
                    
                        elif key == "Velocity":
                            if len(value) > 0:
                                x_coords, y_coords = zip(*value)
                                df[f"{key}_X"] = pd.Series(x_coords, dtype='float64')
                                df[f"{key}_Y"] = pd.Series(y_coords, dtype='float64')
                        
                        elif key == "Cropped Frame Dim":
                            if len(value) > 0:
                                x1, y1, w, h = zip(*value)
                                df[f"{key}_x1"] = pd.Series(x1, dtype='float64')
                                df[f"{key}_y1"] = pd.Series(y1, dtype='float64')
                                df[f"{key}_w"] = pd.Series(w, dtype='float64')
                                df[f"{key}_h"] = pd.Series(h, dtype='float64')
                        
                        elif key == "Area":
                            df[f"{key}"] = pd.Series(value, dtype='float64')

                        elif key == "Blur":
                            df[f"{key}"] = pd.Series(value, dtype='float64')
                        
                        elif key == "Avg Area":
                            df[f"{key}"] = pd.Series(value, dtype='float64')
                    
                        elif key == "Trajectory":
                            if len(value) > 0:
                                x_coords, y_coords = zip(*value)
                                df[f"{key}_X"] = pd.Series(x_coords, dtype='float64')
                                df[f"{key}_Y"] = pd.Series(y_coords, dtype='float64')
                    
                        elif key == "Acoustic Frequency":
                            if len(value) > 0:
                                df[f"{key}"] = pd.Series(value, dtype='float64')

                        elif key == "Magnetic Field":
                            if len(value) > 0:
                                for MFkey, MFvalue in value.items():
                                    df[f"{MFkey}"] = pd.Series(MFvalue, dtype='float64')

                    # Write the DataFrame to a separate sheet with sheet name "Sheet_<idx>"
                    df.to_excel(writer, sheet_name=f"Robot_{idx+1}", index=False)
                
        
    


class Robot:
    """
    Robot class to store and ID all new robots currently being tracked.

    Args:
        None
    """

    def __init__(self):
        self.velocity_list = []  # stores bot velocities per frame
        self.position_list = []  # stores bot positions per frame
        self.blur_list = []  # stores calculated blur values per frame (AKA z-value)
        self.frame_list = []  # unused? stores frames
        self.area_list = []  # stores the cropped areas
        self.cropped_frame = []  # cropped section of a frame representing the bot
        self.avg_area = 0  # current average area of the bot in this frame
        self.trajectory = []  # track points from manual pathing
        self.times = []  #time step per frame in seconds
        self.acoustic_freq = []#acoustic frequency applied
        self.magnetic_field = [] #magnetif field params being sent to arduino
        

    def add_area(self, area: float):
        self.area_list.append(area)

    def add_blur(self, blur: float):
        self.blur_list.append(blur)

    def add_velocity(self, velocity):
        self.velocity_list.append(velocity)

    def add_position(self, position: List[float]):
        self.position_list.append(position)
    
    def add_frame(self, frame: int):
        self.frame_list.append(frame)

    def add_crop(self, crop: List[int]):
        self.cropped_frame.append(crop)

    def set_avg_area(self, avg_area: float):
        self.avg_area = avg_area

    def add_trajectory(self, traj: List[int]):
        self.trajectory.append(traj)

    def add_acoustic_freq(self, freq: float):
        self.acoustic_freq.append(freq)

    def add_time(self, time):
        self.times.append(time)
    
    def add_magnetic_field(self, magnetic_field):
        self.magnetic_field.append(magnetic_field)
    

    def as_dict(self) -> dict:
        """
        Convert's the bot's current frame and position information into a
        readable dictionary

        Args:
            None

        Returns:
            dictionary of the bot's current position and frame information
        """
        mydict = {
            "Frame": self.frame_list,
            "Times": self.times,
            "Position": self.position_list,
            "Velocity": self.velocity_list,
            "Cropped Frame Dim": self.cropped_frame,
            "Area": self.area_list,
            "Blur": self.blur_list,
            "Avg Area": self.avg_area,
            "Trajectory": self.trajectory,
            "Acoustic Frequency": self.acoustic_freq,
            "Magnetic Field": self.magnetic_field,
 
        }

        return mydict




class VideoThread(QThread):
    change_pixmap_signal = pyqtSignal(np.ndarray)
    cropped_frame_signal = pyqtSignal(np.ndarray)




    def __init__(self, cap):
        super().__init__()
        self._run_flag = True
        self._play_flag = True
        self.mask_flag = False
        #self.cap = cv2.VideoCapture(videopath)
        self.cap = cap
        self.fps = FPSCounter()
        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.videofps = int(self.cap.get(cv2.CAP_PROP_FPS))
        self.totalnumframes = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))

        self.framenum = 0
        self.mask_sigma = 0.7
        self.maskinvert = True
        self.crop_length = 40
        self.robot_list = []

       

    def detect_robot(self, frame):
        """
        there are techincally 4 frames for analyse at a time:
        1) displayframe:  this is the raw BGR frame from the videofeed
        2) croppedframe: this is the cropped BGR frame surrounding the nearby robot 
        3) displaymask: this is the raw MONO mask from the videofeed
        4) croppedmask: this is the cropped MONO mask surrounding the nearby robot 
        
        """
        displayframe = frame.copy()
        
        if self.mask_flag == True:
            displaymask = self.find_mask(displayframe)
            displayframe = cv2.cvtColor(displaymask, cv2.COLOR_GRAY2BGR)
        
        if len(self.robot_list) > 0:
            
            color = plt.cm.rainbow(np.linspace(1, 0, len(self.robot_list))) * 255
            for (bot, botcolor) in zip(self.robot_list, color): #for each bot with a position botx, boty, find the cropped frame around the bot
               
                #current cropped frame dim
                x1, y1, w, h = bot.cropped_frame[-1]
                x1 = max(min(x1, self.width), 0)
                y1 = max(min(y1, self.height), 0)

                #crop the frame
                croppedframe = frame[y1 : y1 + h, x1 : x1 + w]
                
                #find the mask
                croppedmask  = self.find_mask(croppedframe)

                #find contours from the mask
                contours, _ = cv2.findContours(croppedmask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                if len(contours) !=0:
                    max_cnt = contours[0]
                    for contour in contours:
                        if cv2.contourArea(contour) > cv2.contourArea(max_cnt): 
                            max_cnt = contour
                    area = cv2.contourArea(max_cnt)
                else:
                    area = 0
            

                #label the mask
                label_im, nb_labels = ndimage.label(croppedmask) 
                sizes = ndimage.sum(croppedmask, label_im, range(nb_labels + 1)) 
                num_bots=np.sum(sizes>50)
                
                if num_bots>0:
                    #find the center of mass from the mask
                    szsorted=np.argsort(sizes)
                    [ycord,xcord]=ndimage.center_of_mass(croppedmask,labels=label_im,index = szsorted[-(1)])
                    
                    
                    #derive the global current location
                    current_pos = [xcord + x1,   ycord + y1] #xcord ycord are relative to the cropped frame. need to convert to the overall frame dim

                    
                    #generate new cropped frame based on the new robots position
                    x1_new = int(current_pos[0] - self.crop_length/2)
                    y1_new = int(current_pos[1] - self.crop_length/2)
                    w_new = int(self.crop_length)
                    h_new = int(self.crop_length)
                    new_crop = [int(x1_new), int(y1_new), int(w_new), int(h_new)]

                    #find velocity:
                    displacement = (np.array(current_pos) - np.array(bot.position_list[-1])).astype(float).round(2)

                    #find blur of original crop
                    blur = cv2.Laplacian(croppedframe, cv2.CV_64F).var()
                    
                    #store the data in the instance of RobotClasss
                    bot.add_frame(self.framenum)
                    bot.add_time(self.cap.get(cv2.CAP_PROP_POS_MSEC)/1000) #original in ms
                    bot.add_position([current_pos[0], current_pos[1]])
                    bot.add_velocity(displacement)
                    bot.add_crop(new_crop)
                    bot.add_area(area)
                    bot.add_blur(blur)
                    bot.set_avg_area(np.mean(bot.area_list))
                    
                    
                    #display visuals
                    cv2.circle(displayframe,(int(xcord+ x1), int(ycord + y1)),5,(botcolor),-1,)
                    cv2.rectangle(displayframe, (x1_new, y1_new), (x1_new + w_new, y1_new + h_new), (botcolor), 3)
                    
                    pts = np.array(bot.position_list, np.int32)
                    cv2.polylines(displayframe, [pts], False, botcolor, 3)
                             

                
            #constantly display the cropped frame mask for parameter tuning
            croppedmask = cv2.cvtColor(croppedmask, cv2.COLOR_GRAY2BGR)
            if len(contours) !=0:
                cv2.drawContours(croppedmask, [max_cnt], -1, (0, 255, 255), 1)

        else:
            croppedmask = np.zeros((200, 200, 3), dtype=np.uint8) 

        return displayframe, croppedmask
 

            


    def find_mask(self, frame):

        im1 = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        invert = self.maskinvert
        mask_sigma= self.mask_sigma
        
        img_threshold=0


        BackgroundImg =  np.ones_like(im1)#BackgroundImage;#I only take every 10th image, any more is likely unnecessary, depends on fps but this can be slow
        meanBackgroundImg = np.median(BackgroundImg)
        
        imgdiff=(-(meanBackgroundImg-np.median(im1))+BackgroundImg-im1)
        if not invert:
            imgdiff = np.mean(imgdiff)-imgdiff;#if invert is true then just keep the img as it is, this is for dark subjects. If invert is false then make it negative and shift by mean, this is for bright subjects
        
        maskdiff = imgdiff < img_threshold
        finalimg= np.where(maskdiff,np.zeros_like(im1),imgdiff)#setting dark pixels to zero
            
        mask = finalimg > (finalimg.mean() + mask_sigma*finalimg.std())
        mask = mask.astype(np.uint8)  #convert to an unsigned byte
        mask*=255
    
        return mask
    




  
        
    def run(self):
        # capture from web cam
        while self._run_flag:
            self.fps.update()

            #set and read frame
            if self._play_flag == True:
                self.framenum +=1


            if self.totalnumframes !=0:
                if self.framenum >  self.totalnumframes:
                    self.framenum = 0
                self.cap.set(1, self.framenum)
            
            
            ret, frame = self.cap.read()
        

            if ret:
                cv2.putText(frame,str(int(self.fps.get_fps())),
                    (int(self.width  / 40),int(self.height / 30)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    fontScale=1, 
                    thickness=4,
                    color = (255, 255, 255))
                
                #step 1 detect robot
                frame, croppedframe = self.detect_robot(frame) 

                
                 #display graphics and emit the frame to the gui
                self.cropped_frame_signal.emit(croppedframe)
                self.change_pixmap_signal.emit(frame)
          

                if self.totalnumframes !=0:
                    interval = int(1000/self.videofps)  #use original fps used to record the video if not live
                    cv2.waitKey(interval)



    def stop(self):
        """Sets run flag to False and waits for thread to finish"""
        #blank = np.zeros((self.width, self.height, 3), dtype=np.uint8) 
        #self.change_pixmap_signal.emit(blank)

        self._run_flag = False
        self.wait()
        self.cap.release()
        cv2.destroyAllWindows




class FPSCounter:
    """
    Class for managing the FPS of the microbot tracker

    Args:
        None
    """

    def __init__(self):
        self.t0 = time.time()
        self.t1 = self.t0
        self.fps = 0

    def update(self):
        self.t1 = time.time()
        try:
            self.fps = 1 / (self.t1 - self.t0)
        except ZeroDivisionError:
            self.fps = 0
        self.t0 = self.t1

    def get_fps(self):
        return self.fps
    




if __name__=="__main__":
    app = QApplication(sys.argv)
    a = App()
    a.show()
    sys.exit(app.exec_())
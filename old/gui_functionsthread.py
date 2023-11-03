from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QApplication
import sys

from PyQt5 import QtGui
from PyQt5.QtWidgets import QWidget
from PyQt5.QtGui import QPixmap,QIcon
from PyQt5.QtCore import Qt, QTimer, PYQT_VERSION_STR
from PyQt5 import QtWidgets, QtGui, QtCore
import cv2
import os
from os.path import expanduser
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import random
import pandas as pd
from datetime import datetime
import sys
from PyQt5.QtWidgets import QApplication
import numpy as np
import cv2
import matplotlib.pyplot as plt 
import time
import platform
import pygame
os.environ["SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS"] = "1"
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"


from classes.tracker_class import VideoThread
from classes.gui_widgets import Ui_MainWindow
from classes.robot_class import Robot
from classes.arduino_class import ArduinoHandler
from classes.joystick_class import ControllerActions
from old.simulation_classthread import HelmholtzSimulator
from classes.projection_class import AxisProjection
from classes.acoustic_class import AcousticClass



class MainWindow(QtWidgets.QMainWindow):
    positionChanged = QtCore.pyqtSignal(QtCore.QPoint)

    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent=parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        self.display_width = 1330# self.ui.frameGeometry().width()
        self.display_height = 700 #keep this fixed, changed the width dpending on the aspect ratio

        self.window_width = 1600
        self.window_height = 860
        self.resize(self.window_width, self.window_height)
        self.ui.frameslider.hide()

        #create folder in homerdiractory of user
        home_dir = expanduser("~")
        new_dir_name = "Tracking Data"
        desktop_path = os.path.join(home_dir, "Desktop")
        self.new_dir_path = os.path.join(desktop_path, new_dir_name)
        if not os.path.exists(self.new_dir_path):
            os.makedirs(self.new_dir_path)


        
        self.result = None
        self.rec_start_time = time.time()
        self.videopath = 0
        self.cap = None
        self.drawing = False
        self.acoustic_frequency = 0
        self.magnetic_field_list = []
        self.actions = [0,0,0,0,0,0,0,0,0]


        #connect to arduino
        if "mac" in platform.platform():
            self.tbprint("Detected OS: macos")
            PORT = "/dev/cu.usbmodem111101"
        elif "Linux" in platform.platform():
            self.tbprint("Detected OS: Linux")
            PORT = "/dev/ttyACM0"
        elif "Windows" in platform.platform():
            self.tbprint("Detected OS:  Windows")
            PORT = "COM3"
        else:
            self.tbprint("undetected operating system")
            PORT = None
        
        self.arduino = ArduinoHandler(self.tbprint)
        self.arduino.connect(PORT)
        
        #if self.arduino.conn is None:
            #self.ui.controlbutton.hide()
        
        #define joystick class, simulator class, pojection class, and acoustic class
        self.controller_actions = ControllerActions()
        #self.simulator = HelmholtzSimulator(self.ui.magneticfieldsimlabel, width=200, height=200, dpi=50)
        self.projection = AxisProjection()
        self.acoustic_module = AcousticClass()
        
        pygame.init()
        if pygame.joystick.get_count() == 0:
            self.tbprint("No Joystick Connected...")
            
        else:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            self.tbprint("Connected to: "+str(self.joystick.get_name()))


        #tracker tab functions
        self.ui.pausebutton.hide()
        self.ui.leftbutton.hide()
        self.ui.rightbutton.hide()
        self.ui.maskbutton.hide()
        
        self.ui.choosevideobutton.clicked.connect(self.selectFile)
        self.ui.trackbutton.clicked.connect(self.track)
        self.ui.pausebutton.clicked.connect(self.pause)
        self.ui.rightbutton.clicked.connect(self.frameright)
        self.ui.leftbutton.clicked.connect(self.frameleft)
        self.ui.maskbutton.clicked.connect(self.showmask)
        self.ui.maskinvert_checkBox.toggled.connect(self.invertmaskcommand)
        self.ui.masksigmabox.valueChanged.connect(self.get_slider_vals)
        self.ui.maskdilationbox.valueChanged.connect(self.get_slider_vals)
        self.ui.croplengthbox.valueChanged.connect(self.get_slider_vals)
        self.ui.savedatabutton.clicked.connect(self.savedata)
        self.ui.frameslider.valueChanged.connect(self.adjustframe)
        self.ui.VideoFeedLabel.installEventFilter(self)
        self.ui.recordbutton.clicked.connect(self.recordfunction)


        #control tab functions
        self.control_status = False
        self.joystick_status = False

        self.ui.led.hide()
        self.ui.acousticfreq_spinBox.hide()
        self.ui.acousticfreqlabel.hide()
        self.ui.applyacousticbutton.hide()

        self.ui.controlbutton.clicked.connect(self.toggle_control_status)
        self.ui.joystickbutton.clicked.connect(self.toggle_joystick_status)
        self.ui.memorybox.valueChanged.connect(self.get_slider_vals)
        self.ui.RRTtreesizebox.valueChanged.connect(self.get_slider_vals)
        self.ui.arrivalthreshbox.valueChanged.connect(self.get_slider_vals)
        self.ui.rollingfrequencybox.valueChanged.connect(self.get_slider_vals)
        self.ui.gammadial.valueChanged.connect(self.get_slider_vals)
        self.ui.psidial.valueChanged.connect(self.get_slider_vals)
        self.ui.applyacousticbutton.clicked.connect(self.apply_acoustic)
        self.ui.acousticfreq_spinBox.valueChanged.connect(self.get_acoustic_frequency)
        self.ui.resetdefaultbutton.clicked.connect(self.resetparams)
        self.ui.simulationbutton.clicked.connect(self.toggle_simulation)
   
        
        self.simulator = HelmholtzSimulator(self.ui.magneticfieldsimlabel, width=200, height=200, dpi=50)
        #self.simulator.figure_ready.connect(self.update_sim)
        self.simulator.start()

        
       
  
    #def update_sim(self,canvas):
        #canvas.draw()
    #    print(canvas)

    def get_slider_vals(self):
        memory = self.ui.memorybox.value()
        RRTtreesize = self.ui.RRTtreesizebox.value()
        arrivalthresh = self.ui.arrivalthreshbox.value()
        rollingfreq = self.ui.rollingfrequencybox.value()
        gamma = self.ui.gammadial.value()
        psi = self.ui.psidial.value()
        sigma = self.ui.masksigmabox.value() /10
        dilation = self.ui.maskdilationbox.value() 
        crop_length = self.ui.croplengthbox.value()

        if self.cap is not None: 
            self.tracker.memory = memory
            self.tracker.RRTtreesize = RRTtreesize
            self.tracker.arrivalthresh = arrivalthresh
            self.tracker.mask_sigma = sigma
            self.tracker.mask_dilation = dilation
            self.tracker.crop_length = crop_length
    

        #self.ui.memorylabel.setText("Memory:            {}".format(memory))
        #self.ui.RRTtreesizelabel.setText("RRT Tree Size:     {}".format(RRTtreesize))
        #self.ui.arrivalthreshlabel.setText("Arrival Thresh:    {}".format(arrivalthresh))
        #self.ui.rollingfrequencylabel.setText("Rolling Frequency: {}".format(rollingfreq))
        self.ui.gammalabel.setText("Gamma: {}".format(gamma))
        self.ui.psilabel.setText("Psi: {}".format(psi))
        #self.ui.masksigmalabel.setText("Mask Sigma:       {}".format(sigma) )
        #self.ui.maskdilationlabel.setText("Mask Dilation:     {}".format(dilation) )
        #if crop_length %2 ==0:
        #    self.ui.croplengthlabel.setText("Crop Length:       {}".format(crop_length) )


        
        

        
    def toggle_simulation(self):
        if self.ui.simulationbutton.isChecked():
            self.simulator.startsim()
            self.tbprint("Simulation Off")
            self.ui.simulationbutton.setText("Simulation Off")
        else:
            self.simulator.stopsim()
            self.tbprint("Simulation On")
            self.ui.simulationbutton.setText("Simulation On")
            
    
    def toggle_control_status(self):
        if self.ui.controlbutton.isChecked():
            self.control_status = True
            self.ui.controlbutton.setText("Stop")
            self.tbprint("Control On: {} Hz".format(self.acoustic_frequency))
            self.ui.led.show()
            self.ui.acousticfreq_spinBox.show()
            self.ui.acousticfreqlabel.show()
            self.ui.applyacousticbutton.show()
  
        
        
        else:
            self.control_status = False
            self.ui.controlbutton.setText("Control")
            self.tbprint("Control Off")
            self.ui.led.hide()
            self.ui.acousticfreq_spinBox.hide()
            self.ui.acousticfreqlabel.hide()
            self.ui.applyacousticbutton.hide()
       
            
    

    def toggle_joystick_status(self):
        if pygame.joystick.get_count() != 0:
            if self.ui.joystickbutton.isChecked():
                self.joystick_status = True
                self.ui.joystickbutton.setText("Stop")
                self.tbprint("Joystick On")
  
            
            else:
                self.joystick_status = False
                self.ui.joystickbutton.setText("Joystick")
                self.tbprint("Joystick Off")
       
        else:
            self.tbprint("No Joystick Connected...")


    


    def update_actions(self, alpha, arrived):
        #alpha argument is signal from tracker_class: actions_signal
        #output actions if control status is on

        gamma = np.radians(self.ui.gammadial.value())
        psi = np.radians(self.ui.psidial.value())
        if self.control_status == True:
            if self.ui.swimradio.isChecked():
                alpha=alpha
                self.simulator.roll = False
            elif self.ui.rollradio.isChecked():
                alpha = alpha - np.pi/2
                self.simulator.roll = True
            
        

            freq = self.ui.rollingfrequencybox.value()
            
            if arrived == True:
                Bx, By, Bz, alpha, gamma, freq, psi = 0,0,0,0,0,0,0
            else:
                Bx,By,Bz = 0,0,0
            self.arduino.send(Bx, By, Bz, alpha, gamma, freq, psi)  
            self.actions = [self.tracker.framenum,Bx,By,Bz,alpha,gamma,freq,psi,self.acoustic_frequency]          
            self.magnetic_field_list.append(self.actions)
            self.simulator.Bx = Bx
            self.simulator.By = By
            self.simulator.Bz = Bz
            self.simulator.alpha = alpha
            self.simulator.gamma = gamma
            self.simulator.freq = freq/10
            self.simulator.omega = 2 * np.pi * self.simulator.freq
            self.simulator.psi = psi
            

        elif self.joystick_status == True:
            Bx, By, Bz, alpha, freq = self.controller_actions.run(self.joystick)
            if self.ui.swimradio.isChecked():
                alpha=alpha
                self.simulator.roll = False
            elif self.ui.rollradio.isChecked():
                alpha = alpha - np.pi/2
                self.simulator.roll = True
            
            
            if freq !=0:
                freq = self.ui.rollingfrequencybox.value()

            self.arduino.send(Bx, By, Bz, alpha, gamma, freq, psi)
            self.actions = [self.tracker.framenum,Bx,By,Bz,alpha,gamma,freq,psi, self.acoustic_frequency]
            self.magnetic_field_list.append(self.actions)
            self.simulator.Bx = Bx
            self.simulator.By = By
            self.simulator.Bz = Bz
            self.simulator.alpha = alpha
            self.simulator.gamma = gamma
            self.simulator.freq = freq/10
            self.simulator.omega = 2 * np.pi * self.simulator.freq
            self.simulator.psi = psi




       
    
    def get_acoustic_frequency(self):
        self.acoustic_frequency = self.ui.acousticfreq_spinBox.value()
        self.tbprint("Control On: {} Hz".format(self.acoustic_frequency))
        self.apply_acoustic()
    
    def apply_acoustic(self):
        if self.ui.applyacousticbutton.isChecked():
            self.ui.applyacousticbutton.setText("Stop")
            self.tbprint("Control On: {} Hz".format(self.acoustic_frequency))
            self.acoustic_frequency = self.ui.acousticfreq_spinBox.value()
            self.acoustic_module.start(self.acoustic_frequency, 0)
            self.ui.led.setStyleSheet("\n"
"                background-color: rgb(0, 255, 0);\n"
"                border-style: outset;\n"
"                border-width: 3px;\n"
"                border-radius: 12px;\n"
"                border-color: rgb(0, 255, 0);\n"
"         \n"
"                padding: 6px;")
        
        else:
            self.ui.applyacousticbutton.setText("Apply")
            self.tbprint("Acoustic Module Off")
            self.acoustic_module.stop()
            self.ui.led.setStyleSheet("\n"
"                background-color: rgb(255, 0, 0);\n"
"                border-style: outset;\n"
"                border-width: 3px;\n"
"                border-radius: 12px;\n"
"                border-color: rgb(255, 0, 0);\n"
"         \n"
"                padding: 6px;")
       
        

    def tbprint(self, text):
        #print to textbox
        self.ui.plainTextEdit.appendPlainText("$ "+ text)
        

    def selectFile(self):
        options = QtWidgets.QFileDialog.Options()
        options |= QtWidgets.QFileDialog.ReadOnly
        file_path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Open File", "", "All Files (*);;Text Files (*.txt);;Python Files (*.py)", options=options)

        if file_path:
            self.videopath = file_path
            file_info = QtCore.QFileInfo(file_path)
            file_name = file_info.fileName()
            print(file_path, file_name)
            self.ui.choosevideobutton.setText(file_name)
            self.tbprint(file_name)
        else:
            self.videopath = 0
            self.ui.choosevideobutton.setText(str(0))
            self.tbprint("Using Video Camera")

    def convert_coords(self,pos):
        #need a way to convert the video position of mouse to the actually coordinate in the window
        newx = int(pos.x() * (self.video_width / self.display_width)) 
        newy = int(pos.y() * (self.video_height / self.display_height))
        return newx, newy

    def eventFilter(self, object, event):
        
        if object is self.ui.VideoFeedLabel: 

            if self.cap is not None:
                
                if event.type() == QtCore.QEvent.MouseButtonDblClick:        
                    if event.buttons() == QtCore.Qt.LeftButton:
                        newx, newy = self.convert_coords(event.pos())
                        #generate original bounding box
                        crop_length = self.ui.croplengthbox.value()

                        x_1 = int(newx - crop_length  / 2)
                        y_1 = int(newy - crop_length  / 2)
                        w = crop_length
                        h = crop_length

                        #reset algorithm nodes
                        self.tracker.control_robot.reset()

                        robot = Robot()  # create robot instance
                        robot.add_frame(self.tracker.framenum)
                        robot.add_time(0)
                        robot.add_position([newx,newy])
                        robot.add_velocity([0,0])
                        robot.add_crop([x_1, y_1, w, h])
                        robot.add_area(0)
                        robot.add_blur(0)
                        
                        
                        self.tracker.robot_list.append(robot)
                        
       
                elif event.type() == QtCore.QEvent.MouseButtonPress:   
                    if event.buttons() == QtCore.Qt.LeftButton: 
                        self.drawing = True
                        newx, newy = self.convert_coords(event.pos())
                        if len(self.tracker.robot_list) > 0:
                            self.tracker.control_robot.reset()
                            self.tracker.control_robot.arrived = False
                            self.tracker.robot_list[-1].add_trajectory([newx, newy])
                
                
                    if event.buttons() == QtCore.Qt.RightButton: 
                        del self.tracker.robot_list[:]
                        del self.magnetic_field_list[:]
                    
                            
                elif event.type() == QtCore.QEvent.MouseMove:
                    if event.buttons() == QtCore.Qt.LeftButton:
                        if self.drawing == True:
                            if len(self.tracker.robot_list)>0:
                                newx, newy = self.convert_coords(event.pos())
                                
                                self.tracker.robot_list[-1].add_trajectory([newx, newy])
                
                elif event.type() == QtCore.QEvent.MouseButtonRelease:
                    if event.buttons() == QtCore.Qt.LeftButton: 
                        self.drawing = False
                        
                    
        return super().eventFilter(object, event)
        



    def update_image(self, frame):
        """Updates the image_label with a new opencv image"""
        if self.result is not None:
            cv2.putText(frame,"frame: " + str(self.tracker.framenum),
                        (int(self.video_width / 15),
                         int(self.video_height / 30)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        fontScale=1, 
                        thickness=4,
                        color = (255, 255, 255))
            self.result.write(frame)
        
        
        if self.control_status == True or self.joystick_status == True:
            _, Bx,By,Bz,alpha,gamma,_,_,_ = self.actions
            self.projection.roll = self.ui.rollradio.isChecked()
            frame, self.projection.draw_sideview(frame,Bx,By,Bz,alpha,gamma,self.video_width,self.video_height)
            frame, self.projection.draw_topview(frame,Bx,By,Bz,alpha,gamma,self.video_width,self.video_height)
        
        rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        
        bytes_per_line = ch * w
        convert_to_Qt_format = QtGui.QImage(rgb_image.data, w, h, bytes_per_line, QtGui.QImage.Format_RGB888)
        p = convert_to_Qt_format.scaled(self.display_width, self.display_height, Qt.KeepAspectRatio)
        qt_img = QPixmap.fromImage(p)
        
        #update frame slider too
        if self.videopath !=0:
            self.ui.framelabel.setText("Frame:"+str(self.tracker.framenum))
            self.ui.frameslider.setSliderPosition(self.tracker.framenum)

        #also update robot info
        if len(self.tracker.robot_list) > 0:
            self.ui.robotarealabel.setText("Area: {0:.2f} px^2".format(self.tracker.robot_list[-1].avg_area))
            self.ui.robotvelocitylabel.setText("Vx: {0:.2f}, Vy: {0:.2f} px/f".format(self.tracker.robot_list[-1].velocity_list[-1][0], self.tracker.robot_list[-1].velocity_list[-1][1]  )  )
            self.ui.robotblurlabel.setText("Blur: {0:.2f} units".format(self.tracker.robot_list[-1].blur_list[-1]))

        self.ui.VideoFeedLabel.setPixmap(qt_img)
        

    

    def update_croppedimage(self, frame):
        """Updates the cropped image_label with a new cropped opencv image"""
        
        rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        
        bytes_per_line = ch * w
        convert_to_Qt_format = QtGui.QImage(rgb_image.data, w, h, bytes_per_line, QtGui.QImage.Format_RGB888)
        p = convert_to_Qt_format.scaled(200, 200, Qt.KeepAspectRatio)
        qt_cimg = QPixmap.fromImage(p)
        self.ui.CroppedVideoFeedLabel.setPixmap(qt_cimg)
        



    
    def track(self):
        if self.videopath is not None:
            if self.ui.trackbutton.isChecked():
                
                #start video thread
                self.ui.pausebutton.show()
                self.ui.leftbutton.show()
                self.ui.rightbutton.show()
                self.ui.maskbutton.show()

                #need to resize the window in order to maintain proper aspect ratios
                self.cap = cv2.VideoCapture(self.videopath)
                self.video_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                self.video_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                self.videofps = int(self.cap.get(cv2.CAP_PROP_FPS))
                self.tbprint("Width: {}  --  Height: {}  --  Fps: {}".format(self.video_width,self.video_height,self.videofps))

                self.totalnumframes = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))

                self.display_width = int(self.display_height * (self.video_width / self.video_height))
                self.ui.VideoFeedLabel.setGeometry(QtCore.QRect(250, 5, self.display_width, self.display_height))
                self.ui.VideoFeedLabel.setStyleSheet("border:2px solid rgb(0, 255, 0); ")
                self.ui.CroppedVideoFeedLabel.setStyleSheet("border:2px solid rgb(0, 255, 0); ")

                self.window_width = self.display_width+250+50
                self.resize(self.window_width, self.window_height)
                
                if self.videopath != 0:
                    self.ui.frameslider.show()
                    self.ui.frameslider.setGeometry(QtCore.QRect(250, 710, self.display_width, 30))
                    self.ui.frameslider.setMaximum(self.totalnumframes)
            
                self.tracker = VideoThread(self)
                self.tracker.change_pixmap_signal.connect(self.update_image)
                self.tracker.cropped_frame_signal.connect(self.update_croppedimage)
                self.tracker.actions_signal.connect(self.update_actions)
                self.tracker.start()
                
                self.ui.trackbutton.setText("Stop")
                
                if self.videopath == 0:
                    self.ui.pausebutton.hide()
                    self.ui.leftbutton.hide()
                    self.ui.rightbutton.hide()
                    self.ui.frameslider.hide()
         
                
            else:
                self.ui.VideoFeedLabel.setStyleSheet("background-color: rgb(0,0,0); border:2px solid rgb(255, 0, 0); ")
                self.ui.CroppedVideoFeedLabel.setStyleSheet("background-color: rgb(0,0,0); border:2px solid rgb(255, 0, 0); ")
         
                
                if self.cap is not None:
                    self.ui.trackbutton.setText("Track")
                    self.tracker.stop()
                    
                    #reset control button
                    self.control_status = False
                    self.ui.controlbutton.setText("Control")
                    self.tbprint("Control Off")
                    self.ui.controlbutton.setChecked(False)

                    #reset joystick button
                    self.joystick_status = False
                    self.ui.joystickbutton.setText("Joystick")
                    self.tbprint("Joystick Off")
                    self.ui.joystickbutton.setChecked(False)

                    #reset mask button
                    self.tracker.mask_flag = False
                    self.ui.maskbutton.setText("Mask")
                    self.ui.maskbutton.setChecked(False)

                    
                    #also reset pause button
                    self.ui.pausebutton.setChecked(False)
                    self.ui.pausebutton.setText("Pause")

                    self.ui.pausebutton.hide()
                    self.ui.leftbutton.hide()
                    self.ui.rightbutton.hide()
                    self.ui.maskbutton.hide()    

                    #zero arduino commands
                    self.arduino.send(0,0,0,0,0,0,0)

        else:
            self.ui.trackbutton.setText("No Video")
            

    def showmask(self):
        if self.cap is not None:
            if self.ui.maskbutton.isChecked():
                self.ui.maskbutton.setText("Original")
                self.tracker.mask_flag = True
            else:
                self.ui.maskbutton.setText("Mask")
                self.tracker.mask_flag = False


    def recordfunction(self):
        if self.cap is not None:
            if self.ui.recordbutton.isChecked():
                self.ui.recordbutton.setText("Stop")
                self.tbprint("Start Record")
                file_path  = os.path.join(self.new_dir_path, str(datetime.now())+".mp4")
                self.rec_start_time = time.time()
                self.result = cv2.VideoWriter(
                    file_path,
                    cv2.VideoWriter_fourcc(*"mp4v"),
                    self.videofps,    
                    (self.video_width, self.video_height), ) 
            else:
                self.ui.recordbutton.setText("Record")
                if self.result is not None:
                    self.result.release()
                    self.result = None
                    self.tbprint("End Record, Data Saved")
                    self.savedata()
                    
         

    def invertmaskcommand(self):
        if self.cap is not None:
            self.ui.maskinvert_checkBox.setText("Invert Mask: " + str(self.ui.maskinvert_checkBox.isChecked()))
            self.tracker.maskinvert = self.ui.maskinvert_checkBox.isChecked()

    def pause(self):
        if self.videopath != 0:
            if self.ui.pausebutton.isChecked():
                self.tracker._play_flag = False
                self.ui.pausebutton.setText("Play")
              
            else:#play
                self.tracker._play_flag = True
                self.ui.pausebutton.setText("Pause")
                
            
    def adjustframe(self):
        if self.videopath != 0:
            self.tracker.framenum = self.ui.frameslider.value()
            self.ui.framelabel.setText("Frame:"+str(self.tracker.framenum))


    def frameright(self):
        if self.videopath != 0:
            self.tracker.framenum+=1
            self.ui.frameslider.setSliderPosition(self.tracker.framenum)
            self.ui.framelabel.setText("Frame:"+str(self.tracker.framenum))

    def frameleft(self):
        if self.videopath != 0:
            self.tracker.framenum-=1
            self.ui.frameslider.setSliderPosition(self.tracker.framenum)
            self.ui.framelabel.setText("Frame:"+str(self.tracker.framenum))



    def savedata(self):
        if self.cap is not None and len(self.tracker.robot_list)>0:      
             #create dictionarys from the robot class
            file_path  = os.path.join(self.new_dir_path, str(datetime.now())+".xlsx")
            robot_dictionary = []
            for bot in self.tracker.robot_list:
                robot_dictionary.append(bot.as_dict())
           

            
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


                        # Write the DataFrame to a separate sheet with sheet name "Sheet_<idx>"
                        df.to_excel(writer, sheet_name=f"Robot_{idx+1}", index=False)
                        
                        #also save the magnetic field params 
                        if len(self.magnetic_field_list) > 0:     
                            df2 = pd.DataFrame()      

                            MFFrame, Bx, By, Bz, alpha, gamma, freq, psi, acoustic_freq = zip(*self.magnetic_field_list)
                            df2[f"Applied on Frame"] = pd.Series(MFFrame, dtype='float64')
                            df2[f"Bx"] = pd.Series(Bx, dtype='float64')
                            df2[f"By"] = pd.Series(By, dtype='float64')
                            df2[f"Bz"] = pd.Series(Bz, dtype='float64')
                            df2[f"alpha"] = pd.Series(alpha, dtype='float64')
                            df2[f"gamma"] = pd.Series(gamma, dtype='float64')
                            df2[f"freq"] = pd.Series(freq, dtype='float64')
                            df2[f"psi"] = pd.Series(psi, dtype='float64')
                            df2[f"acoustic_frequency"] = pd.Series(acoustic_freq, dtype='float64')
                            df2.to_excel(writer, sheet_name=f"Magnetic Field", index=False)
            
    
    def resetparams(self):
        self.ui.masksigmabox.setValue(0.7)
        self.ui.maskdilationbox.setValue(0)
        self.ui.croplengthbox.setValue(40)
        self.ui.memorybox.setValue(15)
        self.ui.RRTtreesizebox.setValue(25)
        self.ui.arrivalthreshbox.setValue(15)
        self.ui.rollingfrequencybox.setValue(10)
        self.ui.gammadial.setSliderPosition(90)
        self.ui.psidial.setSliderPosition(90)


    def closeEvent(self, event):
        """
        called when x button is pressed
        """
        if self.cap is not None:
            self.tracker.stop()
        self.simulator.stopsim()
        self.arduino.close()

# imports from gui_functions.py 

from classes.control_class import Controller
from classes.gui_functions import MainWindow
from classes.path_planning_class import Path_Planner
from classes.simulation_class import HelmholtzSimulator

from classes.gui_widgets import Ui_MainWindow
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QApplication, QFileDialog
import platform
from PyQt5.QtCore import Qt, QTimer, PYQT_VERSION_STR, QEvent
import pygame
try:
    import EasyPySpin
except Exception:
    pass
from classes.joystick_class import Mac_Joystick,Linux_Joystick,Windows_Joystick
from datetime import datetime
import queue
from os.path import expanduser
import os
from os.path import expanduser
from classes.gui_widgets import Ui_MainWindow





def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent=parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        

        
        
        #self.showMaximized()

        #resize some widgets to fit the screen better
        screen  = QtWidgets.QDesktopWidget().screenGeometry(-1)
        
        self.window_width = screen.width()
        self.window_height = screen.height()
        self.resize(self.window_width, self.window_height)
        self.display_width = self.window_width# self.ui.frameGeometry().width()

        self.displayheightratio = 0.79
        self.framesliderheightratio = 0.031
        self.textheightratio = .129
        self.tabheightratio = 0.925
        self.tabheightratio = 0.925
        
        self.aspectratio = 1041/801
        self.resize_widgets()

    
      
        #create folder in homerdiractory of user
        if "Windows" in platform.platform():
            home_dir = expanduser("D:")
            new_dir_name = "Tracking Data"
            desktop_path = os.path.join(home_dir, "Microrobots")
            self.new_dir_path = os.path.join(desktop_path, new_dir_name)
            if not os.path.exists(self.new_dir_path):
                os.makedirs(self.new_dir_path)
        else:
            #home_dir = expanduser("~")
            #new_dir_name = "Tracking Data"
            #desktop_path = os.path.join(home_dir, "Desktop")
            
            self.new_dir_path = "Data"#os.path.join(desktop_path, new_dir_name)
            if not os.path.exists(self.new_dir_path):
                os.makedirs(self.new_dir_path)



        self.zoom_x, self.zoom_y, self.zoomscale, self.scrollamount = 1,0,0,0
        self.croppedresult = None
        self.frame_number = 0
        self.robots = []
        self.cells = []
        self.videopath = 0
        self.cap = None
        self.tracker = None
        self.populate_serial_ports()
        self.arduino = None

        #record variables
        self.recorder = None
        self.frame_queue = queue.Queue(maxsize=100)  # make a queue to store frames in for the recording feature
        self.output_file_name = datetime.now().strftime("%Y-%m-%d_%H-%M-%S_%f")
   

        self.save_status = False
        self.output_workbook = None
        self.ricochet_counter_x = [0]
        self.ricochet_counter_y = [0]
        
        
        self.drawing = False
        self.acoustic_frequency = 0
        self.autoacousticstatus = False
        self.gradient_status = 0
        self.equal_field_status = 0
        self.magnetic_field_list = []
        
        self.actions = [0,0,0,0,0,0,0,0,0,0,0,0,0]
        self.Bx, self.By, self.Bz = 0,0,0
        self.Mx, self.My, self.Mz = 0,0,0
        self.alpha, self.gamma, self.psi, self.freq = 0,0,0,0
        self.field_magnitude = 100

      
        #control tab functions
        self.path_planner_status = False
        self.control_status = False
        self.joystick_status = False
        self.manual_status = False


  
        if "mac" in platform.platform():
            self.tbprint("Detected OS: macos")
            self.joystick_actions = Mac_Joystick()
        elif "Linux" in platform.platform():
            self.tbprint("Detected OS: Linux")
            self.joystick_actions = Linux_Joystick()
        elif "Windows" in platform.platform():
            self.tbprint("Detected OS:  Windows")
            self.joystick_actions = Windows_Joystick()
        else:
            self.tbprint("undetected operating system")
        

        
        
        #define, simulator class, pojection class, and acoustic class
        self.simulator = HelmholtzSimulator(self.ui.magneticfieldsimlabel, width=310, height=310, dpi=200)

        
        #make instance of algorithm class both control and path planning
        self.control_robot = Controller()
        self.path_planner = Path_Planner()
        



        self.setFile()
        
        pygame.init()
        if pygame.joystick.get_count() == 0:
            self.tbprint("No Joystick Connected...")
            
        else:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            self.tbprint("Connected to: "+str(self.joystick.get_name()))
        
      
        self.sensorupdatetimer = QTimer(self)
        self.sensorupdatetimer.timeout.connect(self.update_sensor_label)
        self.sensorupdatetimer.start(25)  # Update every 500 ms
        self.bx_sensor = 0
        self.by_sensor = 0 
        self.bz_sensor = 0

     

        #tracker tab functions
        self.ui.pausebutton.hide()
        self.ui.leftbutton.hide()
        self.ui.rightbutton.hide()
        self.ui.choosevideobutton.clicked.connect(self.selectFile)
        self.ui.startbutton.clicked.connect(self.start)
        self.ui.pausebutton.clicked.connect(self.pause)
        self.ui.rightbutton.clicked.connect(self.frameright)
        self.ui.leftbutton.clicked.connect(self.frameleft)
        self.ui.maskbutton.clicked.connect(self.showmask)
        self.ui.maskinvert_checkBox.toggled.connect(self.invertmaskcommand)
        self.ui.robotmasklowerbox.valueChanged.connect(self.get_slider_vals)
        self.ui.robotmaskupperbox.valueChanged.connect(self.get_slider_vals)
        self.ui.robotmaskdilationbox.valueChanged.connect(self.get_slider_vals)
        self.ui.robotmaskblurbox.valueChanged.connect(self.get_slider_vals)
        self.ui.robotcroplengthbox.valueChanged.connect(self.get_slider_vals)
        self.ui.cellmasklowerbox.valueChanged.connect(self.get_slider_vals)
        self.ui.cellmaskupperbox.valueChanged.connect(self.get_slider_vals)
        self.ui.cellmaskdilationbox.valueChanged.connect(self.get_slider_vals)
        self.ui.cellmaskblurbox.valueChanged.connect(self.get_slider_vals)
        self.ui.cellcroplengthbox.valueChanged.connect(self.get_slider_vals)
        self.ui.gradient_status_checkbox.toggled.connect(self.gradientcommand)
        self.ui.equal_field_checkbox.toggled.connect(self.equalfieldcommand)
        self.ui.savedatabutton.clicked.connect(self.savedata)
        self.ui.VideoFeedLabel.installEventFilter(self)
        self.ui.recordbutton.clicked.connect(self.toggle_recording)
        self.ui.controlbutton.clicked.connect(self.toggle_control_status)
        self.ui.memorybox.valueChanged.connect(self.get_slider_vals)
        self.ui.RRTtreesizebox.valueChanged.connect(self.get_slider_vals)
        self.ui.arrivalthreshbox.valueChanged.connect(self.get_slider_vals)
        self.ui.magneticfrequencydial.valueChanged.connect(self.get_slider_vals)
        self.ui.gammadial.valueChanged.connect(self.get_slider_vals)
        self.ui.psidial.valueChanged.connect(self.get_slider_vals)
        self.ui.applyacousticbutton.clicked.connect(self.apply_acoustic)
        self.ui.acousticfreq_spinBox.valueChanged.connect(self.get_acoustic_frequency)
        self.ui.alphaspinBox.valueChanged.connect(self.spinbox_alphachanged)
        self.ui.alphadial.valueChanged.connect(self.dial_alphachanged)
        self.ui.resetdefaultbutton.clicked.connect(self.resetparams)
        self.ui.simulationbutton.clicked.connect(self.toggle_simulation)

        self.ui.objectivebox.valueChanged.connect(self.get_objective)
        self.ui.exposurebox.valueChanged.connect(self.get_exposure)
        self.ui.joystickbutton.clicked.connect(self.toggle_joystick_status)
        self.ui.autoacousticbutton.clicked.connect(self.toggle_autoacoustic)
        self.ui.manualapplybutton.clicked.connect(self.get_manual_bfieldbuttons)
        self.ui.manualfieldBx.valueChanged.connect(self.get_slider_vals)
        self.ui.manualfieldBy.valueChanged.connect(self.get_slider_vals)
        self.ui.manualfieldBz.valueChanged.connect(self.get_slider_vals)
        self.ui.croppedmasktoggle.clicked.connect(self.showcroppedoriginal)
        self.ui.croppedrecordbutton.clicked.connect(self.croppedrecordfunction)
        self.ui.import_excel_actions.clicked.connect(self.read_excel_actions)
        self.ui.apply_actions.clicked.connect(self.apply_excel_actions)
        self.ui.arduino_portbox.currentTextChanged.connect(self.handle_port_change)

        self.ui.cleartrackingbutton.clicked.connect(self.clear_tracking)

        self.excel_file_name = None
        self.excel_actions_df = None
        self.excel_actions_status = False


        self.ui.make_inf_path.clicked.connect(self.makeinf_trajectory)
# Magnetoacoustic Microrobotic Manipulation System
This system is a magnetic and acoustic microrobotic manipulation device. Its intention is to be a user friendly, portable, micro-robotic
experimentation platform. It contains an embedded single board computer, a microscope, power supplies, power
amplifiers, and control circuitry necessary for generating the complex magnetic fields neccesary for actuating a variety of micro robots. Custom
control software is written in Python and C++ for handling live image feed from a microscope camera, custom tracking
and detection algorthims, and outputting the control signals to the electromagnetic coils and acoustic transducers. Many of the mechanical components can be 3D
printed allowing others to build the device at a low cost. The device can be divided into three
sections (Mechanical, Electrical, and Software) 

# CAD System Overview

<img width="2160" height="1215" alt="Slide3" src="https://github.com/user-attachments/assets/6a907d4f-e47d-4867-a33b-fdfcddf82bfc" />

<img width="4372" height="2722" alt="crosssection" src="https://github.com/user-attachments/assets/cbda2e2e-d56f-45cc-a393-02800b4db7e3" />

# Electrical Component Flowchart and Wiring Diagram

<img width="8969" height="3730" alt="electronics" src="https://github.com/user-attachments/assets/4632f909-fad0-4cf2-bebd-ed72d3d4c59e" />

# Front End and Back End Software Architecture

<img width="6155" height="2612" alt="Figure6" src="https://github.com/user-attachments/assets/3d7c3ec6-8764-427b-abbe-94f78bce155c" />


# Example Control Algorithm:
<img width="699" alt="Screenshot 2023-12-20 at 2 14 26 PM" src="https://github.com/MaxSokolich/Magnetoacoustic-Microrobotic-Manipulation-System/assets/50302377/0eedb007-9db3-4152-8e76-12740618e227">

# System
![Magscope3-2](https://github.com/MaxSokolich/Magnetoacoustic-Microrobotic-Manipulation-System/assets/50302377/697d35fd-252c-45ac-ad69-98e3f2571512)

# Excel Data
Data regarding a microrobots position, velocity, etc and applied magnetic field actions can be recorded and saved to an excel file. Each row in the excel file corresponds to the microrobots tracking data and action signals sent to the arduino at each frame of the camera. 
Also, a predefined excel file with desired action commands can be imported into the system and applied. The software will begin executing each entry (or row) from the excel file once the "Apply Excel Actions" button is pressed. Each row of actions are applied at each subsequent frame from the camera. 
<img width="259" alt="Screenshot 2024-12-06 at 11 27 47 AM" src="https://github.com/user-attachments/assets/9a6c4148-71ba-4c54-972f-bc68fefd5a64">
<img width="775" alt="Screenshot 2024-12-06 at 11 25 11 AM" src="https://github.com/user-attachments/assets/9ce89367-d93f-425e-b5c2-988ce568c746">

# Variations of System for Custom Experiments/Algorithms
Custom GUIs can be designed with different control variables. These allow custom microrobotic algorithms to be tested and experimented with.

a) Model Predictive Control (MPC) of a magnetiized sender cell bot following an infinity path. b) Novel analytical geometry based path planning algorithm for navigating a microrobot around obstacles.
![magscopeUIs](https://github.com/user-attachments/assets/6281b193-272a-4e1b-8e32-efe0712b9729)


# Instructions for initial installation of system components on Nvidia Jetson AGX Orin:
Note*** Software and system also works on Windows and MAC-OS. Jetson Orin System not neccesary.

1) need to configure nvme ssd using nvidia sdkmanager:  
    - https://developer.nvidia.com/embedded/learn-get-started-jetson-agx-orin-devkit

2) need to build opencv with cuda support: 
    - https://github.com/mdegans/nano_build_opencv.git

3) need to add permissions in order to read and write to the arduino port: 
    - add this to /etc/rc.local to execute on boot: $ chmod 666 /dev/ttyACM0

4)  need to install qt5
    - sudo apt install qtbase5-dev qtchooser qt5-qmake qtbase5-dev-tools  
    - sudo apt install qt5-default

5) need to install Spinnaker FLIR camera SDK and python API: 
    - https://flir.app.boxcn.net/v/SpinnakerSDK/file/1093743440079
    - may need: sudo apt-mark manual qtbase5-dev qtchooser qt5-qmake qtbase5-dev-tools for spinview 

7) need to add "self.cam.PixelFormat.SetValue(PySpin.PixelFormat_BGR8)" above self.cam.BeginAcquistion() line in $ .local/lib/python3.8/site-packages/EasyPySpin.videocapture.py

8) need to change in lib/python3.8/site-packages/adafruit_blinka/microcontroller/tegra/t234/pin.py from "GPIO.setmode(GPIO.TEGRA_SOC)" to GPIO.setmode(GPIO.BOARD)
    - otherwise the acoustic class and hall effect class will clash

9) need to install xboxdrv and jstest-gtk for joystick implimentation 
        $ sudo apt-get install -y xboxdrv         
        "https://github.com/FRC4564/Xbox"
        
10) VSCode: https://github.com/JetsonHacksNano/installVSCode.git

11) optional: install arduino using jetsonhacks github and upload main.ino from src/arduino

pyuic5 uis/GUI.ui -o gui_widgets.py
/opt/homebrew/bin/python3.9 -m PyQt5.uic.pyuic uis/GUI.ui -o gui_widgets.py

/opt/homebrew/bin/python3.10 -m PyInstaller --onedir --windowed --icon MagScopeBox.icns --name MagScope main.py


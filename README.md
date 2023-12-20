# Magnetoacoustic Microrobotic Manipulation System
The Magscope is a magnetic and acoustic field generator with real time micro-scale optical
feedback. The system can also be thought of as a micro-robotic, micro scale manipulation platform
with close loop control capabilities. Its intention is to be a user friendly, portable, micro-robotic
experimentation platform. It contains an embedded computer, a microscope, power supplies, power
amplifiers, and control circuitry necessary for micro robotic research and experimentation. Custom
control software is written in Python and C++ for handling live image feed from a camera, tracking
and detection, and outputting the control signals to electromagnetic coils and acoustic transducers.
Furthermore, the system is the first of its kinda to combine both magnetic and acoustic actuation
methods into a single, stand alone, portable device. Many of the mechanical components can be 3D
printed allowing others to build the device at a low cost. The design can be divided into three
sections (Mechanical, Electrical, and Software) 

# CAD System Overview

![MagScope1](https://github.com/MaxSokolich/Magnetoacoustic-Microrobotic-Manipulation-System/assets/50302377/6825d8a5-047d-43c4-af69-0d4fb00fc2c7)

![Magscope2](https://github.com/MaxSokolich/Magnetoacoustic-Microrobotic-Manipulation-System/assets/50302377/c7e8fff9-ff27-48e0-8b8f-d49c7494f22c)


# Tracking and Control Software UI

<img width="1724" alt="Screenshot 2023-12-20 at 2 28 14 PM" src="https://github.com/MaxSokolich/Magnetoacoustic-Microrobotic-Manipulation-System/assets/50302377/4a14a2d1-431e-4f0e-a348-e22e0dc51acd">


# Example Control Algorithm:
<img width="699" alt="Screenshot 2023-12-20 at 2 14 26 PM" src="https://github.com/MaxSokolich/Magnetoacoustic-Microrobotic-Manipulation-System/assets/50302377/0eedb007-9db3-4152-8e76-12740618e227">

# System

![Magscope3-2](https://github.com/MaxSokolich/Magnetoacoustic-Microrobotic-Manipulation-System/assets/50302377/697d35fd-252c-45ac-ad69-98e3f2571512)


# Instructions for initial installation of system components on Nvidia Jetson AGX Orin:
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

6) need to install all python dependencies
    - python3.8 -m pip install -r JetsonOrinReqs.txt

7) need to add "self.cam.PixelFormat.SetValue(PySpin.PixelFormat_BGR8)" above self.cam.BeginAcquistion() line in $ .local/lib/python3.8/site-packages/EasyPySpin.videocapture.py

8) need to change in lib/python3.8/site-packages/adafruit_blinka/microcontroller/tegra/t234/pin.py from "GPIO.setmode(GPIO.TEGRA_SOC)" to GPIO.setmode(GPIO.BOARD)
    - otherwise the acoustic class and hall effect class will clash

9) need to install xboxdrv and jstest-gtk for joystick implimentation 
        $ sudo apt-get install -y xboxdrv         
        "https://github.com/FRC4564/Xbox"
        
10) VSCode: https://github.com/JetsonHacksNano/installVSCode.git

11) optional: install arduino using jetsonhacks github and upload main.ino from src/arduino

pyuic5 uis/CellPusher.ui -o gui_widgets.py

/opt/homebrew/bin/python3.10 -m PyInstaller --onedir --windowed --icon MagScopeBox.icns --name MagScope main.py


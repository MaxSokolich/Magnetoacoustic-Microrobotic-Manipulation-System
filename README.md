# MR_CellPusher
ttracker and control code for cell pusher algorithm

pyuic5 uis/CellPusher.ui -o gui_widgets.py

The idea was for the software to generate 2 masks, one for cells and one for microrobots. Then we could track them sperately and create an algorithm to automatically move a cell using a microrobot. 



/opt/homebrew/bin/python3.10 -m PyInstaller --onedir --windowed --icon MagScopeBox.icns --name MagScope main.py


# Instructions for initial installation of system components:
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
import cv2
import PySpin
import EasyPySpin
import time
import numpy as np

'''
BFS-U3-50S5C-Color
2448x2048
35 fps

NOTE: need to add cam.PixelFormat.SetValue(PySpin.PixelFormat_BGR8) above self.cam.BeginAcquistsion() line in Easypyspin videocapture
'''

class count_fps:
    t0 = 0
    t1 = 0
    fps =0
    def __init__(self):
        self.t0 = time.time()
        self.t1 = self.t0
    def update(self):
        self.t1 = time.time()
        self.fps = 1/(self.t1-self.t0)
        self.t0 = self.t1
    def get_fps(self):
        return self.fps


def mouse_points(event,x,y,flags,params):
    global b
    if event == cv2.EVENT_LBUTTONDOWN:
        
        b = [x,y]


            
b = [0,0]
fps = count_fps()       
#cam = cv2.VideoCapture("/home/max/Documents/MagScopeSystem/src/videos/mickyroll1.mp4")
cam = EasyPySpin.VideoCapture(0)
width = int(cam.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))
cam.set(cv2.CAP_PROP_FPS,30)
fps_cam = int(cam.get(cv2.CAP_PROP_FPS))
print(cam.get(cv2.CAP_PROP_FPS))


cv2.namedWindow("img")  # name of CV2 window
cv2.setMouseCallback("img", mouse_points)  # set callback func

while True:
    fps.update()
    suc, frame = cam.read()
    if not suc:
        print("error")
    #click object on half resd image
    #find coords of obj in
    #do contouring based on full res img


    resize_scale =100
    resize_ratio = (
                width * resize_scale // 100,
                height * resize_scale // 100,
            )
    
    x,y = b[0], b[1]
    #rame = cv2.resize(frame, resize_ratio, interpolation=cv2.INTER_AREA)
    
    #print(width,height, fps_cam)
    cv2.circle(frame,(x,y),10, (255,255,0),-1)



   

    
    cv2.putText(
            frame,
            str(int(fps.get_fps())),
            (
                int((width * resize_scale / 100) / 40),
                int((height * resize_scale / 100) / 20),
            ),
            cv2.FONT_HERSHEY_COMPLEX,
            0.5,
            (0, 255, 0),
            1,
        )
    cv2.putText(
            frame,
            str(
                [
                    int((width * resize_scale / 100)),
                    int((height * resize_scale / 100)),
                ]
            ),
            (
                int((width * resize_scale / 100) / 40),
                int((height * resize_scale / 100) / 60),
            ),
            cv2.FONT_HERSHEY_COMPLEX,
            0.5,
            (0, 255, 0),
            1,
        )

    
    
    cv2.imshow("img",frame)

  

    fps.get_fps()
    if cv2.waitKey(1) & 0xFF == ord('q'):
        #Send(arduino,0,0,4)
        break
      

cv2.destroyAllWindows()
cam.release()


"""


DeprecationWarning: the imp module is deprecated in favour of importlib; see the module's documentation for alternative uses

"""
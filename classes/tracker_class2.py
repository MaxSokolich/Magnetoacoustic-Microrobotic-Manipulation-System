from PyQt5.QtCore import pyqtSignal, pyqtSlot, Qt, QThread,QTimer
import numpy as np
import cv2
import matplotlib.pyplot as plt

from scipy import ndimage 
import time

from classes.algorithm_class import algorithm
from classes.fps_class import FPSCounter
    
#add unique crop length 
class VideoThread(QThread):
    change_pixmap_signal = pyqtSignal(np.ndarray)
    cropped_frame_signal = pyqtSignal(np.ndarray,np.ndarray)
    actions_signal = pyqtSignal(list, bool, list)


    def __init__(self, parent):
        super().__init__(parent=parent)
        self.parent = parent
        self.cap = self.parent.cap 
        video = self.parent.videopath 
        #initiate control class
        self.control_robot = algorithm()
        
    
        self.fps = FPSCounter()
        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.videofps = int(self.cap.get(cv2.CAP_PROP_FPS))
        
        self._run_flag = True
        self._play_flag = True
        self.mask_flag = False
    
        self.croppedmask_flag = True
        self.framenum = 0

        self.orientstatus = False
        self.autoacousticstatus = False
        
        #robot mask attributes
        self.robot_mask_thresh = 128
        self.robot_mask_dilation = 0  
        self.robot_mask_blur = 0
        self.robot_crop_length = 40
        self.robot_mask_flag = True
        self.robot_list = []

        #cell mask attributes
        self.cell_mask_thresh = 128
        self.cell_mask_dilation = 0
        self.cell_mask_blur = 0
        self.cell_crop_length = 40
        self.cell_mask_flag = False
        self.cell_list = []

        
        self.maskinvert = True
        self.crop_length_record = 200
        
        self.exposure = 5000
        self.objective = 10


        self.arrivalthresh = 20
        self.RRTtreesize = 25
        self.memory = 15  #this isnt used as of now
     

        if video != 0:
            self.totalnumframes = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
        else:
            self.totalnumframes = 0
           
        self.pix2metric =  0.28985 * self.objective #.29853 * self.objective#0.28985 * self.objective  
        
            #at 10x objective
            #width_in_pixels = 2448 #pixels
            #divs = 82
            #singledivlength = 10 #um
            #width_in_um = divs * singledivlength # 82 * 10 = 820
            #scale = 2448/820

            #according to website pixel size is Pixel Size, H x V (μm): 3.45 x 3.45
            #1/(pixelSize/magnification)
            #1/(3.45/10) = 2.89

 
    

    def find_robot_mask(self,frame):
        """
        finds a mask of a given image based on a threshold value in black and white for ROBOTS
        """
        invert = self.maskinvert
        robot_mask_thresh= int(self.robot_mask_thresh)

        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if self.robot_mask_blur > 0:
            frame = cv2.blur(frame, (self.robot_mask_blur,self.robot_mask_blur))

        _, robot_mask = cv2.threshold(frame, robot_mask_thresh, 255, cv2.THRESH_BINARY)

        if invert:
            robot_mask = cv2.bitwise_not(robot_mask)

        robot_mask = cv2.dilate(robot_mask, None, iterations=self.robot_mask_dilation)

         

        return robot_mask
    

    
    def find_cell_mask(self,frame):
        """
        finds a mask of a given image based on a threshold value in black and white FOR CELL
        """
        invert = self.maskinvert
        cell_mask_thresh= int(self.cell_mask_thresh)

        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if self.cell_mask_blur > 0:
            frame = cv2.blur(frame, (self.cell_mask_blur,self.cell_mask_blur))

        _, cell_mask = cv2.threshold(frame, cell_mask_thresh, 255, cv2.THRESH_BINARY)

        if invert:
            cell_mask = cv2.bitwise_not(cell_mask)

        cell_mask = cv2.dilate(cell_mask, None, iterations=self.cell_mask_dilation)

        return cell_mask
       


    def track_robot(self, frame):
        """
        Returns:
            cropped_mask: to visualize tracking parameters
        """
        if len(self.robot_list) > 0:
            for bot in self.robot_list: #for each bot with a position botx, boty, find the cropped frame around the bot
                
                #current cropped frame dim
                x1, y1, w, h = bot.cropped_frame[-1]
                x1 = max(min(x1, self.width), 0)
                y1 = max(min(y1, self.height), 0)

                #crop the frame
                croppedframe = frame[y1 : y1 + h, x1 : x1 + w]

          
                #find the mask
                croppedmask  = self.find_robot_mask(croppedframe)
            
              
                #label the mask
                label_im, nb_labels = ndimage.label(croppedmask) 
                sizes = ndimage.sum(croppedmask, label_im, range(nb_labels + 1)) 
                num_bots=np.sum(sizes>50)
                
                if num_bots>0:
                    #find contours from the mask
                    contours, _ = cv2.findContours(croppedmask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                    max_cnt = contours[0]
                    for contour in contours:
                        if cv2.contourArea(contour) > cv2.contourArea(max_cnt): 
                            max_cnt = contour
                    area = cv2.contourArea(max_cnt)* (1/self.pix2metric**2)
                    
                    #find the center of mass from the mask
                    szsorted=np.argsort(sizes)
                    [ycord,xcord]=ndimage.center_of_mass(croppedmask,labels=label_im,index = szsorted[-(1)])
                    ndimage.binary_dilation
                    
                    #derive the global current location
                    current_pos = [xcord + x1,   ycord + y1] #xcord ycord are relative to the cropped frame. need to convert to the overall frame dim

                    #generate new cropped frame based on the new robots position
                    x1_new = int(current_pos[0] - bot.crop_length/2)
                    y1_new = int(current_pos[1] - bot.crop_length/2)
                    w_new = int(bot.crop_length)
                    h_new = int(bot.crop_length)
                    new_crop = [int(x1_new), int(y1_new), int(w_new), int(h_new)]


                    #find velocity:
                    if len(bot.position_list) > self.memory:
                        vx = (current_pos[0] - bot.position_list[-self.memory][0]) * (self.fps.get_fps()/self.memory) / self.pix2metric
                        vy = (current_pos[1] - bot.position_list[-self.memory][1]) * (self.fps.get_fps()/self.memory) / self.pix2metric
                        magnitude = np.sqrt(vx**2 + vy**2)

                        velocity = [vx,vy,magnitude]

                    else:
                        velocity = [0,0,0]

            
                    #find blur of original crop
                    blur = cv2.Laplacian(croppedframe, cv2.CV_64F).var()
                    
                    #store the data in the instance of RobotClasss
                    bot.add_frame(self.framenum)
                    bot.add_time(1/self.fps.get_fps()) #original in ms
                    bot.add_position([current_pos[0], current_pos[1]])
                    bot.add_velocity(velocity)
                    bot.add_crop(new_crop)
                    bot.add_area(area)
                    bot.add_blur(blur)
                    bot.set_avg_area(np.mean(bot.area_list))


                    #stuck condition
                    if len(bot.position_list) > self.memory and velocity[2] < 20 and self.parent.freq > 0:
                        stuck_status = 1
                    else:
                        stuck_status = 0
                    bot.add_stuck_status(stuck_status)

                    #this will toggle between the cropped frame display being the masked version and the original
                    if self.croppedmask_flag == False:
                        croppedmask = frame[y1 : y1 + h, x1 : x1 + w]

        
            
        
            #also crop a second frame at a fixed wdith and heihgt for recording the most recent robots suroundings
            x1_record = int(bot.position_list[-1][0] - self.crop_length_record/2)
            y1_record = int(bot.position_list[-1][1] - self.crop_length_record/2)
            recorded_cropped_frame = frame[y1_record : y1_record + self.crop_length_record, x1_record : x1_record + self.crop_length_record]
            
            #adjust most recent bot crop_length 
            bot.crop_length = self.robot_crop_length

        else:
            recorded_cropped_frame = np.zeros((self.crop_length_record, self.crop_length_record, 3), dtype=np.uint8) 
            croppedmask = np.zeros((310, 310, 3), dtype=np.uint8)
        
        return croppedmask, recorded_cropped_frame
    




    def track_cell(self, frame):
        """
        Returns:
            cropped_mask: to visualize tracking parameters
        """
        if len(self.cell_list) > 0:
            for cell in self.cell_list: #for each bot with a position botx, boty, find the cropped frame around the bot
                
                #current cropped frame dim
                x1, y1, w, h = cell.cropped_frame[-1]
                x1 = max(min(x1, self.width), 0)
                y1 = max(min(y1, self.height), 0)

                #crop the frame
                croppedframe = frame[y1 : y1 + h, x1 : x1 + w]

          
                #find the mask
                croppedmask  = self.find_cell_mask(croppedframe)
            
              
                #label the mask
                label_im, nb_labels = ndimage.label(croppedmask) 
                sizes = ndimage.sum(croppedmask, label_im, range(nb_labels + 1)) 
                num_cells=np.sum(sizes>50)
                
                if num_cells>0:
                    #find contours from the mask
                    contours, _ = cv2.findContours(croppedmask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                    max_cnt = contours[0]
                    for contour in contours:
                        if cv2.contourArea(contour) > cv2.contourArea(max_cnt): 
                            max_cnt = contour
                    area = cv2.contourArea(max_cnt)* (1/self.pix2metric**2)
                    
                    #find the center of mass from the mask
                    szsorted=np.argsort(sizes)
                    [ycord,xcord]=ndimage.center_of_mass(croppedmask,labels=label_im,index = szsorted[-(1)])
                    ndimage.binary_dilation
                    
                    #derive the global current location
                    current_pos = [xcord + x1,   ycord + y1] #xcord ycord are relative to the cropped frame. need to convert to the overall frame dim

                    #generate new cropped frame based on the new robots position
                    x1_new = int(current_pos[0] - cell.crop_length/2)
                    y1_new = int(current_pos[1] - cell.crop_length/2)
                    w_new = int(cell.crop_length)
                    h_new = int(cell.crop_length)
                    new_crop = [int(x1_new), int(y1_new), int(w_new), int(h_new)]


                    #find velocity:
                    if len(cell.position_list) > self.memory:
                        vx = (current_pos[0] - cell.position_list[-self.memory][0]) * (self.fps.get_fps()/self.memory) / self.pix2metric
                        vy = (current_pos[1] - cell.position_list[-self.memory][1]) * (self.fps.get_fps()/self.memory) / self.pix2metric
                        magnitude = np.sqrt(vx**2 + vy**2)

                        velocity = [vx,vy,magnitude]

                    else:
                        velocity = [0,0,0]

            
                    #find blur of original crop
                    blur = cv2.Laplacian(croppedframe, cv2.CV_64F).var()
                    
                    #store the data in the instance of RobotClasss
                    cell.add_frame(self.framenum)
                    cell.add_time(1/self.fps.get_fps()) #original in ms
                    cell.add_position([current_pos[0], current_pos[1]])
                    cell.add_velocity(velocity)
                    cell.add_crop(new_crop)
                    cell.add_area(area)
                    cell.add_blur(blur)
                    cell.set_avg_area(np.mean(cell.area_list))

                    
                
                    #stuck condition
                    if len(cell.position_list) > self.memory and velocity[2] < 20 and self.parent.freq > 0:
                        stuck_status = 1
                    else:
                        stuck_status = 0
                    cell.add_stuck_status(stuck_status)
                    
                    #this will toggle between the cropped frame display being the masked version and the original
                    if self.croppedmask_flag == False:
                        croppedmask = frame[y1 : y1 + h, x1 : x1 + w]
                    
                
            #adjust most recent bot crop_length 
            cell.crop_length = self.cell_crop_length

        else:
            croppedmask = np.zeros((310, 310, 3), dtype=np.uint8)
        

        return croppedmask
    
    





    def display_hud(self, frame):
        
        display_frame = frame.copy()
        if len(self.robot_list) > 0:
            for botnum in range(len(self.robot_list)):
                bot  = self.robot_list[botnum]
                x1, y1, w, h = bot.cropped_frame[-1]

                cv2.rectangle(display_frame, (x1, y1), (x1 + w, y1 + h), (255,0,0), 4)
                cv2.putText(display_frame,str(botnum+1),(x1 + w,y1 + h),cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, thickness=4,color = (255, 255, 255))
                
                pts = np.array(bot.position_list, np.int32)
                cv2.polylines(display_frame, [pts], False, (255,0,0), 4)

                targets = bot.trajectory
                if len(targets) > 0:
                    pts = np.array(bot.trajectory, np.int32)
                    cv2.polylines(display_frame, [pts], False, (0, 0, 255), 4)
                    tar = targets[-1]
                    cv2.circle(display_frame,(int(tar[0]), int(tar[1])),6,(0,0,255), -1,)
        
        if len(self.cell_list) > 0:
            for cellnum in range(len(self.cell_list)):
                cell  = self.cell_list[cellnum]
                x1, y1, w, h = cell.cropped_frame[-1]

                cv2.rectangle(display_frame, (x1, y1), (x1 + w, y1 + h), (0,255,0), 4)
                cv2.putText(display_frame,str(cellnum+1),(x1 + w, y1 + h),cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, thickness=4,color = (255, 255, 255))
                
                pts = np.array(cell.position_list, np.int32)
                cv2.polylines(display_frame, [pts], False, (0,255,0), 4)

        
        cv2.putText(display_frame,"fps:"+str(int(self.fps.get_fps())),
                    (int(self.width  / 80),int(self.height / 14)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    fontScale=1, 
                    thickness=4,
                    color = (255, 255, 255))
        
        cv2.putText(display_frame,"100 um",
            (int(self.width / 80),int(self.height / 30)),
            cv2.FONT_HERSHEY_SIMPLEX,
            fontScale=1, 
            thickness=4,
            color = (255, 255, 255),
          
        )
        cv2.line(
            display_frame, 
            (int(self.width / 8),int(self.height /40)),
            (int(self.width / 8) + int(100 * (self.pix2metric)),int(self.height / 40)), 
            (255, 255, 255), 
            thickness=4
        )

        return display_frame






    def display_hud2(self, frame, croppedmask, cellmask):
        """there are techincally 4 frames for analyse at a time:
        1) displayframe:  this is the raw BGR frame from the videofeed
        2) croppedframe: this is the cropped BGR frame surrounding the nearby robot 
        3) displaymask: this is the raw MONO mask from the videofeed
        4) croppedmask: this is the cropped MONO mask surrounding the nearby robot 
        5) robot mask
        6) cell mask"""

        displayframe = frame.copy()

        #replace the botslocaton with black squre so dilation is not performed in it
        #ideally we subtract out the robot mask somehow before finding the cell mask
        if len(self.robot_list) > 0 and croppedmask is not None:
            x,y,w,h = self.robot_list[-1].cropped_frame[-1]
            cv2.rectangle(cellmask, (x, y), (x + w, y + h), (0, 0, 0), -1)
   

                

            
            #if len(self.robot_list) > 0 and croppedmask is not None:
            #    #replace the botslocaton with black squre so dilation is not performed in it
            #    try:
            #        if len(self.robot_list[-1].cropped_frame) > 1:
            #            x,y,w,h = self.robot_list[-1].cropped_frame[-2]
            #            displaymask[y:y+w , x:x+h] =  croppedmask #= croppedmask
            #    except Exception:
            #        pass
            #displayframe = cv2.cvtColor(displaymask, cv2.COLOR_GRAY2BGR)


        if len(self.robot_list) > 0 and croppedmask is not None:

            color = plt.cm.rainbow(np.linspace(1, 0, len(self.robot_list))) * 255
            for (botnum, botcolor) in zip(range(len(self.robot_list)), color):
                    bot  = self.robot_list[botnum]
                    posx = bot.position_list[-1][0]
                    posy = bot.position_list[-1][1]
                    x1, y1, w, h = bot.cropped_frame[-1]


                    #cv2.circle(displayframe,(int(posx), int(posy)),6,(botcolor),-1,)
                    cv2.rectangle(displayframe, (x1, y1), (x1 + w, y1 + h), (botcolor), 4)
                    cv2.putText(displayframe,str(botnum+1),(x1 + w,y1 + h),cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, thickness=4,color = (255, 255, 255))
                    
                    pts = np.array(bot.position_list, np.int32)
                    cv2.polylines(displayframe, [pts], False, botcolor, 4)

                    targets = bot.trajectory
                    if len(targets) > 0:
                        pts = np.array(bot.trajectory, np.int32)
                        cv2.polylines(displayframe, [pts], False, (1, 1, 255), 4)
                        tar = targets[-1]
                        cv2.circle(displayframe,(int(tar[0]), int(tar[1])),6,(botcolor), -1,)
            
    
            croppedmask = cv2.cvtColor(croppedmask, cv2.COLOR_GRAY2BGR)
            
            #this will toggle between the cropped frame display being the masked version and the original
            if self.croppedmask_flag == False:
                croppedmask = frame[y1 : y1 + h, x1 : x1 + w]


        else:
            croppedmask = np.zeros((310, 310, 3), dtype=np.uint8) 
        

        cv2.putText(displayframe,"fps:"+str(int(self.fps.get_fps())),
                    (int(self.width  / 80),int(self.height / 14)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    fontScale=1, 
                    thickness=4,
                    color = (255, 255, 255))
        
        cv2.putText(displayframe,"100 um",
            (int(self.width / 80),int(self.height / 30)),
            cv2.FONT_HERSHEY_SIMPLEX,
            fontScale=1, 
            thickness=4,
            color = (255, 255, 255),
          
        )
        cv2.line(
            displayframe, 
            (int(self.width / 8),int(self.height /40)),
            (int(self.width / 8) + int(100 * (self.pix2metric)),int(self.height / 40)), 
            (255, 255, 255), 
            thickness=4
        )
        

        return displayframe, croppedmask 
    






    def run(self):
    
        # capture from web camx
        while self._run_flag:
            self.fps.update()

            #set and read frame
            if self._play_flag == True:
                self.framenum +=1
            
            
            if self.totalnumframes !=0:
                if self.framenum >= self.totalnumframes:
                    self.framenum = 0
                
                self.cap.set(1, self.framenum)
            
            
            ret, frame = self.cap.read()
        
            #control_mask = None
            if ret:       
                if self.totalnumframes ==0:         
                    self.cap.set(cv2.CAP_PROP_EXPOSURE, self.exposure)
                    self.pix2metric =  0.28985 * self.objective
                    
                #step 1 track robot
                robot_mask = self.find_robot_mask(frame)
                robotcroppedmask, recorded_cropped_frame = self.track_robot(frame) 
                

                #step 2: track cell
                cell_mask = self.find_cell_mask(frame)
                cellcroppedmask = self.track_cell(frame)

                #step 2.5: subtract robots from cell mask
                for bot in self.robot_list:    
                    x,y,w,h = bot.cropped_frame[-1]
                    cv2.rectangle(cell_mask, (x, y), (x + w, y + h), (0, 0, 0), -1)


                #step 3 display
                if self.parent.ui.robotmask_radio.isChecked():
                    croppedmask = robotcroppedmask
                    if self.mask_flag == True:
                        frame = cv2.cvtColor(robot_mask, cv2.COLOR_GRAY2BGR)

                elif self.parent.ui.cellmask_radio.isChecked():
                    croppedmask = cellcroppedmask
                    if self.mask_flag == True:
                        frame = cv2.cvtColor(cell_mask, cv2.COLOR_GRAY2BGR)
                
                displayframe = self.display_hud(frame)

                #step 2 control robot
                if len(self.robot_list)>0:
                    displayframe, actions, stopped = self.control_robot.run(displayframe, cell_mask, self.robot_list, self.RRTtreesize, self.arrivalthresh, self.orientstatus, self.autoacousticstatus)
                else:
                    actions = [0,0,0,0,0,0,0,0]
                    stopped = True    
                    
                #gather most recent robot params
                
                #step 3: emit croppedframe, frame from this thread to the main thread
                self.cropped_frame_signal.emit(croppedmask, recorded_cropped_frame)
                self.change_pixmap_signal.emit(displayframe)
                self.actions_signal.emit(actions, stopped, self.robot_list)
                
                

                #step 4: delay based on fps
                if self.totalnumframes !=0:
                    interval = 1/self.videofps  #use original fps used to record the video if not live
                    time.sleep(interval)

    
            
           


    def stop(self):
        """Sets run flag to False and waits for thread to finish"""
        #blank = np.zeros((self.width, self.height, 3), dtype=np.uint8) 
        #self.change_pixmap_signal.emit(blank)

        self._run_flag = False
        self.wait()
        self.cap.release()


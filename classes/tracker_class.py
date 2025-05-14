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
    actions_signal = pyqtSignal(list, bool, list, list)


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
        self.framenum = 1
        self.time_stamp = 0
        self.start_time = time.time()

        self.orientstatus = False
        self.pushstatus = False
        self.autoacousticstatus = False
        
        #robot mask attributes
        self.robot_mask_lower = 0
        self.robot_mask_upper = 128
        self.robot_mask_dilation = 0  
        self.robot_mask_blur = 0
        self.robot_crop_length = 40
        self.robot_mask_flag = True
        self.robot_list = []

        #cell mask attributes
        self.cell_mask_lower = 0
        self.cell_mask_upper = 128
        self.cell_mask_dilation = 0
        self.cell_mask_blur = 0
        self.cell_crop_length = 40
        self.cell_mask_flag = False
        self.cell_list = []

        
        self.maskinvert = False
        self.crop_length_record = 200
        
        self.exposure = 5000
        self.objective = 10


        self.arrivalthresh = 100
        self.RRTtreesize = 25
        self.memory = 15  #this isnt used as of now
     

        if video != 0:
            self.totalnumframes = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
        else:
            self.totalnumframes = 0
           
        self.pixel2um =  3.45 / self.objective # each pixel is 3.35 um in size at 1x. [um] / [px]
        
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
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if self.robot_mask_blur > 0:
            frame = cv2.blur(frame, (self.robot_mask_blur,self.robot_mask_blur))

        #threshold the mask
        #_, robot_mask = cv2.threshold(frame, robot_mask_thresh, 255, cv2.THRESH_BINARY)
        #robot_mask = cv2.adaptiveThreshold(frame, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, self.robot_mask_blocksize, self.robot_mask_thresh)
        robot_mask = cv2.inRange(frame, self.robot_mask_lower, self.robot_mask_upper)
        
        if self.maskinvert:
            robot_mask = cv2.bitwise_not(robot_mask)

        #subtract cell from robot mask
        try:
            for cell in self.cell_list:    
                x,y,w,h = cell.cropped_frame[-1]
                blank = np.zeros((w, h), dtype=np.uint8) 
                robot_mask[y:y+w , x:x+h] = blank 
        except Exception:
            pass
     

        robot_mask = cv2.dilate(robot_mask, None, iterations=self.robot_mask_dilation)

        return robot_mask
    
    def find_cell_mask(self,frame):
        """
        finds a mask of a given image based on a threshold value in black and white FOR CELL
        """
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if self.cell_mask_blur > 0:
            frame = cv2.blur(frame, (self.cell_mask_blur,self.cell_mask_blur))
            
        #threshold the mask
        #_, cell_mask = cv2.threshold(frame, cell_mask_thresh, 255, cv2.THRESH_BINARY)
        #cell_mask = cv2.adaptiveThreshold(frame, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, self.cell_mask_blocksize, self.cell_mask_thresh)
        cell_mask = cv2.inRange(frame, self.cell_mask_lower, self.cell_mask_upper)
        
        if self.maskinvert:
            cell_mask = cv2.bitwise_not(cell_mask)

        #sibtract robot from cell mask 
        try:
            for bot in self.robot_list:    
                x,y,w,h = bot.cropped_frame[-1]
                blank = np.zeros((w, h), dtype=np.uint8) 
                cell_mask[y:y+w , x:x+h] = blank 
        except Exception:
            pass
        
        cell_mask = cv2.dilate(cell_mask, None, iterations=self.cell_mask_dilation)

        return cell_mask

       


    def track_robot(self, frame, robotmask):
        """
        Returns:
            cropped_mask: to visualize tracking parameters
        """
        if len(self.robot_list) > 0:
            for i in range(len(self.robot_list)): #for each bot with a position botx, boty, find the cropped frame around the bot
                try:
                    bot = self.robot_list[i]
                    #current cropped frame dim
                    x1, y1, w, h = bot.cropped_frame[-1]
                    x1 = max(min(x1, self.width), 0)
                    y1 = max(min(y1, self.height), 0)

                    #crop the frame and mask
                    croppedframe = frame[y1 : y1 + h, x1 : x1 + w]
                    croppedmask  = robotmask[y1 : y1 + h, x1 : x1 + w]
                
                
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
                        area = cv2.contourArea(max_cnt)#* (self.pixel2um**2)
                        
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
                            vx = (current_pos[0] - bot.position_list[-self.memory][0]) * (self.fps.get_fps()/self.memory) #* self.pixel2um
                            vy = (current_pos[1] - bot.position_list[-self.memory][1]) * (self.fps.get_fps()/self.memory) #* self.pixel2um

               
                            magnitude = np.sqrt(vx**2 + vy**2)

                            velocity = [vx,vy,magnitude]

                        else:
                            velocity = [0,0,0]

                
                        #find blur of original crop
                        blur = cv2.Laplacian(croppedframe, cv2.CV_64F).var()
                        
                        #store the data in the instance of RobotClasss
                        bot.add_frame(self.framenum)
                        bot.add_time(self.time_stamp) #original in ms
                        bot.add_position([current_pos[0], current_pos[1]])
                        bot.add_velocity(velocity)
                        bot.add_crop(new_crop)
                        bot.add_area(area)
                        bot.add_blur(blur)
                        bot.set_avg_area(np.mean(bot.area_list))
                        bot.add_um2pixel(self.pixel2um)

                        #this will toggle between the cropped frame display being the masked version and the original
                        if self.croppedmask_flag == False:
                            croppedmask = croppedframe

                    else:
                        if len(self.robot_list) > 0:
                            del self.robot_list[i]
                
                except Exception:
                    pass
                       
        
            #also crop a second frame at a fixed wdith and heihgt for recording the most recent robots suroundings
            if len(self.robot_list)>0:
                x1_record = int(bot.position_list[-1][0] - self.crop_length_record/2)
                y1_record = int(bot.position_list[-1][1] - self.crop_length_record/2)
                recorded_cropped_frame = frame[y1_record : y1_record + self.crop_length_record, x1_record : x1_record + self.crop_length_record]
                
                #adjust most recent bot crop_length 
                bot.crop_length = self.robot_crop_length
            else:
                recorded_cropped_frame = np.zeros((self.crop_length_record, self.crop_length_record, 3), dtype=np.uint8) 

        else:
            recorded_cropped_frame = np.zeros((self.crop_length_record, self.crop_length_record, 3), dtype=np.uint8) 
            croppedmask = np.zeros((310, 310, 3), dtype=np.uint8)
        
        return croppedmask, recorded_cropped_frame
    




    def track_cell(self, frame, cellmask):
        """
        Returns:
            cropped_mask: to visualize tracking parameters
        """
        if len(self.cell_list) > 0:
            for j in range(len(self.cell_list)): #for each bot with a position botx, boty, find the cropped frame around the bot
                try:    
                    cell = self.cell_list[j]
                    #current cropped frame dim
                    x1, y1, w, h = cell.cropped_frame[-1]
                    x1 = max(min(x1, self.width), 0)
                    y1 = max(min(y1, self.height), 0)

                    #crop the frame and mask
                    croppedframe = frame[y1 : y1 + h, x1 : x1 + w]
                    croppedmask = cellmask[y1 : y1 + h, x1 : x1 + w]

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
                        area = cv2.contourArea(max_cnt)#* (self.pixel2um**2)
                    
                        
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
                            vx = (current_pos[0] - cell.position_list[-self.memory][0]) * (self.fps.get_fps()/self.memory) #* self.pixel2um
                            vy = (current_pos[1] - cell.position_list[-self.memory][1]) * (self.fps.get_fps()/self.memory) #* self.pixel2um
                            magnitude = np.sqrt(vx**2 + vy**2)

                            velocity = [vx,vy,magnitude]

                        else:
                            velocity = [0,0,0]

                
                        #find blur of original crop
                        blur = cv2.Laplacian(croppedframe, cv2.CV_64F).var()
                        
                        #store the data in the instance of RobotClasss
                        cell.add_frame(self.framenum)
                        cell.add_time(self.time_stamp) #original in ms
                        cell.add_position([current_pos[0], current_pos[1]])
                        cell.add_velocity(velocity)
                        cell.add_crop(new_crop)
                        cell.add_area(area)
                        cell.add_blur(blur)
                        cell.set_avg_area(np.mean(cell.area_list))
                        cell.add_um2pixel(self.pixel2um)
                        
                        #this will toggle between the cropped frame display being the masked version and the original
                        if self.croppedmask_flag == False:
                            croppedmask = croppedframe
                    else:
                        if len(self.cell_list) > 0:
                            del self.cell_list[j]
                except Exception:
                    pass
                  
                
            #adjust most recent bot crop_length 
            cell.crop_length = self.cell_crop_length

        else:
            croppedmask = np.zeros((310, 310, 3), dtype=np.uint8)

        return croppedmask
    
    





    def display_hud(self, frame):
        display_frame = frame.copy()
        if self.parent.ui.toggledisplayvisualscheckbox.isChecked():
            
            if len(self.robot_list) > 0:
                color = plt.cm.rainbow(np.linspace(1, 0.2, len(self.robot_list))) * 255
        
                for (botnum,botcolor) in zip(range(len(self.robot_list)), color):
                    try:
                        bot  = self.robot_list[botnum]
                        x1, y1, w, h = bot.cropped_frame[-1]

                        cv2.rectangle(display_frame, (x1, y1), (x1 + w, y1 + h), botcolor, 4)
                        cv2.putText(display_frame,str(botnum+1),(x1 + w,y1 + h),cv2.FONT_HERSHEY_SIMPLEX, fontScale=1.5, thickness=4,color = (255, 255, 255))
                        
                        pts = np.array(bot.position_list, np.int32)
                        cv2.polylines(display_frame, [pts], False, botcolor, 5)

                        targets = bot.trajectory
                        if len(targets) > 0:
                            pts = np.array(bot.trajectory, np.int32)
                            cv2.polylines(display_frame, [pts], False, (0, 0, 255), 5)
                            tar = targets[-1]
                            cv2.circle(display_frame,(int(tar[0]), int(tar[1])),10,(0,0,0), -1,)
                    except Exception:
                        pass
            
            if len(self.cell_list) > 0:
                color = plt.cm.rainbow(np.linspace(0.5, 0, len(self.cell_list))) *0
                for (cellnum,cellcolor) in zip(range(len(self.cell_list)),color):
                    cell  = self.cell_list[cellnum]
                    x1, y1, w, h = cell.cropped_frame[-1]

                    cv2.rectangle(display_frame, (x1, y1), (x1 + w, y1 + h), (0,255,0), 5)
                    cv2.putText(display_frame,str(cellnum+1),(x1 + w, y1 + h),cv2.FONT_HERSHEY_SIMPLEX, fontScale=1.5, thickness=5,color = (255, 255, 255))
                    
                    pts = np.array(cell.position_list, np.int32)
                    cv2.polylines(display_frame, [pts], False, cellcolor, 5)

            
            cv2.putText(display_frame,"fps:"+str(int(self.fps.get_fps())),
                        (int(self.width  / 80),int(self.height / 14)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        fontScale=1.5, 
                        thickness=5,
                        color = (0, 0, 0))
            
            
            
            cv2.putText(display_frame,"100 um",
                (int(self.width / 80),int(self.height / 30)),
                cv2.FONT_HERSHEY_SIMPLEX,
                fontScale=1.5, 
                thickness=5,
                color = (0, 0, 0),
            
            )
            cv2.line(
                display_frame, 
                (int(self.width / 8),int(self.height /40)),
                (int(self.width / 8) + int(100 / (self.pixel2um)),int(self.height / 40)), 
                (0, 0, 0), 
                thickness=20
            )
            
    

        return display_frame




    def run(self):
    
        # capture from web camx
        while self._run_flag:
            self.fps.update()

            #set and read frame
            if self._play_flag == True:
                self.framenum +=1
                self.time_stamp = time.time() - self.start_time
            
            
            if self.totalnumframes !=0:
                if self.framenum >= self.totalnumframes:
                    self.framenum = 0
                
                self.cap.set(1, self.framenum)
            
            
            ret, frame = self.cap.read()
        
            #control_mask = None
            if ret:       
                if self.totalnumframes ==0:         
                    self.cap.set(cv2.CAP_PROP_EXPOSURE, self.exposure)
                    self.pixel2um =   3.45 / self.objective
                    

                #step 1 track robot
                robot_mask = self.find_robot_mask(frame)
                robotcroppedmask, recorded_cropped_frame = self.track_robot(frame, robot_mask) 
                

                #step 2.5: subtract robots from cell mask. first create an inital mask using cell mask params. then remove the robot from this inital mask. then find the mask again on this first mask and use this for tracking
                #create inital cell mask
           
                #on cell mask initial, replace all
                
                #step 2: track cell            
                cell_mask = self.find_cell_mask(frame)
                cellcroppedmask = self.track_cell(frame, cell_mask)

           

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
                self.actions_signal.emit(actions, stopped, self.robot_list, self.cell_list)
                
                

                

    
            
           


    def stop(self):
        """Sets run flag to False and waits for thread to finish"""
        #blank = np.zeros((self.width, self.height, 3), dtype=np.uint8) 
        #self.change_pixmap_signal.emit(blank)

        self._run_flag = False
        self.wait()
        self.cap.release()



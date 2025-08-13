from PyQt5.QtCore import pyqtSignal, pyqtSlot, Qt, QThread,QTimer
import numpy as np
import cv2



class Controller:
   
    def __init__(self):
        self.reset()
     
        

    def reset(self):
        self.node = 0
        self.count = 0
        self.stopped = False
        self.actions = [0,0,0,0,0,0,0,0]


        #orient stuff
        self.B_vec = np.array([0,1])
        self.T_R = 1
        self.theta_maps = np.array([]) #added this so that we store the mapped angles
        self.theta = 0

        #acoustic stuff
        self.min_freq = 800000   #current minimum freq to start at
        self.max_freq = 1600000  #current maximum freq to stop at
        self.current_freq = self.min_freq   #current freq we are applying (start at 0)
        self.increment = 10000   #increment to step frequency by
        self.optimal_freq = None
        self.resistance = 0
        self.vmag_min = 3   #um/s
        self.vmag_max = 20   #um/s
        self.iter = 0



        self.Bx, self.By, self.Bz, self.alpha, self.gamma, self.freq, self.psi, self.acoustic_frequency = 0,0,0,0,0,0,0,0



        
    def run_orient(self, frame, robot_list, arrivialthresh):
        
        #this whole algorithm is in pixels!!!
        if len(robot_list[-1].trajectory) > 0:
            
            #define target coordinate
            targetx = robot_list[-1].trajectory[self.node][0]
            targety = robot_list[-1].trajectory[self.node][1]

            #define robots current position
            robotx = robot_list[-1].position_list[-1][0]
            roboty = robot_list[-1].position_list[-1][1]
            
            #calculate error between node and robot
            direction_vec = [targetx - robotx, targety - roboty]
            error = np.sqrt(direction_vec[0] ** 2 + direction_vec[1] ** 2)
            self.alpha = np.arctan2(-direction_vec[1], direction_vec[0])

            self.Bx, self.By, self.Bz = self.orient(robot_list[-1], direction_vec)
            self.freq = 0

            #step condition
            if self.node == len(robot_list[-1].trajectory):
                self.node = len(robot_list[-1].trajectory)
            else:
                if error < arrivialthresh:
                    self.node += 1


            #draw error arrow
            cv2.arrowedLine(
                frame,
                (int(robotx), int(roboty)),
                (int(targetx), int(targety)),
                [0, 0, 0],
                3,
            )

        self.actions = [self.Bx,self.By,self.Bz,self.alpha, self.gamma, self.freq, self.psi, self.acoustic_frequency]
        
        return frame, self.actions




    def run_roll(self, frame, robot_list, arrivialthresh):
        
        #this whole algorithm is in pixels!!!
        if len(robot_list[-1].trajectory) > 0:
            
            #define target coordinate
            targetx = robot_list[-1].trajectory[self.node][0]
            targety = robot_list[-1].trajectory[self.node][1]

            #define robots current position
            robotx = robot_list[-1].position_list[-1][0]
            roboty = robot_list[-1].position_list[-1][1]
            
            #calculate error between node and robot
            direction_vec = [targetx - robotx, targety - roboty]
            error = np.sqrt(direction_vec[0] ** 2 + direction_vec[1] ** 2)
            self.alpha = np.arctan2(-direction_vec[1], direction_vec[0])

            #step condition
            if self.node != len(robot_list[-1].trajectory):
                if error < arrivialthresh:
                    self.node += 1


            #draw error arrow
            cv2.arrowedLine(
                frame,
                (int(robotx), int(roboty)),
                (int(targetx), int(targety)),
                [0, 0, 0],
                3,
            )

        self.actions = [self.Bx,self.By,self.Bz,self.alpha, self.gamma, self.freq, self.psi, self.acoustic_frequency]
        
        return frame, self.actions


















    def run_push(self, frame, robot_list):
        pass



        


    def find_optimal_acoustic_freq(self, robot_list, pixel2um):
        #take a rolling average of the velocity from past 10 frames and average
        if len(robot_list[-1].velocity_list) > 0:
            vmag_avg = robot_list[-1].velocity_list[-1][2] * pixel2um

            ## CASE #1
            if vmag_avg < self.vmag_min: 
                
                if self.optimal_freq is not None:
                    self.optimal_freq = None

                if self.current_freq < self.max_freq: #if we increment and get to max
                    if self.count % 10 == 0:  #every 10 frames adjust frequency by the increment value may need to change this
                        self.current_freq += self.increment  #increment. once a frequency is found that puts vmag > vmin, the current_freq will be the optmial
                        #self.AcousticMod.start(self.current_freq, self.resistance) #apply the crrent freq at 0 resistance

                
                ## SUBCASE #2
                elif self.current_freq >= self.max_freq: 
                    if self.increment <= 50:
                        pass
                    else:
                        self.increment = int(self.increment/2)    #reduce the incriment to increase resolution of frequency sweep
                    self.current_freq = self.min_freq  #reset current freq to new minimum

            
            ## CASE #2
            elif vmag_avg > self.vmag_min and vmag_avg < self.vmag_max:
                self.optimal_freq = self.current_freq  #set the optimal frequency to be used later if vmag dips < vmin
                #self.AcousticMod.start(self.optimal_freq, self.resistance)
        
            ## CASE #3
            elif vmag_avg > self.vmag_max:
                if self.count % 20 == 0 and self.current_freq > self.min_freq:
                    self.current_freq -= 75000
                    #self.increment =  int(self.increment/3)
                    
        
        return self.current_freq


    

    def orient(self, bot, direction_vec):
        if len(bot.velocity_list) >= 0:
            
            #find the velocity avearge over the last memory number of frames to mitigate noise: 
            vx = bot.velocity_list[-1][0] 
            vy = bot.velocity_list[-1][1] 
            
            vel_bot = np.array([vx, vy])  # current velocity of self propelled robot
            vd = np.linalg.norm(vel_bot)
            bd = np.linalg.norm(self.B_vec)
            self.iter = self.iter +1
            
            #if vd != 0 and bd != 0:
            if vd > 1 and bd != 0: #changed it so that the velocity needs to be greater than 1um/s for it to start recording theta maps
                costheta = np.dot(vel_bot, self.B_vec) / (vd * bd)
                sintheta = (vel_bot[0] * self.B_vec[1] - vel_bot[1] * self.B_vec[0]) / (vd * bd)
                self.theta =  np.arctan2(sintheta, costheta)   
                if len(self.theta_maps) > 0:
                    previous = self.theta_maps[-1]
                    self.theta = self.theta + np.sign(previous-self.theta)*(2*np.pi)*(np.abs((previous-self.theta))//(2*np.pi*0.8))
                    
                
                self.theta_maps = np.append(self.theta_maps,self.theta)
        
                if len(self.theta_maps) > 40:
                    self.theta_maps = self.theta_maps[-40:len(self.theta_maps)]#this makes sure that we only look at the latest 150 frames of data to keep it adaptable. It should be bigger if there's a lot of noise (slow bot) and smaller if its traj is well defined (fast bot) 
                if self.iter % 20 == 0:
                    thetaNew = np.mean(self.theta_maps)#take the average, or median, so that the mapped angle is robust to noise                        
                    self.T_R = np.array([[np.cos(thetaNew), -np.sin(thetaNew)], [np.sin(thetaNew), np.cos(thetaNew)]])#only update the mapping every 40 frames
                
                #self.T_R = np.array([[costhetaNew, -sinthetaNew], [sinthetaNew, costhetaNew]])
            
        self.B_vec = np.dot(self.T_R, direction_vec)

        
        Bx = self.B_vec[0] / np.sqrt(self.B_vec[0] ** 2 + self.B_vec[1] ** 2)
        By = -self.B_vec[1] / np.sqrt(self.B_vec[0] ** 2 + self.B_vec[1] ** 2)  #needs to be negative because of coordinate system flip in the y direction
        Bz = .5
        
        return Bx,By,Bz
    
    




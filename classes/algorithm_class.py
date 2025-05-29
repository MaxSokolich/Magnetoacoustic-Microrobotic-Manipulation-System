from PyQt5.QtCore import pyqtSignal, pyqtSlot, Qt, QThread,QTimer
import numpy as np
import cv2
import random
import math
from classes.mpc.rrtstar import RrtStar

class algorithm:
   
    def __init__(self):
        self.reset()
        self.pixel2um =   3.45 / 10  #um/s
        self.continue_status = False

    def reset(self):
        self.node = 0
        self.count = 0
        self.stopped = False
        self.actions = [0,0,0,0,0,0,0,0]

        self.B_vec = np.array([0,1])
        self.T_R = 1
        self.theta_maps = np.array([])#added this so that we store the mapped angles
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

    


    def run(self, frame, mask, robot_list, stepsize, arrivialthresh, orientstatus, autoacoustic_status, pixel2um):
        self.pixel2um = pixel2um
        
    
        if len(robot_list[-1].trajectory) > 0:
            if self.count == 0: #% 10
              
                #define start end
                startpos = robot_list[-1].position_list[-1] #the most recent position at the time of clicking run algo
                endpos = robot_list[-1].trajectory[-1]
                

                #remove robot so that itself is not mistaken for an obstacle
                x,y,w,h = robot_list[-1].cropped_frame[-1]
                cv2.rectangle(mask, (x, y), (x + w, y + h), (0, 0, 0), -1)


                #RRT STAR planner
                """rrt_star = RrtStar(img = mask, x_start = startpos, x_goal=endpos, step_len=50,
                                         goal_sample_rate=.1, search_radius=2, iter_max=3000,plotting_flag=True)
                
                robot_list[-1].trajectory = rrt_star.planning()"""
                
                
                #RRT planner
                
                pathplanner = RRT(mask, startpos, endpos, stepsize)
                trajectory = pathplanner.run()
                trajectory.append(endpos)    
            
        
                #record robot list trajectory
                robot_list[-1].trajectory= trajectory


            #logic for arrival condition
            if self.node == len(robot_list[-1].trajectory):
                #weve arrived
                self.stopped = True
                self.Bx,self.By,self.Bz,self.alpha, self.gamma, self.freq, self.psi, self.acoustic_frequency = 0,0,0,0,0,0,0,0
                if self.continue_status == True:
                    self.reset()


            #closed loop algorithm 
            else:
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
                
                if orientstatus == True:
                    self.Bx, self.By, self.Bz, self.alpha = self.orient(robot_list[-1], direction_vec)
                else:
                    self.Bx, self.By, self.Bz = 0,0,0
                
                

                #draw error arrow
                cv2.arrowedLine(
                    frame,
                    (int(robotx), int(roboty)),
                    (int(targetx), int(targety)),
                    [0, 0, 0],
                    3,
                )
        
                if error < arrivialthresh:
                    self.node += 1
                
         
        

        if autoacoustic_status == True:
            self.acoustic_frequency = self.find_optimal_acoustic_freq(robot_list)

      
        
  
        self.actions = [self.Bx,self.By,self.Bz,self.alpha, self.gamma, self.freq, self.psi, self.acoustic_frequency]
        self.count += 1

    
        return frame, self.actions, self.stopped

    def push(self, bot, frame):
        print("pushing")


    def find_optimal_acoustic_freq(self, robot_list):
        #take a rolling average of the velocity from past 10 frames and average
        if len(robot_list[-1].velocity_list) > 0:
            vmag_avg = robot_list[-1].velocity_list[-1][2] * self.pixel2um


            

            
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
            vx = bot.velocity_list[-1][0] * self.pixel2um
            vy = bot.velocity_list[-1][1] * self.pixel2um
            
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
                if self.iter % 40 == 0:
                    thetaNew = np.mean(self.theta_maps)#take the average, or median, so that the mapped angle is robust to noise                        
                    self.T_R = np.array([[np.cos(thetaNew), -np.sin(thetaNew)], [np.sin(thetaNew), np.cos(thetaNew)]])#only update the mapping every 40 frames
                
                #self.T_R = np.array([[costhetaNew, -sinthetaNew], [sinthetaNew, costhetaNew]])
            
        self.B_vec = np.dot(self.T_R, direction_vec)

        
        Bx = self.B_vec[0] / np.sqrt(self.B_vec[0] ** 2 + self.B_vec[1] ** 2)
        By = -self.B_vec[1] / np.sqrt(self.B_vec[0] ** 2 + self.B_vec[1] ** 2)  #needs to be negative because of coordinate system flip in the y direction
        Bz = 0
        alpha = np.arctan2(By, Bx)

        return Bx,By,Bz,alpha
    
    



class Nodes:
    """Class to store the RRT graph"""
    def __init__(self, x,y):
        self.x = x
        self.y = y
        self.parent_x = []
        self.parent_y = []

class RRT:
    def __init__(self, img, start, end, stepsize):
        
        #imagepath = "/Users/bizzarohd/Desktop/mask.png"
        self.img = img #cv2.imread(imagepath,0) # load grayscale maze image
        self.start = start#(20,20) #(20,20) # starting coordinate
        self.end = end#(650,450) #(450,250) # target coordinate
        self.stepSize = stepsize # stepsize for RRT
        self.node_list = [0]

    # check collision
    def collision(self, x1,y1,x2,y2):
        color=[]
        x = list(np.arange(x1,x2,(x2-x1)/100))
        y = list(((y2-y1)/(x2-x1))*(x-x1) + y1)

        for i in range(len(x)):
            #print(int(x[i]),int(y[i]))
        
            color.append(self.img[int(y[i]),int(x[i])])

        #print(color, "\n\n")
        if (255 in color):
            return True #collision
        else:
            return False #no-collision

    # check the  collision with obstacle and trim
    def check_collision(self, x1,y1,x2,y2):
        _,theta = self.dist_and_angle(x2,y2,x1,y1)
        x=x2 + self.stepSize*np.cos(theta)
        y=y2 + self.stepSize*np.sin(theta)
        #print(x2,y2,x1,y1)
        #print("theta",theta)
        #print("check_collision",x,y)

        # TODO: trim the branch if its going out of image area
        # print("Image shape",img.shape)
        hy,hx=self.img.shape
        if y<0 or y>hy or x<0 or x>hx:
            #print("Point out of image bound")
            directCon = False
            nodeCon = False
        else:
            # check direct connection
            if self.collision(x,y,self.end[0],self.end[1]):
                directCon = False
            else:
                directCon=True

            # check connection between two nodes
            if self.collision(x,y,x2,y2):
                nodeCon = False
            else:
                nodeCon = True

        return(x,y,directCon,nodeCon)

    # return dist and angle b/w new point and nearest node
    def dist_and_angle(self,x1,y1,x2,y2):
        dist = math.sqrt( ((x1-x2)**2)+((y1-y2)**2) )
        angle = math.atan2(y2-y1, x2-x1)
        return(dist,angle)

    # return the neaerst node index
    def nearest_node(self,x,y):
        temp_dist=[]
        for i in range(len(self.node_list)):
            dist,_ = self.dist_and_angle(x,y,self.node_list[i].x,self.node_list[i].y)
            temp_dist.append(dist)
        return temp_dist.index(min(temp_dist))

    # generate a random point in the image space
    def rnd_point(self,h,l):
        new_y = random.randint(0, h)
        new_x = random.randint(0, l)
        return (new_x,new_y)


    def run(self):
        h,l= self.img.shape # dim of the loaded image
        # print(img.shape) # (384, 683)
        # print(h,l)

        # insert the starting point in the node class
        # node_list = [0] # list to store all the node points         
        self.node_list[0] = Nodes(self.start[0],self.start[1])
        self.node_list[0].parent_x.append(self.start[0])
        self.node_list[0].parent_y.append(self.start[1])


        i=1
        pathFound = False
        for k in range(10000):#
            nx,ny = self.rnd_point(h,l)  #generate random point
            #print("Random points:",nx,ny)

            nearest_ind = self.nearest_node(nx,ny)
            nearest_x = self.node_list[nearest_ind].x
            nearest_y = self.node_list[nearest_ind].y
            #print("Nearest node coordinates:",nearest_x,nearest_y)

            #check direct connection
            tx,ty,directCon,nodeCon = self.check_collision(nx,ny,nearest_x,nearest_y)
            #print("Check collision:",tx,ty,directCon,nodeCon)

            if directCon and nodeCon:
                self.node_list.append(i)
                self.node_list[i] = Nodes(tx,ty)
                self.node_list[i].parent_x = self.node_list[nearest_ind].parent_x.copy()
                self.node_list[i].parent_y = self.node_list[nearest_ind].parent_y.copy()
                self.node_list[i].parent_x.append(tx)
                self.node_list[i].parent_y.append(ty)


                #print("Path has been found")
            
                trajectory = list(zip(self.node_list[i].parent_x,self.node_list[i].parent_y))
                return trajectory
                

            elif nodeCon:
                #print("Nodes connected")
                self.node_list.append(i)
                self.node_list[i] = Nodes(tx,ty)
                self.node_list[i].parent_x = self.node_list[nearest_ind].parent_x.copy()
                self.node_list[i].parent_y = self.node_list[nearest_ind].parent_y.copy()
                # print(i)
                # print(node_list[nearest_ind].parent_y)
                self.node_list[i].parent_x.append(tx)
                self.node_list[i].parent_y.append(ty)
                i=i+1
                continue

            else:
                #print("No direct con. and no node con. :( Generating new rnd numbers")
                continue
                    
        
        return [] #if the for loop ends without finding a path return an empty lst

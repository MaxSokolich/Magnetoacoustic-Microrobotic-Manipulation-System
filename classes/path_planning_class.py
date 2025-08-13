import cv2
import numpy as np
import random
import math
from classes.mpc.rrtstar import RrtStar




class Path_Planner:
    def __init__(self):
        self.count = 0


    def run(self, robot_list, mask, stepsize):
        

            
              
        #define start end
        startpos = robot_list[-1].position_list[-1] #the most recent position at the time of clicking run algo
        endpos = robot_list[-1].trajectory[-1]
        

        #remove robot so that itself is not mistaken for an obstacle
        x,y,w,h = robot_list[-1].cropped_frame[-1]
        mask = cv2.rectangle(mask, (x, y), (x + w, y + h), (0, 0, 0), -1)


        #RRT STAR planner
        """rrt_star = RrtStar(img = mask, x_start = startpos, x_goal=endpos, step_len=50,
                                    goal_sample_rate=.1, search_radius=2, iter_max=3000,plotting_flag=True)
        
        robot_list[-1].trajectory = rrt_star.planning()"""
        
        
        #RRT planner
        pathplanner = RRT(mask, startpos, endpos, stepsize)
        trajectory = pathplanner.run()
        trajectory.append(endpos)    
        
    

  
        
        return trajectory

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
    
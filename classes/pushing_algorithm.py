import cv2
import numpy as np

class pushing_algorithm:
    def __init__(self):

        self.objective = 10
        self.pixel2um = 3.45 / self.objective
        self.corridor_width = 100  # thicness of guiding corridor in pixels
        self.corridor_width = 100 / self.pixel2um


        self.approach_distance = 100
        self.goal_threshold = 20
        self.approach_threshold = 50 #how close the robot needs to come within the approach coord
        
        
        self.approach_coord = None
        self.arrived_at_approach = False
        
        self.node = 0
        self.alpha = 0 #heading angle
        self.gamma = 0 #polar angle
        self.microvortice_freq = 0  #rotating field frequency
        self.pushingfreq = 0    #rotating field frequency
        self.freq = 0
        self.release_distance = 500
        
        


    def calculate_approach_coord(self, frame, start, goal):
        """
        args: frame, start coordinate, goal coordinate
        returns: updated frame, and approach cooridnate behind object
        """
        OG = goal - start
    
        # Normalize OG to get the unit vector in the direction of OG
        OG_unit = OG / np.linalg.norm(OG)
        
        # Calculate point A by moving d units in the opposite direction of OG
        A = (start - self.approach_distance  * OG_unit).astype(int) #approach coordinate

        cv2.circle(frame, A, 10, (255, 0, 255), -1)

        return frame, A



    def draw_guiding_corridor(self, frame, start, goal):
        
          # guiding corridate thickness

        # Compute the direction vector of the central line
        AG = start - goal

        # Compute the perpendicular (normal) vector
        perpendicular_vector = np.array([-AG[1], AG[0]])

        # Normalize the perpendicular vector
        perpendicular_vector = perpendicular_vector / np.linalg.norm(perpendicular_vector)

        # Compute the offset vector for the parallel lines
        offset_vector = ((self.corridor_width / self.pixel2um) / 2) * perpendicular_vector


        # Compute the endpoints of the two parallel lines
        R1 = (start - offset_vector).astype(int)
        R2 = (goal - offset_vector).astype(int)

        L1 = (start + offset_vector).astype(int)
        L2 = (goal + offset_vector).astype(int)


        # Draw the two parallel lines
        cv2.arrowedLine(frame, L1, L2, (255, 0, 255), 2)
        cv2.arrowedLine(frame, R1, R2, (0, 255, 255), 2)

        goal_error = np.sqrt((goal[1]- start[1])**2 + (goal[0] - start[0])**2)
        #cv2.line(frame, (int(start[0]), int(start[1])), (int(goal[0]), int(goal[1])), [0, 0, 0],3,)


        
        return frame, L1, L2, R1, R2
    
    
    def calculate_corridor_determinant(self,frame, start, end, object):
        #cv2.line(frame, (int(object[0]), int(object[1])), (int(start[0]), int(start[1])), [0, 0, 0],3,)
        #cv2.line(frame, (int(object[0]), int(object[1])), (int(end[0]), int(end[1])), [0, 0, 0],3,)
        D = (end[0] - start[0]) * (object[1] - start[1]) - (end[1] - start[1]) * (object[0] - start[0])
       
        return D, frame



    def release(self, frame, bot_pos, cell_pos):
        #release mechanism
        #step 1: define line pointing away from object
        BC = cell_pos - bot_pos
        BC_unit = BC / np.linalg.norm(BC)
        
        #step 2: move a distance self.release_distance away from object
        R = (cell_pos - self.release_distance  * BC_unit).astype(int) #approach coordinate
        cv2.circle(frame, R, 5, (255, 0, 255), -1)
        cv2.arrowedLine(frame, (int(bot_pos[0]), int(bot_pos[1])), (int(R[0]), int(R[1])), [0, 0, 0],5,)
        
        #step 2: move a distance self.release_distance away from object
        robot_direction_vec = [R[0] - bot_pos[0], R[1] - bot_pos[1]]
        
        alpha = np.arctan2(-robot_direction_vec[1], robot_direction_vec[0])
        gamma = np.radians(90)
        freq = 1

        return frame, alpha, gamma, freq

    



    def run(self, frame, robot_list, cell_list, arrivalthresh, corridor_width,approach_distance, spinning_freq, pushingfreq, pixel2um):
        
        self.corridor_width = corridor_width
        self.approach_distance = approach_distance
        self.microvortice_freq = spinning_freq
        self.pushingfreq = pushingfreq
        self.goal_threshold = arrivalthresh
        self.pixel2um = pixel2um
        #input:  robot_list which stores all the attributes for each robot you select
        
        
        

        if len(robot_list) > 0 and len(cell_list) > 0:
            bot_pos = np.array(robot_list[-1].position_list[-1])
            cell_pos = np.array(cell_list[-1].position_list[-1])
            #cv2.line(frame, (0,0), (int(bot_pos[0]), int(bot_pos[1])), [0, 100, 100],5,)
            #cv2.line(frame, (0,0), (int(cell_pos[0]), int(cell_pos[1])), [100, 100, 0],5,)

        
            if len(robot_list[-1].trajectory) > 0:  #if a trajectory is defined
                #display trajecotry node status
                cv2.putText(frame, "Node = {}/{}".format(self.node, len(robot_list[-1].trajectory)),(int(2448 / 2.2),int(2048 / 8)),cv2.FONT_HERSHEY_SIMPLEX, fontScale=1.5, thickness=3,
                                            color = (255, 255, 255),)
                
                if self.node == len(robot_list[-1].trajectory): #if we are at the end of the trajectory, stop the algorithm

                    self.alpha, self.gamma, self.freq = 0,0,0# self.release(frame, bot_pos, cell_pos) # and relase the object by slowly moving away


                    

                     


                else:  #apply the algorithm
                    target_pos = np.array(robot_list[-1].trajectory[self.node])


                    # if we havent arrived at the approach point, guide the microrobot there
                    if self.arrived_at_approach == False:
                        #check if an approach coord is defined. if it is, guide the microrobot to the approach coord using the error.
                        #if its not, define it
                        if self.approach_coord is not None:
                            approach_error = np.sqrt((self.approach_coord[1]- bot_pos[1])**2 + (self.approach_coord[0] - bot_pos[0])**2) # calcualte error
                            
                            if approach_error > self.approach_threshold:
                                #calculate the direction vector
                                direction_vec = [self.approach_coord[0] - bot_pos[0], self.approach_coord[1] - bot_pos[1]]
                                
                                #visualize the error
                                cv2.arrowedLine(frame, (int(bot_pos[0]), int(bot_pos[1])), (int(self.approach_coord[0]), int(self.approach_coord[1])), [0, 0, 255],5,)
                                cv2.putText(frame, "approach error = {:.1f}".format(approach_error),(int(2448 / 2.2),int(2048 / 9)),cv2.FONT_HERSHEY_SIMPLEX, fontScale=1.5, thickness=3,
                                            color = (255, 255, 255),)
                                
                                #output signals to get the particle there                                
                                self.alpha = np.arctan2(-direction_vec[1], direction_vec[0])
                                self.freq = self.pushingfreq
                                self.gamma = np.radians(90)
                            else:
                                self.arrived_at_approach = True
                                #this is where the rest of the algorithm comes in
                        
                        
                        #define the approach coordinate
                        else:       
                            frame, self.approach_coord = self.calculate_approach_coord(frame, cell_pos, target_pos)
                    


                    # once were at the approach coordinate, begin pushing object in direction of goal
                    else:
                
                        goal_error = np.sqrt((target_pos[1]- cell_pos[1])**2 + (target_pos[0] - cell_pos[0])**2)
                        
                        #Step 1: define guiding corridor
                        frame, L1, L2, R1, R2 = self.draw_guiding_corridor(frame, bot_pos, target_pos)
                        
                        #Step 2: calculate the determinate of the triangle the forms the left guiding corridor coords and the objects position
                        DL, frame = self.calculate_corridor_determinant(frame, L1, L2, cell_pos)
                        #Step 3: calculate the determinate of the triangle the forms the right guiding corridor coords and the objects position
                        DR, frame = self.calculate_corridor_determinant(frame, R1, R2, cell_pos)

                        cv2.putText(frame, "DL = {:.1f}, DR = {:.1f}".format(DL, DR),(int(2448 / 2.2),int(2048 / 10)),cv2.FONT_HERSHEY_SIMPLEX, fontScale=1.5, thickness=3,
                                            color = (255, 255, 255),)
            

                        if goal_error < self.goal_threshold: # if object is within threshold distance to targt position, move ont ot he next node in the array
                            self.node += 1
                        
                        else:  #push object towards target/goal 
                            
                            #Step 4: apply conditions 
                            if DL < 0 and DR < 0: # object is to the left of guiding corridor (y is flipped making negative)
                                #spin CW
                                #print("WE MOVED TO THE LEFT OF LEFT CORRIDOR")
                                self.alpha = 0
                                self.freq = self.microvortice_freq
                                self.gamma = np.radians(180)

                            elif DR > 0 and DL > 0: #object is to the right of guiding corridor (y is flipped making negative)
                                #spin CCW
                                #print("WE MOVED TO THE RIGHT OF RIGHT CORRIDOR")
                                self.alpha = 0
                                self.freq = self.microvortice_freq
                                self.gamma = np.radians(0)

                            else:
                                #print("PUSHING TOWARDS GOAL")
                                robot_direction_vec = [target_pos[0] - bot_pos[0], target_pos[1] - bot_pos[1]]
                                self.alpha = np.arctan2(-robot_direction_vec[1], robot_direction_vec[0])
                                self.freq = self.pushingfreq
                                self.gamma = np.radians(90)
                            
    
                    
                   
        
    

        return frame, self.alpha, self.gamma, self.freq

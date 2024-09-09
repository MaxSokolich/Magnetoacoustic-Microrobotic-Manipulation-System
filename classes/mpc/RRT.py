
import os
import sys
import math
import numpy as np
import cv2
import queue
import matplotlib.pyplot as plt

class Node:
    def __init__(self, n):
        self.x = n[0]
        self.y = n[1]
        self.parent = None


class RrtStar:
    def __init__(self, img, x_start, x_goal, step_len,
                 goal_sample_rate, search_radius, iter_max,stepsize = 15,plotting_flag = True,directCon_flag = False):       
        self.s_start = Node(x_start)
        self.s_goal = Node(x_goal)
        self.step_len = step_len
        self.goal_sample_rate = goal_sample_rate
        self.search_radius = search_radius
        self.iter_max = iter_max
        self.vertex = [self.s_start]
        self.path = []
        self.delta = 0.5
        self.plotting_flag = plotting_flag
        # uBots attributes:
        self.img = img 
        self.y_range, self.x_range = self.img.shape
        self.stepSize = stepsize
        self.directCon_flag = directCon_flag

    # check collision: (Based on checking the change of pixle values)
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
    def check_collision(self, node_1,node_2):
        x1,y1,x2,y2 = node_1.x,node_1.y,node_2.x,node_2.y
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
            if self.collision(x,y,self.s_goal.x,self.s_goal.y):
                directCon = False
            else:
                directCon=True

            # check connection between two nodes
            if self.collision(x,y,x2,y2):
                nodeCon = False
            else:
                nodeCon = True

        return(x,y,directCon,nodeCon)
    
    def dist_and_angle(self,x1,y1,x2,y2):
        dist = math.sqrt( ((x1-x2)**2)+((y1-y2)**2) )
        angle = math.atan2(y2-y1, x2-x1)
        return(dist,angle)

    def planning(self):
        for k in range(self.iter_max) :#range(self.iter_max):
            node_rand = self.generate_random_node(self.goal_sample_rate)
            node_near = self.nearest_neighbor(self.vertex, node_rand)
            node_new = self.new_state(node_near, node_rand)

            # if k % 2000 == 0:
            #     print('rrtStar sampling iterations: ', k)
            #     # TODO plotting
            #     # self.plotting.animation_online(self.vertex, "rrtStar", True)

            tx,ty,directCon,nodeCon = self.check_collision(node_near, node_new)
            
            # If direct connection to the goal exist: 
            if node_new and directCon and nodeCon and self.directCon_flag:
                self.vertex.append(node_near)
                node_new = Node((tx,ty))
                node_new.parent = node_near
                self.path = self.extract_path(node_new)
                if self.plotting_flag:
                    self.plot_visited(self.vertex, img = self.img)
                    path = np.array(self.path)
                    plt.plot(path[:,0],path[:,1])
                return self.path

            if node_new and nodeCon:
                neighbor_index = self.find_near_neighbor(node_new)
                self.vertex.append(node_new)

                if neighbor_index:
                    self.choose_parent(node_new, neighbor_index)
                    self.rewire(node_new, neighbor_index)
            print(k)
        
        index = self.search_goal_parent()
        self.path = self.extract_path(self.vertex[index])
        self.path.reverse()
        path = np.array(self.path)
        if self.plotting_flag:
            self.plot_visited(self.vertex, img = self.img)
            plt.plot(path[:,0],path[:,1])
        list(zip(path[:,0],path[:,1]))
        return list(zip(path[:,0],path[:,1]))
        # self.plotting.animation(self.vertex, self.path, "rrt*, N = " + str(self.iter_max))

    def new_state(self, node_start, node_goal):
        dist, theta = self.get_distance_and_angle(node_start, node_goal)

        dist = min(self.step_len, dist)
        node_new = Node((node_start.x + dist * math.cos(theta),
                         node_start.y + dist * math.sin(theta)))

        node_new.parent = node_start

        return node_new

    def choose_parent(self, node_new, neighbor_index):
        cost = [self.get_new_cost(self.vertex[i], node_new) for i in neighbor_index]

        cost_min_index = neighbor_index[int(np.argmin(cost))]
        node_new.parent = self.vertex[cost_min_index]

    def rewire(self, node_new, neighbor_index):
        for i in neighbor_index:
            node_neighbor = self.vertex[i]

            if self.cost(node_neighbor) > self.get_new_cost(node_new, node_neighbor):
                node_neighbor.parent = node_new
                # update the cost to reach the node: 


    def search_goal_parent(self):
        dist_list = [math.hypot(n.x - self.s_goal.x, n.y - self.s_goal.y) for n in self.vertex]
        node_index = [i for i in range(len(dist_list)) if dist_list[i] <= self.step_len]

        if len(node_index) > 0:
            cost_list = [dist_list[i] + self.cost(self.vertex[i]) for i in node_index
                         if self.check_collision(self.vertex[i], self.s_goal)[3]]
            return node_index[int(np.argmin(cost_list))]

        return len(self.vertex) - 1

    def get_new_cost(self, node_start, node_end):
        dist, _ = self.get_distance_and_angle(node_start, node_end)

        return self.cost(node_start) + dist

    def generate_random_node(self, goal_sample_rate):
        delta = self.delta

        if np.random.random() > goal_sample_rate:
            return Node((np.random.uniform(0 + delta, self.x_range - delta),
                         np.random.uniform(0+ delta, self.y_range - delta)))

        return self.s_goal

    def find_near_neighbor(self, node_new):
        n = len(self.vertex) + 1
        r = min(self.search_radius * math.sqrt((math.log(n) / n)), self.step_len)
        r = 55
        dist_table = [math.hypot(nd.x - node_new.x, nd.y - node_new.y) for nd in self.vertex]
        # dist_table_index = [ind for ind in range(len(dist_table)) if dist_table[ind] <= r and
        #                     not self.utils.is_collision(node_new, self.vertex[ind])]
        dist_table_index = [ind for ind in range(len(dist_table)) if dist_table[ind] <= r and
                             self.check_collision(node_new, self.vertex[ind])[3]]
    
        return dist_table_index

    @staticmethod
    def nearest_neighbor(node_list, n):
        return node_list[int(np.argmin([math.hypot(nd.x - n.x, nd.y - n.y)
                                        for nd in node_list]))]

    @staticmethod
    def cost(node_p):
        node = node_p
        cost = 0.0

        while node.parent:
            cost += math.hypot(node.x - node.parent.x, node.y - node.parent.y)
            node = node.parent

        return cost

    def update_cost(self, parent_node):
        OPEN = queue.QueueFIFO()
        OPEN.put(parent_node)

        while not OPEN.empty():
            node = OPEN.get()

            if len(node.child) == 0:
                continue

            for node_c in node.child:
                node_c.Cost = self.get_new_cost(node, node_c)
                OPEN.put(node_c)

    def extract_path(self, node_end):
        path = [[self.s_goal.x, self.s_goal.y]]
        node = node_end

        while node.parent is not None:
            path.append([node.x, node.y])
            node = node.parent
        path.append([node.x, node.y])

        return path
    
    def plot_visited(self,nodelist, img,animation=False):
        if animation:
            count = 0
            for node in nodelist:
                count += 1
                if node.parent:
                    plt.plot([node.parent.x, node.x], [node.parent.y, node.y], "-g")
                    plt.gcf().canvas.mpl_connect('key_release_event',
                                                 lambda event:
                                                 [exit(0) if event.key == 'escape' else None])
                    if count % 10 == 0:
                        plt.pause(0.001)
        else:
            fig, ax = plt.subplots()
            ax.imshow(img,cmap='gray')
            for node in nodelist:
                if node.parent:
                    ax.plot([node.parent.x, node.x], [node.parent.y, node.y], "-g")
        a = 1
    @staticmethod
    def get_distance_and_angle(node_start, node_end):
        dx = node_end.x - node_start.x
        dy = node_end.y - node_start.y
        return math.hypot(dx, dy), math.atan2(dy, dx)
    


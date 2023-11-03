#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Jun 18 14:07:19 2022

@author: bizzarohd
"""
import datetime as dt
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import time as time
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D 
import matplotlib.colors as colors
#plt.style.use('Solarize Light2')

class Helmholtz_Simulator:
    def __init__(self, alpha, gamma, psi, freq, memory, Bx, By, Bz):
        self.start = time.time()


     
        #define lists to store sineusoid in
        self.t_list = []
        self.Ix_List = []
        self.Iy_List = []
        self.Iz_List = []
        self.memory = memory

        #rolling parameters
        self.Bx = Bx
        self.By = By
        self.Bz = Bz

        self.A = 1 #amplitude of rotating magetnic field
        self.alpha = alpha* (np.pi/180)   # yaw angle converted to radians
        self.gamma = gamma * (np.pi/180) # pitch angle converted to radians
        self.psi = psi * (np.pi/180)
        self.omega = 2*np.pi* float(freq)  #angular velocity of rotating field defined from input from Rotating Frequency Entry
        
 


        # Create figure for plotting
        self.fig = plt.figure(figsize=(14,4))
        self.fig.suptitle("alpha = {}     gamma = {}     freq = {}     ".format(alpha,gamma,freq))
        
        
        self.ax = self.fig.add_subplot(141, projection='3d')  #3d field sim
        self.ax.view_init(elev=30, azim=45)
       
        self.ax1 = self.fig.add_subplot(142, projection='3d')#self.fig.add_subplot(132)  #2d sine sim
        self.ax1.view_init(elev=30, azim=45)

        self.ax2 = self.fig.add_subplot(143)  #2d sine sim
        self.ax3 = self.fig.add_subplot(144)  #2d diagram image
        


        #params for field
        self.milli = 10**(-6)
        self.mu = 4*np.pi * (10**(-7)) 

        #define 3D grid
        grid_res = 8 *self.milli
        min_x,min_y,min_z = -25*self.milli, -25*self.milli,-25*self.milli #mm
        max_x, max_y, max_z = 25*self.milli, 25*self.milli,25*self.milli

        X = np.arange(min_x, max_x, grid_res)
        Y = np.arange(min_y, max_y, grid_res)
        Z = np.arange(min_z, max_z, grid_res)
        self.x,self.y,self.z = np.meshgrid(X, Y, Z)

    #helmholtz X field equation
    def xb_field(self,x, Ix):
        a = 54*self.milli #radius
        c = (84/2)*self.milli #distance between /2
        N = 260 #number of turns
        #the field at x away from the coil
        term1 = 1/ (a**2 + (c-x)**2)**(3/2)
        term2 = 1/ (a**2 + (c+x)**2)**(3/2)
        B = ((self.mu * N * Ix * a**2)/2 ) * (term1 + term2)
        return B

    #helmholtz Y field equation
    def yb_field(self,y, Iy):
        a = 35*self.milli #radius
        c = (66/2)*self.milli #distance between /2
        N = 368 #number of turns
        #the field at x away from the coil
        term1 = 1/ ((a**2 + (c-y)**2)**(3/2))
        term2 = 1/ ((a**2 + (c+y)**2)**(3/2))
        B = ((self.mu * N * Iy * a**2)/2 ) * (term1 + term2)
        return B

    #helmholtz Z field equation
    def zb_field(self, z, Iz):
        a = 26*self.milli #radius
        c = (26/2)*self.milli #distance between /2
        N = 368 #number of turns
        #the field at x away from the coil
        term1 = 1/ (a**2 + (c-z)**2)**(3/2)
        term2 = 1/ (a**2 + (c+z)**2)**(3/2)
        B = ((self.mu * N * Iz * a**2)/2 ) * (term1 + term2)
        return B

    # This function is called periodically from FuncAnimation

    def animate(self,i):
        tp = time.time() - self.start
        #self.alpha = i * np.pi/180    #<<--- uncomment this line to sweep alpha from 0 -360
        #self.psi = (i %90) *np.pi/180
        #self.ax.view_init(elev=90, azim=45)
        #self.ax1.view_init(elev=90, azim=45)
        if self.omega != 0:
            
            Brollx =  ((-np.sin(self.alpha) * np.sin(self.omega*tp)) + (-np.cos(self.alpha) * np.cos(self.gamma)  * np.cos(self.omega*tp))) 
            Brolly =  ((np.cos(self.alpha) * np.sin(self.omega*tp)) + (-np.sin(self.alpha) * np.cos(self.gamma) *  np.cos(self.omega*tp))) 
            Brollz =  np.sin(self.gamma) * np.cos(self.omega*tp)

            
            if self.psi < np.pi/2:
                #if self.alpha % (np.pi/2) == 0:
                #    self.alpha = self.alpha + 0.000001#for some strange reason the eqns give wrong answers when alpha is pi/2
                #if self.gamma == 0 or self.gamma % (np.pi/2) == 0:
                #    self.gamma = self.gamma + 0.000001
                c = 1/np.tan(self.psi)
                BxPer = c* np.cos(self.alpha) * np.sin(self.gamma)
                ByPer = np.tan(self.alpha) * BxPer
                #print(ByPer)
                BzPer = BxPer * (1/np.cos(self.alpha)) * (1/np.tan(self.gamma))
            else:
                BxPer = 0
                ByPer = 0
                BzPer = 0
                c = 0

            Brollx = (Brollx + BxPer) / (1+c)
            Brolly = (Brolly + ByPer) / (1+c)
            Brollz = (Brollz + BzPer) / (1+c)
        else:
            Brollx = 0
            Brolly = 0
            Brollz = 0


        #super impose orient field on top of already agumented roll field that includes a perp compnent from 
        #allows for alittle more control of vector field
        #cc = np.sqrt(Brollx*Brollx + Brolly*Brolly + Brollz*Brollz )
        #ccc = np.sqrt(self.Bx*self.Bx + self.By*self.By + self.Bz*self.Bz )
        #print(ccc)
        #print(cc)
        Ix = round((self.Bx + Brollx),3) ##/ (ccc+cc)
        Iy = round((self.By + Brolly),3) #/ (ccc+cc)
        Iz = round((self.Bz + Brollz),3) #/ (ccc+cc)
        
        uniform_mag = round(np.sqrt(self.Bx**2 + self.By**2 + self.Bz**2),3)
        rolling_mag = round(np.sqrt(Brollx**2 + Brolly**2 + Brollz**2),3)

        mag = max(uniform_mag, rolling_mag)
        denom = round(np.sqrt(Ix**2 + Iy**2 + Iz**2),3)

   
        Ix_final = mag*(Ix / denom)
        Iy_final = mag*(Iy / denom)
        Iz_final = mag*(Iz / denom)
        m = round(np.sqrt(Ix_final**2 + Iy_final**2 + Iz_final**2),3)
        print("final normalize field", m)
       
        BX = self.zb_field(self.x, Ix_final)  
        BY = self.zb_field(self.y, Iy_final)
        BZ = self.zb_field(self.z, Iz_final)

        print(Ix_final,Iy_final,Iz_final)
        
        
        #update Bfield lists for sinusoids
        self.t_list.append(tp)
        self.Ix_List.append(Ix_final)
        self.Iy_List.append(Iy_final)
        self.Iz_List.append(Iz_final)

        # Limit x and y lists to 20 items
        t_list = self.t_list[-self.memory:]
        Ix_List = self.Ix_List[-self.memory:]
        Iy_List = self.Iy_List[-self.memory:]
        Iz_List = self.Iz_List[-self.memory:]

        
        # Draw Bx,By,Bz field
        self.ax.clear()
       
        self.show_axis_rotation(self.ax, 0.0001)
        speed = np.sqrt((BX)**2+(BY)**2+(BZ)**2).flatten()
        self.ax.quiver(self.x,self.y,self.z,BX,BY,BZ, color='black',length=0.000001) #norm = colors.LogNorm(vmin=speed.min(), vmax=speed.max() ))#,density = 2)#norm = colors.LogNorm(vmin=speed.min(), vmax=speed.max() ))
        self.ax.scatter(self.x,self.y,self.z, s = 1, c= "b")
        self.ax.set_xlabel('x (mm)') 
        self.ax.set_ylabel('y (mm)') 
        self.ax.set_zlabel('z (mm)') 
        self.ax.set_title("Bvectorfield = [Bx, By, Bz]\n and Axis or Rotation") 


        # Draw x and y lists
        self.ax2.clear()
        self.ax2.plot(t_list, Ix_List, label = "Ix (A)", color = "red")
        self.ax2.plot(t_list, Iy_List, label = "Iy (A)", color = "yellow")
        self.ax2.plot(t_list, Iz_List, label = "Iz (A)", color = "blue")
        self.ax2.set_ylim([-2,2])
        self.ax2.legend(loc='upper right')
        self.ax2.set_title("signals") 

        #plot IX,IY, IZ in 3D
        self.ax1.clear()    
        self.show_axis_rotation(self.ax1, 1)
        self.ax1.set_xlim([-1,1])
        self.ax1.set_ylim([-1,1])
        self.ax1.plot(Ix_List, Iy_List, Iz_List, label = "Iz (A)", color = "blue")
        self.ax1.set_xlabel('Ix (A)') 
        self.ax1.set_ylabel('Iy (A)') 
        self.ax1.set_zlabel('Iz (A)') 
      
        self.ax1.set_title("if line 108 uncommented\nalpha = {}".format(i)) 

     
        
    def show_axis_rotation(self, ax, length):
        #plot rotation axis
        x = 1 * np.sin(self.gamma) * np.cos(self.alpha)
        y = 1 * np.sin(self.gamma) * np.sin(self.alpha)  
        z = 1 * np.cos(self.gamma)
        ax.quiver(0,0,0,x,y,z,color='red',length=length)


    def run(self):
        # Set up plot to call animate() function periodically
        anim = animation.FuncAnimation(self.fig, self.animate,frames = range(360), interval=10, blit = False)

        plt.show()





if __name__ == "__main__":
    Bx = 0   #constant Bz field
    By = 0   #constant By field
    Bz = 0   #constant Bz field
    alpha = 90  # polar angle measure from the positive z axis
    gamma = 90  # azimuthal angle measure from the positive z axis
    psi = 90   # cone angle measure from the axis of rotation
    freq = .5

    

    memory = 15  # for sinuoisd, so its only plot the last 15 points in the list
    
    sim = Helmholtz_Simulator(alpha, gamma,psi, freq, memory, Bx, By, Bz)
    sim.run()

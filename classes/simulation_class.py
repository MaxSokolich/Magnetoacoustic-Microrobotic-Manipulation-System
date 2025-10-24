import sys
import time as time
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.animation import FuncAnimation
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget
from PyQt5.QtCore import Qt, QTimer

class HelmholtzSimulator(FigureCanvas):
    def __init__(self, parent=None, width=310, height=310, dpi=200):
        fig = plt.figure(figsize=(width/dpi, height/dpi), dpi=dpi)
        fig.tight_layout()
        fig.subplots_adjust(top=1, bottom=0, left=-0.1, right=1)
        self.ax = fig.add_subplot(111, projection='3d')

        super().__init__(fig)
        self.setParent(parent)
        
        #rolling parameters
        self.Bx = 0
        self.By = 0
        self.Bz = 0
        self.A = 1 #amplitude of rotating magetnic field
        self.alpha = 0
        self.gamma = np.pi/2
        self.psi = 0
        self.freq = 0
        self.omega = 2*np.pi* float(self.freq)  #angular velocity of rotating field defined from input from Rotating Frequency Entry

    
        #params for field
        self.milli = 10**(-6)
        self.mu = 4*np.pi * (10**(-7)) /self.milli
        self.start_time = time.time()

        #define 3D grid
        grid_res = 16
        min_x,min_y,min_z = -25, -25,-25
        max_x, max_y, max_z = 25, 25,25

        X = np.arange(min_x, max_x, grid_res)
        Y = np.arange(min_y, max_y, grid_res)
        Z = np.arange(min_z, max_z, grid_res)
        self.x,self.y,self.z = np.meshgrid(X, Y, Z)
        self.ax.scatter(self.x,self.y,self.z, s = 2, c= "b")
        self.ax.set_xlabel('x',labelpad=-15) 
        self.ax.set_ylabel('y',labelpad=-15) 
        self.ax.set_zlabel('z',labelpad=-17)
        self.ax.set_xticks([])
        self.ax.set_yticks([])
        self.ax.set_zticks([])
        

        
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.animate)
          



    #helmholtz X field equation
    def xb_field(self,x, Ix):
        a = 60 #radius
        c = (155/2) #distance between /2
        N = 1200 #number of turns
        #the field at x away from the coil
        term1 = 1/ (a**2 + (c-x)**2)**(3/2)
        term2 = 1/ (a**2 + (c+x)**2)**(3/2)
        B = ((self.mu * N * Ix * a**2)/2 ) * (term1 + term2)
        return B

    #helmholtz Y field equation
    def yb_field(self,y, Iy):
        a = 60 #radius
        c = (155/2) #distance between /2
        N = 1200 #number of turns
        #the field at x away from the coil
        term1 = 1/ ((a**2 + (c-y)**2)**(3/2))
        term2 = 1/ ((a**2 + (c+y)**2)**(3/2))
        B = ((self.mu * N * Iy * a**2)/2 ) * (term1 + term2)
        return B

    #helmholtz Z field equation
    def zb_field(self, z, Iz):
        a = 60 #radius
        c = (155/2) #distance between /2
        N = 1200 #number of turns
        #the field at x away from the coil
        term1 = 1/ (a**2 + (c-z)**2)**(3/2)
        term2 = 1/ (a**2 + (c+z)**2)**(3/2)
        B = ((self.mu * N * Iz * a**2)/2 ) * (term1 + term2)
        return B
    

    def animate(self):
        tp = time.time() - self.start_time
       
      
        if [self.Bx, self.By, self.Bz, self.freq] == [0,0,0,0]:
            self.zero()
 
        else:
            
            
            if self.freq == 0:
                Brollx = 0
                Brolly = 0
                Brollz = 0
            else:
                Brollx =  ((-np.sin(self.alpha) * np.sin(-self.omega*tp)) + (-np.cos(self.alpha) * np.cos(self.gamma)  * np.cos(-self.omega*tp))) 
                Brolly =  ((np.cos(self.alpha) * np.sin(-self.omega*tp)) + (-np.sin(self.alpha) * np.cos(self.gamma) *  np.cos(-self.omega*tp))) 
                Brollz =  np.sin(self.gamma) * np.cos(-self.omega*tp)
        

            if self.psi < np.pi/2 and self.psi !=0:
                c = 1/np.tan(self.psi)
                BxPer = c* np.cos(self.alpha) * np.sin(self.gamma)
                ByPer = np.tan(self.alpha) * BxPer
                BzPer = BxPer * (1/np.cos(self.alpha)) * (1/np.tan(self.gamma))
            else:
                BxPer = 0
                ByPer = 0
                BzPer = 0
                c = 0
        
            Brollx = (Brollx + BxPer) / (1+c)
            Brolly = (Brolly + ByPer) / (1+c)
            Brollz = (Brollz + BzPer) / (1+c)

            #super impose orient field on top of already agumented roll field that includes a perp compnent from 
            #allows for alittle more control of vector field
            Ix = self.Bx + Brollx
            Iy = self.By + Brolly
            Iz = self.Bz + Brollz
      
            
            Ix = Ix / np.sqrt(Ix**2 + Iy**2 + Iz**2)
            Iy = Iy / np.sqrt(Ix**2 + Iy**2 + Iz**2)
            Iz = Iz / np.sqrt(Ix**2 + Iy**2 + Iz**2)
            
            BX = self.zb_field(self.x, Ix)  
            BY = self.zb_field(self.y, Iy)
            BZ = self.zb_field(self.z, Iz)

            
            # Draw Bx,By,Bz field
            self.ax.clear()
            if self.freq !=0:
                self.show_axis_rotation(self.ax, 40)
            self.ax.set_xlabel('x') 
            self.ax.set_ylabel('y') 
            self.ax.set_zlabel('z')
            self.ax.set_xticks([])
            self.ax.set_yticks([])
            self.ax.set_zticks([])
            speed = np.sqrt((BX)**2+(BY)**2+(BZ)**2).flatten()
            self.ax.quiver(self.x,self.y,self.z,BX,BY,BZ, color='black',length=1.5) #norm = colors.LogNorm(vmin=speed.min(), vmax=speed.max() ))#,density = 2)#norm = colors.LogNorm(vmin=speed.min(), vmax=speed.max() ))
            self.ax.scatter(self.x,self.y,self.z, s = 2, c= "b")
            self.draw()


    def show_axis_rotation(self, ax, length):
        #plot rotation axis
 
        x = 1 * np.sin(self.gamma) * np.cos(self.alpha)
        y = 1 * np.sin(self.gamma) * np.sin(self.alpha)  
        z = 1 * np.cos(self.gamma)
        ax.quiver(0,0,0,x,y,z,color='red',length=length)



    def start(self):
       self.timer.start(75)# Update plot every 10 ms

    def zero(self):
        self.ax.clear()
        self.ax.scatter(self.x,self.y,self.z, s = 2, c= "b")
        self.ax.set_xlabel('x') 
        self.ax.set_ylabel('y') 
        self.ax.set_zlabel('z')
        self.ax.set_xticks([])
        self.ax.set_yticks([])
        self.ax.set_zticks([])
        self.draw()


    def stop(self):
        self.timer.stop()
        self.zero()




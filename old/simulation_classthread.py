import sys
import time as time
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.animation import FuncAnimation
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget
from PyQt5.QtCore import Qt, QTimer

import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget
from PyQt5.QtCore import Qt, QThread, pyqtSignal
import random
from matplotlib import animation


class HelmholtzSimulator(QThread):
    figure_ready = pyqtSignal(int)
    
    def __init__(self, parent, width, height, dpi):
        super().__init__(parent=parent)
   
        
        #self.canvas = canvas
        #self.figure = figure
        
        fig = plt.figure(figsize=(width/dpi, height/dpi), dpi=dpi)
        fig.tight_layout()
        fig.subplots_adjust(top=1, bottom=0, left=-0.2, right=1)
        self.ax = fig.add_subplot(111, projection='3d')
        self.canvas = FigureCanvas(fig)
        self.canvas.setParent(parent)
        #self.run_flag = True
        
        
        
        #rolling parameters
        self.Bx = 0
        self.By = 0
        self.Bz = 0
        self.A = 1 #amplitude of rotating magetnic field
        self.alpha =0
        self.gamma = np.pi/2
        self.psi = 0
        self.freq = 1
        self.omega = 2*np.pi* float(self.freq)  #angular velocity of rotating field defined from input from Rotating Frequency Entry
        self.roll = False


        #params for field
        self.milli = 10**(-6)
        self.mu = 4*np.pi * (10**(-7)) /self.milli
        self.start_time = time.time()

        #define 3D grid
        grid_res = 8 
        min_x,min_y,min_z = -25, -25,-25
        max_x, max_y, max_z = 25, 25,25

        X = np.arange(min_x, max_x, grid_res)
        Y = np.arange(min_y, max_y, grid_res)
        Z = np.arange(min_z, max_z, grid_res)
        self.x,self.y,self.z = np.meshgrid(X, Y, Z)
        self.ax.scatter(self.x,self.y,self.z, s = 1, c= "b")
        self.ax.set_xlabel('x') 
        self.ax.set_ylabel('y') 
        self.ax.set_zlabel('z')


        self.timer = QTimer(self)
        self.timer.timeout.connect(self.animate)
        #anim = animation.FuncAnimation(fig, self.animate, frames = range(360), interval=10, blit = False)
        

        
    #helmholtz X field equation
    def xb_field(self,x, Ix):
        a = 54 #radius
        c = (84/2) #distance between /2
        N = 260 #number of turns
        #the field at x away from the coil
        term1 = 1/ (a**2 + (c-x)**2)**(3/2)
        term2 = 1/ (a**2 + (c+x)**2)**(3/2)
        B = ((self.mu * N * Ix * a**2)/2 ) * (term1 + term2)
        return B

    #helmholtz Y field equation
    def yb_field(self,y, Iy):
        a = 35 #radius
        c = (66/2) #distance between /2
        N = 368 #number of turns
        #the field at x away from the coil
        term1 = 1/ ((a**2 + (c-y)**2)**(3/2))
        term2 = 1/ ((a**2 + (c+y)**2)**(3/2))
        B = ((self.mu * N * Iy * a**2)/2 ) * (term1 + term2)
        return B

    #helmholtz Z field equation
    def zb_field(self, z, Iz):
        a = 26 #radius
        c = (26/2) #distance between /2
        N = 368 #number of turns
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
            Brollx =  ((-np.sin(self.alpha) * np.sin(self.omega*tp)) + (-np.cos(self.alpha) * np.cos(self.gamma)  * np.cos(self.omega*tp))) 
            Brolly =  ((np.cos(self.alpha) * np.sin(self.omega*tp)) + (-np.sin(self.alpha) * np.cos(self.gamma) *  np.cos(self.omega*tp))) 
            if self.freq == 0:
                Brollz = 0
            else:
                Brollz =  np.sin(self.gamma) * np.cos(self.omega*tp)

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
            self.show_axis_rotation(self.ax, 60)
            self.ax.set_xlabel('x') 
            self.ax.set_ylabel('y') 
            self.ax.set_zlabel('z')
            speed = np.sqrt((BX)**2+(BY)**2+(BZ)**2).flatten()
            self.ax.quiver(self.x,self.y,self.z,BX,BY,BZ, color='black',length=1) #norm = colors.LogNorm(vmin=speed.min(), vmax=speed.max() ))#,density = 2)#norm = colors.LogNorm(vmin=speed.min(), vmax=speed.max() ))
            self.ax.scatter(self.x,self.y,self.z, s = 1, c= "b")
            self.canvas.draw()
        

        #self.ax.plot(data, '*-')
        self.figure_ready.emit(1)
    
    
    def show_axis_rotation(self, ax, length):
        #plot rotation axis
        if self.roll == True:
            alpha = self.alpha + np.pi/2
        else:
            alpha = self.alpha
        x = 1 * np.sin(self.gamma) * np.cos(alpha)
        y = 1 * np.sin(self.gamma) * np.sin(alpha)  
        z = 1 * np.cos(self.gamma)
        ax.quiver(0,0,0,x,y,z,color='red',length=length)

    def zero(self):
        self.ax.clear()
        self.ax.scatter(self.x,self.y,self.z, s = 1, c= "b")
        self.ax.set_xlabel('x') 
        self.ax.set_ylabel('y') 
        self.ax.set_zlabel('z')
        #self.figure_ready.emit(self.canvas)
        self.canvas.draw()
    
    def startsim(self):
        self.run_flag = True
        self.timer.start(50)
        
    def stopsim(self):
        self.run_flag = False
        self.zero()
        self.timer.stop()


    #def run_animation(self):
    #    # Set up plot to call animate() function periodically
    #    anim = animation.FuncAnimation(self.fig, self.animate,frames = range(360), interval=10, blit = False)

  



  
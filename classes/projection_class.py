import numpy as np
import cv2
import math

class AxisProjection:
    """
    class for displaying a 3D axis projection on the 2D window,
    displays both rotating field direction and uniform field direction depending on which is being applied
    """
    def __init__(self):
        self.roll = True
        self.gradient = 0



    def projection(self, window, Bx,By,Bz,alpha, gamma, pitch, yaw, roll, w, h, offsetx, offsety,title):
        
        if self.roll == False:
            alpha = alpha - np.pi/2
            

        def spherical_to_cartesian(rho, theta, phi):
            x = rho * np.sin(phi) * np.cos(theta)
            y = rho * np.sin(phi) * np.sin(theta)
            z = rho * np.cos(phi)
            return x, y, z

        scaleline = 1.3
        #projection from rotating field
        if [Bx, By, Bz] != [0,0,0]:
            x2,y2,z2 = Bx*scaleline,By*scaleline,Bz*scaleline
            if self.gradient == 1:
                title = title + "(gradient)"
            else:
                title = title + "(uniform)"
        #projection from constant field
        
        
        else:
            alpha = alpha + np.pi/2
            x2, y2, z2 = spherical_to_cartesian(scaleline, alpha, gamma)
            title = title + "(rotating)"
        
        
        axis_points = np.float32([[0, 0, 0], [scaleline, 0, 0], [0, -scaleline, 0], [0, 0, scaleline], [x2, -y2, z2]])

        # rotate axis
        rotation_vector = np.array([np.radians(pitch), np.radians(yaw), np.radians(roll)], dtype=np.float32) #[x pitch, y roll, z yaw]
        translation_vector = np.array([0, 0, 5], dtype=np.float32)  # position of the coordinate system in relation to the camera.
        # Define the camera intrinsic parameters
        camera_matrix = np.array([[800, 0, w//2], [0, 800, h//2], [0, 0, 1]], dtype=np.float32)
        dist_coeffs = np.zeros((4, 1))

        # Project the 3D points onto the 2D image plane
        image_points, _ = cv2.projectPoints(axis_points, rotation_vector, translation_vector, camera_matrix, dist_coeffs)
        image_points += np.array([offsetx, offsety], dtype=np.float32)

        # Draw the projected axis lines on the image
        origin = tuple(image_points[0].reshape(2).astype(int))
        x_axis = tuple(image_points[1].reshape(2).astype(int))
        y_axis = tuple(image_points[2].reshape(2).astype(int))
        z_axis = tuple(image_points[3].reshape(2).astype(int))
        vec2 = tuple(image_points[4].reshape(2).astype(int))
    
        #draw axis
        cv2.line(window, origin, x_axis, (0, 0, 255), thickness=6)  # Draw x-axis (red)
        cv2.line(window, origin, y_axis, (0, 255, 0), thickness=6)  # Draw y-axis (green)
        cv2.line(window, origin, z_axis, (255, 0, 0), thickness=6)  # Draw z-axis (blue)
        cv2.arrowedLine(window, origin, vec2, (0, 0, 0), thickness=6)  # Draw second vector (white)

        title_loc = (int(w//2+offsetx), int(h//2+offsety*(9.5/10)))
        
        # Add labels to each axis
    
        cv2.putText(window, 'X', x_axis, cv2.FONT_HERSHEY_SIMPLEX,fontScale=1.2, thickness=3, color = (0, 0, 255))
        cv2.putText(window, 'Y', y_axis, cv2.FONT_HERSHEY_SIMPLEX,fontScale=1.2, thickness=3, color = (0, 255, 0))
        cv2.putText(window, 'Z', z_axis, cv2.FONT_HERSHEY_SIMPLEX,fontScale=1.2, thickness=3, color = (255, 0, 0))
        cv2.putText(window, 'B', vec2, cv2.FONT_HERSHEY_SIMPLEX,fontScale=1.2, thickness=3, color = (255, 255, 255))
        cv2.putText(window, title, title_loc, cv2.FONT_HERSHEY_SIMPLEX,fontScale=1.2, thickness=3, color = (0, 255, 255))
        

        return window

    def draw_topview(self, window,Bx,By,Bz,alpha, gamma, window_width, window_height):
        title = "top"
        pitch,yaw,roll = 0,0,0
        offsetx, offsety = int((window_width/2)*(5/10)) , -int((window_height/2)*(6.5/10))#pixel offset from center 
        window = self.projection(window,Bx,By,Bz,alpha,gamma,pitch,yaw,roll,window_width, window_height, offsetx,offsety, title)
        return window
    
    def draw_sideview(self, window, Bx,By,Bz,alpha, gamma,window_width, window_height):
        #side view 
        title = "side"
        pitch,yaw,roll = 90,0,0
        offsetx, offsety = int((window_width/2)*(7.5/10)) , -int((window_height/2)*(6.5/10))#pixel offset from center 
        window = self.projection(window,Bx,By,Bz,alpha,gamma,pitch,yaw,roll,window_width, window_height, offsetx,offsety, title)
        return window




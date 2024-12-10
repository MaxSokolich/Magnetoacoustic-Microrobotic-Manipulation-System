import cv2
import math
import numpy as np 

def generate_circle_coordinates(width, height, node_number):
    center_x = width // 2
    center_y = height // 2
    radius = min(width, height) // 4  # Assume circle fits within a quarter of the image
    
    coordinates = []
    for i in range(node_number):
        theta = 2 * math.pi * i / node_number
        x = center_x + int(radius * math.cos(theta))
        y = center_y + int(radius * math.sin(theta))
        coordinates.append((x, y))
    
    return coordinates

width = 2448
height = 2048
node_number = 2  # Adjust this number to change the number of nodes in the circle
circle_coordinates = generate_circle_coordinates(width, height, node_number)

# Create a blank image
image = np.zeros((height, width, 3), np.uint8)

# Draw the circle
for coord in circle_coordinates:
    cv2.circle(image, coord, 5, (255, 255, 255), -1)  # -1 indicates filled circle

# Display the image
cv2.imshow('Circle', image)
cv2.waitKey(0)
cv2.destroyAllWindows()

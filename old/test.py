import cv2
import numpy as np
import math

# Create a blank image (black background)
width, height = 800, 600
image = np.zeros((height, width, 3), dtype=np.uint8)

# Parameters for the lemniscate (infinity symbol)
a = 200  # Controls the size
center_x, center_y = width // 2, height // 2  # Center of the symbol

# Generate points using parametric equations for a lemniscate
points = []
for t in np.linspace(0, 2 * math.pi, 1000):
    denom = 1 + math.sin(t)**2
    if denom == 0:
        continue  # avoid division by zero
    x = (a * math.cos(t)) / denom
    y = (a * math.cos(t) * math.sin(t)) / denom

    # Convert to image coordinates
    img_x = int(center_x + x)
    img_y = int(center_y + y)
    points.append((img_x, img_y))

# Draw the curve by connecting the points
for i in range(1, len(points)):
    cv2.line(image, points[i - 1], points[i], (0, 255, 0), 2)

# Show and save the image
cv2.imshow('Infinity Symbol', image)
cv2.waitKey(0)
cv2.destroyAllWindows()

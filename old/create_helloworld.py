import cv2
import numpy as np

# Global variables
drawing = False
last_x, last_y = -1, -1
coordinates = []

def draw(event, x, y, flags, param):
    global last_x, last_y, drawing

    if event == cv2.EVENT_LBUTTONDOWN:
        drawing = True
        last_x, last_y = x, y
    elif event == cv2.EVENT_MOUSEMOVE:
        if drawing:
            cv2.line(image, (last_x, last_y), (x, y), (255, 255, 255), 2)
            last_x, last_y = x, y
            coordinates.append((x, y))
    elif event == cv2.EVENT_LBUTTONUP:
        drawing = False

# Create a black image window
image = np.zeros((512, 512, 3), np.uint8)
cv2.namedWindow('Drawing Program')
cv2.setMouseCallback('Drawing Program', draw)

while True:
    cv2.imshow('Drawing Program', image)
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break


# Draw the saved coordinates on a new window
drawing_image = np.zeros((512, 512, 3), np.uint8)
for i in range(1, len(coordinates)):
    cv2.line(drawing_image, coordinates[i-1], coordinates[i], (255, 255, 255), 2)


cv2.imshow("Drawing", drawing_image)
key = cv2.waitKey(1) & 0xFF
if key == ord('q'):
    cv2.destroyAllWindows()

print("Coordinates:", coordinates)




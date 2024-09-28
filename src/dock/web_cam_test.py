import cv2
import sys
import numpy as np

def nothing(x):
    pass

# Initialize the webcam
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Camera open failed!")
    sys.exit()

# Create a window for the trackbars
cv2.namedWindow('HSV_Toolbar')

# Create trackbars for HSV values
cv2.createTrackbar('H_min', 'HSV_Toolbar', 0, 179, nothing)
cv2.createTrackbar('S_min', 'HSV_Toolbar', 0, 255, nothing)
cv2.createTrackbar('V_min', 'HSV_Toolbar', 0, 255, nothing)
cv2.createTrackbar('H_max', 'HSV_Toolbar', 179, 179, nothing)
cv2.createTrackbar('S_max', 'HSV_Toolbar', 255, 255, nothing)
cv2.createTrackbar('V_max', 'HSV_Toolbar', 255, 255, nothing)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Apply Gaussian blur to smoothen the image
    blurred = cv2.GaussianBlur(frame, (5, 5), 0)

    # Convert the frame to HSV
    hsv_frame = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # Get current positions of the trackbars for HSV limits
    h_min = cv2.getTrackbarPos('H_min', 'HSV_Toolbar')
    s_min = cv2.getTrackbarPos('S_min', 'HSV_Toolbar')
    v_min = cv2.getTrackbarPos('V_min', 'HSV_Toolbar')
    h_max = cv2.getTrackbarPos('H_max', 'HSV_Toolbar')
    s_max = cv2.getTrackbarPos('S_max', 'HSV_Toolbar')
    v_max = cv2.getTrackbarPos('V_max', 'HSV_Toolbar')

    # Define HSV range based on trackbar values
    lower_hsv = np.array([h_min, s_min, v_min])
    upper_hsv = np.array([h_max, s_max, v_max])

    # Create a mask with the specified HSV range
    mask = cv2.inRange(hsv_frame, lower_hsv, upper_hsv)

    # Apply the mask to get the result
    result = cv2.bitwise_and(frame, frame, mask=mask)

    # Show the original, mask, and result windows
    cv2.imshow('Original Frame', frame)
    cv2.imshow('HSV Mask', mask)
    cv2.imshow('Filtered Result', result)

    # Break the loop on ESC press
    if cv2.waitKey(10) == 27:
        break

# Release the camera and close all windows
cap.release()
cv2.destroyAllWindows()

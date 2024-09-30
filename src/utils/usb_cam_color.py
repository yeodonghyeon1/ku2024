#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def nothing(x):
    pass

class HSVFilterNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('hsv_filter_node', anonymous=True)

        # Create a window for the trackbars
        cv2.namedWindow('HSV_Toolbar')

        # Create trackbars for HSV values
        cv2.createTrackbar('H_min', 'HSV_Toolbar', 0, 179, nothing)
        cv2.createTrackbar('H_max', 'HSV_Toolbar', 179, 179, nothing)
        cv2.createTrackbar('S_min', 'HSV_Toolbar', 0, 255, nothing)
        cv2.createTrackbar('S_max', 'HSV_Toolbar', 255, 255, nothing)
        cv2.createTrackbar('V_min', 'HSV_Toolbar', 0, 255, nothing)
        cv2.createTrackbar('V_max', 'HSV_Toolbar', 255, 255, nothing)

        # Set the scaling factor for image resizing
        self.scaling_factor = 0.5  # 50% of the original size

        # Initialize the CvBridge class
        self.bridge = CvBridge()

        # Subscribe to the image topic (change the topic name based on your camera)
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)

    def image_callback(self, data):
        try:
            # Convert the ROS Image message to an OpenCV image
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        # Apply Gaussian blur to smoothen the image
        blurred = cv2.GaussianBlur(frame, (5, 5), 0)

        # Convert the frame to HSV
        hsv_frame = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # Get current positions of the trackbars for HSV limits
        h_min = cv2.getTrackbarPos('H_min', 'HSV_Toolbar')
        h_max = cv2.getTrackbarPos('H_max', 'HSV_Toolbar')
        s_min = cv2.getTrackbarPos('S_min', 'HSV_Toolbar')
        s_max = cv2.getTrackbarPos('S_max', 'HSV_Toolbar')
        v_min = cv2.getTrackbarPos('V_min', 'HSV_Toolbar')
        v_max = cv2.getTrackbarPos('V_max', 'HSV_Toolbar')

        # Define HSV range based on trackbar values
        lower_hsv = np.array([h_min, s_min, v_min])
        upper_hsv = np.array([h_max, s_max, v_max])

        # Create a mask with the specified HSV range
        mask = cv2.inRange(hsv_frame, lower_hsv, upper_hsv)

        # Apply the mask to get the result
        result = cv2.bitwise_and(frame, frame, mask=mask)

        # Resize the original, mask, and result images
        frame_resized = cv2.resize(frame, None, fx=self.scaling_factor, fy=self.scaling_factor)
        mask_resized = cv2.resize(mask, None, fx=self.scaling_factor, fy=self.scaling_factor)
        result_resized = cv2.resize(result, None, fx=self.scaling_factor, fy=self.scaling_factor)

        # Convert the mask to a BGR image for display
        mask_resized_bgr = cv2.cvtColor(mask_resized, cv2.COLOR_GRAY2BGR)

        # Stack the resized images horizontally
        combined_display = np.hstack((frame_resized, mask_resized_bgr, result_resized))

        # Show the combined window
        cv2.imshow('Combined View', combined_display)

        # Break the loop on ESC press
        if cv2.waitKey(10) == 27:
            rospy.signal_shutdown("ESC Pressed")
            cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        hsv_filter_node = HSVFilterNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
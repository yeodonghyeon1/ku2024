#!/usr/bin/env python
# -*- coding:utf-8 -*-

"""Subscribe IMU Magnetometer data and calculate boat's heading direction

Notes:
    Heading: -180 (to West) ~ 180 (to East) (deg), Magnetic North is 0 deg
"""

import math
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
import tf
from tf2_msgs.msg import TFMessage
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class HeadingAngle:
    def __init__(self):
        self.pub = rospy.Publisher("/heading", Float64, queue_size=0)
        self.pub2 = rospy.Publisher("/one", TFMessage, queue_size=1)
        self.pub_imu = rospy.Publisher("/imu/frame_trans", Imu, queue_size=1)
        
        rospy.Subscriber("/imu/data", Imu, self.IMU_callback, queue_size=1) #edit
        # rospy.Subscriber("/tf", TFMessage, self.tf_callback, queue_size=1) #edit        
        # rospy.Subscriber("/imu_fix",Float64,self.IMU_Fix_callback,queue_size=1 )


        self.imu = Imu()
        self.orientation_x = 0.0
        self.orientation_y = 0.0
        self.orientation_z = 0.0 
        self.last_heading_angle = 0.0
        self.imu_fix = 0.0

        self.roll = 0
        self.pitch = 0
        self.yaw = 0
    def IMU_callback(self, imu):
        # self.orientation_x = imu.orientation.x 
        # self.orientation_y = imu.orientation.y
        # self.orientation_z = imu.orientation.z 
        orientation_q = imu.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion (orientation_list)
        
        self.imu = imu
        self.imu.header.frame_id = "base_link"
        
        self.pub_imu.publish(self.imu)
        # self.imu = imu.frame    

    def tf_callback(self, data):
        self.orientation_x = data.transforms[0].transform.rotation.x 
        self.orientation_y = data.transforms[0].transform.rotation.y
        self.orientation_z = data.transforms[0].transform.rotation.z


    def IMU_Fix_callback(self,fix):
        self.imu_fix =  fix.data
        

    def getRotationMatrix(self, R, I, gravity, geomagnetic):
        Ax = gravity[0]
        Ay = gravity[1]
        Az = gravity[2]

        normsqA = Ax * Ax + Ay * Ay + Az * Az
        g = 9.81
        freeFallGravitySquared = 0.01 * g * g

        if normsqA < freeFallGravitySquared:
            # gravity less than 10% of normal value
            return False

        Ex = geomagnetic[0]
        Ey = geomagnetic[1]
        Ez = geomagnetic[2]
        Hx = Ey * Az - Ez * Ay
        Hy = Ez * Ax - Ex * Az
        Hz = Ex * Ay - Ey * Ax
        normH = math.sqrt(Hx * Hx + Hy * Hy + Hz * Hz)

        if normH < 0.1:
            # device is close to free fall (or in space?), or close to
            # magnetic north pole. Typical values are  > 100.
            return False

        invH = 1.0 / normH
        Hx *= invH
        Hy *= invH
        Hz *= invH

        invA = 1.0 / math.sqrt(Ax * Ax + Ay * Ay + Az * Az)
        Ax *= invA
        Ay *= invA
        Az *= invA

        Mx = Ay * Hz - Az * Hy
        My = Az * Hx - Ax * Hz
        Mz = Ax * Hy - Ay * Hx
        if R != None:
            if len(R) == 9:
                R[0] = Hx
                R[1] = Hy
                R[2] = Hz
                R[3] = Mx
                R[4] = My
                R[5] = Mz
                R[6] = Ax
                R[7] = Ay
                R[8] = Az
            elif len(R) == 16:
                R[0] = Hx
                R[1] = Hy
                R[2] = Hz
                R[3] = 0
                R[4] = Mx
                R[5] = My
                R[6] = Mz
                R[7] = 0
                R[8] = Ax
                R[9] = Ay
                R[10] = Az
                R[11] = 0
                R[12] = 0
                R[13] = 0
                R[14] = 0
                R[15] = 1

        if I != None:
            # compute the inclination matrix by projecting the geomagnetic
            # vector onto the Z (gravity) and X (horizontal component
            # of geomagnetic vector) axes.
            invE = 1.0 / math.sqrt(Ex * Ex + Ey * Ey + Ez * Ez)
            c = (Ex * Mx + Ey * My + Ez * Mz) * invE
            s = (Ex * Ax + Ey * Ay + Ez * Az) * invE

            if len(I) == 9:
                I[0] = 1
                I[1] = 0
                I[2] = 0
                I[3] = 0
                I[4] = c
                I[5] = s
                I[6] = 0
                I[7] = -s
                I[8] = c
            elif len(I) == 16:
                I[0] = 1
                I[1] = 0
                I[2] = 0
                I[4] = 0
                I[5] = c
                I[6] = s
                I[8] = 0
                I[9] = -s
                I[10] = c
                I[3] = I[7] = I[11] = I[12] = I[13] = I[14] = 0
                I[15] = 1

        return True

    def getOrientation(self, R, values):
        """
        4x4 (length=16) case:
            R[ 0]   R[ 1]   R[ 2]   0
            R[ 4]   R[ 5]   R[ 6]   0
            R[ 8]   R[ 9]   R[10]   0
            0       0       0    1

        3x3 (length=9) case:
            R[ 0]   R[ 1]   R[ 2]
            R[ 3]   R[ 4]   R[ 5]
            R[ 6]   R[ 7]   R[ 8]
        """
        if len(R) == 9:
            values[0] = math.atan2(R[1], R[4])
            values[1] = math.asin(-R[7])
            values[2] = math.atan2(-R[6], R[8])
        else:
            values[0] = math.atan2(R[1], R[5])
            values[1] = math.asin(-R[9])
            values[2] = math.atan2(-R[8], R[10])

        return values

    def calculate_heading_angle(self):
        gravity = [0, 0, -9.8]
        geomagnetic = [
            self.orientation_x * 10**6,
            self.orientation_y * 10**6,
            self.orientation_z * 10**6,
        ]
        R = [0] * 9
        orientation = [0] * 3
        success = self.getRotationMatrix(R, None, gravity, geomagnetic)
        if success is True:
            self.getOrientation(R, orientation)
            heading_angle = orientation[0] * (180 / math.pi)
            self.last_heading_angle = heading_angle
        else:
            heading_angle = self.last_heading_angle

        return heading_angle


def main():
    rospy.init_node("heading_calculator", anonymous=False)
    rate = rospy.Rate(20)  # 10 Hz
    heading = HeadingAngle()
 
    while not rospy.is_shutdown():
        # heading_angle = heading.calculate_heading_angle()
        # heading_angle = heading.orientation_z * (180/math.pi)
        # heading_angle = heading.orientation_z * 180
        heading_angle = heading.yaw * (180/math.pi)
        heading.pub.publish(heading_angle)
        # heading.pub.publish(round(heading_angle, -1))

        rate.sleep()

if __name__ == "__main__":
    main()

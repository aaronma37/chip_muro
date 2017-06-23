#!/usr/bin/env python

import rospy

import struct
import time

from std_msgs.msg import String
from std_msgs.msg import Float32
from mouse_localization.msg import mouse_event1
#
# from geometry_msgs.msg import Pose2D
from math import *
# from numpy import matrix
# from numpy import absolute

# all mice inputs
# miceInputs = open( "/dev/input/mice", "rb" )
# miceData = open("miceData.csv", "wb")

# each mouse input
mouseInputs1 = open( "/dev/input/mouse1", "rb" )

# data files for each mouse output
mouseData1 = open("mouseData1.csv", "wb")

# numerical integrator
def fwdEuler(x,v,h1):
    return x + h1 * v

# get mouse pose from dual mouse sensors
def calc_mouse_pose(mousePose, mouseVel1, mouseVel2, h1):
    # distance between sensors
    LENGTH = 0

    # robot rotation speed
    rRot = ( mouseVel2[0] - mouseVel1[0] ) / LENGTH

    # update orientation
    rPose[2] = fwdEuler(mousePose[2], rRot, h1)

    # inertial velocity of robot center in terms of global coordinates
    rVel[0] = cos(mousePose[2]) * mouseVel1[0] \
                    - sin(mousePose[2]) * mouseVel1[1]
    rVel[1] = cos(mousePose[2]) * mouseVel1[0] \
                    + sin(mousePose[2]) * mouseVel1[1]

    # position in terms of global coordinates
    rPose[0] = fwdEuler(mousePose[0], rVel[0], h1)
    rPose[1] = fwdEuler(mousePose[1], rVel[0], h1)

    return rPose, rVel



# works with one mouse, cannot tell two mice apart
# :TODO: ensure that initial values are preserved, ensure that time step is minimal after mouse is stationary for a while
# :TODO: choose appropriate MAX_STEP_SIZE
# :TODO: modify to read data from two mice
# :TODO: send velX1 and velY1 for each mouse to calc_mouse_pose (call within loop)
def writeMouseData(mFile1, mData1):
    PI = 3.14159265
    LENGTH = 0
    MAX_STEP_SIZE = 0.0846 # Prevent delays from affecting integration
    ANGLE_BETWEEN_SENSORS=PI/4
    INIT_TIME = time.time() #

    # initialize at zero time
    t10=0

    # initialize other sensor positionlocated away from first sensor
    x10=LENGTH * cos( ANGLE_BETWEEN_SENSORS )
    y10=LENGTH * sin( ANGLE_BETWEEN_SENSORS )

    # column headers
    mData1.write("time, timestep, velX1, velY1, x1, y1\n")

    pub = rospy.Publisher('mouse_publisher1', Float32, queue_size=10)
    rospy.init_node('mouse_publisher1', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        buffer1 = mFile1.read(3)

        velX1,velY1 = struct.unpack( "bb", buffer1[1:] )

        # time stamp, step size
        t1 = time.time() - INIT_TIME
        h1 = t1 - t10

        # ignore when mouse is stationary
        if ( h1 > MAX_STEP_SIZE ):
            print ("restart measurement\n")
            print("t1: %f, h1: %f, velX1: %f, velY1: %f, " % (t1, MAX_STEP_SIZE, velX1, velY1))

            mData1.write("%f, %f, %f, %f, " \
                % (t1, MAX_STEP_SIZE, velX1, velY1))

            x1 = fwdEuler(x10,velX1,MAX_STEP_SIZE)
            y1 = fwdEuler(y10,velY1,MAX_STEP_SIZE)
        else:
            print("t1: %f, h1: %f, velX1: %f, velY1: %f, " % (t1, h1, velX1, velY1))


            x1 = fwdEuler(x10,velX1,h1)
            y1 = fwdEuler(y10,velY1,h1)

            print("t1: %f, h1: %f, velX1: %f, velY1: %f, x1: %f, y1: %f\n" \
                % (t1, h1, velX1, velY1, x1, y1))

        print("x1: %f, y1: %f\n" \
                % (x1, y1))

        mData1.write("%f, %f\n" % (x1,y1))

        # initialize next time step
        x10 = x1
        y10 = y1

        t10 = t1 # can be later than previous t

        mouse_msg = Event1()
        mouse_msg.t1 = t1
        mouse_msg.h1 = h1
        mouse_msg.velX1 = velX1
        mouse_msg.velY1 = velY1
        mouse_msg.x1 = x1
        mouse_msg.y1 = y1

        rospy.loginfo(mouse_msg)
        pub.publish(mouse_msg)
        rate.sleep()

    # close files
    mData1.close()
    mFile1.close()


# def mouse_publisher1():
#     pub = rospy.Publisher('mouse_publisher1', mouseEvents1, queue_size=10)
#     rospy.init_node('mouse_publisher1', anonymous=True)
#     rate = rospy.Rate(10) # 10hz
#     while not rospy.is_shutdown():
#         hello_str = "hello world %s" % rospy.get_time()
#         rospy.loginfo(hello_str)
#         pub.publish(hello_str)
#         rate.sleep()
#
# def mouse_publisher1(wheel_output):
#         if not rospy.is_shutdown():
#                 pwm_msg = Speeds()
#                 pwm_msg.s1 = wheel_output[0].round(0)
#                 pwm_msg.s2 = wheel_output[1].round(0)
#                 pwm_msg.s3 = wheel_output[2].round(0)
#                 pwm_pub = rospy.Publisher('speeds', Speeds, queue_size=10)
#                 pwm_pub.publish(pwm_msg)
#                 rospy.loginfo("\nPublished to topic /speeds:\nMotor 1: %f\nMotor 2: %f\nMotor 3: %f\n",\
#                                                                                                                                  wheel_output[0].round(0),\
#                                                                                                                                  wheel_output[1].round(0),\
#                                                                                                                                  wheel_output[2].round(0))

def main():
    writeMouseData(mouseInputs1, mouseData1)


if __name__ == '__main__':
        main()

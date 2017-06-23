#!/usr/bin/env python

import rospy

import struct
import time

from std_msgs.msg import String
from std_msgs.msg import Float32
from mouse_localization.msg import mouse_event0
#
# from geometry_msgs.msg import Pose2D
from math import *
# from numpy import matrix
# from numpy import absolute

# all mice inputs
# miceInputs = open( "/dev/input/mice", "rb" )
# miceData = open("miceData.csv", "wb")

# each mouse input
mouseInputs0 = open( "/dev/input/mouse0", "rb" )

# data files for each mouse output
mouseData0 = open("mouseData0.csv", "wb")

# numerical integrator
def fwdEuler(x,v,h0):
    return x + h0 * v

# get mouse pose from dual mouse sensors
def calc_mouse_pose(mousePose, mouseVel1, mouseVel2, h0):
    # distance between sensors
    LENGTH = 0

    # robot rotation speed
    rRot = ( mouseVel2[0] - mouseVel1[0] ) / LENGTH

    # update orientation
    rPose[2] = fwdEuler(mousePose[2], rRot, h0)

    # inertial velocity of robot center in terms of global coordinates
    rVel[0] = cos(mousePose[2]) * mouseVel1[0] \
                    - sin(mousePose[2]) * mouseVel1[1]
    rVel[1] = cos(mousePose[2]) * mouseVel1[0] \
                    + sin(mousePose[2]) * mouseVel1[1]

    # position in terms of global coordinates
    rPose[0] = fwdEuler(mousePose[0], rVel[0], h0)
    rPose[1] = fwdEuler(mousePose[1], rVel[0], h0)

    return rPose, rVel



# works with one mouse, cannot tell two mice apart
# :TODO: ensure that initial values are preserved, ensure that time step is minimal after mouse is stationary for a while
# :TODO: choose appropriate MAX_STEP_SIZE
# :TODO: modify to read data from two mice
# :TODO: send velX0 and velY0 for each mouse to calc_mouse_pose (call within loop)
def writeMouseData(mFile0, mData0):
    PI = 3.14159265
    LENGTH = 0
    MAX_STEP_SIZE = 0.0846 # Prevent delays from affecting integration
    ANGLE_BETWEEN_SENSORS=PI/4
    INIT_TIME = time.time() #

    # initialize at zero time
    t00=0

    # initialize at zero position (assuming this sensor is located at robot reference point)
    x00=0
    y00=0

    # column headers
    mData0.write("time, timestep, velX0, velY0, x0, y0\n")

    pub = rospy.Publisher('mouse_publisher0', Event0, queue_size=10)
    rospy.init_node('mouse_publisher0', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        buffer0 = mFile0.read(3)

        velX0,velY0 = struct.unpack( "bb", buffer0[1:] )

        # time stamp, step size
        t0 = time.time() - INIT_TIME
        h0 = t0 - t00

        # ignore when mouse is stationary
        if ( h0 > MAX_STEP_SIZE ):
            print ("restart measurement\n")
            print("t0: %f, h0: %f, velX0: %f, velY0: %f, " % (t0, MAX_STEP_SIZE, velX0, velY0))

            mData0.write("%f, %f, %f, %f, " \
                % (t0, MAX_STEP_SIZE, velX0, velY0))

            x0 = fwdEuler(x00,velX0,MAX_STEP_SIZE)
            y0 = fwdEuler(y00,velY0,MAX_STEP_SIZE)
        else:
            print("t0: %f, h0: %f, velX0: %f, velY0: %f, " % (t0, h0, velX0, velY0))


            x0 = fwdEuler(x00,velX0,h0)
            y0 = fwdEuler(y00,velY0,h0)

            print("t0: %f, h0: %f, velX0: %f, velY0: %f, x0: %f, y0: %f\n" \
                % (t0, h0, velX0, velY0, x0, y0))

        print("x0: %f, y0: %f\n" \
                % (x0, y0))


        # mData0.write("%f, %f\n" % (x0,y0))

        # initialize next time step
        x00 = x0
        y00 = y0

        t00 = t0 # can be later than previous t

        mouse_msg = Event0()
        mouse_msg.t0 = t0
        mouse_msg.h0 = h0
        mouse_msg.velX0 = velX0
        mouse_msg.velY0 = velY0
        mouse_msg.x0 = x0
        mouse_msg.y0 = y0

        rospy.loginfo(mouse_msg)
        pub.publish(mouse_msg)
        rate.sleep()

    # close files
    mData0.close()
    mFile0.close()


# def mouse_publisher0():
#     pub = rospy.Publisher('mouse_publisher0', mouseEvents0, queue_size=10)
#     rospy.init_node('mouse_publisher0', anonymous=True)
#     rate = rospy.Rate(10) # 10hz
#     while not rospy.is_shutdown():
#         hello_str = "hello world %s" % rospy.get_time()
#         rospy.loginfo(hello_str)
#         pub.publish(hello_str)
#         rate.sleep()
#
# def mouse_publisher0(wheel_output):
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
    writeMouseData(mouseInputs0, mouseData0)


if __name__ == '__main__':
        main()

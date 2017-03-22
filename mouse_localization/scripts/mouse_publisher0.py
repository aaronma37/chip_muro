#!/usr/bin/env python

import rospy

import struct
import time

from std_msgs.msg import String
from std_msgs.msg import Float32
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
def fwdEuler(x,v,h):
    return x + h * v

# get mouse pose from dual mouse sensors
def calc_mouse_pose(mousePose, mouseVel1, mouseVel2, h):
    # distance between sensors
    LENGTH = 0

    # robot rotation speed
    rRot = ( mouseVel2[0] - mouseVel1[0] ) / LENGTH

    # update orientation
    rPose[2] = fwdEuler(mousePose[2], rRot, h)

    # inertial velocity of robot center in terms of global coordinates
    rVel[0] = cos(mousePose[2]) * mouseVel1[0] \
                    - sin(mousePose[2]) * mouseVel1[1]
    rVel[1] = cos(mousePose[2]) * mouseVel1[0] \
                    + sin(mousePose[2]) * mouseVel1[1]

    # position in terms of global coordinates
    rPose[0] = fwdEuler(mousePose[0], rVel[0], h)
    rPose[1] = fwdEuler(mousePose[1], rVel[0], h)

    return rPose, rVel



# works with one mouse, cannot tell two mice apart
# :TODO: ensure that initial values are preserved, ensure that time step is minimal after mouse is stationary for a while
# :TODO: choose appropriate MAX_STEP_SIZE
# :TODO: modify to read data from two mice
# :TODO: send velX and velY for each mouse to calc_mouse_pose (call within loop)
def writeMouseData(mFile0, mData0):
    PI = 3.14159265
    LENGTH = 0
    MAX_STEP_SIZE = 0.0846 # Prevent delays from affecting integration
    ANGLE_BETWEEN_SENSORS=PI/4
    INIT_TIME = time.time() #

    # initialize at zero time
    t0=0

    # initialize at zero position (assuming this sensor is located at robot reference point)
    x00=0
    y00=0

    # column headers
    mData0.write("time, timestep, velX, velY, x, y\n")

    pub = rospy.Publisher('mouse_publisher', Float32, queue_size=10)
    rospy.init_node('mouse_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        buffer0 = mFile0.read(3)

        velX0,velY0 = struct.unpack( "bb", buffer0[1:] )

        # time stamp, step size
        t = time.time() - INIT_TIME
        h = t - t0

        # ignore when mouse is stationary
        if ( h > MAX_STEP_SIZE ):
            #print ("restart measurement\n")
            #print("t: %f, h: %f, velX0: %f, velY0: %f, " % (t, MAX_STEP_SIZE, velX0, velY0))

            mData0.write("%f, %f, %f, %f, " \
                % (t, MAX_STEP_SIZE, velX0, velY0))

            x0 = fwdEuler(x00,velX0,MAX_STEP_SIZE)
            y0 = fwdEuler(y00,velY0,MAX_STEP_SIZE)
        else:
            #print("t: %f, h: %f, velX0: %f, velY0: %f, " % (t, h, velX0, velY0))


            x0 = fwdEuler(x00,velX0,h)
            y0 = fwdEuler(y00,velY0,h)

            # #print("t: %f, h: %f, velX: %f, velY: %f, x: %f, y: %f\n" \
                # % (t, h, velX, velY, x, y))
        #print("x0: %f, y0: %f\n" \
                #% (x0, y0))

        mData0.write("%f, %f\n" % (x0,y0))

        # initialize next time step
        x00 = x0
        y00 = y0

        t0 = t # can be later than previous t

        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

    # close files
    mData0.close()
    mFile0.close()


def mouse_publisher():
    pub = rospy.Publisher('mouse_publisher', mouseEvent0, queue_size=10)
    rospy.init_node('mouse_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()
#
# def mouse_publisher(wheel_output):
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




#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
from chip_controller.msg import Speeds

from geometry_msgs.msg import Pose2D
from math import *
from numpy import matrix
from numpy import absolute

target_pose = Pose2D()
robot_pose = Pose2D()

def target_pose_CB(pose):
        global target_pose
        target_pose = pose
        rospy.loginfo("\ntarget:\nx: %f\ny: %f\ntheta: %f\n", pose.x, pose.y, pose.theta)

def robot_pose_CB(pose):
        global robot_pose
        robot_pose = pose
        rospy.loginfo("\nrobot:\nx: %f\ny: %f\ntheta: %f\n", pose.x, pose.y, pose.theta)

        calc_pose_vector()

def calc_pose_vector():
        pose_vector = Pose2D()
        pose_vector = subtract_pose(target_pose,robot_pose)
        phi_init = robot_pose.theta;
        desired_xyphi = matrix([[pose_vector.x],[pose_vector.y],[0]])
        rospy.loginfo("\nDesired xyphi_dot:\nx_dot: %f\ny_dot: %f\nphi_dot: %f\n",\
                                                                                                                         desired_xyphi[0].round(0),\
                                                                                                                         desired_xyphi[1].round(0),\
                                                                                                                         desired_xyphi[2].round(0))
        wheel_output = calc_wheel_contribution(desired_xyphi,phi_init)
        motor_publisher(wheel_output)

def motor_publisher(wheel_output):
        if not rospy.is_shutdown():
                pwm_msg = Speeds()
                pwm_msg.s1 = wheel_output[0].round(0)
                pwm_msg.s2 = wheel_output[1].round(0)
                pwm_msg.s3 = wheel_output[2].round(0)
                pwm_pub = rospy.Publisher('speeds', Speeds, queue_size=10)
                pwm_pub.publish(pwm_msg)
                rospy.loginfo("\nPublished to topic /speeds:\nMotor 1: %f\nMotor 2: %f\nMotor 3: %f\n",\
                                                                                                                                 wheel_output[0].round(0),\
                                                                                                                                 wheel_output[1].round(0),\
                                                                                                                                 wheel_output[2].round(0))

def subtract_pose(t_pose,r_pose):
        diff_pose = Pose2D()
        diff_pose.x = t_pose.x - r_pose.x
        diff_pose.y = t_pose.y - r_pose.y
        diff_pose.theta = t_pose.theta - r_pose.theta
        return diff_pose

def calc_wheel_contribution(desired_xyphi,phi_init):
        d = pi/6;                #angle of front wheels from xaxis
        L = 74*10^-3;    #length from center to wheel
        #phi_init = 0;    #angle orientation of robot from 1 2 y pos
        R = 25*10^-3;
        max_pwm = 100;
        A_ss = 1/R*matrix([[-sin(d-phi_init), -cos(d-phi_init), L],\
                                           [-sin(d+phi_init), cos(d+phi_init), L],\
                                           [ cos(phi_init), sin(phi_init), L]])
        input_ss = desired_xyphi;
        output_ss = A_ss*input_ss
        abs_ss = absolute(output_ss)
        val_max = abs_ss.max()
        scale_factor = max_pwm/val_max
        norm_ss = scale_factor*output_ss
        print output_ss
        print scale_factor
        return(norm_ss)

def main():

        rospy.init_node('motor_handler', anonymous=True)

        rospy.Subscriber("top_cam/robot_pose", Pose2D, robot_pose_CB)
        rospy.Subscriber("top_cam/target_pose", Pose2D, target_pose_CB)

        rospy.spin()


if __name__ == '__main__':
        main()
        # test_input = matrix([[0],[1],[pi/3]])
        # calc_wheel_contribution(test_input)












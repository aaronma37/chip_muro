  GNU nano 2.2.6                                                                                File: pwm.py                                                                                                                                                                       

#!/usr/bin/env python

import rospy
from chip_driver.msg import Speeds
# from std_msgs.msg import Int32 
import CHIP_IO.SOFTPWM as SPWM
import CHIP_IO.Utilities as UT

def callback(cmd):
    if cmd.s1 > 0:
        SPWM.set_duty_cycle("CSID0", cmd.s1)
        SPWM.set_duty_cycle("CSID1", 0)
        print "pin 0 set to ", cmd.s1
        print "pin 1 set to ", 0
    else:
        SPWM.set_duty_cycle("CSID0", 0)
        SPWM.set_duty_cycle("CSID1", -cmd.s1)
        print "pin 0 set to ", 0
        print "pin 1 set to ", -cmd.s1

    if cmd.s2 > 0:
        SPWM.set_duty_cycle("CSID2", cmd.s2)
        SPWM.set_duty_cycle("CSID3", 0)
        print "pin 2 set to ", cmd.s2
        print "pin 3 set to ", 0
    else:
        SPWM.set_duty_cycle("CSID2", 0)
        SPWM.set_duty_cycle("CSID3", -cmd.s2)
        print "pin 2 set to ", 0
        print "pin 3 set to ", -cmd.s2

    if cmd.s3 > 0:
        SPWM.set_duty_cycle("CSID4", cmd.s3)
        SPWM.set_duty_cycle("CSID5", 0)
        print "pin 4 set to ", cmd.s3
        print "pin 5 set to ", 0
    else:
        SPWM.set_duty_cycle("CSID4", 0)
        SPWM.set_duty_cycle("CSID5", -cmd.s3)
        print "pin 4 set to ", 0
        print "pin 5 set to ", -cmd.s3


def init_pins():
    SPWM.start("CSID0", 0, 100)
    SPWM.start("CSID1", 0, 100)
    SPWM.start("CSID2", 0, 100)
    SPWM.start("CSID3", 0, 100)
    SPWM.start("CSID4", 0, 100)
    SPWM.start("CSID5", 0, 100)

def pwm():

    rospy.init_node('pwm')
    rospy.Subscriber('speeds', Speeds, callback)

    while not rospy.is_shutdown():
        rospy.spin()

    SPWM.cleanup()
    UT.unexport_all()


if __name__ == '__main__':
    init_pins()
    pwm()








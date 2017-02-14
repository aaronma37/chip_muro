#!/usr/bin/env python
# USAGE
# python match.py --template cod_logo.png --images images

# import the necessary packages
import numpy as np
import rospy
from std_msgs.msg import String
import time
import math
import sys
import glob
import cv2

import tf
import tf2_ros

from cv2 import __version__
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped
from tf2_msgs.msg import TFMessage


#Define a global localization class

template = np.zeros((480,640,3), np.uint8)


#Define a global localization class
class GL:

###DEFINE OUR CLASS
	def __init__(self):
		self.bridge = CvBridge()

		self.active=[]
		for i in range(50):
			self.active.append(False)

		self.tf_list = TFMessage()
		for i in range(50):
			self.tf_list.transforms.append(TransformStamped())

		self.voronoi_list = TFMessage()
		for i in range(50):
			self.voronoi_list.transforms.append(TransformStamped())


		self.sub =rospy.Subscriber('/PoseEstimationC', TFMessage, self.pose_cb)
		self.sub =rospy.Subscriber('/VList', TFMessage, self.voronoi_cb)

###GATHER TURTLEBOT LOCATIONS
	def pose_cb(self,data):
		for i in range(len(data.transforms)):
			self.tf_list.transforms[i].transform.translation=data.transforms[i].translation
			if self.tf_list.transforms[i].transform.translation.x is not 0:
				self.active[i]=True
			else:
				self.active[i]=False

###GATHER VORONOI POINTS
	def voronoi_cb(self,data):
		for i in range(len(self.voronoi_list.transforms)):
				if data.transforms[i].transforms.rotation.z is 1:
					self.voronoi_list.transforms[i].transform.rotation.z=1
					self.voronoi_list.transforms[i].transform=data.transforms[i].transform
				else:
					self.voronoi_list.transforms[i].transform.rotation.z=0


###REFRESH THE IMAGES
	def refresh(self):
		while not rospy.is_shutdown():
			template = np.zeros((480,640,3), np.uint8)
			cv2.rectangle(template, (0, 0), (480, 640), (0,0,0), -1)

			for i in range(len(self.voronoi_list.transforms)):
				if self.voronoi_list.transforms[i].transform.rotation.z is 1:
					cv2.line(template, (int(self.voronoi_list.transforms[i].transform.translation.x), int(self.voronoi_list.transforms[i].transform.translation.y)), (int(self.voronoi_list.transforms[i].transform.rotation.x), int(self.voronoi_list.transforms[i].transform.rotation.y)), (255,255,0), 5)

			for i in range(len(self.tf_list.transforms)):
				if self.active[i] is True:
					cv2.rectangle(template, (int(self.tf_list.transforms[i].transform.translation.x-10), int(self.tf_list.transforms[i].transform.translation.y-10)), (int(self.tf_list.transforms[i].transform.translation.x+10), int(self.tf_list.transforms[i].transform.translation.y+10)), (255,0,0), -1)

			cv2.imshow("Template", template)
			cv2.waitKey(100)




###MAIN
def main(args):
	rospy.init_node('image_converter', anonymous=True)

	gl = GL()
	gl.refresh()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")


	cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)



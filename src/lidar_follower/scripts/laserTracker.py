#!/usr/bin/env python
# test mail: chutter@uos.de

import rospy
import thread, threading
import time
import math
import numpy as np
from sensor_msgs.msg import Joy, LaserScan
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3, PointStamped
from lidar_follower.msg import position as PositionMsg
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry

		
class laserTracker:
	def __init__(self):
		self.lastScan = None
		self.winSize = rospy.get_param('~winSize')
		self.deltaDist = rospy.get_param('~deltaDist')
		self.scanSubscriber = rospy.Subscriber('/scan', LaserScan, self.registerScan)
		self.positionPublisher = rospy.Publisher('/object_tracker/current_position', PositionMsg,queue_size=3)
		self.infoPublisher = rospy.Publisher('/object_tracker/info', String, queue_size=3)

		self.robot_pose = {'x': 0, 'y': 0, 'theta': 0}
		self.pose_sub = rospy.Subscriber('/robot_pose', Odometry, self.getSelfPoseCallback)

		self.self_point_pub = rospy.Publisher('/selfPoint', PointStamped, queue_size=1)
		self.self_point_pub_ = PointStamped()
		self.self_point_pub_.header.frame_id = 'map'
		self.self_point_pub_.header.stamp = rospy.Time()
		self.self_point_pub_.point.x, self.self_point_pub_.point.y, self.self_point_pub_.point.z = 0, 0, 0

		self.track_point_pub = rospy.Publisher('/trackPoint', PointStamped, queue_size=1)
		self.track_point_pub_ = PointStamped()
		self.track_point_pub_.header.frame_id = 'map'
		self.track_point_pub_.header.stamp = rospy.Time()
		self.track_point_pub_.point.x, self.track_point_pub_.point.y, self.track_point_pub_.point.z = 0, 0, 0
		self.sign = 1

	def getSelfPoseCallback(self, data):
		self.robot_pose['x'] = data.pose.pose.position.x
		self.robot_pose['y'] = data.pose.pose.position.y
		qx = data.pose.pose.orientation.x
		qy = data.pose.pose.orientation.y
		qz = data.pose.pose.orientation.z
		qw = data.pose.pose.orientation.w
		rad = euler_from_quaternion((qx,qy,qz,qw))
		self.robot_pose['theta'] = rad[2] # rad

		self.self_point_pub_.point.x = self.robot_pose['x']
		self.self_point_pub_.point.y = self.robot_pose['y']
		self.self_point_pub.publish(self.self_point_pub_)


	def registerScan(self, scan_data):
		# registers laser scan and publishes position of closest object (or point rather)
		ranges = np.array(scan_data.ranges)
		# sort by distance to check from closer to further away points if they might be something real
		sortedIndices = np.argsort(ranges)
		
		minDistanceID = None
		minDistance = float('inf')		

		if not self.lastScan is None:
			# if we already have a last scan to compare to:
			for i in sortedIndices:
				# check all distance measurements starting from the closest one
				tempMinDistance   = ranges[i]
				
				# now we check if this might be noise:
				# get a window. in it we will check if there has been a scan with similar distance
				# in the last scan within that window
				
				# we kneed to clip the window so we don't have an index out of bounds
				windowIndex = np.clip([i-self.winSize, i+self.winSize+1], 0, len(self.lastScan))
				window = self.lastScan[windowIndex[0]:windowIndex[1]]

				with np.errstate(invalid='ignore'):
					# check if any of the scans in the window (in the last scan) has a distance close enough to the current one
					if(np.any(abs(window-tempMinDistance) <= self.deltaDist)):
					# this will also be false for all tempMinDistance = NaN or inf

						# we found a plausible distance
						minDistanceID = i
						minDistance = ranges[minDistanceID]
						break # at least one point was equally close
						# so we found a valid minimum and can stop the loop
			
		self.lastScan=ranges	
		
		#catches no scan, no minimum found, minimum is actually inf
		if minDistance > scan_data.range_max:
			#means we did not really find a plausible object
			
			# publish warning that we did not find anything
			rospy.logwarn('laser no object found')
			self.infoPublisher.publish(String('laser:nothing found'))

			self.track_point_pub_.point.x, self.track_point_pub_.point.y, self.track_point_pub_.point.z = 0, 0, 0
			self.track_point_pub.publish(self.track_point_pub_)
		
		else:
			# calculate angle of the objects location. 0 is straight ahead
			minDistanceAngle = scan_data.angle_min + minDistanceID * scan_data.angle_increment
			# here we only have an x angle, so the y is set arbitrarily
			self.positionPublisher.publish(PositionMsg(minDistanceAngle, 42, minDistance))

			delta_theta = self.robot_pose['theta'] + self.sign * minDistanceAngle
			self.track_point_pub_.point.x = self.robot_pose['x'] + math.cos(delta_theta) * minDistance
			self.track_point_pub_.point.y = self.robot_pose['x'] + math.sin(delta_theta) * minDistance
			self.track_point_pub.publish(self.track_point_pub_)
			

if __name__ == '__main__':
	print('starting')
	rospy.init_node('laser_tracker')
	tracker = laserTracker()
	print('seems to do something')
	try:
		rospy.spin()
	except rospy.ROSInterruptException:
		print('exception')

# sensor_msgs/LaserScan Message
# File: sensor_msgs/LaserScan.msg
# Raw Message Definition
# # Single scan from a planar laser range-finder
# #
# # If you have another ranging device with different behavior (e.g. a sonar
# # array), please find or create a different message, since applications
# # will make fairly laser-specific assumptions about this data

# Header header            # timestamp in the header is the acquisition time of 
#                          # the first ray in the scan.
#                          #
#                          # in frame frame_id, angles are measured around 
#                          # the positive Z axis (counterclockwise, if Z is up)
#                          # with zero angle being forward along the x axis
                         
# float32 angle_min        # start angle of the scan [rad]
# float32 angle_max        # end angle of the scan [rad]
# float32 angle_increment  # angular distance between measurements [rad]

# float32 time_increment   # time between measurements [seconds] - if your scanner
#                          # is moving, this will be used in interpolating position
#                          # of 3d points
# float32 scan_time        # time between scans [seconds]

# float32 range_min        # minimum range value [m]
# float32 range_max        # maximum range value [m]

# float32[] ranges         # range data [m] (Note: values < range_min or > range_max should be discarded)
# float32[] intensities    # intensity data [device-specific units].  If your
#                          # device does not provide intensities, please leave
#                          # the array empty.



#!/usr/bin/env python

import rospy
import sensor_msgs.point_cloud2
from sensor_msgs.msg import PointCloud2

pub = None

class z_axix_process:

	def __init__(self):
		self.pc2 = None
	
	def process(self, pc2):
		self.pc2 = pc2
		self.z_process()

	def z_process(self):
		points = sensor_msgs.point_cloud2.read_points(self.pc2)
		res_points = []
		points_list = [(p[0], p[1], p[2], p[3]) for p in points]
		for p in points_list:
			res_points.append((p[0], p[1], 0, p[3]))
		self.pc2 = sensor_msgs.point_cloud2.create_cloud(self.pc2.header, self.pc2.fields, res_points)
		# for p in points_list:
		# 	rospy.loginfo("%f %f %f %d", p[0],p[1],p[2],p[3])

	def genrate_res(self):
		return self.pc2

        
def z_projection(data):
	z_process_Ins = z_axix_process()
	z_process_Ins.process(data)
	pub.publish(z_process_Ins.genrate_res())


def listener():
    rospy.init_node('z_projection')
    rospy.Subscriber('xyzi_filt_out', PointCloud2, z_projection)
    rospy.spin()


if __name__ == '__main__':
    pub = rospy.Publisher('/z_projection', PointCloud2, queue_size=10)
    listener()

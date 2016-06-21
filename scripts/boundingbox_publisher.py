#!/usr/bin/env python

from std_srvs.srv import *
import rospy
import commands
import tf
from tf import transformations
from jsk_recognition_msgs.msg import *
from geometry_msgs.msg import *
import numpy

def box_pub_server():
    pub = rospy.Publisher('/attention_clipper/input/box_array', BoundingBoxArray, queue_size=10)
    rospy.init_node('box_pub', anonymous=True)
    r = rospy.Rate(1) # 10hz
    box_array = BoundingBoxArray()
    x_y_r_list = [[0.6*2*numpy.sin(numpy.deg2rad(15)) * 0.5, 0.6*2*numpy.sin(numpy.deg2rad(15)) * 0.86, 45], [0.6*2*numpy.sin(numpy.deg2rad(15)) * 0.5, -0.6*2*numpy.sin(numpy.deg2rad(15)) * 0.86, -45]]
    for x_y_r in x_y_r_list:
        box = BoundingBox()
        box.dimensions = Vector3(x=0.6*2*numpy.sin(15) * 0.5 * 1.1, y=0.4, z=0.8)
        box.header.frame_id = "ground"
        box.pose.position = Point(x=x_y_r[0], y=x_y_r[1], z=0.5)
        box.pose.orientation = Quaternion(0, 0, numpy.sin(numpy.deg2rad(x_y_r[2]/2)), numpy.cos(numpy.deg2rad(x_y_r[2]/2)))
        box_array.boxes.append(box)
    box_array.header = box.header
    while not rospy.is_shutdown():
        pub.publish(box_array)
        r.sleep()

if __name__ == "__main__":
    box_pub_server()

#!/usr/bin/env python

import grasp_finder_with_rrt_node
import grasp_finder_node
from openravepy import *
from openrave_test_utility import *
import numpy, time
import rospy
from jsk_recognition_msgs.msg import BoundingBox
from std_srvs.srv import Empty
import geometry_msgs.msg
from geometry_msgs.msg import *
import tf
from tf import transformations
from openrave_test.srv import *
import commands
from pickle import *

if __name__ == '__main__':
    grasp_finder_with_rrt_node.grasp_finder()
    grasp_finder_with_rrt_node.try_grasp()
    # grasp_finder_node.grasp_finder()
    # grasp_finder_node.try_grasp()

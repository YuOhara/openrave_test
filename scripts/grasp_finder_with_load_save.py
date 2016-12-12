import grasp_finder_with_rrt_node
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
import sys
import os

HOME_PATH = os.environ.get("HOME")

if __name__ == '__main__':
    formatstring = sys.argv[1]
    f = open('%s/.ros/grasps/grasp_%s.txt' % (HOME_PATH, formatstring))
    approachrays = pickle.load(f)
    f.close()
    success_grasp_list = []
    half_success_grasp_list = []
    left_hand = True if "left" in formatstring else False
    robot_name = "hrp2"
    grasp_finder_with_rrt_node.trial(approachrays, success_grasp_list, half_success_grasp_list, len(approachrays), left_hand, robot_name)
    f2 = open('%s/.ros/grasps/result_%s.txt' % (HOME_PATH, formatstring) , 'w')
    pickle.dump(success_grasp_list, f2)
    f2.close()
    f2 = open('%s/.ros/grasps/resultfalse_%s.txt' % (HOME_PATH, formatstring) , 'w')
    pickle.dump(half_success_grasp_list, f2)
    f2.close()

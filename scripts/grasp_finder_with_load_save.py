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

if __name__ == '__main__':
    formatstring = sys.argv[1]
    f = open('/home/leus/.ros/grasps/grasp_%s.txt' % (formatstring))
    approachrays = pickle.load(f)
    f.close()
    success_grasp_list = []
    half_success_grasp_list = []
    grasp_finder_with_rrt_node.trial(approachrays, success_grasp_list, half_success_grasp_list, len(approachrays))
    f2 = open('/home/leus/.ros/grasps/result_%s.txt' % (formatstring) , 'w')
    pickle.dump(success_grasp_list, f2)
    f2.close()

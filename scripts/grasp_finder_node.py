#!/usr/bin/env python

from openravepy import *
import numpy, time
import rospy
from jsk_recognition_msgs.msg import BoundingBox
from std_srvs.srv import Empty

env=Environment()

def callback(data):
    ## call service for save mesh
    env.Load('hand_and_world.env.xml')
    robot = env.GetRobots()[0]
    target = env.GetKinBody('mug1')
    gmodel = databases.grasping.GraspingModel(robot,target)
    taskmanip = interfaces.TaskManipulation(robot)
    taskmanip.robot.SetDOFValues([90, 90, 0, 0, 0, 0])
    gmodel.autogenerate()
def grasp_finder():
    rospy.init_node('grasp_finder', anonymous=True)
    rospy.Subscriber("box", BoundingBox, callback)
    rospy.spin()

if __name__ == '__main__':
    grasp_finder()

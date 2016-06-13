#!/usr/bin/env python

from std_srvs.srv import *
import rospy
import commands

def file_get(req):
    check = commands.getoutput("scp leus@koi:/home/leus/.ros/tmp_model.l /home/leus/.ros/tmp_model.l")
    print check
    return EmptyResponse()

def file_getter_server():
    rospy.init_node('file_getter_server')
    s = rospy.Service('/file_getter', Empty, file_get)
    print "Ready to add two ints."
    rospy.spin()

if __name__ == "__main__":
    file_getter_server()

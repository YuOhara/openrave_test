#!/usr/bin/env python

from openravepy import *
import numpy, time
import rospy
from jsk_recognition_msgs.msg import BoundingBox
from std_srvs.srv import Empty
import geometry_msgs.msg
import tf
from tf import transformations
import commands

def return_approach_rays(gmodel, box):
    # ode gives the most accurate rays
    rospy.loginfo("start approach rays")
    env = gmodel.env
    delta = 0.02
    normalanglerange = 0
    target = gmodel.target
    cc = RaveCreateCollisionChecker(env,'ode')
    dim = box.dimensions
    position = box.pose.position
    try:
        with target:
            # target.SetTransform(numpy.eye(4))
            maxlen = numpy.sqrt(dim.x * dim.x + dim.y * dim.y + dim.z * dim.z)+0.03
            theta,pfi = misc.SpaceSamplerExtra().sampleS2(angledelta=delta)
            dirs = numpy.c_[numpy.cos(theta),numpy.sin(theta)*numpy.cos(pfi),numpy.sin(theta)*numpy.sin(pfi)]
            rays = numpy.c_[numpy.tile(numpy.array([position.x, position.y, position.z]),(len(dirs),1))-maxlen*dirs,2*maxlen*dirs]
            collision, info = cc.CheckCollisionRays(rays,target)
            # make sure all normals are the correct sign: pointing outward from the object)
            approachrays = info[collision,:]
            if len(approachrays) > 0:
                approachrays[sum(rays[collision,3:6]*approachrays[:,3:6],1)>0,3:6] *= -1
            if normalanglerange > 0:
                theta,pfi = misc.SpaceSamplerExtra().sampleS2(angledelta=directiondelta)
                dirs = numpy.c_[numpy.cos(theta),numpy.sin(theta)*numpy.cos(pfi),numpy.sin(theta)*numpy.sin(pfi)]
                dirs = array([dir for dir in dirs if numpy.arccos(dir[2])<=normalanglerange]) # find all dirs within normalanglerange
                if len(dirs) == 0:
                    dirs = array([[0,0,1]])
                newapproachrays = zeros((0,6))
                for approachray in approachrays:
                    R = rotationMatrixFromQuat(quatRotateDirection(array((0,0,1)),approachray[3:6]))
                    newapproachrays = numpy.r_[newapproachrays,numpy.c_[numpy.tile(approachray[0:3],(len(dirs),1)),numpy.dot(dirs,transpose(R))]]
                approachrays = newapproachrays
            return approachrays
    finally:
        cc.DestroyEnvironment()
def return_rave_params(gmodel, box):
    friction = None
    preshapes = None
    manipulatordirections = None
    approachrays = None
    standoffs = None
    rolls = None
    avoidlinks = None
    graspingnoise = None
    plannername = None
    normalanglerange = 0
    directiondelta=0
    translationstepmult=None
    finestep=None
    avoidlinks = []
    friction = 0.3
    forceclosure = True
    forceclosurethreshold=1e-9
    approachrays = return_approach_rays(gmodel, box)
    return preshapes,standoffs,rolls,approachrays, graspingnoise,forceclosure,forceclosurethreshold,None,manipulatordirections,translationstepmult,finestep,friction,avoidlinks,plannername

def callback(box):
    print "callback start!"
    ## call service for save mesh
    rospy.ServiceProxy('/kinfu/save_mesh', Empty)()
    ## change ply -> dae
    check = commands.getoutput("meshlabserver -i /home/leus/.ros/mesh.ply -o /home/leus/.ros/mesh.dae")
    print check
    env=Environment()
    env.Load('/home/leus/ros/indigo/src/openrave_test/scripts/hand_and_world.env.xml')
    env.SetViewer('qtcoin')
    robot = env.GetRobots()[0]
    target = env.GetKinBody('mug1')
    listener = tf.TransformListener()
    try:
        now = rospy.Time(0)
        listener.waitForTransform(box.header.frame_id, 'kinfu_origin', now, rospy.Duration(2.0))
        (trans,rot) = listener.lookupTransform('kinfu_origin', box.header.frame_id, now)
        mat44 = numpy.dot(transformations.translation_matrix(trans), transformations.quaternion_matrix(rot))
        pose44 = numpy.dot(tf.listener.xyz_to_mat44(box.pose.position), tf.listener.xyzw_to_mat44(box.pose.orientation))
        txpose = numpy.dot(mat44, pose44)
        xyz = tuple(transformations.translation_from_matrix(txpose))[:3]
        quat = tuple(transformations.quaternion_from_matrix(txpose))
        # assemble return value PoseStampe
        box.pose = geometry_msgs.msg.Pose(geometry_msgs.msg.Point(*xyz), geometry_msgs.msg.Quaternion(*quat))
        box.header = "kinfu_origin"
        #target.SetTransform(numpy.dot(transformations.translation_matrix(trans), transformations.quaternion_matrix(rot)))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), e:
        print "tf error: %s" % e
        return
    gmodel = databases.grasping.GraspingModel(robot,target)
    taskmanip = interfaces.TaskManipulation(robot)
    taskmanip.robot.SetDOFValues([90, 90, 0, 0, 0, 0])
    # gmodel.generate(*gmodel.autogenerateparams())
    gmodel.generate(*return_rave_params(gmodel, box))
    ## respected to frame, kinfu outputs with camera frame.


def grasp_finder():
    rospy.init_node('grasp_finder', anonymous=True)
    rospy.Subscriber("/bounding_box_marker/selected_box", BoundingBox, callback)
    rospy.spin()


if __name__ == '__main__':
    grasp_finder()

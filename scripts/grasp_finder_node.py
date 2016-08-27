#!/usr/bin/env python

from openravepy import *
from openrave_test_utility import *
import numpy, time
import rospy
from jsk_recognition_msgs.msg import BoundingBox
from std_srvs.srv import Empty
import geometry_msgs.msg
import tf
from tf import transformations
import commands

def return_rave_params(gmodel, box, approachrays = None):
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
    if (approachrays == None):
        approachrays = return_box_approach_rays(gmodel, box)
    return preshapes,standoffs,rolls,approachrays, graspingnoise,forceclosure,forceclosurethreshold,None,manipulatordirections,translationstepmult,finestep,friction,avoidlinks,plannername

def callback(box):
    print "callback start!"
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
        box.header.frame_id = "kinfu_origin"
        #target.SetTransform(numpy.dot(transformations.translation_matrix(trans), transformations.quaternion_matrix(rot)))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), e:
        print "tf error: %s" % e
        return
    ## call service for save mesh
    rospy.loginfo("save mesh start")
    rospy.ServiceProxy('/kinfu/save_mesh', Empty)()
    rospy.loginfo("save mesh end")

    ## change ply -> dae
    check = commands.getoutput("meshlabserver -i /home/leus/.ros/mesh.ply -o /home/leus/.ros/mesh.dae")
    print check
    # check = commands.getoutput("cp /home/leus/.ros/mesh.dae $(rospack find openrave_test)/scripts/mesh.dae")
    # print check
    # check = commands.getoutput("rosrun collada_urdf_jsk_patch urdf_to_collada $(rospack find openrave_test)/scripts/tmp_model.urdf $(rospack find openrave_test)/scripts/tmp_model.dae")
    # print check
    # check = commands.getoutput("rosrun euscollada collada2eus $(rospack find openrave_test)/scripts/tmp_model.dae $(rospack find openrave_test)/scripts/tmp_model.l")
    # print check
    ## estimated
    check = commands.getoutput("meshlabserver -i /home/leus/.ros/mesh_estimated.ply -o /home/leus/.ros/mesh_estimated.dae")
    print check
    # check = commands.getoutput("cp /home/leus/.ros/mesh_estimated.dae $(rospack find openrave_test)/scripts/mesh_estimated.dae")
    # print check
    # check = commands.getoutput("rosrun collada_urdf_jsk_patch urdf_to_collada $(rospack find openrave_test)/scripts/tmp_model_estimated.urdf $(rospack find openrave_test)/scripts/tmp_model_estimated.dae")
    # print check
    # check = commands.getoutput("rosrun euscollada collada2eus $(rospack find openrave_test)/scripts/tmp_model_estimated.dae $(rospack find openrave_test)/scripts/tmp_model_estimated.l")
    # print check

    env=Environment()
    env.Load('/home/leus/ros/indigo/src/openrave_test/scripts/hand_and_world.env.xml')
    env.SetViewer('qtcoin')
    robot = env.GetRobots()[0]
    target1 = env.GetKinBody('mug1')
    gmodel1 = databases.grasping.GraspingModel(robot,target1)
    target2 = env.GetKinBody('mug2')
    target2.Enable(False)
    # gmodel2 = databases.grasping.GraspingModel(robot,target1)
    taskmanip = interfaces.TaskManipulation(robot)
    taskmanip.robot.SetDOFValues([90, 90, 0, 0, 0, 0])
    # gmodel.generate(*gmodel.autogenerateparams())
    approachrays = return_box_approach_rays(gmodel1, box)
    gmodel1.generate(*return_rave_params(gmodel1, box, approachrays = approachrays))
    # gmodel2.generate(*return_rave_params(gmodel2, box, approachrays = approachrays))
    publish_result(gmodel1)
    gmodel1.save()
    # publish_result(gmodel2)
    # gmodel2.save()
    ## respected to frame, kinfu outputs with camera frame.

def publish_result(gmodel):
    result = gmodel.grasps
    print "the total grasp is %d" % len(result)
    i = 0
    pose_array = geometry_msgs.msg.PoseArray()
    for grasp in result:
        Tgrasp = gmodel.getGlobalGraspTransform(grasp)
        xyz = tuple(transformations.translation_from_matrix(Tgrasp))[:3]
        quat = tuple(transformations.quaternion_from_matrix(Tgrasp))
        # assemble return value PoseStampe
        pose_array.poses.append(geometry_msgs.msg.Pose(geometry_msgs.msg.Point(*xyz), geometry_msgs.msg.Quaternion(*quat)))
    pose_array.header.frame_id = "/kinfu_origin"
    pose_array.header.stamp = rospy.Time.now()
    grasp_array_pub.publish(pose_array)
    rospy.on_shutdown(shut_down_hook)

def shut_down_hook():
    print "shutting down node"

def grasp_finder():
    rospy.init_node('grasp_finder', anonymous=True)
    global grasp_array_pub
    grasp_array_pub = rospy.Publisher('/grasp_caluculation_result', geometry_msgs.msg.PoseArray)
    rospy.Subscriber("/bounding_box_marker/selected_box", BoundingBox, callback)
    rospy.spin()


if __name__ == '__main__':
    grasp_finder()

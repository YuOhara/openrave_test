#!/usr/bin/env python

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
from openrave_test.srv import GraspAssess
import commands


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
        box.header = "kinfu_origin"
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
    check = commands.getoutput("cp /home/leus/.ros/mesh.dae $(rospack find openrave_test)/scripts/mesh.dae")
    print check
    check = commands.getoutput("rosrun collada_urdf_jsk_patch urdf_to_collada $(rospack find openrave_test)/scripts/tmp_model.urdf $(rospack find openrave_test)/scripts/tmp_model.dae")
    print check
    check = commands.getoutput("rosrun euscollada collada2eus $(rospack find openrave_test)/scripts/tmp_model.dae $(rospack find openrave_test)/scripts/tmp_model.l")
    # print check
    ## estimated
    check = commands.getoutput("meshlabserver -i /home/leus/.ros/mesh_estimated.ply -o /home/leus/.ros/mesh_estimated.dae")
    print check
    check = commands.getoutput("cp /home/leus/.ros/mesh_estimated.dae $(rospack find openrave_test)/scripts/mesh_estimated.dae")
    print check
    check = commands.getoutput("rosrun collada_urdf_jsk_patch urdf_to_collada $(rospack find openrave_test)/scripts/tmp_model_estimated.urdf $(rospack find openrave_test)/scripts/tmp_model_estimated.dae")
    print check
    check = commands.getoutput("rosrun euscollada collada2eus $(rospack find openrave_test)/scripts/tmp_model_estimated.dae $(rospack find openrave_test)/scripts/tmp_model_estimated.l")
    # print check
    global env, robot, target1, target2, taskmanip
    env=Environment()
    env.Load('/home/leus/ros/indigo/src/openrave_test/scripts/hand_and_world.env.xml')
    env.SetViewer('qtcoin')
    robot = env.GetRobots()[0]
    target1 = env.GetKinBody('mug1')
    target2 = env.GetKinBody('mug2')
    taskmanip = interfaces.TaskManipulation(robot)
    taskmanip.robot.SetDOFValues([90, 90, 0, 0, 0, 0])
    target1.IsEnable(False)
    approachrays = return_box_approach_rays(gmodel, box)
    pose_array_msg = PoseArray()
    for approachray in approachrays:
        pose_msg = Pose()
        matrix = poseFromGraspParams(approachray[3:6], 0, approachray[0:3])
        quat = quatFromMatrix(matrix)
        pos = matrix[0:3,3]
        pose_msg.position = Position(pos[0], pos[1], pos[2])
        pose_msg.orientation = Quarternion(rot[1], rot[2], rot[3], rot[0])
        pose_array_msg.poses.append(pose_msg)
    pose_array_msg.header = box.header
    approarch_array_pub.publish(pose_array_msg)
    # gmodel.generate(*gmodel.autogenerateparams())
    ## respected to frame, kinfu outputs with camera frame.

def shut_down_hook():
    print "shutting down node"

def grasp_assess_service(req):
    # do not need to change frame
    pos_msg = req.pose_stamped.pose.position
    rot_msg = req.pose_stamped.pose.orientation
    pos_array = numpy.array([pos_msg.x, pos_msg.y, pos_msg.z])
    rot_array = numpy.array([rot_msg.w, rot_msg.x, rot_msg.y, rot_msg.z])
    pose_mat = matrixFromQuat(rot_array)
    pose_mat[0:3, 3] = pos_array
    direction, roll, position = graspParamsFromPose(pose)
    standoffs = [0, 0.025]
    contacts,finalconfig,mindist,volume = grasper.Grasp(direction=direction, roll=roll, position=position, standoff=standoffs[0], manipulatordirection=manipulatordirection, target=target, graspingnoise = 0.0, forceclosure=True, execute=False, outputfinal=True,translationstepmult=None, finestep=None, vintersectplane=numpy.array([0.0, 0.0, 0.0, 0.0]), chuckingdirection=manip.GetChuckingDirection())
    res = GraspAssessResponse()
    if finalconfig:
        res.assessment_point = mindist
        pose_msg = Pose()
        matrix = finalconfig[1]
        quat = quatFromMatrix(matrix)
        pos = matrix[0:3,3]
        pose_msg.position = Position(pos[0], pos[1], pos[2])
        pose_msg.orientation = Quarternion(rot[1], rot[2], rot[3], rot[0])
        res.grasp_pose_stamped = pose_msg
    else:
        res.assessment_point = -100
    return res
def grasp_finder():
    rospy.init_node('grasp_finder', anonymous=True)
    global pose_array_pub
    pose_array_pub = rospy.Publisher('/grasp_caluculation_result', geometry_msgs.msg.PoseArray)
    rospy.Subscriber("/bounding_box_marker/selected_box", BoundingBox, callback)
    rospy.Service('/grasp_assess', GraspAssess, grasp_assess_service)
    rospy.spin()


if __name__ == '__main__':
    grasp_finder()

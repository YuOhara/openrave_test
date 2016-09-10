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
from openrave_test.srv import *
import commands
from pickle import *
import sys
from std_msgs.msg import String
from jsk_interactive_marker.msg import *
from jsk_interactive_marker.srv import *


def callback(box):
    print "callback start!"
    listener = tf.TransformListener()
    global mat44
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
    check = commands.getoutput("cp /home/leus/.ros/mesh.dae $(rospack find openrave_test)/scripts/mesh.dae")
    print check
    check = commands.getoutput("rosrun collada_urdf_jsk_patch urdf_to_collada $(rospack find openrave_test)/scripts/tmp_model.urdf $(rospack find openrave_test)/scripts/tmp_model.dae")
    print check
    check = commands.getoutput("rosrun euscollada collada2eus $(rospack find openrave_test)/scripts/tmp_model.dae /home/leus/.ros/tmp_model.l")
    # print check
    ## estimated
    check = commands.getoutput("meshlabserver -i /home/leus/.ros/mesh_estimated.ply -o /home/leus/.ros/mesh_estimated.dae")
    print check
    check = commands.getoutput("cp /home/leus/.ros/mesh_estimated.dae $(rospack find openrave_test)/scripts/mesh_estimated.dae")
    print check
    check = commands.getoutput("rosrun collada_urdf_jsk_patch urdf_to_collada $(rospack find openrave_test)/scripts/tmp_model_estimated.urdf $(rospack find openrave_test)/scripts/tmp_model_estimated.dae")
    print check
    check = commands.getoutput("rosrun euscollada collada2eus $(rospack find openrave_test)/scripts/tmp_model_estimated.dae /home/leus/.ros/tmp_model_estimated.l")
    # print check
    # pickle
    f = open('/home/leus/.ros/temp_box.txt', 'w')
    pickle.dump(box, f)
    f.close()
    try_grasp()

def try_grasp():
    global env, robot, target1, target2, taskmanip, gmodel, manip, manipulatordirection
    env=Environment()
    # pickle
    f = open('/home/leus/.ros/temp_box.txt')
    box = pickle.load(f)
    f.close()

    check = commands.getoutput("rosrun openrave_test ply_clipper _dim_x:=%f _dim_y:=%f _dim_z:=%f _p_x:=%f _p_y:=%f _p_z:=%f _r_x:=%f _r_y:=%f _r_z:=%f _r_w:=%f" % (box.dimensions.x+0.15, box.dimensions.y+0.15, box.dimensions.z+ 0.02, box.pose.position.x, box.pose.position.y, box.pose.position.z, box.pose.orientation.x, box.pose.orientation.y, box.pose.orientation.z, box.pose.orientation.w))
    print check
    check = commands.getoutput("meshlabserver -i /home/leus/.ros/mesh_estimated2.ply -o /home/leus/.ros/mesh_estimated2.dae")
    print check

    env.Load('/home/leus/ros/indigo/src/openrave_test/scripts/hand_and_world.env.xml')
    env.SetViewer('qtcoin')
    robot = env.GetRobots()[0]
    target1 = env.GetKinBody('mug1')
    target2 = env.GetKinBody('mug2')
    # robot.GetLink("RARM_LINK6").Enable(False)
    taskmanip = interfaces.TaskManipulation(robot)
    manip = robot.GetActiveManipulator()
    manipulatordirection = manip.GetLocalToolDirection()
    target2.Enable(False)
    target2.SetVisible(True)
    gmodel = databases.grasping.GraspingModel(robot,target1)
    approachrays = return_box_approach_rays(gmodel, box)
    pose_array_msg = geometry_msgs.msg.PoseArray()
    com_array_msg = geometry_msgs.msg.PoseArray()
    global grasper
    grasper = interfaces.Grasper(robot, friction=0.3)
    len_approach = len(approachrays)
    print "len %d %d" % (len_approach,  approachrays.shape[0])
    try_num = 0
    taskmanip.robot.SetDOFValues([90, 90, 0, 0, -40, -40])
    final,traj = taskmanip.ReleaseFingers(execute=False,outputfinal=True)
    preshape = final
    success_grasp_list = []
    half_success_grasp_list = []
    for approachray in approachrays:
        standoffs = [0, 0.025]
        for standoff in standoffs:
            pose_msg = Pose()
            matrix = None
            mindist2 = -0.1
            if False:
                matrix = poseFromGraspParams(-approachray[3:6], 0, approachray[0:3], manipulatordirection)
                mindist = mindist2 = 1.0
            else:
                rolls = [0, numpy.pi/2, numpy.pi, numpy.pi*3/2]
                for roll in rolls:
                    mindist = mindist2 = -0.1
                    robot.SetActiveManipulator(manip)
                    robot.SetTransform(numpy.eye(4))
                    robot.SetDOFValues(preshape, manip.GetGripperIndices())
                    robot.SetActiveDOFs(manip.GetGripperIndices(),DOFAffine.X+DOFAffine.Y+DOFAffine.Z if True else 0)
                    sys.stdout.write("\rtry %d/%d " % (try_num, len_approach*4*2))
                    sys.stdout.flush()
                    try_num = try_num + 1
                    # grasper.robot.SetTransform(poseFromGraspParams(-approachray[3:6], roll, approachray[0:3], manipulatordirection))
                    try:
                        target1.Enable(True)
                        target2.Enable(False)
                        contacts,finalconfig,mindist,volume = grasper.Grasp(direction=-approachray[3:6], roll=roll, position=approachray[0:3], standoff=standoff, manipulatordirection=manipulatordirection, target=target1, graspingnoise = 0.0, forceclosure=True, execute=False, outputfinal=True,translationstepmult=None, finestep=None, vintersectplane=numpy.array([0.0, 0.0, 0.0, 0.0]), chuckingdirection=manip.GetChuckingDirection())
                        # print "mindist! %f" % mindist
                        if mindist > 1e-9:
                            grasper.robot.SetTransform(finalconfig[1])
                            env.UpdatePublishedBodies()
                            Tgrasp = manip.GetEndEffectorTransform()
                            # Tgrasp = robot.GetTransform()
                            direction2, roll2, position2 = (-approachray[3:6], roll, approachray[0:3])# graspParamsFromPose(Tgrasp, manipulatordirection) # need debug, Tgrasp not changed?
                            matrix = Tgrasp ##finalconfig[1]
                            # print finalconfig[1]
                            ## start 2nd
                            target1.Enable(False)
                            target2.Enable(True)
                            robot.SetActiveDOFs(manip.GetGripperIndices(), 0)
                            robot.SetTransform(numpy.eye(4))
                            robot.SetDOFValues(preshape, manip.GetGripperIndices())
                            robot.SetActiveDOFs(manip.GetGripperIndices(),DOFAffine.X+DOFAffine.Y+DOFAffine.Z if True else 0)
                            contacts2,finalconfig2,mindist2,volume2 = grasper.Grasp(direction=direction2, roll=roll2, position=position2, standoff=standoff, manipulatordirection=manipulatordirection, target=target2, graspingnoise = 0.0, forceclosure=True, execute=False, outputfinal=True,translationstepmult=None, finestep=None, vintersectplane=numpy.array([0.0, 0.0, 0.0, 0.0]), chuckingdirection=manip.GetChuckingDirection())
                            print "mindists %f %f" % (mindist, mindist2)
                            if mindist > 1e-9 and mindist2 > 1e-9:
                            # if True:
                                grasper.robot.SetTransform(finalconfig[1])
                                grasper.robot.SetDOFValues(finalconfig[0])
                                drawContacts(contacts)
                                env.UpdatePublishedBodies()
                                # raw_input('press any key to continue:(1) ')
                                grasper.robot.SetTransform(finalconfig2[1])
                                grasper.robot.SetDOFValues(finalconfig2[0])
                                drawContacts(contacts2)
                                env.UpdatePublishedBodies()
                                # raw_input('press any key to continue:(2) ')
                                grasper.robot.SetTransform(finalconfig[1])
                                pose_array_msg.poses.append(matrix2pose(robot.GetTransform()))
                                success_grasp_list.append([contacts, contacts2, finalconfig, finalconfig2])
                            else:
                                half_success_grasp_list.append([contacts, contacts2, finalconfig, finalconfig2])
                        else:
                            mindist2 = 1.0
                        # print "hoge"
                        # print mindist, mindist2
                    except (PlanningError), e:
                        print "warn! planning error occured!"
                        continue
    pose_array_msg.header = box.header
    pose_array_msg.header.stamp = rospy.Time(0)
    # pose_array_msg.header.frame_id = "ground"
    pose_array_pub.publish(pose_array_msg)
    for grasp_node in success_grasp_list:
        contact_num = 0
        ave_x = ave_y = ave_z = 0
        temp_pose = Pose()
        for contact in grasp_node[0]:
            ave_x = ave_x + contact[0]
            ave_y = ave_y + contact[1]
            ave_z = ave_z + contact[2]
            contact_num = contact_num + 1
        temp_pose.position.x = ave_x/contact_num
        temp_pose.position.y = ave_y/contact_num
        temp_pose.position.z = ave_z/contact_num
        com_array_msg.poses.append(temp_pose)
    com_array_msg.header = pose_array_msg.header
    com_array_pub.publish(com_array_msg)
    show_result(success_grasp_list)
    print "Finished"
    print "Num!"
    print len(pose_array_msg.poses)
    # gmodel.generate(*gmodel.autogenerateparams())
    ## respected to frame, kinfu outputs with camera frame.

def show_result(grasp_list_array):
    global grasper, env
    for grasp_list in grasp_list_array:
        grasper.robot.SetTransform(grasp_list[2][1])
        grasper.robot.SetDOFValues(grasp_list[2][0])
        drawContacts(grasp_list[0])
        env.UpdatePublishedBodies()
        raw_input('press any key to continue:(1) ')
        grasper.robot.SetTransform(grasp_list[3][1])
        grasper.robot.SetDOFValues(grasp_list[3][0])
        drawContacts(grasp_list[1])
        env.UpdatePublishedBodies()
        raw_input('press any key to continue:(2) ')
        grasper.robot.SetTransform(finalconfig[1])
        pose_array_msg.poses.append(matrix2pose(robot.GetTransform()))


def drawContacts(contacts,conelength=0.03,transparency=0.5):
    angs = numpy.linspace(0,2*numpy.pi,10)
    global grasper, env
    conepoints = numpy.r_[[[0,0,0]],conelength*numpy.c_[grasper.friction*numpy.cos(angs),grasper.friction*numpy.sin(angs),numpy.ones(len(angs))]]
    triinds = numpy.array(numpy.c_[numpy.zeros(len(angs)),range(2,1+len(angs))+[1],range(1,1+len(angs))].flatten(),int)
    allpoints = numpy.zeros((0,3))
    for c in contacts:
        R = rotationMatrixFromQuat(quatRotateDirection(numpy.array((0,0,1)),c[3:6]))
        points = numpy.dot(conepoints,numpy.transpose(R)) + numpy.tile(c[0:3],(conepoints.shape[0],1))
        allpoints = numpy.r_[allpoints,points[triinds,:]]
    return env.drawtrimesh(points=allpoints,indices=None,colors=numpy.array((1,0.4,0.4,transparency)))

def shut_down_hook():
    print "shutting down node"

def marker_callback(msg):
    box = BoundingBox()
    get_box_pose_srv = rospy.ServiceProxy("/transformable_server_sample/get_pose", GetTransformableMarkerPose)
    resp = get_box_pose_srv(target_name=msg.data)
    box.pose = resp.pose_stamped.pose
    box.header = resp.pose_stamped.header
    get_box_dim_srv = rospy.ServiceProxy("/transformable_server_sample/get_dimensions", GetMarkerDimensions)
    resp2 = get_box_dim_srv(target_name=msg.data)
    box.dimensions.x = resp2.dimensions.x
    box.dimensions.y = resp2.dimensions.y
    box.dimensions.z = resp2.dimensions.z
    callback(box)

def grasp_finder():
    rospy.init_node('grasp_finder', anonymous=True)
    global pose_array_pub, com_array_pub
    pose_array_pub = rospy.Publisher('/grasp_caluculation_result', geometry_msgs.msg.PoseArray, latch=True)
    com_array_pub = rospy.Publisher('/grasp_caluculation_com_result', geometry_msgs.msg.PoseArray, latch=True)
    rospy.Subscriber("/bounding_box_marker/selected_box", BoundingBox, callback)
    rospy.Subscriber("/select_box", String, marker_callback)

def matrix2pose(matrix):
    quat = quatFromRotationMatrix(matrix[0:3, 0:3])
    pos = matrix[0:3,3]
    pose_msg = Pose()
    pose_msg.position = Point(pos[0], pos[1], pos[2])
    pose_msg.orientation = Quaternion(quat[1], quat[2], quat[3], quat[0])
    return pose_msg

if __name__ == '__main__':
    grasp_finder()
    rospy.spin()

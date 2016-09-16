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
from multiprocessing import Process, Queue
import os.path

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
    left_hand = rospy.get_param("~left_hand", False)
    if left_hand:
        rospy.loginfo("left hand")
    else:
        rospy.loginfo("right hand")
    if not left_hand:
        rospy.loginfo("save mesh start")
        rospy.ServiceProxy('/kinfu/save_mesh', Empty)()
        rospy.loginfo("save mesh end")
        ## change ply -> dae
        check = commands.getoutput("meshlabserver -i /home/leus/.ros/mesh.ply -o /home/leus/.ros/mesh.dae")
        print check
        check = commands.getoutput("rosrun collada_urdf_jsk_patch urdf_to_collada $(rospack find openrave_test)/scripts/tmp_model.urdf $(rospack find openrave_test)/scripts/tmp_model.dae")
        print check
        check = commands.getoutput("rosrun euscollada collada2eus $(rospack find openrave_test)/scripts/tmp_model.dae /home/leus/.ros/tmp_model.l")
        # print check
        ## estimated
        check = commands.getoutput("rosrun collada_urdf_jsk_patch urdf_to_collada $(rospack find openrave_test)/scripts/tmp_model_estimated.urdf $(rospack find openrave_test)/scripts/tmp_model_estimated.dae")
        print check
        check = commands.getoutput("rosrun euscollada collada2eus $(rospack find openrave_test)/scripts/tmp_model_estimated.dae /home/leus/.ros/tmp_model_estimated.l")
        # print check
        # pickle
        f = open('/home/leus/.ros/temp_box.txt', 'w')
        pickle.dump(box, f)
        f.close()
        check = commands.getoutput("rosrun openrave_test ply_clipper _dim_x:=%f _dim_y:=%f _dim_z:=%f _p_x:=%f _p_y:=%f _p_z:=%f _r_x:=%f _r_y:=%f _r_z:=%f _r_w:=%f _input_file_name:=/home/leus/.ros/mesh_estimated.ply _output_file_name:=/home/leus/.ros/mesh_estimated2.ply" % (box.dimensions.x+0.15, box.dimensions.y+0.15, box.dimensions.z+ 0.02, box.pose.position.x, box.pose.position.y, box.pose.position.z, box.pose.orientation.x, box.pose.orientation.y, box.pose.orientation.z, box.pose.orientation.w))
        print check
        check = commands.getoutput("meshlabserver -i /home/leus/.ros/mesh_estimated2.ply -o /home/leus/.ros/mesh_estimated2.dae")
        print check
        check = commands.getoutput("rosrun openrave_test ply_clipper _dim_x:=%f _dim_y:=%f _dim_z:=%f _p_x:=%f _p_y:=%f _p_z:=%f _r_x:=%f _r_y:=%f _r_z:=%f _r_w:=%f _input_file_name:=/home/leus/.ros/mesh.ply _output_file_name:=/home/leus/.ros/mesh0.ply" % (box.dimensions.x+0.15, box.dimensions.y+0.15, box.dimensions.z+ 0.02, box.pose.position.x, box.pose.position.y, box.pose.position.z, box.pose.orientation.x, box.pose.orientation.y, box.pose.orientation.z, box.pose.orientation.w))
        print check
        check = commands.getoutput("meshlabserver -i /home/leus/.ros/mesh0.ply -o /home/leus/.ros/mesh0.dae")
        print check
    try_grasp()

def initialize_env(left_hand):
    env=Environment()
    env.Load('/home/leus/ros/indigo/src/openrave_test/scripts/hand_and_world.env.xml')
    # env.SetViewer('qtcoin')
    hand1 = env.GetRobots()[0]
    hand2 = env.GetRobots()[1]
    robot = hand2 if left_hand else hand1
    target1 = env.GetKinBody('mug1')
    target2 = env.GetKinBody('mug2')
    # robot.GetLink("RARM_LINK6").Enable(False)
    taskmanip = interfaces.TaskManipulation(robot)
    manip = robot.GetActiveManipulator()
    manipulatordirection = manip.GetLocalToolDirection()
    gmodel = databases.grasping.GraspingModel(robot,target1)
    grasper = interfaces.Grasper(robot, friction=0.3)
    return env, hand1, hand2, robot, target1, target2, taskmanip, manip, manipulatordirection, gmodel, grasper

def load_and_save_trial(approachrays, success_grasp_list, half_success_grasp_list, len_approach, formatstring):
    thread_num = 40
    ps = []
    for i in range(thread_num):
        approachrays_save = approachrays[len_approach/thread_num*i: len_approach/thread_num*(i+1)]
        f2 = open('/home/leus/.ros/grasps/grasp_%s%d.txt' % (formatstring, i) , 'w')
        pickle.dump(approachrays_save, f2)
        f2.close()
        p = Process(target=load_and_save_trial_single, args=(i, formatstring))
        p.start()
        ps.append(p)
        print i
    for i in range(thread_num):
        ps[i].join()
        f = open('/home/leus/.ros/grasps/result_%s%d.txt' % (formatstring, i))
        success_grasp_list_load = pickle.load(f)
        success_grasp_list.extend(success_grasp_list_load)
        f.close()


def load_and_save_trial_single(index, formatstring):
    check = commands.getoutput("ipython `rospack find openrave_test`/scripts/grasp_finder_with_load_save.py %s%d" % (formatstring, index))

def trial(approachrays, success_grasp_list, half_success_grasp_list, len_approach, left_hand):
    approachrays_queue = Queue()
    success_grasp_list_queue = Queue()
    half_success_grasp_list_queue = Queue()
    for approachray in approachrays:
        approachrays_queue.put(approachray)
    trial_queue(approachrays_queue, success_grasp_list_queue, half_success_grasp_list_queue, len_approach, left_hand)
    while not success_grasp_list_queue.empty():
        success_grasp_list.append(success_grasp_list_queue.get())
    while not half_success_grasp_list_queue.empty():
        half_success_grasp_list.append(half_success_grasp_list_queue.get())

def trial_queue(approachrays, success_grasp_list, half_success_grasp_list, len_approach, left_hand):
    env, hand1, hand2, robot, target1, target2, taskmanip, manip, manipulatordirection, gmodel, grasper=initialize_env(left_hand)
    taskmanip.robot.SetDOFValues([90, 90, 0, 0, -40, -40])
    print "hoge"
    final,traj = taskmanip.ReleaseFingers(execute=False,outputfinal=True)
    preshape = final
    # preshape = robot.GetDOFValues(manip.GetGripperIndices())
    print "fuga"
    # for approachray in approachrays:
    while not approachrays.empty():
        approachray = approachrays.get()
        standoffs = [0, 0.025]
        for standoff in standoffs:
            pose_msg = Pose()
            matrix = None
            mindist2 = -0.1
            if False:
                matrix = poseFromGraspParams(-approachray[3:6], 0, approachray[0:3], manipulatordirection)
                mindist = mindist2 = 1.0
            else:
                rolls = [0, numpy.pi/4 ,numpy.pi/2,numpy.pi/4*3, numpy.pi, numpy.pi/4*5, numpy.pi*3/2, numpy.pi/4*7]
                for roll in rolls:
                    mindist = mindist2 = -0.1
                    robot.SetActiveManipulator(manip)
                    robot.SetTransform(numpy.eye(4))
                    robot.SetDOFValues(preshape, manip.GetGripperIndices())
                    robot.SetActiveDOFs(manip.GetGripperIndices(),DOFAffine.X+DOFAffine.Y+DOFAffine.Z if True else 0)
                    try_num = 0 # temp
                    sys.stdout.write("\rtry %d/%d " % (try_num, len_approach*4*2))
                    sys.stdout.flush()
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
                                drawContacts(contacts, grasper, env)
                                env.UpdatePublishedBodies()
                                # raw_input('press any key to continue:(1) ')
                                grasper.robot.SetTransform(finalconfig2[1])
                                grasper.robot.SetDOFValues(finalconfig2[0])
                                drawContacts(contacts2, grasper, env)
                                env.UpdatePublishedBodies()
                                # raw_input('press any key to continue:(2) ')
                                grasper.robot.SetTransform(finalconfig[1])
                                success_grasp_list.put([contacts, contacts2, finalconfig, finalconfig2])
                            else:
                                half_success_grasp_list.put([contacts, contacts2, finalconfig, finalconfig2])
                        else:
                            mindist2 = 1.0
                        # print "hoge"
                        # print mindist, mindist2
                    except (PlanningError), e:
                        print "warn! planning error occured!"
                        continue


def try_grasp():
    # pickle
    left_hand = rospy.get_param("~left_hand", False)
    if left_hand:
        commands.getoutput("rm /home/leus/.ros/temp_box.txt")
        while not os.path.exists('/home/leus/.ros/temp_box.txt'):
            rospy.loginfo("wait for right")
            time.sleep(3)
    f = open('/home/leus/.ros/temp_box.txt')
    box = pickle.load(f)
    f.close()
    if left_hand:
        rospy.loginfo("left hand")
    else:
        rospy.loginfo("right hand")
    env, hand1, hand2, robot, target1, target2, taskmanip, manip, manipulatordirection, gmodel, grasper = initialize_env(left_hand)
    env.SetViewer('qtcoin')
    target2.Enable(False)
    target2.SetVisible(True)

    target0 = env.GetKinBody('mug0')

    if False:
        target1.Enable(False)
        target1.SetVisible(True)
        target0.Enable(True)
        target0.SetVisible(True)
        gmodel0 = databases.grasping.GraspingModel(robot,target0)
        approachrays = return_box_approach_rays(gmodel0, box)
        target0.Enable(False)
        target0.SetVisible(False)
    else:
        approachrays = return_box_approach_rays(gmodel, box)

    pose_array_msg = geometry_msgs.msg.PoseArray()
    com_array_msg = geometry_msgs.msg.PoseArray()

    len_approach = len(approachrays)
    print "len %d %d" % (len_approach,  approachrays.shape[0])
    try_num = 0
    success_grasp_list_queue = Queue()
    half_success_grasp_list_queue = Queue()
    success_grasp_list = []
    half_success_grasp_list = []
    approachrays_queue = Queue()
    format_string = "left" if left_hand else "right"
    start = time.time()
    load_and_save_trial(approachrays, success_grasp_list, half_success_grasp_list, len_approach, format_string)
    elapsed_time = time.time() - start
    print ("elapsed_time:{0}".format(elapsed_time)) + "[sec]"
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
        grasper.robot.SetTransform(grasp_node[3][1])
        pose_array_msg.poses.append(matrix2pose(robot.GetTransform()))
    com_array_msg.header = pose_array_msg.header
    com_array_pub.publish(com_array_msg)
    show_result(success_grasp_list, grasper, env)
    print "Finished"
    print "Num!"
    print len(pose_array_msg.poses)
    # gmodel.generate(*gmodel.autogenerateparams())
    ## respected to frame, kinfu outputs with camera frame.

def show_result(grasp_list_array, grasper, env):
    if env.GetViewer() is not None:
        for grasp_list in grasp_list_array:
            grasper.robot.SetTransform(grasp_list[2][1])
            grasper.robot.SetDOFValues(grasp_list[2][0])
            drawContacts(grasp_list[0], grasper, env)
            env.UpdatePublishedBodies()
            raw_input('press any key to continue:(1) ')
            grasper.robot.SetTransform(grasp_list[3][1])
            grasper.robot.SetDOFValues(grasp_list[3][0])
            drawContacts(grasp_list[1], grasper, env)
            env.UpdatePublishedBodies()
            raw_input('press any key to continue:(2) ')

def drawContacts(contacts,grasper, env, conelength=0.03,transparency=0.5):
    if env.GetViewer() is not None:
        angs = numpy.linspace(0,2*numpy.pi,10)
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
    pose_array_pub = rospy.Publisher('~grasp_caluculation_result', geometry_msgs.msg.PoseArray, latch=True)
    com_array_pub = rospy.Publisher('~grasp_caluculation_com_result', geometry_msgs.msg.PoseArray, latch=True)
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

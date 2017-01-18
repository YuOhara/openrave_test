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
from openrave_test.msg import *
import commands
from pickle import *
import sys
from std_msgs.msg import String, Float32MultiArray
from jsk_interactive_marker.msg import *
from jsk_interactive_marker.srv import *
from multiprocessing import Process, Queue
import os.path
import os
import rospkg

debug_mode = False
# RaveSetDebugLevel(DebugLevel.Debug)
HOME_PATH = os.environ.get("HOME")
OPENRAVE_TEST_PATH = rospkg.RosPack().get_path("openrave_test")

linelist = None
linelist_temp = None
contactlist = None
only_read = False
box_minus = None

def change_frame(box):
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
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), e:
        print "tf error: %s" % e
        return


def convert_mesh(box, box_minus):
    check = commands.getoutput("rosrun openrave_test ply_clipper _dim_x:=%f _dim_y:=%f _dim_z:=%f _p_x:=%f _p_y:=%f _p_z:=%f _r_x:=%f _r_y:=%f _r_z:=%f _r_w:=%f _input_file_name:=%s/.ros/mesh.ply _output_file_name:=%s/.ros/mesh0.ply" % (box.dimensions.x+1.00, box.dimensions.y+1.00, box.dimensions.z+ 1.00, box.pose.position.x, box.pose.position.y, box.pose.position.z, box.pose.orientation.x, box.pose.orientation.y, box.pose.orientation.z, box.pose.orientation.w, HOME_PATH, HOME_PATH))
    print check
    check = commands.getoutput("meshlabserver -i ~/.ros/mesh0.ply -o ~/.ros/mesh.dae")
    print check
    check = commands.getoutput("cp ~/.ros/mesh.dae $(rospack find openrave_test)/scripts/mesh.dae")
    print check
    check = commands.getoutput("rosrun collada_urdf_jsk_patch urdf_to_collada $(rospack find openrave_test)/scripts/tmp_model.urdf $(rospack find openrave_test)/scripts/tmp_model.dae")
    print check
    check = commands.getoutput("rosrun euscollada collada2eus $(rospack find openrave_test)/scripts/tmp_model.dae %s/.ros/tmp_model.l" % HOME_PATH)
    # print check
    ## estimated
    check = commands.getoutput("rosrun collada_urdf_jsk_patch urdf_to_collada $(rospack find openrave_test)/scripts/tmp_model_estimated.urdf $(rospack find openrave_test)/scripts/tmp_model_estimated.dae")
    print check
    check = commands.getoutput("rosrun euscollada collada2eus $(rospack find openrave_test)/scripts/tmp_model_estimated.dae %s/.ros/tmp_model_estimated.l" % HOME_PATH)
    # print check
    # pickle
    f = open('%s/.ros/temp_box.txt' % HOME_PATH, 'w')
    pickle.dump(box, f)
    f.close()
    f = open('%s/.ros/temp_box_minus.txt' % HOME_PATH, 'w')
    pickle.dump(box_minus, f)
    f.close()
    check = commands.getoutput("rosrun openrave_test ply_clipper _dim_x:=%f _dim_y:=%f _dim_z:=%f _p_x:=%f _p_y:=%f _p_z:=%f _r_x:=%f _r_y:=%f _r_z:=%f _r_w:=%f _input_file_name:=%s/.ros/mesh_estimated.ply _output_file_name:=%s/.ros/mesh_estimated2.ply" % (box.dimensions.x+0.15, box.dimensions.y+0.15, box.dimensions.z+ 0.02, box.pose.position.x, box.pose.position.y, box.pose.position.z, box.pose.orientation.x, box.pose.orientation.y, box.pose.orientation.z, box.pose.orientation.w, HOME_PATH, HOME_PATH))
    print check
    check = commands.getoutput("meshlabserver -i ~/.ros/mesh_estimated2.ply -o ~/.ros/mesh_estimated2.dae")
    print check
    check = commands.getoutput("meshlabserver -i ~/.ros/mesh0.ply -o ~/.ros/mesh0.dae")
    print check

def callback(box):
    print "callback start!"
    change_frame(box)
    if box_minus:
        change_frame(box_minus)
    ## call service for save mesh
    left_hand = rospy.get_param("~left_hand", False)
    robot_name = rospy.get_param("~robot_name", "hrp2")
    if not left_hand:
        rospy.loginfo("save mesh start")
        start = time.time()
        rospy.ServiceProxy('/kinfu/save_mesh', Empty)()
        rospy.loginfo("save mesh end")
        ## change ply -> dae
        convert_mesh(box, box_minus)
        elapsed_time = time.time() - start
        print ("elapsed_time for convert:{0}".format(elapsed_time)) + "[sec]"
    else:
        commands.getoutput("rm ~/.ros/temp_box.txt")
        while not os.path.exists('%s/.ros/temp_box.txt' % HOME_PATH):
            rospy.loginfo("wait for right")
            time.sleep(10)
    try_grasp()

def callback_minus(box):
    global box_minus
    box_minus = box

def initialize_env(left_hand, robot_name):
    env=Environment()
    hand1 = None
    hand2 = None
    if robot_name == "hrp2":
        env.Load('%s/scripts/config/hand_and_world.env.xml' % OPENRAVE_TEST_PATH)
        hand1 = env.GetRobots()[0]
        hand2 = env.GetRobots()[1]
        hand1.SetDOFValues([90, 90, 0, 0, -40, -40])
        hand2.SetDOFValues([90, 90, 0, 0, -40, -40])
    elif robot_name == "baxter":
        env.Load('%s/scripts/config/world.env.xml' % OPENRAVE_TEST_PATH)
        module = RaveCreateModule(env, "urdf")
        APC_PATH = rospkg.RosPack().get_path("jsk_2016_01_baxter_apc")
        module.SendCommand("LoadURI %s/robots/openrave_config/right_vacuum_gripper_only.urdf %s/robots/openrave_config/right_vacuum_gripper_only.srdf" % (APC_PATH, APC_PATH))
        hand1 = hand2 = env.GetRobots()[0] # todo load left hand if left hand is made
        manip = hand1.GetManipulators()[0]
        manip.SetChuckingDirection([1, 1])
        manip.SetLocalToolDirection([0, 0, 1])
        trans = numpy.eye(4)
        trans[0:3, 3] = numpy.array([0.035, 0, 0.3])
        manip.SetLocalToolTransform(trans)
        target = env.GetKinBody('mug1')
        hand1.SetDOFValues([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    else:
        rospy.logerr("Robot %s Not Found", robot_name)
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
    if not only_read:
        for i in range(thread_num):
            approachrays_save = approachrays[(len_approach/thread_num)*i: (len_approach/thread_num)*(i+1)]
            f2 = open('%s/.ros/grasps/grasp_%s%d.txt' % (HOME_PATH, formatstring, i) , 'w')
            pickle.dump(approachrays_save, f2)
            f2.close()
            p = Process(target=load_and_save_trial_single, args=(i, formatstring))
            p.start()
            ps.append(p)
            print i
        for i in range(thread_num):
            ps[i].join()
    for i in range(thread_num):
        f = open('%s/.ros/grasps/result_%s%d.txt' % (HOME_PATH, formatstring, i))
        success_grasp_list_load = pickle.load(f)
        success_grasp_list.extend(success_grasp_list_load)
        f = open('%s/.ros/grasps/resultfalse_%s%d.txt' % (HOME_PATH, formatstring, i))
        half_grasp_list_load = pickle.load(f)
        half_success_grasp_list.extend(half_grasp_list_load)
        f.close()

def load_and_save_trial_single(index, formatstring):
    check = commands.getoutput("ipython `rospack find openrave_test`/scripts/grasp_finder_with_load_save.py %s%d" % (formatstring, index))

def trial(approachrays, success_grasp_list, half_success_grasp_list, len_approach, left_hand, robot_name):

    env, hand1, hand2, robot, target1, target2, taskmanip, manip, manipulatordirection, gmodel, grasper=initialize_env(left_hand, robot_name)
    # collisionChecker = RaveCreateCollisionChecker(env,'fcl')
    # env.SetCollisionChecker(collisionChecker)
    # debug
    if debug_mode:
        env.SetViewer('qtcoin')
        show_approachrays(approachrays, env)
        # show_box(box, env)

    print "len_approach %d" % len_approach
    final,traj = taskmanip.ReleaseFingers(execute=False,outputfinal=True)
    preshape = final
    # preshape = robot.GetDOFValues(manip.GetGripperIndices())
    # for approachray in approachrays:
    try_num = 0
    graspnoise = -0.001
    rolls = [0, numpy.pi/4, numpy.pi/2, numpy.pi*3/4, numpy.pi, numpy.pi*5/4, numpy.pi*3/2, numpy.pi*7/4]
    # rolls = [0]
    rolls_size = len(rolls)
    standoffs = [0, #0.025
    ]
    standoffs_size = len(standoffs)
    offset_rolls = [0, numpy.pi / 2.0]
    offset_rolls_size = len(rolls)
    translationstepmult = 0.01
    finestep = 0.0003
    # approachrays = approachrays[100:] # for debug, skipping
    for approachray in approachrays:
        for standoff in standoffs:
            pose_msg = Pose()
            matrix = None
            mindist2 = -0.1
            if False:
                matrix = poseFromGraspParams(-approachray[3:6], 0, approachray[0:3], manipulatordirection)
                mindist = mindist2 = 1.0
            else:
                for roll in rolls:
                  for offset_roll in offset_rolls:
                    mindist = mindist2 = -0.1
                    robot.SetActiveManipulator(manip)
                    robot.SetTransform(numpy.eye(4))
                    robot.SetDOFValues(preshape, manip.GetGripperIndices())
                    robot.SetActiveDOFs(manip.GetGripperIndices(),DOFAffine.X+DOFAffine.Y+DOFAffine.Z if True else 0)
                    try_num = try_num + 1 # temp
                    sys.stdout.write("\rtry %d/%d " % (try_num, len_approach*rolls_size*standoffs_size*offset_rolls_size))
                    sys.stdout.flush()
                    # grasper.robot.SetTransform(poseFromGraspParams(-approachray[3:6], roll, approachray[0:3], manipulatordirection))
                    try:
                        target1.Enable(True)
                        target2.Enable(False)
                        direction =  approachray[3:6] # maybe + is true!
                        position = approachray[0:3]
                        ## debug
                        if debug_mode:
                            global linelist_temp
                            linelist_temp = env.drawlinelist(points=numpy.array([position, position - 0.05 * direction]), linewidth=7, colors=numpy.array((0, 0, 1, 1)))
                            # linelist_temp = env.drawlinelist(points=numpy.array([[0, 0, 0], [0, 0, 1]]), linewidth=10, colors=numpy.array((1, 0, 0, 1))) # example
                            print "rolls_before"
                            print direction, roll, position
                            print "rolls_after"
                            print graspParamsFromPose(poseFromGraspParams(direction, roll, position, manipulatordirection), manipulatordirection)
                            pose1 = poseFromGraspParams(direction, roll, approachray[0:3], manipulatordirection)
                            grasper.robot.SetTransform(pose1)
                            env.UpdatePublishedBodies()
                            raw_input("hoge")
                        ## end debug
                        direction, roll, position, new_roll_mat = offset_direction(direction, roll, position, manipulatordirection, - offset_roll if left_hand else offset_roll)
                        ## debug
                        if debug_mode:
                            pose2 = poseFromGraspParams(direction, roll, position, manipulatordirection)
                            grasper.robot.SetTransform(pose2)
                            env.UpdatePublishedBodies()
                            raw_input("fuga")
                        ## end debug
                        contacts,finalconfig,mindist,volume = grasper.Grasp(direction=direction, roll=roll, position=position, standoff=standoff, manipulatordirection=manipulatordirection, target=target1, graspingnoise = 0.0, forceclosure=True, execute=False, outputfinal=True,translationstepmult=translationstepmult, finestep=finestep, vintersectplane=numpy.array([0.0, 0.0, 0.0, 0.0]), chuckingdirection=manip.GetChuckingDirection())
                        if graspnoise > 0.00:
                            for i in range(20):
                                contacts_n,finalconfig_n,mindist_n,volume_n = grasper.Grasp(direction=direction, roll=roll, position=position, standoff=standoff, manipulatordirection=manipulatordirection, target=target1, graspingnoise = graspnoise, forceclosure=True, execute=False, outputfinal=True,translationstepmult=translationstepmult, finestep=finestep, vintersectplane=numpy.array([0.0, 0.0, 0.0, 0.0]), chuckingdirection=manip.GetChuckingDirection())
                                if mindist_n < 1e-4:
                                    mindist = mindist_n
                                    break
                        ## debug
                        if debug_mode:
                            grasper.robot.SetTransform(finalconfig[1])
                            raw_input("piyo")
                            grasper.robot.SetDOFValues(finalconfig[0])
                            drawContacts(contacts, grasper, env)
                            env.UpdatePublishedBodies()
                            raw_input("hogera")
                            drawContacts([], grasper, env)
                        ## end debug
                        # print "mindist! %f" % mindist
                        if mindist > 1e-4:
                            grasper.robot.SetTransform(finalconfig[1])
                            # env.UpdatePublishedBodies()
                            Tgrasp = manip.GetEndEffectorTransform()
                            # Tgrasp = robot.GetTransform()
                            direction2, roll2, position2 = (direction, roll, approachray[0:3])# graspParamsFromPose(Tgrasp, manipulatordirection) # need debug, Tgrasp not changed?
                            matrix = Tgrasp ##finalconfig[1]
                            # print finalconfig[1]
                            ## start 2nd
                            target1.Enable(False)
                            target2.Enable(True)
                            robot.SetActiveDOFs(manip.GetGripperIndices(), 0)
                            robot.SetTransform(numpy.eye(4))
                            robot.SetDOFValues(preshape, manip.GetGripperIndices())
                            robot.SetActiveDOFs(manip.GetGripperIndices(),DOFAffine.X+DOFAffine.Y+DOFAffine.Z if True else 0)
                            contacts2,finalconfig2,mindist2,volume2 = grasper.Grasp(direction=direction2, roll=roll2, position=position2, standoff=standoff, manipulatordirection=manipulatordirection, target=target2, graspingnoise = 0.0, forceclosure=True, execute=False, outputfinal=True,translationstepmult=translationstepmult, finestep=finestep, vintersectplane=numpy.array([0.0, 0.0, 0.0, 0.0]), chuckingdirection=manip.GetChuckingDirection())
                            if graspnoise > 0.00:
                                for i in range(20):
                                    contacts2_n,finalconfig2_n,mindist2_n,volume2_n = grasper.Grasp(direction=direction2, roll=roll2, position=position2, standoff=standoff, manipulatordirection=manipulatordirection, target=target2, graspingnoise = graspnoise, forceclosure=True, execute=False, outputfinal=True,translationstepmult=translationstepmult, finestep=finestep, vintersectplane=numpy.array([0.0, 0.0, 0.0, 0.0]), chuckingdirection=manip.GetChuckingDirection())
                                    if mindist2_n < 1e-4:
                                        mindist2 = mindist2_n
                                        break
                            print "mindists %f %f" % (mindist, mindist2)
                            if mindist > 1e-4 and mindist2 > 1e-4 and numpy.linalg.norm(finalconfig[1][:,3][0:3] - finalconfig2[1][:,3][0:3]) < 0.015:
                            # if True:
                                # grasper.robot.SetTransform(finalconfig[1])
                                # grasper.robot.SetDOFValues(finalconfig[0])
                                # drawContacts(contacts, grasper, env)
                                # env.UpdatePublishedBodies()
                                # raw_input('press any key to continue:(1) ')
                                # grasper.robot.SetTransform(finalconfig2[1])
                                # grasper.robot.SetDOFValues(finalconfig2[0])
                                # drawContacts(contacts2, grasper, env)
                                # env.UpdatePublishedBodies()
                                # raw_input('press any key to continue:(2) ')
                                grasper.robot.SetTransform(finalconfig[1])
                                success_grasp_list.append([contacts, contacts2, finalconfig, finalconfig2])
                            else:
                                if True: #numpy.linalg.norm(finalconfig[1][:,3][0:3] - finalconfig2[1][:,3][0:3]) < 0.015:
                                    half_success_grasp_list.append([contacts, contacts2, finalconfig, finalconfig2])
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
    robot_name = rospy.get_param("~robot_name", "hrp2")
    f = open('%s/.ros/temp_box.txt' % HOME_PATH)
    box = pickle.load(f)
    f.close()
    f = open('%s/.ros/temp_box_minus.txt' % HOME_PATH)
    box_minus = pickle.load(f)
    f.close()
    if True:
        convert_mesh(box, box_minus)
    if left_hand:
        rospy.loginfo("left hand")
    else:
        rospy.loginfo("right hand")
    env, hand1, hand2, robot, target1, target2, taskmanip, manip, manipulatordirection, gmodel, grasper = initialize_env(left_hand, robot_name)
    if not debug_mode:
        pass
        # env.SetViewer('qtcoin')
        # set_camera_pos(env, box)
    target2.Enable(False)
    target2.SetVisible(True)
    approachrays = return_box_approach_rays(gmodel, box)

    # target0 = env.GetKinBody('mug0')
    # if False:
    #     target1.Enable(False)
    #     target1.SetVisible(True)
    #     target0.Enable(True)
    #     target0.SetVisible(True)
    #     gmodel0 = databases.grasping.GraspingModel(robot,target0)
    #     approachrays = return_box_approach_rays(gmodel0, box)
    #     target0.Enable(False)
    #     target0.SetVisible(False)
    # else:
    #     target0.Enable(False)
    #     target0.SetVisible(False)
    #     approachrays = return_box_approach_rays(gmodel, box)
    # show_approachrays(approachrays, env)
    pose_array_msg = geometry_msgs.msg.PoseArray()
    com_array_msg = geometry_msgs.msg.PoseArray()
    float_array_msg_list = []
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
    if debug_mode:
        # env.SetViewer('qtcoin')
        # show_approachrays(approachrays, env)
        # show_box(box, env)
        trial(approachrays, success_grasp_list, half_success_grasp_list, len_approach, left_hand, robot_name) #debug
    else:
        load_and_save_trial(approachrays, success_grasp_list, half_success_grasp_list, len_approach, format_string)
    elapsed_time = time.time() - start
    print ("elapsed_time for trial:{0}".format(elapsed_time)) + "[sec]"
    pose_array_msg.header = box.header
    pose_array_msg.header.stamp = rospy.Time(0)
    # pose_array_msg.header.frame_id = "ground"
    print "Finished"
    print "Num!"
    print len(success_grasp_list)
    print "Num False!"
    print len(half_success_grasp_list)
    dim = box.dimensions
    dimx = dim.x/2+0.01
    dimy = dim.y/2+0.01
    dimz = dim.z/2+0.015
    b_position = box.pose.position
    p = numpy.array([b_position.x, b_position.y, b_position.z])
    box_pose_mat = tf.listener.xyzw_to_mat44(box.pose.orientation)
    box_pose_mat[:,3][0:3] = p
    if box_minus:
        box_minus_pose_mat = tf.listener.xyzw_to_mat44(box_minus.pose.orientation)
        box_minus_pose_mat[:,3][0:3] = p
        dim_minus = box_minus.dimensions
        dimx_minus = dim_minus.x/2
        dimy_minus = dim_minus.y/2
        dimz_minus = dim_minus.z/2 - 0.01
        print "dims"
        print dimx_minus, dimy_minus, dimz_minus
    full_success_grasp_list = []
    full_half_success_grasp_list = []
    for success_grasp_list, full_success_grasp_list in [
            (half_success_grasp_list, full_half_success_grasp_list), ## for debug
            (success_grasp_list, full_success_grasp_list)]:
        pose_array_msg = geometry_msgs.msg.PoseArray()
        com_array_msg = geometry_msgs.msg.PoseArray()
        float_array_msg_list = []
        for grasp_node in success_grasp_list:
            contact_num = 0
            ave_x = ave_y = ave_z = 0
            temp_pose = Pose()
            success_flug = True
            for contact in grasp_node[0]:
                if box_minus:
                    local_pos_minus = numpy.dot(numpy.linalg.inv(box_minus_pose_mat)
                                                , (contact[0], contact[1], contact[2], 1))[0:3]
                    if local_pos_minus[0] < dimx_minus and local_pos_minus[0] > -dimx_minus and local_pos_minus[1] < dimy_minus and local_pos_minus[1] > -dimy_minus and local_pos_minus[2] < dimz_minus and local_pos_minus[2] > -dimz_minus:
                        pass
                    else:
                        success_flug = False
                        print local_pos_minus
                ave_x = ave_x + contact[0]
                ave_y = ave_y + contact[1]
                ave_z = ave_z + contact[2]
                contact_num = contact_num + 1
                temp_pose.position.x = ave_x/contact_num
                temp_pose.position.y = ave_y/contact_num
                temp_pose.position.z = ave_z/contact_num
                local_pos = numpy.dot(numpy.linalg.inv(box_pose_mat)
                                      , (temp_pose.position.x, temp_pose.position.y, temp_pose.position.z, 1))[0:3]
            if local_pos[0] < dimx and local_pos[0] > -dimx and local_pos[1] < dimy and local_pos[1] > -dimy and local_pos[2] < dimz and local_pos[2] > -dimz:
                pass
            else:
                success_flug = False
            if success_flug:
                com_array_msg.poses.append(temp_pose)
                grasper.robot.SetTransform(grasp_node[2][1])
                pose_array_msg.poses.append(matrix2pose(robot.GetTransform()))
                float_array_msg_list.append(Float32MultiArray(data=grasp_node[2][0]))
                full_success_grasp_list.append(grasp_node)
    com_array_msg.header = pose_array_msg.header
    com_array_pub.publish(com_array_msg)
    pose_array_pub.publish(pose_array_msg)
    rave_grasp_array_msg = RaveGraspArray()
    rave_grasp_array_msg.pose_array = pose_array_msg
    rave_grasp_array_msg.header = pose_array_msg.header
    rave_grasp_array_msg.grasp_array = float_array_msg_list
    grasp_array_pub.publish(rave_grasp_array_msg)
    print "len full %d" % len(full_success_grasp_list)
    print "len half %d" % len(full_half_success_grasp_list)
    show_result(full_success_grasp_list, grasper, env)
    # gmodel.generate(*gmodel.autogenerateparams())
    ## respected to frame, kinfu outputs with camera frame.

def show_result(grasp_list_array, grasper, env):
    if env.GetViewer() is not None:
        for grasp_list in grasp_list_array:
            grasper.robot.SetTransform(grasp_list[2][1])
            grasper.robot.SetDOFValues(grasp_list[2][0])
            drawContacts(grasp_list[0], grasper, env)
            env.UpdatePublishedBodies()
            # raw_input('press any key to continue:(1) ')
            target2 = env.GetKinBody('mug2')
            target2.SetVisible(False)
            env.UpdatePublishedBodies()
            raw_input('press any key to continue:(1.5) ')
            target2.SetVisible(True)
            grasper.robot.SetTransform(grasp_list[3][1])
            grasper.robot.SetDOFValues(grasp_list[3][0])
            drawContacts(grasp_list[1], grasper, env)
            env.UpdatePublishedBodies()
            raw_input('press any key to continue:(2) ')

def drawContacts(contacts,grasper, env, conelength=0.07,transparency=0.5):
    if env.GetViewer() is not None:
        angs = numpy.linspace(0,2*numpy.pi,10)
        conepoints = numpy.r_[[[0,0,0]],conelength*numpy.c_[grasper.friction*numpy.cos(angs),grasper.friction*numpy.sin(angs),numpy.ones(len(angs))]]
        triinds = numpy.array(numpy.c_[numpy.zeros(len(angs)),range(2,1+len(angs))+[1],range(1,1+len(angs))].flatten(),int)
        allpoints = numpy.zeros((0,3))
        global contactlist
        for c in contacts:
            R = rotationMatrixFromQuat(quatRotateDirection(numpy.array((0,0,1)),c[3:6]))
            points = numpy.dot(conepoints,numpy.transpose(R)) + numpy.tile(c[0:3],(conepoints.shape[0],1))
            allpoints = numpy.r_[allpoints,points[triinds,:]]
        contactlist = env.drawtrimesh(points=allpoints,indices=None,colors=numpy.array((1,0.4,0.4,transparency)))

def shut_down_hook():
    print "shutting down node"

def marker_callback(msg):
    callback(get_box(msg))

def get_box(msg):
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
    return box

def marker_callback_clip(msg):
    box = get_box(msg)
    clip_box_pub.publish(box)

def grasp_finder():
    rospy.init_node('grasp_finder', anonymous=True)
    global pose_array_pub, com_array_pub, grasp_array_pub
    pose_array_pub = rospy.Publisher('~grasp_caluculation_result', geometry_msgs.msg.PoseArray, latch=True)
    com_array_pub = rospy.Publisher('~grasp_caluculation_com_result', geometry_msgs.msg.PoseArray, latch=True)
    grasp_array_pub = rospy.Publisher('~rave_grasp_result', RaveGraspArray, latch=True)
    clip_box_pub = rospy.Publisher('~attension_clip', BoundingBox, latch=False)
    rospy.Subscriber("/bounding_box_marker/selected_box", BoundingBox, callback)
    rospy.Subscriber("/select_box", String, marker_callback)
    rospy.Subscriber("/offset_box_publisher/output", BoundingBox, callback_minus)
    rospy.Subscriber("/attension_clip_request", String, marker_callback_clip)


def matrix2pose(matrix):
    quat = quatFromRotationMatrix(matrix[0:3, 0:3])
    pos = matrix[0:3,3]
    pose_msg = Pose()
    pose_msg.position = Point(pos[0], pos[1], pos[2])
    pose_msg.orientation = Quaternion(quat[1], quat[2], quat[3], quat[0])
    return pose_msg

def offset_direction(direction, roll, position, manipulatordirection, new_roll):
    pose_mat = poseFromGraspParams(direction, roll, position, manipulatordirection)
    quat = [numpy.cos(new_roll/2), numpy.sin(new_roll/2), 0, 0]
    quat_mat = matrixFromQuat(quat)
    new_roll_mat = numpy.dot(pose_mat, quat_mat)
    new_d, new_r, new_p = graspParamsFromPose(new_roll_mat, manipulatordirection)
    # pose2 = poseFromGraspParams(new_d, new_r, new_p, manipulatordirection)
    ## todo pose2 != new_roll_mat
    return new_d, new_r, new_p, new_roll_mat

def show_approachrays(approachrays, env): ## todo
    gapproachrays = numpy.c_[approachrays[:,0:3],approachrays[:,3:6]]
    N = approachrays.shape[0]
    global linelist
    linelist = [env.plot3(points=gapproachrays[:,0:3],pointsize=5,colors=numpy.array((1,0,0))), env.drawlinelist(points=numpy.reshape(numpy.c_[gapproachrays[:,0:3],gapproachrays[:,0:3]-0.0035*gapproachrays[:,3:6]],(2*N,3)),linewidth=2,colors=numpy.array((1,0,0,1)))]
    env.UpdatePublishedBodies()

def show_box(box, env):
    global linelist_box
    linelist_box = []
    pose44 = numpy.dot(tf.listener.xyz_to_mat44(box.pose.position), tf.listener.xyzw_to_mat44(box.pose.orientation))
    pos = pose44[0:3,3]
    dx = pose44[0:3,0] * box.dimensions.x / 2
    dy = pose44[0:3,1] * box.dimensions.y / 2
    dz = pose44[0:3,2] * box.dimensions.z / 2
    for d1 in (dx, dy, dz):
        for d2 in (dx, dy, dz):
            for d3 in (dx, dy, dz):
                if numpy.dot(d1, d2) > 1e-5 or numpy.dot(d2, d3) > 1e-5 or numpy.dot(d3, d1) > 1e-5:
                    continue
                for d2_a in (d2, -d2):
                    for d3_a in (d3, -d3):
                        linelist_box.append(env.drawlinelist(points=numpy.array([pos + d2_a + d3_a - d1,  pos + d2_a + d3_a + d1]), linewidth=10, colors=numpy.array((0, 1, 0, 1))))

def show():
    left_hand = rospy.get_param("~left_hand", False)
    robot_name = rospy.get_param("~robot_name", "hrp2")
    f = open('%s/.ros/temp_box.txt' % HOME_PATH)
    box = pickle.load(f)
    f.close()
    f = open('%s/.ros/temp_box_minus.txt' % HOME_PATH)
    box_minus = pickle.load(f)
    f.close()
    if left_hand:
        rospy.loginfo("left hand")
    else:
        rospy.loginfo("right hand")
    env, hand1, hand2, robot, target1, target2, taskmanip, manip, manipulatordirection, gmodel, grasper = initialize_env(left_hand, robot_name)
    env.SetViewer('qtcoin')
    show_box(box, env)
    env.GetViewer().SetSize(2500, 1600)
    time.sleep(2.0)
    set_camera_pos(env, box)
    check = commands.getoutput("gnome-screenshot --file=tmp.png")

def set_camera_pos(env, box):
    camera_pos = [box.pose.position.x, box.pose.position.y, box.pose.position.z - 0.8, 0]
    print camera_pos
    ex = [1.0, 0.0, 0.0, 0.0]
    ey = [0.0, 1.0, 0.0, 0.0]
    ez = [0.0, 0.0, 1.0, 0.0]
    ew = [0.0, 0.0, 0.0, 0.0]
    camera_pose = numpy.array([ex, ey, ez, camera_pos])
    camera_pose = camera_pose.transpose()
    env.GetViewer().SetCamera(camera_pose)

if __name__ == '__main__':
    grasp_finder()
    rospy.spin()

from openravepy import *
import numpy, time
from openrave_test_utility import *

print "hoge"
env=Environment()
print "huga"
env.Load('config/hrp.env.xml')
# env.Load('data/lab1.env.xml')
# env.SetViewer('qtcoin')
robot = env.GetRobots()[0]
target = env.GetKinBody('mug1')
# gmodel = databases.grasping.GraspingModel(robot,target)
taskmanip = interfaces.TaskManipulation(robot)
taskmanip.robot.SetDOFValues([90, 90, 0, 0, 0, 0])
manip = robot.GetActiveManipulator()
grasper = interfaces.Grasper(robot)
manipulatordirection = manip.GetLocalToolDirection()

standoffs = [0, 0.025]
direction = numpy.array([-1, 0, 1])
roll= numpy.pi / 4 * 2
position = numpy.array([0.3, 0, 0])

posematrix = poseFromGraspParams(direction, roll, position, manipulatordirection)
grasper.robot.SetTransform(posematrix)

RaveSetDebugLevel(DebugLevel.Debug)
robot.SetActiveManipulator(manip)
robot.SetTransform(numpy.eye(4))
robot.SetDOFValues([90, 90, 0, 0, 0, 0])
robot.SetActiveDOFs(manip.GetGripperIndices(),DOFAffine.X+DOFAffine.Y+DOFAffine.Z if True else 0)

contacts,finalconfig,mindist,volume = grasper.Grasp(direction=direction, roll=roll, position=position, standoff=standoffs[0], manipulatordirection=manipulatordirection, target=target, graspingnoise = 0.0, forceclosure=True, execute=False, outputfinal=True,translationstepmult=None, finestep=None, vintersectplane=numpy.array([0.0, 0.0, 0.0, 0.0]), chuckingdirection=manip.GetChuckingDirection())
grasper.robot.SetTransform(finalconfig[1])

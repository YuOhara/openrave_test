from openravepy import *
import numpy, time
env=Environment()
env.Load('hrp.env.xml')
# env.Load('data/lab1.env.xml')
env.SetViewer('qtcoin')
robot = env.GetRobots()[0]
target = env.GetKinBody('mug1')
# gmodel = databases.grasping.GraspingModel(robot,target)
taskmanip = interfaces.TaskManipulation(robot)
taskmanip.robot.SetDOFValues([90, 90, 0, 0, 0, 0])
manip = robot.GetActiveManipulator()
grasper = interfaces.Grasper(robot)
manipulatordirection = manip.GetLocalToolDirection()

standoffs = [0, 0.025]

## todo Generate Mat with pose
grasper.robot.SetTransform(matrixFromAxisAngle(numpy.array([1, 0, 0]), 0))
robot.SetActiveDOFs(self.manip.GetGripperIndices(),DOFAffine.X+DOFAffine.Y+DOFAffine.Z if translate else 0)
contacts,finalconfig,mindist,volume = grasper.Grasp(direction=numpy.array([1, 0, 0]), roll=0, position=numpy.array([1, 0, 0]), standoff=standoffs[0], manipulatordirection=manipulatordirection, target=target, graspingnoise = 0.0, forceclosure=True, execute=False, outputfinal=True,translationstepmult=None, finestep=None, vintersectplane=None, chuckingdirection=manip.GetChuckingDirection())

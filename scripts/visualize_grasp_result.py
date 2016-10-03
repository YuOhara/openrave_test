from openravepy import *
import numpy, time
env=Environment()
OPENRAVE_TEST_PATH = rospkg.RosPack().get_path("openrave_test")
env.Load('/scripts/config/hand_and_world.env.xml' % OPENRAVE_TEST_PATH)
# env.Load('data/lab1.env.xml')
env.SetViewer('qtcoin')
robot = env.GetRobots()[0]
target1 = env.GetKinBody('mug2')
gmodel = databases.grasping.GraspingModel(robot,target1)
taskmanip = interfaces.TaskManipulation(robot)
taskmanip.robot.SetDOFValues([90, 90, 0, 0, 0, 0])
# taskmanip.robot.SetDOFValues([0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0, 0.0, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.349066, -0.523599, 0.523599, 0.0, 0.0, 0.0, -0.174533, 0.261799, -0.349066, 0.523599, -0.523599, 0.0, 0.0, 0.0, -0.174533, -0.261799])
parser = databases.DatabaseGenerator.CreateOptionParser()
(options, hoge) = parser.parse_args(args=["--numthread=40"])

# gmodel.autogenerate()
# gmodel.autogenerate(options=options)
if not gmodel.load():
    print "model is empty!!"

# validgrasps, validindicees = gmodel.computeValidGrasps()
# gmodel.moveToPreshape(validgrasps[0])
# Tgoal = gmodel.getGlobalGraspTransform(validgrasps[0],collisionfree=True)
# basemanip = interfaces.BaseManipulation(robot)
# basemanip.MoveToHandPosition(matrices=[Tgoal])
# robot.WaitForController(0)
# taskmanip.CloseFingers()
# robot.WaitForController(0)

def show_result(result=gmodel.grasps):
    print "the total grasp is %d" % len(result)
    i = 0
    for grasp in result:
        print i
        gmodel.showgrasp(grasp)
        i = i+1

show_result()

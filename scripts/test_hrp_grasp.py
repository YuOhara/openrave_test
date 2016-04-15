from openravepy import *
import numpy, time
env=Environment()
env.Load('hrp.env.xml')
# env.Load('data/lab1.env.xml')
# env.SetViewer('qtcoin')
robot = env.GetRobots()[0]
target = env.GetKinBody('mug1')
gmodel = databases.grasping.GraspingModel(robot,target)
taskmanip = interfaces.TaskManipulation(robot)
taskmanip.robot.SetDOFValues([90, 90, 0, 0, 0, 0])
parser = databases.DatabaseGenerator.CreateOptionParser()
(options, hoge) = parser.parse_args(args=["--numthread=40"])

if not gmodel.load():
    gmodel.autogenerate()
    # gmodel.autogenerate(options=options)



# validgrasps, validindicees = gmodel.computeValidGrasps()
# gmodel.moveToPreshape(validgrasps[0])
# Tgoal = gmodel.getGlobalGraspTransform(validgrasps[0],collisionfree=True)
# basemanip = interfaces.BaseManipulation(robot)
# basemanip.MoveToHandPosition(matrices=[Tgoal])
# robot.WaitForController(0)
# taskmanip.CloseFingers()
# robot.WaitForController(0)
env.SetViewer('qtcoin')

def show_result():
    print "the total grasp is %d" % len(gmodel.grasps)
    for grasp in gmodel.grasps:
        gmodel.showgrasp(grasp)

show_result()

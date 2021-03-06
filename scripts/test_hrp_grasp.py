from openravepy import *
import numpy, time
env=Environment()
env.Load('config/hrp.env.xml')
# env.Load('data/lab1.env.xml')
env.SetViewer('qtcoin')

collisionChecker = RaveCreateCollisionChecker(env,"ode")
env.SetCollisionChecker(collisionChecker)

RaveSetDebugLevel(DebugLevel.Debug)
robot = env.GetRobots()[0]
target = env.GetKinBody('mug1')
gmodel = databases.grasping.GraspingModel(robot,target)
taskmanip = interfaces.TaskManipulation(robot)
taskmanip.robot.SetDOFValues([90, 90, 0, 0, 0, 0])
# taskmanip.robot.SetDOFValues([0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0, 0.0, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.349066, -0.523599, 0.523599, 0.0, 0.0, 0.0, -0.174533, 0.261799, -0.349066, 0.523599, -0.523599, 0.0, 0.0, 0.0, -0.174533, -0.261799])
parser = databases.DatabaseGenerator.CreateOptionParser()
(options, hoge) = parser.parse_args(args=["--numthread=1"])

gmodel.autogenerate()
# gmodel.autogenerate(options=options)
# if not gmodel.load():
#     gmodel.autogenerate()
    # gmodel.autogenerate(options=options)



# validgrasps, validindicees = gmodel.computeValidGrasps()
# gmodel.moveToPreshape(validgrasps[0])
# Tgoal = gmodel.getGlobalGraspTransform(validgrasps[0],collisionfree=True)
# basemanip = interfaces.BaseManipulation(robot)
# basemanip.MoveToHandPosition(matrices=[Tgoal])
# robot.WaitForController(0)
# taskmanip.CloseFingers()
# robot.WaitForController(0)

removed_grasp = []
def get_removed():
    global removed_grasp
    removed_grasp = []
    for grasp in gmodel.grasps:
        T = gmodel.getGlobalGraspTransform(grasp)
        if T[1][3] < -0.03:
            removed_grasp.append(grasp)

def show_result(result=gmodel.grasps):
    print "the total grasp is %d" % len(result)
    i = 0
    for grasp in result:
        print i
        gmodel.showgrasp(grasp)
        i = i+1

get_removed()
show_result(removed_grasp)

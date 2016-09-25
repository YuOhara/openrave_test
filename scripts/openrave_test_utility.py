from openravepy import *
import numpy, time
import rospy
from jsk_recognition_msgs.msg import BoundingBox
from std_srvs.srv import Empty
import geometry_msgs.msg
import tf
from tf import transformations
import commands

def return_box_approach_rays(gmodel, box):
    # ode gives the most accurate rays
    rospy.loginfo("start approach rays")
    env = gmodel.env
    delta = 0.01
    normalanglerange = 0
    target = gmodel.target
    cc = RaveCreateCollisionChecker(env,'ode')
    dim = box.dimensions
    position = box.pose.position
    orientation = box.pose.orientation
    # temp
    # from orientation
    mat44 = tf.listener.xyzw_to_mat44(box.pose.orientation)
    ex = mat44[:,0][0:3]
    ey = mat44[:,1][0:3]
    ez = mat44[:,2][0:3]
    # temp
    try:
        with target:
            # target.SetTransform(numpy.eye(4))
            # ab = target.ComputeAABB()
            p = numpy.array([position.x, position.y, position.z])
            dim.x = dim.x/2
            dim.y = dim.y/2
            dim.x = dim.z/2
            dx = ex * dim.x
            dy = ey * dim.y
            dz = ez * dim.z
            print "dims! %f %f %f" % (dim.x, dim.y, dim.z)
            sides = numpy.array((numpy.r_[dz, -ez, dx, dy],
                                 numpy.r_[-dz, ez, dx, dy],
                                 numpy.r_[dy, -ey, dx, dz],
                                 numpy.r_[-dy, ey, dx, dz],
                                 numpy.r_[dx, -ex, dy, dz],
                                 numpy.r_[-dx, ex, dy, dz]
                       ))
            maxlen = 2*numpy.sqrt(dim.x**2 + dim.y**2 + dim.z**2)+0.03
            approachrays = numpy.zeros((0,6))
            for side in sides:
                ex = numpy.sqrt(sum(side[6:9]**2))
                ey = numpy.sqrt(sum(side[9:12]**2))
                if ex/delta > 1000:
                    raise ValueError('object is way too big for its discretization! %f > 1000'%(ex/delta))
                XX,YY = numpy.meshgrid(numpy.r_[numpy.arange(-ex,-0.25*delta,delta),0,numpy.arange(delta,ex,delta)],
                                 numpy.r_[numpy.arange(-ey,-0.25*delta,delta),0,numpy.arange(delta,ey,delta)])
                localpos = numpy.outer(XX.flatten(),side[6:9]/ex)+numpy.outer(YY.flatten(),side[9:12]/ey)
                N = localpos.shape[0]
                rays = numpy.c_[numpy.tile(p+side[0:3],(N,1))+localpos,maxlen*numpy.tile(side[3:6],(N,1))]
                collision, info = cc.CheckCollisionRays(rays,target)
                # make sure all normals are the correct sign: pointing outward from the object)
                newinfo = info[collision,:]
                if len(newinfo) > 0:
                    newinfo[sum(rays[collision,3:6]*newinfo[:,3:6],1)>0,3:6] *= -1
                    approachrays = numpy.r_[approachrays,newinfo]
            if normalanglerange > 0:
                theta,pfi = SpaceSamplerExtra().sampleS2(angledelta=directiondelta)
                dirs = numpy.c_[numpy.cos(theta),numpy.sin(theta)*numpy.cos(pfi),numpy.sin(theta)*numpy.sin(pfi)]
                dirs = numpy.array([dir for dir in dirs if numpy.arccos(dir[2])<=normalanglerange]) # find all dirs within normalanglerange
                if len(dirs) == 0:
                    dirs = numpy.array([[0,0,1]])
                newapproachrays = numpy.zeros((0,6))
                for approachray in approachrays:
                    R = rotationMatrixFromQuat(quatRotateDirection(numpy.array((0,0,1)),approachray[3:6]))
                    newapproachrays = numpy.r_[newapproachrays,c_[numpy.tile(approachray[0:3],(len(dirs),1)),numpy.dot(dirs,transpose(R))]]
                approachrays = newapproachrays
            return approachrays
    finally:
        cc.DestroyEnvironment()

def return_sphere_approach_rays(gmodel, box):
    # ode gives the most accurate rays
    rospy.loginfo("start approach rays")
    env = gmodel.env
    delta = 0.02
    normalanglerange = 0
    target = gmodel.target
    cc = RaveCreateCollisionChecker(env,'ode')
    dim = box.dimensions
    position = box.pose.position
    try:
        with target:
            # target.SetTransform(numpy.eye(4))
            maxlen = numpy.sqrt(dim.x * dim.x + dim.y * dim.y + dim.z * dim.z)+0.03
            theta,pfi = misc.SpaceSamplerExtra().sampleS2(angledelta=delta)
            dirs = numpy.c_[numpy.cos(theta),numpy.sin(theta)*numpy.cos(pfi),numpy.sin(theta)*numpy.sin(pfi)]
            rays = numpy.c_[numpy.tile(numpy.array([position.x, position.y, position.z]),(len(dirs),1))-maxlen*dirs,2*maxlen*dirs]
            collision, info = cc.CheckCollisionRays(rays,target)
            # make sure all normals are the correct sign: pointing outward from the object)
            approachrays = info[collision,:]
            if len(approachrays) > 0:
                approachrays[sum(rays[collision,3:6]*approachrays[:,3:6],1)>0,3:6] *= -1
            if normalanglerange > 0:
                theta,pfi = misc.SpaceSamplerExtra().sampleS2(angledelta=directiondelta)
                dirs = numpy.c_[numpy.cos(theta),numpy.sin(theta)*numpy.cos(pfi),numpy.sin(theta)*numpy.sin(pfi)]
                dirs = array([dir for dir in dirs if numpy.arccos(dir[2])<=normalanglerange]) # find all dirs within normalanglerange
                if len(dirs) == 0:
                    dirs = array([[0,0,1]])
                newapproachrays = numpy.zeros((0,6))
                for approachray in approachrays:
                    R = rotationMatrixFromQuat(quatRotateDirection(array((0,0,1)),approachray[3:6]))
                    newapproachrays = numpy.r_[newapproachrays,numpy.c_[numpy.tile(approachray[0:3],(len(dirs),1)),numpy.dot(dirs,transpose(R))]]
                approachrays = newapproachrays
            return approachrays
    finally:
        cc.DestroyEnvironment()

def poseFromGraspParams(direction, roll, position, manipulatordirection):
    tbasematrix = matrixFromQuat(quatFromAxisAngle(manipulatordirection, roll))
    posematrix_tmp = matrixFromQuat(quatRotateDirection(manipulatordirection, direction))
    posematrix = numpy.dot(posematrix_tmp, tbasematrix)
    posematrix[0:3,3] = position
    return posematrix

def graspParamsFromPose(pose, manipulatordirection):
    ## maybe incorect
    ## not used in
    direction_2 = numpy.dot(pose[0:3, 0:3], manipulatordirection)
    posematrix_tmp_2 = matrixFromQuat(quatRotateDirection(manipulatordirection, direction_2))
    position_2 = pose[0:3, 3]
    tbasematrix_2 = numpy.dot(numpy.linalg.inv(posematrix_tmp_2), pose)
    # roll_2 = numpy.linalg.norm(axisAngleFromQuat(quatFromRotationMatrix(tbasematrix_2[0:3, 0:3])))
    # roll_2 = axisAngleFromQuat(quatFromRotationMatrix(tbasematrix_2[0:3, 0:3]))[3]
    roll_2 = rollFromQuat(quatFromRotationMatrix(tbasematrix_2[0:3, 0:3]), manipulatordirection)
    return direction_2, roll_2, position_2

def rollFromQuat(quat, manipulatordirection):
    sinang = quat[1]*quat[1] + quat[2]*quat[2] + quat[3]*quat[3]
    if sinang == 0:
        return 0
    sinang = numpy.sqrt(sinang)
    if (numpy.dot(quat[1:4], manipulatordirection) > 0 and quat[0] > 0) or (numpy.dot(quat[1:4], manipulatordirection) < 0 and quat[0] < 0):
        return numpy.arcsin(sinang) * 2
    else:
        return (numpy.pi - numpy.arcsin(sinang)) * 2

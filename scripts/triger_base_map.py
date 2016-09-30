#!/usr/bin/env python

from std_srvs.srv import *
import rospy
import numpy
import tf

def get_transform():
    try:
        (trans,rot) = listener.lookupTransform('kinfu_origin', "odom", rospy.Time(0))
        return tf.transformations.concatenate_matrices(
            tf.transformations.translation_matrix(trans),
            tf.transformations.quaternion_matrix(rot)
            )
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        return []

def start_record(req):
    rospy.loginfo("fuga")

    global recorded_transform, transform_odom_start
    recorded_transform = get_transform()
    transform_odom_start = transform_odom
    if len(recorded_transform) == 0:
        return False
    else:
        return EmptyResponse()

def stop_record(req):
    global recorded_transform
    recorded_transform = []
    return EmptyResponse()

def publish_tf(event):
    global transform_odom, recorded_transform
    if len(recorded_transform) != 0:
        recorded_transform_new =  get_transform()
        if len(recorded_transform_new) != 0:
            transform_diff = numpy.dot(numpy.linalg.inv(recorded_transform), recorded_transform_new)
            transform_odom = numpy.dot(transform_diff, transform_odom_start)
    transform_odom_inv = numpy.linalg.inv(transform_odom)
    br.sendTransform(tf.transformations.translation_from_matrix(transform_odom_inv),
                     tf.transformations.quaternion_from_matrix(transform_odom_inv),
                     rospy.Time.now(),
                     "triger_base_map",
                     "odom"
    )

if __name__ == "__main__":
    rospy.init_node("triger_base_map")
    global transform_odom, listener, br, recorded_transform
    transform_odom = numpy.eye(4)
    recorded_transform = []
    listener = tf.TransformListener()
    br = tf.TransformBroadcaster()
    rospy.Service("start_record", Empty, start_record)
    rospy.Service("stop_record", Empty, stop_record)
    rospy.Timer(rospy.Duration(0.04), publish_tf)
    rospy.spin()

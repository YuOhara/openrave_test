#include "ros/ros.h"
#include <image_geometry/pinhole_camera_model.h>
#include "std_msgs/String.h"
#include "geometry_msgs/PoseArray.h"
#include "sensor_msgs/CameraInfo.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <fstream>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <tf_conversions/tf_kdl.h>
#include <tf/transform_listener.h>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <math.h>
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */

class CPoint {
public:
  CPoint() {}
  float x,y,frame;
};

class CSimpleTrack {
public:
  CSimpleTrack() {}
  int mLabel;
  std::vector<CPoint> mPoints;
};

sensor_msgs::CameraInfo::ConstPtr cam_info_;
tf::TransformListener tf_listener_;
std::vector<Eigen::Vector3d> grasp_array_ground;
std::vector<Eigen::Vector3d> com_array_ground;


void readTracks(std::vector<CSimpleTrack> &mTracks) {
  // open a file
  std::ifstream aFile("/home/leus/ros/indigo/src/openrave_test/moseg/TrainingSet/Results/OchsBroxMalik4_all_0000020.00/fromrobot/Tracks20.dat");
  // Read number of frames considered
  int mSequenceLength;
  aFile >> mSequenceLength;
  // Read number of tracks
  int aCount;
  aFile >> aCount;
  // Read each track
  for (int i = 0; i < aCount; i++) {
    // Read label and length of track
    int aSize;
    mTracks.push_back(CSimpleTrack());
    CSimpleTrack& aTrack = mTracks.back();
    aFile >> aTrack.mLabel;
    aFile >> aSize;
    aTrack.mPoints.resize(aSize);
    // Read x,y coordinates and frame number of the tracked point 
    for (int j = 0; j < aSize; j++) {
      aFile >> aTrack.mPoints[j].x;
      aFile >> aTrack.mPoints[j].y;
      aFile >> aTrack.mPoints[j].frame;
    }
  }
}

void graspPoseCallback(const geometry_msgs::PoseArrayConstPtr& grasp, const geometry_msgs::PoseArrayConstPtr& com)
{
  // transform to ground frame
}

void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& info)
{
  cam_info_ = info;
}

void loadMovementFile()
{
  if (!cam_info_) {
    ROS_INFO("no camera info is available");
    return;
  }
  std::vector<CSimpleTrack> mTracks;
  readTracks(mTracks);
  // move to camera frame
  tf::StampedTransform tf_transform;
  ros::Time now = ros::Time::now();
  tf_listener_.waitForTransform(cam_info_->header.frame_id, "ground", now, ros::Duration(2.0));
  tf_listener_.lookupTransform(cam_info_->header.frame_id, "ground", now, tf_transform);
  Eigen::Affine3d transform;
  tf::transformTFToEigen(tf_transform, transform);
  // project all grasps to movement
  image_geometry::PinholeCameraModel model;
  bool model_success_p = model.fromCameraInfo(cam_info_);
  if (!model_success_p) {
    ROS_INFO("failed to create camera model");
    return;
  }
  int hand_label = 1;
  std::vector<cv::Point2d> local_points;
  for (size_t i = 0; i < com_array_ground.size(); i++) {
    Eigen::Vector3d grasp_cam;
    grasp_cam = transform * com_array_ground[i];
    cv::Point3d p(grasp_cam.x(), grasp_cam.y(), grasp_cam.z());
    cv::Point2d uv = model.project3dToPixel(p);
    local_points.push_back(uv);
    double min = 10000;
    int label = -1;
    for (size_t j = 0; j < mTracks.size(); j++) {
      double dist = pow((uv.x - mTracks[j].mPoints.back().x), 2.0) + pow((uv.y - mTracks[j].mPoints.back().y), 2.0);
      if (min > dist){
        label = mTracks[j].mLabel;
        min = dist;
      }
    }
    if (label != hand_label) {
      // push back grasp pose
      
    }
  }

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "second_grasp_finder");
  ros::NodeHandle n;

  message_filters::Subscriber<geometry_msgs::PoseArray> grasp_sub(n, "image", 1);
  message_filters::Subscriber<geometry_msgs::PoseArray> com_sub(n, "camera_info", 1);
  typedef message_filters::sync_policies::ExactTime<geometry_msgs::PoseArray, geometry_msgs::PoseArray> MySyncPolicy;
  // ExactTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), grasp_sub, com_sub);
  sync.registerCallback(boost::bind(&graspPoseCallback, _1, _2));

  return 0;
}

// subscribe grasp pose

// subscribe center pose

// load movement file

// find point before

// find near point

// publish grasp pose in moved points

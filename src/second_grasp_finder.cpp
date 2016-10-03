#include "ros/ros.h"
#include <image_geometry/pinhole_camera_model.h>
#include "std_msgs/String.h"
#include "geometry_msgs/PoseArray.h"
#include "sensor_msgs/CameraInfo.h"
#include "openrave_test/SecondGrasp.h"
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
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <openrave_test/RaveGraspArray.h>
#include <rospack/rospack.h>
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
image_geometry::PinholeCameraModel model_;
tf::TransformListener *tf_listener_;
std::vector<Eigen::Affine3d> grasp_array_odom;
std::vector<Eigen::Vector3d> com_array_odom;
std::vector<std_msgs::Float32MultiArray> finger_angle_array_;
ros::Publisher *second_grasp_array_pub_;
ros::Publisher *second_grasp_array_debug_pub_;
ros::Publisher *second_rave_grasp_array_pub_;
cv::Mat debug_img = cv::Mat::zeros(500, 500, CV_8UC3);

void readTracks(std::vector<CSimpleTrack> &mTracks) {
  // open a file

  rospack::Rospack rp;
  std::vector<std::string> search_path;
  rp.getSearchPathFromEnv(search_path);
  rp.crawl(search_path, 1);
  std::string path;
  if (rp.find("openrave_test",path)==false) {
    ROS_ERROR("openrave_test not found!");
    return;
  }
  std::ifstream aFile((path + std::string("/moseg/TrainingSet/Results/OchsBroxMalik4_all_0000020.00/fromrobot/Tracks20.dat")).c_str());
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

Eigen::Affine3d getTransform(std::string from, std::string to) {
  tf::StampedTransform tf_transform;
  ros::Time now = ros::Time::now();
  tf_listener_->waitForTransform(from, to, now, ros::Duration(2.0));
  tf_listener_->lookupTransform(from, to,
                                //ros::Time(0)
                                now
                                , tf_transform);
  Eigen::Affine3d transform;
  tf::transformTFToEigen(tf_transform, transform);
  return transform;
}


int getLabel(double x, double y, std::vector<CSimpleTrack> &mTracks)
{
  double min = 10000;
  int label = -1;
  for (int j = 0; j < mTracks.size(); j++) {
    double dist = pow((x - mTracks[j].mPoints.front().x), 2.0) + pow((y - mTracks[j].mPoints.front().y), 2.0);
    if (min > dist){
      label = mTracks[j].mLabel;
      min = dist;
    }
  }
  std::stringstream ss;
  ss << " label: " << label << " val: " << min;
  std::string output_txt = ss.str();
  cv::circle(debug_img, cv::Point(x, y), 3, cv::Scalar(200,0,0), -1, CV_AA);
  cv::putText(debug_img, output_txt.c_str(), cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,200), 2, CV_AA);
  return label;
}

int getLabel(Eigen::Vector3d grasp_cam, std::vector<CSimpleTrack> &mTracks)
{
    cv::Point3d p(grasp_cam.x(), grasp_cam.y(), grasp_cam.z());
    cv::Point2d uv = model_.project3dToPixel(p);
    ROS_INFO("%f, %f, %f -> %f, %f", p.x, p.y, p.z, uv.x, uv.y);
    return getLabel(uv.x, uv.y, mTracks);
}

void graspPoseCallback(const openrave_test::RaveGraspArrayConstPtr& rave_grasp, const geometry_msgs::PoseArrayConstPtr& com)
{
  // transform to odom frame
  const geometry_msgs::PoseArray *grasp;
  grasp = &(rave_grasp -> pose_array);
  ROS_INFO("recieve grasp coords");
  Eigen::Affine3d transform = getTransform("triger_base_map", grasp->header.frame_id);
  grasp_array_odom.clear();
  com_array_odom.clear();
  for (int i=0; i<grasp->poses.size(); i++) {
    Eigen::Affine3d grasp_affine;
    Eigen::Affine3d com_affine_pose;
    tf::poseMsgToEigen(grasp->poses[i], grasp_affine);
    tf::poseMsgToEigen(com->poses[i], com_affine_pose);
    Eigen::Vector3d com_affine = com_affine_pose.translation();
    grasp_affine = transform * grasp_affine;
    com_affine = transform * com_affine;
    grasp_array_odom.push_back(grasp_affine);
    com_array_odom.push_back(com_affine);
    finger_angle_array_.push_back(rave_grasp->grasp_array[i]);
  }
}

void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& info)
{
  if (!cam_info_) {
    cam_info_ = info;
  }
}

bool loadMovementFile(openrave_test::SecondGrasp::Request  &req,
                      openrave_test::SecondGrasp::Response &res)
{
  if (!cam_info_) {
    ROS_INFO("no camera info is available");
    return false;
  }
  std::vector<CSimpleTrack> mTracks;
  readTracks(mTracks);
  // debug_img = cv::Mat::zeros(cam_info_->height, cam_info_->width, CV_8UC3);
  {
    std::stringstream ss;
    ss << std::getenv("HOME") << "/.ros/left0000.jpg";
    debug_img = cv::imread(ss.str().c_str());
  }
  // move to camera frame
  Eigen::Affine3d transform = getTransform(cam_info_->header.frame_id, "/triger_base_map");
  // project all grasps to movement
  bool model_success_p = model_.fromCameraInfo(cam_info_);
  if (!model_success_p) {
    ROS_INFO("failed to create camera model");
    return false;
  }
  int hand_label = getLabel(getTransform(cam_info_->header.frame_id, "rarm_end_coords").translation() ,mTracks);
  ROS_INFO("label! %d", hand_label);
  int back_label = 0; // may be 0
  geometry_msgs::PoseArray second_grasp_pose_array;
  geometry_msgs::PoseArray second_grasp_pose_array_debug;
  // get transform
  std::vector<std_msgs::Float32MultiArray> finger_angle_array_out;
  for (int i = 0; i < com_array_odom.size(); i++) {
    Eigen::Vector3d grasp_cam;
    grasp_cam = transform * com_array_odom[i];
    int label = getLabel(grasp_cam, mTracks);
    ROS_INFO("label%d:, %d", i, label);
    if (label != hand_label && label != back_label && label != -1) {
      // push back grasp pose
      ROS_INFO("succeeded");
      geometry_msgs::Pose grasp_pose;
      tf::poseEigenToMsg(grasp_array_odom[i], grasp_pose);
      second_grasp_pose_array.poses.push_back(grasp_pose);
      finger_angle_array_out.push_back(finger_angle_array_[i]);
    }
    else {
      geometry_msgs::Pose grasp_pose;
      Eigen::Affine3d debug_grasp = grasp_array_odom[i];
      debug_grasp.translation() = grasp_cam;
      tf::poseEigenToMsg(debug_grasp, grasp_pose);
      second_grasp_pose_array_debug.poses.push_back(grasp_pose);
    }
  }
  second_grasp_pose_array.header.frame_id = "/triger_base_map";
  second_grasp_pose_array.header.stamp = ros::Time::now();
  second_grasp_pose_array_debug.header = second_grasp_pose_array.header;
  second_grasp_pose_array_debug.header.frame_id = cam_info_->header.frame_id; // debug
  second_grasp_array_pub_->publish(second_grasp_pose_array);
  second_grasp_array_debug_pub_->publish(second_grasp_pose_array_debug);
  openrave_test::RaveGraspArray second_rave_grasp_array;
  second_rave_grasp_array.pose_array = second_grasp_pose_array;
  second_rave_grasp_array.grasp_array = finger_angle_array_out;
  second_rave_grasp_array_pub_->publish(second_rave_grasp_array);
  res.second_grasp_pose_array = second_grasp_pose_array;
  // cv::imshow("debug_moseg", debug_img);
  {
    std::stringstream ss;
    ss << std::getenv("HOME") << "/.ros/test.jpg";
    cv::imwrite(ss.str().c_str(), debug_img);
  }
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "second_grasp_finder");
  ros::NodeHandle n;
  second_grasp_array_pub_ = new ros::Publisher(n.advertise<geometry_msgs::PoseArray>("second_grasp_array", 1000));
  second_grasp_array_debug_pub_ = new ros::Publisher(n.advertise<geometry_msgs::PoseArray>("second_grasp_array_false", 1000));
  second_rave_grasp_array_pub_ = new ros::Publisher(n.advertise<openrave_test::RaveGraspArray>("second_rave_grasp_array", 1000));
  tf_listener_ = new tf::TransformListener();
  message_filters::Subscriber<openrave_test::RaveGraspArray> grasp_sub(n, "/grasp_finder_left/rave_grasp_result", 1);
  message_filters::Subscriber<geometry_msgs::PoseArray> com_sub(n, "/grasp_finder_left/grasp_caluculation_com_result", 1);
  typedef message_filters::sync_policies::ExactTime<openrave_test::RaveGraspArray, geometry_msgs::PoseArray> MySyncPolicy;
  // ExactTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), grasp_sub, com_sub);
  sync.registerCallback(boost::bind(&graspPoseCallback, _1, _2));
  ros::Subscriber sub = n.subscribe("camera_info", 1000, cameraInfoCallback);
  ros::ServiceServer service = n.advertiseService("second_grasp", loadMovementFile);
  grasp_array_odom.clear();
  com_array_odom.clear();
  ros::spin();
}

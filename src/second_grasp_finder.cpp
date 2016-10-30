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

void readTracks(std::vector<CSimpleTrack> &mTracks, std::string file_name=std::string("/moseg/TrainingSet/Results/OchsBroxMalik4_all_0000020.00/fromrobot/Tracks20.dat")) {
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
  std::ifstream aFile((path + file_name).c_str());
  if(!aFile.is_open())
  {
    ROS_WARN("track file not exists");
    return;
  }
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
                                ros::Time(0)
                                // now
                                , tf_transform);
  Eigen::Affine3d transform;
  tf::transformTFToEigen(tf_transform, transform);
  return transform;
}

CPoint getBefore(CPoint after_point, std::vector<CSimpleTrack> &mTracksGrasp) {
  if (mTracksGrasp.size() == 0)
    {
      return after_point;
    }
  double x = after_point.x; double y = after_point.y;
  double min = 10000000;
  int index = -1;
  for (int j = 0; j < mTracksGrasp.size(); j++) {
    double dist = pow((x - mTracksGrasp[j].mPoints.back().x), 2.0) + pow((y - mTracksGrasp[j].mPoints.back().y), 2.0);
    if (min > dist){
      min = dist;
      index = j;
    }
  }
  if (index == -1) {ROS_INFO("hoge"); return after_point; }

  CPoint return_point;
  return_point.x = mTracksGrasp[index].mPoints.front().x;
  return_point.y = mTracksGrasp[index].mPoints.front().y;
  return return_point;
}

int getLabel(double x, double y, std::vector<CSimpleTrack> &mTracks, std::vector<CSimpleTrack> &mTracksGrasp, cv::Mat &H)
{
  int label = -1;
  int store_size = 8;
  std::vector<double> mins(store_size);
  std::vector<CPoint> points(store_size);
  std::vector<CPoint> points_after(store_size);
  std::fill( mins.begin(), mins.end(), 500);
  int filled_size = 0;
  for (int j = 0; j < mTracks.size(); j++) {
    if (mTracks[j].mPoints.size() != 20) {
      continue;
    }
    CPoint after_point = mTracks[j].mPoints.front();
    CPoint before_point = after_point;//getBefore(after_point, mTracksGrasp);
    double dist = pow((x - before_point.x), 2.0) + pow((y - before_point.y), 2.0);
    std::vector<double>::iterator it = mins.begin();
    std::vector<CPoint>::iterator point_it = points.begin();
    std::vector<CPoint>::iterator point_after_it = points_after.begin();
    for (;it!=mins.end();)
    {
      if (*it > dist){
        mins.insert(it, dist);
        points.insert(point_it, before_point);
        points_after.insert(point_after_it, 
                            before_point
                            //mTracks[j].mPoints.back()
                            );
        mins.pop_back();
        points.pop_back();
        points_after.pop_back();
        filled_size++;
        if (it == mins.begin()) {
          label = mTracks[j].mLabel;
        }
        break;
      }
      it++; point_it++; point_after_it++;
    }
  }
  if (filled_size < store_size) {ROS_INFO("hoge"); return -1; }
  for (int i=0; i<store_size; i++){
    ROS_INFO("min %f", mins[i]);
  }
  // std::vector<cv::Point2f>
  //   before_points(store_size), after_points(store_size);
  cv::Point2f
    before_points[store_size], after_points[store_size];
  for (int i=0; i<store_size; i++) {
    before_points[i] = cv::Point2f(points[i].x, points[i].y);
    after_points[i] = cv::Point2f(points_after[i].x, points_after[i].y);
    ROS_INFO("%f %f -> %f %f", points[i].x, points[i].y, points_after[i].x, points_after[i].y);
  }
  // cv::Mat(before_points);
  cv::Mat before_points_mat (cv::Size(store_size, 1), CV_32FC2, before_points);
  cv::Mat after_points_mat (cv::Size(store_size, 1), CV_32FC2, after_points);
  H = cv::findHomography(before_points_mat, after_points_mat, CV_RANSAC, 3 /* reprojection thre*/);
  ROS_INFO("hige %f %f %f", H.at<double>(0, 0),  H.at<double>(0, 1),  H.at<double>(0, 2));

  std::stringstream ss;
  ss << " label: " << label;
  std::string output_txt = ss.str();
  cv::circle(debug_img, cv::Point(x, y), 3, cv::Scalar(200,0,0), -1, CV_AA);
  cv::putText(debug_img, output_txt.c_str(), cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,200), 2, CV_AA);
  return label;
}

int getLabel(Eigen::Vector3d grasp_cam, std::vector<CSimpleTrack> &mTracks, std::vector<CSimpleTrack> &mTracksGrasp, cv::Mat &H)
{
    cv::Point3d p(grasp_cam.x(), grasp_cam.y(), grasp_cam.z());
    cv::Point2d uv = model_.project3dToPixel(p);
    return getLabel(uv.x, uv.y, mTracks, mTracksGrasp, H);
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
    bool model_success_p = model_.fromCameraInfo(cam_info_);
    if (!model_success_p) {
      ROS_INFO("failed to create camera model");
      return;
    }
  }
}

Eigen::Affine3d transFromHomography(cv::Mat H)
{
  // cv::Mat pose = cv::Mat::eye(3, 4, CV_64FC1);      // 3x4 matrix, the camera pose
  // float norm1 = (float)norm(H.col(0));  
  // float norm2 = (float)norm(H.col(1));  
  // float tnorm = (norm1 + norm2) / 2.0f; // Normalization value
  // cv::Mat p1 = H.col(0);       // Pointer to first column of H
  // cv::Mat p2 = pose.col(0);    // Pointer to first column of pose (empty)
  // cv::normalize(p1, p2);   // Normalize the rotation, and copies the column to pose
  // ROS_INFO("hage %f %f %f", H.col(0).at<double>(0),  H.col(0).at<double>(1),  H.col(0).at<double>(2));
  // p1 = H.col(1);           // Pointer to second column of H
  // p2 = pose.col(1);        // Pointer to second column of pose (empty)
  // cv::normalize(p1, p2);   // Normalize the rotation and copies the column to pose
  // p1 = pose.col(0);
  // p2 = pose.col(1);
  // cv::Mat p3 = p1.cross(p2);   // Computes the cross-product of p1 and p2
  // cv::Mat c2 = pose.col(2);    // Pointer to third column of pose
  // p3.copyTo(c2);       // Third column is the crossproduct of columns one and two
  // pose.col(3) = H.col(2) / tnorm;  //vector t [R|t] is the last column of pose
  // tf::Transform checktf;
  // ROS_INFO("piyo %f %f %f", pose.at<double>(0, 3), pose.at<double>(1, 3), pose.at<double>(2, 3));
  // checktf.setOrigin( tf::Vector3(pose.at<double>(0, 3), pose.at<double>(1, 3), pose.at<double>(2, 3)) );
  // tf::Matrix3x3 rotation_mat(pose.at<double>(0, 0), pose.at<double>(0, 1), pose.at<double>(0, 2),
  //                            pose.at<double>(1, 0), pose.at<double>(1, 1), pose.at<double>(1, 2),
  //                            pose.at<double>(2, 0), pose.at<double>(2, 1), pose.at<double>(2, 2));
  // checktf.setBasis(rotation_mat);
  // Eigen::Affine3d transform;
  // tf::transformTFToEigen(checktf, transform);
  // return transform;

  cv::Point2f corners2d[4] = {cv::Point2f(0,0),
                              cv::Point2f(100,0),
                              cv::Point2f(100,100),
                              cv::Point2f(0,100)};
  cv::Mat corners2d_mat (cv::Size(4, 1), CV_32FC2, corners2d);
  cv::Point3f corners3d[4] = {cv::Point3f(0,0,0),
                              cv::Point3f(0,0.1,0),
                              cv::Point3f(0.1,0.1,0),
                              cv::Point3f(0.1,0,0)};
  cv::Mat corners3d_mat (cv::Size(4, 1), CV_32FC3, corners3d);
  cv::Mat corners2d_mat_trans;
  cv::perspectiveTransform (corners2d_mat, corners2d_mat_trans, H);
  corners2d_mat.at<cv::Point2f>();
  ROS_INFO("translated corner (%f %f) (%f %f)...",
           corners2d_mat_trans.at<cv::Point2f>(0).x,
           corners2d_mat_trans.at<cv::Point2f>(0).y,
           corners2d_mat_trans.at<cv::Point2f>(1).x,
           corners2d_mat_trans.at<cv::Point2f>(1).y
           );
  double fR3[3], fT3[3];
  cv::Mat rvec(3, 1, CV_64FC1, fR3);
  cv::Mat tvec(3, 1, CV_64FC1, fT3);
  cv::Mat zero_distortion_mat = cv::Mat::zeros(4, 1, CV_64FC1);
  cv::solvePnP (corners3d_mat, corners2d_mat_trans, 
                model_.intrinsicMatrix(),
                zero_distortion_mat,//if unrectified: pcam.distortionCoeffs()
                rvec, tvec);

  tf::Transform checktf;
  checktf.setOrigin( tf::Vector3(fT3[0], fT3[1], fT3[2] ) );
  double rx = fR3[0], ry = fR3[1], rz = fR3[2];
  tf::Quaternion quat;
  double angle = cv::norm(rvec);
  quat.setRotation(tf::Vector3(rx/angle, ry/angle, rz/angle), angle);
  checktf.setRotation( quat );
  Eigen::Affine3d transform;
  tf::transformTFToEigen(checktf, transform);
  return transform;
}


bool loadMovementFile(openrave_test::SecondGrasp::Request  &req,
                      openrave_test::SecondGrasp::Response &res)
{
  if (!cam_info_) {
    ROS_INFO("no camera info is available");
    return false;
  }
  std::vector<CSimpleTrack> mTracks;
  std::vector<CSimpleTrack> mTracksGrasp;
  readTracks(mTracks);
  readTracks(mTracksGrasp, std::string("/moseg/TrainingSet/Results/OchsBroxMalik4_all_0000020.00/fromrobot2/Tracks10.dat"));
  // debug_img = cv::Mat::zeros(cam_info_->height, cam_info_->width, CV_8UC3);
  {
    std::stringstream ss;
    ss << std::getenv("HOME") << "/.ros/left0000.jpg";
    debug_img = cv::imread(ss.str().c_str());
  }
  // move to camera frame
  Eigen::Affine3d transform = getTransform(cam_info_->header.frame_id, "/triger_base_map");
  // project all grasps to movement

  std::vector<CSimpleTrack> nullTracks;
  cv::Mat hand_trans; // not needed
  int hand_label = getLabel(getTransform(cam_info_->header.frame_id, "rarm_end_coords").translation() ,mTracks, nullTracks, hand_trans);
  ROS_INFO("label! %d", hand_label);
  int back_label = 0; // may be 0
  geometry_msgs::PoseArray second_grasp_pose_array;
  geometry_msgs::PoseArray second_grasp_pose_array_debug;
  // get transform
  std::vector<std_msgs::Float32MultiArray> finger_angle_array_out;
  omp_lock_t writelock;
  omp_init_lock(&writelock);

// #ifdef _OPENMP
// #pragma omp parallel for
// #endif
  for (int i = 0; i < com_array_odom.size(); i++) {
    Eigen::Vector3d grasp_cam;
    grasp_cam = transform * com_array_odom[i];
    cv::Mat H;
    int label = getLabel(grasp_cam, mTracks, mTracksGrasp, H);
    ROS_INFO("label%d:, %d", i, label);
    omp_set_lock(&writelock);
    // one thread at a time stuff
    if (label != hand_label && label != back_label && label != -1) {
      ROS_INFO("succeeded");
      // push back grasp pose
      // H to Eigen
      Eigen::Affine3d local_trans = transFromHomography(H);
      ROS_INFO("fuga %f %f %f", local_trans.translation().x(), local_trans.translation().y(), local_trans.translation().z());
      geometry_msgs::Pose grasp_pose;
      tf::poseEigenToMsg(transform.inverse()* local_trans * transform *
                         grasp_array_odom[i], grasp_pose);
      second_grasp_pose_array.poses.push_back(grasp_pose);
      finger_angle_array_out.push_back(finger_angle_array_[i]);
      geometry_msgs::Pose grasp_pose2;
      tf::poseEigenToMsg(
                         grasp_array_odom[i], grasp_pose2);
      second_grasp_pose_array_debug.poses.push_back(grasp_pose2);
    }
    else {
      geometry_msgs::Pose grasp_pose;
      Eigen::Affine3d debug_grasp = grasp_array_odom[i];
      // debug_grasp.translation() = grasp_cam; // for debug
      tf::poseEigenToMsg(debug_grasp, grasp_pose);
      // second_grasp_pose_array_debug.poses.push_back(grasp_pose);
    }
    omp_unset_lock(&writelock);
    // some stuff
  }
  omp_destroy_lock(&writelock);
  second_grasp_pose_array.header.frame_id = "/triger_base_map";
  second_grasp_pose_array.header.stamp = ros::Time::now();
  second_grasp_pose_array_debug.header = second_grasp_pose_array.header;
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

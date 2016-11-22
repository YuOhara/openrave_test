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
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <openrave_test/RaveGraspArray.h>
#include <rospack/rospack.h>
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_openv");
  ros::NodeHandle n;
  cv::Point2f before_points[7], after_points[7];
  before_points[0] = cv::Point2f(0, 0);
  before_points[1] = cv::Point2f(0, 10);
  before_points[2] = cv::Point2f(10, 0);
  before_points[3] = cv::Point2f(20, 0);
  before_points[4] = cv::Point2f(30, 40);
  before_points[5] = cv::Point2f(50, 40);
  before_points[6] = cv::Point2f(150, 100);

  after_points[0] = cv::Point2f(0, 0);
  after_points[1] = cv::Point2f(0, 10);
  after_points[2] = cv::Point2f(10, 0);
  after_points[3] = cv::Point2f(20, 0);
  after_points[4] = cv::Point2f(30, 40);
  after_points[5] = cv::Point2f(50, 40);
  after_points[6] = cv::Point2f(150, 100);

  cv::Matx33d R;
  cv::Matx31d T;
  float focal =
    // 1;
    529.59520819;
  cv::Point2d pp =
    // cv::Point2d(0, 0);
    cv::Point2d(313.313022493709, 235.043504038408);
  cv::Mat before_points_mat (cv::Size(7, 1), CV_32FC2, before_points);
  cv::Mat after_points_mat (cv::Size(7, 1), CV_32FC2, after_points);
  cv::Mat E = cv::findEssentialMat(before_points_mat, after_points_mat, focal, pp);
  std::cout << E << std::endl;
  cv::recoverPose(E, before_points_mat, after_points_mat, R, T, focal, pp);
  std::cout << R << std::endl;
  std::cout << T << std::endl;

  cv::Mat img0 = cv::imread("/home/leus/ros/indigo/src/openrave_test/data/left0000.jpg");
  cv::Mat img1 = cv::imread("/home/leus/ros/indigo/src/openrave_test/data/left0001.jpg");
  cv::Ptr<cv::FeatureDetector> detector;
  cv::Ptr <cv::DescriptorExtractor> extractor;
  detector=cv::ORB::create();
  extractor=cv::ORB::create();

  cv::Mat descriptors0, descriptors1;
  std::vector<cv::KeyPoint> keypoints0, keypoints1;
  detector->detect(img0, keypoints0);
  detector->detect(img1, keypoints1);
  extractor->compute(img0, keypoints0, descriptors0);
  extractor->compute(img1, keypoints1, descriptors1);
 
  // マッチング
  std::vector<cv::DMatch> matches;
  cv::BFMatcher matcher(cv::NORM_HAMMING, true);
  matcher.match(descriptors0, descriptors1, matches);
 
  // 最大・最小距離
  double min_dist = DBL_MAX;
  for (int j = 0; j < (int)matches.size(); j++) { 
    double dist = matches[j].distance;
    if (dist < min_dist) min_dist = (dist < 1.0) ? 1.0 : dist;
  }

  // 良いペアのみ残す
  double cutoff = 5.0 * min_dist;
  std::set<int> existing_trainIdx;
  std::vector<cv::DMatch> matches_good;
  for (int j = 0; j < (int)matches.size(); j++) { 
    if (matches[j].trainIdx <= 0) matches[j].trainIdx = matches[j].imgIdx;
    if (matches[j].distance > 0.0 && matches[j].distance < cutoff) {
      if (existing_trainIdx.find(matches[j].trainIdx) == existing_trainIdx.end() && matches[j].trainIdx >= 0 && matches[j].trainIdx < (int)keypoints1.size()) {
        matches_good.push_back(matches[j]);
        existing_trainIdx.insert(matches[j].trainIdx);
      }
    }
  }

  cv::Mat matchImage;
  cv::drawMatches(img0, keypoints0, img1, keypoints1, matches_good, matchImage, cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
  cv::imshow("match", matchImage);
  cv::waitKey(0);
 
  // 5ペア以上は必要
  if (matches_good.size() > 100) {
    // ステレオペア
    std::vector<cv::Point2f>pts0, pts1;
    for (int j = 0; j < (int)matches_good.size(); j++) {
      pts0.push_back(keypoints0[matches_good[j].queryIdx].pt);
      pts1.push_back(keypoints1[matches_good[j].trainIdx].pt);
    }
    // 焦点距離とレンズ主点
    // double focal = 529.595208192375;
    // cv::Point2d pp = cv::Point2d(313.31, 235.04);
    // 5点アルゴリズムで基礎行列を計算
    cv::Matx33d E = cv::findEssentialMat(pts0, pts1, focal, pp);
    // カメラ姿勢
    cv::Matx33d R;
    cv::Matx31d t;
    cv::recoverPose(E, pts0, pts1, R, t, focal, pp); 
    std::cout << "R" << std::endl;
    std::cout << R << std::endl;
    std::cout << "t" << std::endl;
    std::cout << t << std::endl;

  }
}

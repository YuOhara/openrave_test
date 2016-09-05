#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/crop_box.h>

template<class FromT, class ToT>
void convertMatrix4(const FromT& from,
                    ToT& to)
{
  for (size_t i = 0; i < 4; i++) {
    for (size_t j = 0; j < 4; j++) {
      to(i, j) = from(i, j);
    }
  }
}

void convertEigenAffine3(const Eigen::Affine3d& from,
                         Eigen::Affine3f& to)
{
  Eigen::Matrix4d from_mat = from.matrix();
  Eigen::Matrix4f to_mat;
  convertMatrix4<Eigen::Matrix4d, Eigen::Matrix4f>(from_mat, to_mat);
  to = Eigen::Affine3f(to_mat);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;
  ros::NodeHandle local_nh("~");
  boost::shared_ptr< pcl::PointCloud<pcl::PointXYZ> > point_ptr( new pcl::PointCloud<pcl::PointXYZ>() );

  boost::shared_ptr<pcl::PolygonMesh> mesh_ptr( new pcl::PolygonMesh() );
  double dim_x, dim_y, dim_z;
  double p_x, p_y, p_z, r_x, r_y, r_z, r_w;
  std::string input_file_name, output_file_name;
  // p_x = p_y = p_z = 0;
  // r_x = r_y = r_z = 0;
  // r_w = 1.0;
  // input_file_name="/home/leus/.ros/mesh_estimated.ply";
  // output_file_name="/home/leus/.ros/mesh_estimated2.ply";
  local_nh.param("dim_x", dim_x, 1.0);
  local_nh.param("dim_y", dim_y, 1.0);
  local_nh.param("dim_z", dim_z, 1.0);
  local_nh.param("p_x", p_x, 0.0);
  local_nh.param("p_y", p_y, 0.0);
  local_nh.param("p_z", p_z, 0.0);
  local_nh.param("r_x", r_x, 0.0);
  local_nh.param("r_y", r_y, 0.0);
  local_nh.param("r_z", r_z, 0.0);
  local_nh.param("r_w", r_w, 1.0);
  local_nh.param("input_file_name", input_file_name, std::string("/home/leus/.ros/mesh_estimated.ply"));
  local_nh.param("output_file_name", output_file_name, std::string("/home/leus/.ros/mesh_estimated2.ply"));

  ROS_INFO("dim: %f %f %f, p: %f %f %f, r: %f %f %f %f", dim_x, dim_y, dim_z, p_x, p_y, p_z, r_x, r_y, r_z, r_w);
  pcl::io::loadPLYFile(input_file_name, *point_ptr);
  pcl::CropBox<pcl::PointXYZ> crop_box(false);
  pcl::PointIndices::Ptr indices (new pcl::PointIndices);

  Eigen::Vector4f max_points(dim_x/2,
                             dim_y/2,
                             dim_z/2,
                             0);
  Eigen::Vector4f min_points(-dim_x/2,
                             -dim_y/2,
                             -dim_z/2,
                             0);

  Eigen::Affine3f box_pose;
  Eigen::Affine3d box_posed =
    Eigen::Translation3d(p_x,
                         p_y,
                         p_z) *
    Eigen::Quaterniond(r_w,
                       r_x,
                       r_y,
                       r_z);
  convertEigenAffine3(box_posed, box_pose);
  crop_box.setInputCloud(point_ptr);
  crop_box.setMax(max_points);
  crop_box.setMin(min_points);
  crop_box.setTranslation(box_pose.translation());
  float roll, pitch, yaw;
  pcl::getEulerAngles(box_pose, roll, pitch, yaw);
  crop_box.setRotation(Eigen::Vector3f(roll, pitch, yaw));
  crop_box.filter(indices->indices);
  ROS_INFO("box clip end, before=%d, after=%d", point_ptr->points.size(), indices->indices.size());
  int new_index = 0;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (size_t i = 0; i < indices->indices.size()-2; ++i)
  {
    int index=indices->indices[i];
    if (index%3) {
      continue;
    }
    if (! (index+1)==indices->indices[i+1] || ! (index+2)==indices->indices[i+2]) {
      continue;
    }
    cloud.points.push_back(point_ptr->points[index]);
    cloud.points.push_back(point_ptr->points[index+1]);
    cloud.points.push_back(point_ptr->points[index+2]);
    pcl::Vertices v;
    v.vertices.push_back(new_index); new_index++;
    v.vertices.push_back(new_index); new_index++;
    v.vertices.push_back(new_index); new_index++;
    mesh_ptr->polygons.push_back(v);
  }
  cloud.width = cloud.points.size();
  cloud.height = 1;
  pcl::toPCLPointCloud2(cloud, mesh_ptr->cloud);
  pcl::io::savePLYFile(output_file_name, *mesh_ptr);
}

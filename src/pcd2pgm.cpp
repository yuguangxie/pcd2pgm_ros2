#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/srv/get_map.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>


#include <opencv2/opencv.hpp>
#include <fstream>

std::string file_directory;
std::string file_name;
std::string pcd_file;

std::string map_topic_name;

const std::string pcd_format = ".pcd";

nav_msgs::msg::OccupancyGrid map_topic_msg;
double thre_z_min = 0.3;
double thre_z_max = 2.0;
int flag_pass_through = 0;
double map_resolution = 0.05;
double thre_radius = 0.1;
int thres_point_count = 10;

pcl::PointCloud<pcl::PointXYZ>::Ptr
    cloud_after_PassThrough(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr
    cloud_after_Radius(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr
    pcd_cloud(new pcl::PointCloud<pcl::PointXYZ>);

void PassThroughFilter(const double &thre_low, const double &thre_high,
                       const bool &flag_in);
void RadiusOutlierFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pcd_cloud,
                         const double &radius, const int &thre_count);
void SetMapTopicMsg(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                    nav_msgs::msg::OccupancyGrid &msg);

void SavePGMAndYAML(const nav_msgs::msg::OccupancyGrid &msg, 
                    const std::string &directory, const std::string &name);


int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("pcd_filters");

  rclcpp::Rate loop_rate(1.0);

  node->declare_parameter("file_directory", "/home/");
  node->declare_parameter("file_name", "map");
  node->declare_parameter("thre_z_min", 0.2);
  node->declare_parameter("thre_z_max", 2.0);
  node->declare_parameter("flag_pass_through", 0);
  node->declare_parameter("thre_radius", 0.5);
  node->declare_parameter("map_resolution", 0.05);
  node->declare_parameter("thres_point_count", 10);
  node->declare_parameter("map_topic_name", "map");

  node->get_parameter("file_directory", file_directory);
  node->get_parameter("file_name", file_name);
  node->get_parameter("thre_z_min", thre_z_min);
  node->get_parameter("thre_z_max", thre_z_max);
  node->get_parameter("flag_pass_through", flag_pass_through);
  node->get_parameter("thre_radius", thre_radius);
  node->get_parameter("map_resolution", map_resolution);
  node->get_parameter("thres_point_count", thres_point_count);
  node->get_parameter("map_topic_name", map_topic_name);

  auto map_topic_pub = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
                      map_topic_name, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());

  pcd_file = file_directory + file_name + pcd_format;

  if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file, *pcd_cloud) == -1) {
    RCLCPP_ERROR(node->get_logger(), "Couldn't read file: %s", pcd_file.c_str());
    return -1;
  }

  std::cout << "Initial point cloud size: " << pcd_cloud->points.size() << std::endl;
  PassThroughFilter(thre_z_min, thre_z_max, bool(flag_pass_through));
  RadiusOutlierFilter(cloud_after_PassThrough, thre_radius, thres_point_count);
  SetMapTopicMsg(cloud_after_Radius, map_topic_msg);
  SavePGMAndYAML(map_topic_msg, file_directory, file_name);

  while (rclcpp::ok()) {
    map_topic_pub->publish(map_topic_msg);
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}

void PassThroughFilter(const double &thre_low, const double &thre_high,
                       const bool &flag_in) {
  pcl::PassThrough<pcl::PointXYZ> passthrough;
  passthrough.setInputCloud(pcd_cloud);
  passthrough.setFilterFieldName("z");
  passthrough.setFilterLimits(thre_low, thre_high);
  // passthrough.setFilterLimitsNegative(flag_in);
  passthrough.setNegative(flag_in);
  passthrough.filter(*cloud_after_PassThrough);
  pcl::io::savePCDFile<pcl::PointXYZ>(file_directory + "map_filter.pcd",
                                      *cloud_after_PassThrough);
  std::cout << "Point cloud size after passthrough filter: "
            << cloud_after_PassThrough->points.size() << std::endl;
}

void RadiusOutlierFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pcd_cloud0,
                         const double &radius, const int &thre_count) {
  pcl::RadiusOutlierRemoval<pcl::PointXYZ> radiusoutlier;
  radiusoutlier.setInputCloud(pcd_cloud0);
  radiusoutlier.setRadiusSearch(radius);
  radiusoutlier.setMinNeighborsInRadius(thre_count);
  radiusoutlier.filter(*cloud_after_Radius);
  pcl::io::savePCDFile<pcl::PointXYZ>(file_directory + "map_radius_filter.pcd",
                                      *cloud_after_Radius);
  std::cout << "Point cloud size after radius filter: "
            << cloud_after_Radius->points.size() << std::endl;
}

void SetMapTopicMsg(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                    nav_msgs::msg::OccupancyGrid &msg) {
  msg.header.stamp = rclcpp::Clock().now();
  msg.header.frame_id = "map";
  msg.info.map_load_time = rclcpp::Clock().now();
  msg.info.resolution = map_resolution;

  double x_min, x_max, y_min, y_max;
  double z_max_grey_rate = 0.05;
  double z_min_grey_rate = 0.95;
  double k_line =
      (z_max_grey_rate - z_min_grey_rate) / (thre_z_max - thre_z_min);
  double b_line =
      (thre_z_max * z_min_grey_rate - thre_z_min * z_max_grey_rate) /
      (thre_z_max - thre_z_min);

  if (cloud->points.empty()) {
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "PCD is empty!");
    return;
  }

  for (size_t i = 0; i < cloud->points.size() - 1; i++) {
    if (i == 0) {
      x_min = x_max = cloud->points[i].x;
      y_min = y_max = cloud->points[i].y;
    }

    double x = cloud->points[i].x;
    double y = cloud->points[i].y;

    if (x < x_min)
      x_min = x;
    if (x > x_max)
      x_max = x;

    if (y < y_min)
      y_min = y;
    if (y > y_max)
      y_max = y;
  }

  msg.info.origin.position.x = x_min;
  // msg.info.origin.position.y = y_min;
  int h = int((y_max - y_min) / map_resolution);
  msg.info.origin.position.y = y_max - h*map_resolution;
  msg.info.origin.position.z = 0.0;
  msg.info.origin.orientation.w = 1.0;

  msg.info.width = int((x_max - x_min) / map_resolution);
  msg.info.height = int((y_max - y_min) / map_resolution);
  msg.data.resize(msg.info.width * msg.info.height);
  msg.data.assign(msg.info.width * msg.info.height, 0);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Data size: %ld", msg.data.size());

  for (size_t iter = 0; iter < cloud->points.size(); iter++) {
    int i = int((cloud->points[iter].x - x_min) / map_resolution);
    if (i < 0 || i >= msg.info.width)
      continue;

    int j = int((cloud->points[iter].y - y_min) / map_resolution);
    if (j < 0 || j >= msg.info.height - 1)
      continue;
    
    msg.data[i + j * msg.info.width] = 100;
  }
}


void SavePGMAndYAML(const nav_msgs::msg::OccupancyGrid &msg, 
                    const std::string &directory, const std::string &name) {
  int width = msg.info.width;
  int height = msg.info.height;

  // Save PGM
  cv::Mat image(height, width, CV_8UC1);

  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      int8_t data = msg.data[x + y * width];
      if (data == -1) {
        image.at<uchar>(y, x) = 205; // Unknown
      } else {
        image.at<uchar>(y, x) = 255 - data * 255 / 100; // Occupied
      }
    }
  }
  cv::flip(image, image, 0); //resolve image mirroring issues
  std::string pgm_file = directory + name + ".pgm";
  cv::imwrite(pgm_file, image);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Saved PGM file: %s", pgm_file.c_str());

  // Save YAML
  std::string yaml_file = directory + name + ".yaml";
  std::ofstream yaml_output(yaml_file);
  yaml_output << "image: " << name << ".pgm" << std::endl;
  yaml_output << "resolution: " << msg.info.resolution << std::endl;
  yaml_output << "origin: [" << msg.info.origin.position.x << ", " 
              << msg.info.origin.position.y << ", " 
              << msg.info.origin.position.z << "]" << std::endl;
  yaml_output << "negate: 0" << std::endl;
  yaml_output << "occupied_thresh: 0.65" << std::endl;
  yaml_output << "free_thresh: 0.196" << std::endl;
  yaml_output.close();
  
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Saved YAML file: %s", yaml_file.c_str());
}

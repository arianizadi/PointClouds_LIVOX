#include "livox_lidar_api.h"
#include "livox_lidar_def.h"

#include <arpa/inet.h>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <unistd.h>

pcl::PointCloud< pcl::PointXYZ >::Ptr
    cloud(new pcl::PointCloud< pcl::PointXYZ >);
int counter = 0;
const int MIN_POINTS = 1000; // Adjust this value based on your needs

void PointCloudCallback(uint32_t /*handle*/,
                        const uint8_t /*dev_type*/,
                        LivoxLidarEthernetPacket *data,
                        void * /*client_data*/) {
  if(data == nullptr) {
    return;
  }

  auto *p_point_data
      = reinterpret_cast< LivoxLidarCartesianHighRawPoint * >(data->data);

  for(uint32_t i = 0; i < data->dot_num; ++i) {
    pcl::PointXYZ point;
    point.x = static_cast< float >(p_point_data[i].x) / 1000.0F;
    point.y = static_cast< float >(p_point_data[i].y) / 1000.0F;
    point.z = static_cast< float >(p_point_data[i].z) / 1000.0F;
    cloud->push_back(point);
  }
  counter += data->dot_num;
}

void WorkModeCallback(livox_status status,
                      uint32_t handle,
                      LivoxLidarAsyncControlResponse *response,
                      void * /*client_data*/) {
  if(response == nullptr) {
    return;
  }
  std::cout << "WorkModeCallack, status:" << status
            << ", handle:" << handle
            << ", ret_code:" << response->ret_code
            << ", error_key:" << response->error_key << std::endl;
}

void RebootCallback(livox_status status,
                    uint32_t handle,
                    LivoxLidarRebootResponse *response,
                    void * /*client_data*/) {
  if(response == nullptr) {
    return;
  }
  std::cout << "RebootCallback, status:" << status
            << ", handle:" << handle
            << ", ret_code:" << response->ret_code << std::endl;
}

void SetIpInfoCallback(livox_status status,
                       uint32_t handle,
                       LivoxLidarAsyncControlResponse *response,
                       void * /*client_data*/) {
  if(response == nullptr) {
    return;
  }

  std::cout << "LivoxLidarIpInfoCallback, status:" << status
            << ", handle:" << handle
            << ", ret_code:" << response->ret_code
            << ", error_key:" << response->error_key << std::endl;

  if(response->ret_code == 0 && response->error_key == 0) {
    LivoxLidarRequestReboot(handle, RebootCallback, nullptr);
  }
}

void QueryInternalInfoCallback(
    livox_status status,
    uint32_t handle,
    LivoxLidarDiagInternalInfoResponse *response,
    void * /*client_data*/) {
  if(status != kLivoxLidarStatusSuccess) {
    std::cout << "Query lidar internal info failed.\n";
    QueryLivoxLidarInternalInfo(
        handle, QueryInternalInfoCallback, nullptr);
    return;
  }

  if(response == nullptr) {
    return;
  }

  std::array< uint8_t, 4 > host_point_ipaddr {0};
  uint16_t host_point_port = 0;
  uint16_t lidar_point_port = 0;

  std::array< uint8_t, 4 > host_imu_ipaddr {0};
  uint16_t host_imu_data_port = 0;
  uint16_t lidar_imu_data_port = 0;

  uint16_t off = 0;
  for(uint16_t i = 0; i < response->param_num; ++i) {
    auto *kv = reinterpret_cast< LivoxLidarKeyValueParam * >(
        &response->data[off]);
    if(kv->key == kKeyLidarPointDataHostIpCfg) {
      std::copy_n(&kv->value[0], 4, host_point_ipaddr.begin());
      std::memcpy(&host_point_port, &kv->value[4], sizeof(uint16_t));
      std::memcpy(&lidar_point_port, &kv->value[6], sizeof(uint16_t));
    } else if(kv->key == kKeyLidarImuHostIpCfg) {
      std::copy_n(&kv->value[0], 4, host_imu_ipaddr.begin());
      std::memcpy(&host_imu_data_port, &kv->value[4], sizeof(uint16_t));
      std::memcpy(&lidar_imu_data_port, &kv->value[6], sizeof(uint16_t));
    }
    off += sizeof(uint16_t) * 2 + kv->length;
  }

  std::cout << "Host point cloud ip addr:" << host_point_ipaddr[0] << '.'
            << host_point_ipaddr[1] << '.' << host_point_ipaddr[2] << '.'
            << host_point_ipaddr[3]
            << ", host point cloud port:" << host_point_port
            << ", lidar point cloud port:" << lidar_point_port
            << std::endl;

  // Output IMU info
  std::cout << "Host imu ip addr:" << host_imu_ipaddr[0] << '.'
            << host_imu_ipaddr[1] << '.' << host_imu_ipaddr[2] << '.'
            << host_imu_ipaddr[3]
            << ", host imu port:" << host_imu_data_port
            << ", lidar imu port:" << lidar_imu_data_port << std::endl;
}

void LidarInfoChangeCallback(const uint32_t handle,
                             const LivoxLidarInfo *info,
                             void * /*client_data*/) {
  if(info == nullptr) {
    std::cerr << "lidar info change callback failed, the info is nullptr."
              << std::endl;
    return;
  }
  std::cout << "LidarInfoChangeCallback Lidar handle: " << handle
            << " SN: " << static_cast< const char * >(info->sn)
            << std::endl;

  SetLivoxLidarWorkMode(
      handle, kLivoxLidarNormal, WorkModeCallback, nullptr);

  QueryLivoxLidarInternalInfo(handle, QueryInternalInfoCallback, nullptr);
}

pcl::PointCloud< pcl::PointXYZ >::Ptr
removeFloor(pcl::PointCloud< pcl::PointXYZ >::Ptr cloud) {
  pcl::PointCloud< pcl::PointXYZ >::Ptr cloud_filtered(
      new pcl::PointCloud< pcl::PointXYZ >);
  pcl::SACSegmentation< pcl::PointXYZ > seg;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::ExtractIndices< pcl::PointXYZ > extract;

  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.02); // 2cm threshold, adjust if needed
  seg.setMaxIterations(100);

  // Set the axis to which the plane should be perpendicular (assuming Z
  // is up)
  Eigen::Vector3f axis = Eigen::Vector3f::UnitZ();
  seg.setAxis(axis);

  // Set the angle epsilon (in radians)
  // 0.15 radians is about 8.6 degrees
  seg.setEpsAngle(0.15);

  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);

  if(inliers->indices.size() == 0) {
    std::cerr
        << "Could not estimate a planar model for the given dataset."
        << std::endl;
    return cloud;
  }

  // Calculate the angle between the detected plane normal and the
  // vertical axis
  double dot_product
      = coefficients->values[2]; // cos(angle) = normal_z / 1
  double angle = std::acos(std::abs(dot_product)) * 180.0 / M_PI;

  std::cout << "Detected plane angle with vertical: " << angle
            << " degrees" << std::endl;

  // Only remove the plane if it's within our angle threshold
  if(angle < 10.0) { // 10 degrees threshold, adjust as needed
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_filtered);
    std::cout << "Floor plane removed" << std::endl;
  } else {
    std::cout
        << "Detected plane not considered as floor, no points removed"
        << std::endl;
    *cloud_filtered = *cloud;
  }

  return cloud_filtered;
}

void create2DMap(pcl::PointCloud< pcl::PointXYZ >::Ptr cloud,
                 const std::string &output_file) {
  // Find the boundaries of the point cloud
  pcl::PointXYZ min_pt, max_pt;
  pcl::getMinMax3D(*cloud, min_pt, max_pt);

  // Define the grid resolution (in meters)
  float resolution = 0.01; // 1cm grid cells

  // Calculate the grid size
  int grid_width
      = static_cast< int >((max_pt.x - min_pt.x) / resolution) + 1;
  int grid_height
      = static_cast< int >((max_pt.y - min_pt.y) / resolution) + 1;

  // Create a 2D grid
  cv::Mat grid = cv::Mat::zeros(grid_height, grid_width, CV_8UC1);

  // Project points onto the 2D grid
  for(const auto &point : cloud->points) {
    int x = static_cast< int >((point.x - min_pt.x) / resolution);
    int y = static_cast< int >((point.y - min_pt.y) / resolution);
    if(x >= 0 && x < grid_width && y >= 0 && y < grid_height) {
      grid.at< uchar >(grid_height - 1 - y, x) = 255; // Occupied cell
    }
  }

  // Create a colored map
  cv::Mat colored_map = cv::Mat::zeros(grid_height, grid_width, CV_8UC3);
  colored_map.setTo(cv::Scalar(
      200, 200, 200)); // Light grey background for unknown areas

  // Find contours of objects
  std::vector< std::vector< cv::Point > > contours;
  cv::findContours(
      grid, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  // Draw filled objects and red outlines
  for(const auto &contour : contours) {
    cv::drawContours(colored_map,
                     std::vector< std::vector< cv::Point > > {contour},
                     0,
                     cv::Scalar(0, 0, 0),
                     cv::FILLED); // Fill objects with black
    cv::drawContours(colored_map,
                     std::vector< std::vector< cv::Point > > {contour},
                     0,
                     cv::Scalar(0, 0, 255),
                     2); // Draw red outline
  }

  // Fill empty spaces with white
  cv::Mat empty_spaces;
  cv::dilate(grid, empty_spaces, cv::Mat(), cv::Point(-1, -1), 3);
  colored_map.setTo(cv::Scalar(255, 255, 255), empty_spaces == 0);

  // Save the map
  cv::imwrite(output_file, colored_map);
  std::cout << "2D map with red object outlines (floor removed) saved as "
            << output_file << std::endl;
}

int main(int argc, const char *argv[]) {
  if(argc != 2) {
    std::cout << "Params Invalid, must input config path.\n";
    return -1;
  }
  const std::string path = argv[1];

  // Initialize Livox SDK2
  if(!LivoxLidarSdkInit(path.c_str())) {
    std::cout << "Livox Init Failed\n";
    LivoxLidarSdkUninit();
    return -1;
  }

  SetLivoxLidarPointCloudCallBack(PointCloudCallback, nullptr);
  SetLivoxLidarInfoChangeCallback(LidarInfoChangeCallback, nullptr);

  // Run for a longer duration to collect more data
  while(counter < MIN_POINTS) {
    sleep(1);
    std::cout << "Collected " << counter << " points\n";
  }

  LivoxLidarSdkUninit();
  std::cout << "Data collection complete. Processing point cloud...\n";

  // Downsample the point cloud using a voxel grid filter
  pcl::VoxelGrid< pcl::PointXYZ > vg;
  pcl::PointCloud< pcl::PointXYZ >::Ptr cloud_filtered(
      new pcl::PointCloud< pcl::PointXYZ >);
  vg.setInputCloud(cloud);
  vg.setLeafSize(0.05F, 0.05F, 0.05F); // 5cm voxel size
  vg.filter(*cloud_filtered);

  // Remove the floor plane
  pcl::PointCloud< pcl::PointXYZ >::Ptr cloud_no_floor
      = removeFloor(cloud_filtered);

  // Create and save 2D map with red object outlines (floor included)
  create2DMap(cloud_filtered, "room_2d_map.png");

  // Create and save 2D map with red object outlines (floor removed)
  create2DMap(cloud_no_floor, "room_2d_map_no_floor_angle_based.png");

  // Create room.pcd
  pcl::PCDWriter writer;
  writer.writeBinary("room.pcd", *cloud_no_floor);

  return 0;
}

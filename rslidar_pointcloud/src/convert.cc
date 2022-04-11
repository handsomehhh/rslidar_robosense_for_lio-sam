/*
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2011 Jesse Vera
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2017 Robosense, Tony Zhang
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    This class converts raw RSLIDAR 3D LIDAR packets to PointCloud2.

*/
#include "convert.h"
#include <pcl_conversions/pcl_conversions.h>

/**
 * @brief rs_to_velodyne中定义点云格式
 * 
 */
std::string output_type;

static int RING_ID_MAP_RUBY[] = {
        3, 66, 33, 96, 11, 74, 41, 104, 19, 82, 49, 112, 27, 90, 57, 120,
        35, 98, 1, 64, 43, 106, 9, 72, 51, 114, 17, 80, 59, 122, 25, 88,
        67, 34, 97, 0, 75, 42, 105, 8, 83, 50, 113, 16, 91, 58, 121, 24,
        99, 2, 65, 32, 107, 10, 73, 40, 115, 18, 81, 48, 123, 26, 89, 56,
        7, 70, 37, 100, 15, 78, 45, 108, 23, 86, 53, 116, 31, 94, 61, 124,
        39, 102, 5, 68, 47, 110, 13, 76, 55, 118, 21, 84, 63, 126, 29, 92,
        71, 38, 101, 4, 79, 46, 109, 12, 87, 54, 117, 20, 95, 62, 125, 28,
        103, 6, 69, 36, 111, 14, 77, 44, 119, 22, 85, 52, 127, 30, 93, 60
};
static int RING_ID_MAP_16[] = {
        0, 1, 2, 3, 4, 5, 6, 7, 15, 14, 13, 12, 11, 10, 9, 8
};

// rslidar和velodyne的格式有微小的区别
// rslidar的点云格式
struct RsPointXYZIRT {
    PCL_ADD_POINT4D;
    uint8_t intensity;
    uint16_t ring = 0;
    double timestamp = 0;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(RsPointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(uint8_t, intensity, intensity)
                                          (uint16_t, ring, ring)(double, timestamp, timestamp))

// velodyne的点云格式
struct VelodynePointXYZIRT {
    PCL_ADD_POINT4D

    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointXYZIRT,
                                   (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
                                           (uint16_t, ring, ring)(float, time, time)
)



namespace rslidar_pointcloud
{
std::string model;

/** @brief Constructor. */
Convert::Convert(ros::NodeHandle node, ros::NodeHandle private_nh) : data_(new rslidar_rawdata::RawData()) 
{
  data_->loadConfigFile(node, private_nh);  // load lidar parameters //加载雷达的参数
  private_nh.param("model", model, std::string("RS16"));

  // advertise output point cloud (before subscribing to input data)
  output_ = node.advertise<sensor_msgs::PointCloud2>("rslidar_points", 10); //发布话题（重点）（应当是处理后的点云格式）

  srv_ = boost::make_shared<dynamic_reconfigure::Server<rslidar_pointcloud::CloudNodeConfig> >(private_nh); //应该就是动态参数配置的
  dynamic_reconfigure::Server<rslidar_pointcloud::CloudNodeConfig>::CallbackType f;
  f = boost::bind(&Convert::callback, this, _1, _2);
  srv_->setCallback(f);

  // subscribe to rslidarScan packets //订阅之前在driver节点发布的原始数据
  rslidar_scan_ = node.subscribe("rslidar_packets", 10, &Convert::processScan, (Convert*)this, //回调函数 重点
                                 ros::TransportHints().tcpNoDelay(true));
}

void Convert::callback(rslidar_pointcloud::CloudNodeConfig& config, uint32_t level)
{
  ROS_INFO("Reconfigure Request");
  // config_.time_offset = config.time_offset;
}

/** @brief Callback for raw scan messages. */
void Convert::processScan(const rslidar_msgs::rslidarScan::ConstPtr& scanMsg)
{
  //分析一下driver如何处理数据的时间戳
  //雷达的时间戳 还包含在1248byte的数据包里
  //这里要修改*** 没有ring和timestamp
  pcl::PointCloud<pcl::PointXYZI>::Ptr outPoints(new pcl::PointCloud<pcl::PointXYZI>);
  outPoints->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp; //这里的时间戳是从ros中获取的系统时间；不是雷达提供的时间戳
  outPoints->header.frame_id = scanMsg->header.frame_id; //"rslidar"
  outPoints->clear();
  if (model == "RS16")
  {
    outPoints->height = 16;
    outPoints->width = 24 * (int)scanMsg->packets.size();
    outPoints->is_dense = false;
    outPoints->resize(outPoints->height * outPoints->width);
  }
  else if (model == "RS32")
  {
    outPoints->height = 32;
    outPoints->width = 12 * (int)scanMsg->packets.size();
    outPoints->is_dense = false;
    outPoints->resize(outPoints->height * outPoints->width);
  }

  // process each packet provided by the driver 处理每个packet（一次又84个packet 或者等有了雷达实际可能有变化）

  data_->block_num = 0;
  for (size_t i = 0; i < scanMsg->packets.size(); ++i)
  {
    //解析每个packet
    data_->unpack(scanMsg->packets[i], outPoints);
  }
  sensor_msgs::PointCloud2 outMsg;
  pcl::toROSMsg(*outPoints, outMsg);
  //发布
  output_.publish(outMsg);
}
}  // namespace rslidar_pointcloud

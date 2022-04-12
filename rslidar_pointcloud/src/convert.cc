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



  // subscribe to rslidarScan packets //订阅之前在driver节点发布的原始数据
  rslidar_scan_ = node.subscribe("rslidar_packets", 10, &Convert::processScan, (Convert*)this, //回调函数 重点
                                 ros::TransportHints().tcpNoDelay(true));
}


/** @brief Callback for raw scan messages. */
void Convert::processScan(const rslidar_msgs::rslidarScan::ConstPtr& scanMsg)
{
  //分析一下driver如何处理数据的时间戳
  //雷达的时间戳 还包含在1248byte的数据包里
  //这里要修改*** 没有ring和timestamp
  // pcl::PointCloud<pcl::PointXYZI>::Ptr outPoints(new pcl::PointCloud<pcl::PointXYZI>); //修改成带有ring和timestamp信息的格式
  pcl::PointCloud<RsPointXYZIRT>::Ptr outPoints(new pcl::PointCloud<RsPointXYZIRT>);
  pcl::PointCloud<VelodynePointXYZIRT>::Ptr outPoints_velodyne(new pcl::PointCloud<VelodynePointXYZIRT>);
  outPoints->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp; //这里的时间戳是从ros中获取的系统时间；不是雷达提供的时间戳
  outPoints->header.frame_id = scanMsg->header.frame_id; //"rslidar"

  outPoints_velodyne->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp; //这里的时间戳是从ros中获取的系统时间；不是雷达提供的时间戳
  outPoints_velodyne->header.frame_id = "velodyne";//scanMsg->header.frame_id; //"rslidar"
  

  outPoints->clear();
  if (model == "RS16")
  {
    outPoints->height = 16;
    outPoints->width = 24 * (int)scanMsg->packets.size();
    outPoints->is_dense = false;
    outPoints->resize(outPoints->height * outPoints->width);

    //velodyne
    //项目中实际收到的点云数据是height = 1的（无序）
    //所以这里暂时先设置成无序的
    outPoints_velodyne->height = 1;
    outPoints_velodyne->width = 16 * 24 * (int)scanMsg->packets.size();
    outPoints_velodyne->is_dense = false;
    outPoints_velodyne->resize(outPoints_velodyne->height * outPoints_velodyne->width);
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
    if(i == 0)
      data_->unpack(scanMsg->packets[i], outPoints, outPoints_velodyne, true);
    else 
      data_->unpack(scanMsg->packets[i], outPoints, outPoints_velodyne, false);
  }
  sensor_msgs::PointCloud2 outMsg;
  pcl::toROSMsg(*outPoints, outMsg);
  //发布
  output_.publish(outMsg);

  // outPoint_pcl是velodyne格式的
}
}  // namespace rslidar_pointcloud

/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2007 Austin Robot Technology, Yaxin Liu, Patrick Beeson
 *  Copyright (C) 2009, 2010, 2012 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2017 Robosense, Tony Zhang
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file
 *
 *  @brief Interfaces for interpreting raw packets from the Robosense 3D LIDAR.
 *
 *  @author Yaxin Liu
 *  @author Patrick Beeson
 *  @author Jack O'Quin
 *  @author Tony Zhang
 */

#ifndef _RAWDATA_H
#define _RAWDATA_H

#include <ros/ros.h>
#include <ros/package.h>
#include <rslidar_msgs/rslidarPacket.h>
#include <rslidar_msgs/rslidarScan.h>
#include "std_msgs/String.h"
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <stdio.h>

#include <time.h>
#include <locale.h>


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

namespace rslidar_rawdata
{




// static const float  ROTATION_SOLUTION_ = 0.18f;  //水平角分辨率 10hz
static const int SIZE_BLOCK = 100;
static const int RAW_SCAN_SIZE = 3;
static const int SCANS_PER_BLOCK = 32;
static const int BLOCK_DATA_SIZE = (SCANS_PER_BLOCK * RAW_SCAN_SIZE);  // 96

static const float ROTATION_RESOLUTION = 0.01f;   /**< degrees 旋转角分辨率*/
static const uint16_t ROTATION_MAX_UNITS = 36000; /**< hundredths of degrees */

static const float DISTANCE_MAX = 200.0f;       /**< meters */
static const float DISTANCE_MIN = 0.2f;         /**< meters */
static const float DISTANCE_RESOLUTION = 0.01f; /**< meters */
static const float DISTANCE_MAX_UNITS = (DISTANCE_MAX / DISTANCE_RESOLUTION + 1.0f);
/** @todo make this work for both big and little-endian machines */
static const uint16_t UPPER_BANK = 0xeeff;  //
static const uint16_t LOWER_BANK = 0xddff;

/** Special Defines for RS16 support **/
static const int RS16_FIRINGS_PER_BLOCK = 2;
static const int RS16_SCANS_PER_FIRING = 16;
static const float RS16_BLOCK_TDURATION = 100.0f;  // [µs]
static const float RS16_DSR_TOFFSET = 3.0f;        // [µs]
static const float RS16_FIRING_TOFFSET = 50.0f;    // [µs]

/** Special Defines for RS32 support **/
static const int RS32_FIRINGS_PER_BLOCK = 1;
static const int RS32_SCANS_PER_FIRING = 32;
static const float RS32_BLOCK_TDURATION = 50.0f;  // [µs]
static const float RS32_DSR_TOFFSET = 3.0f;       // [µs]
static const float RL32_FIRING_TOFFSET = 50.0f;   // [µs]

static const int TEMPERATURE_MIN = 31;

/** \brief Raw rslidar data block.
 *
 *  Each block contains data from either the upper or lower laser
 *  bank.  The device returns three times as many upper bank blocks.
 *
 *  use stdint.h types, so things work with both 64 and 32-bit machines
 */
// block
typedef struct raw_block //一个block 2byte 的标志位，使用 0xffee 表示；2byte 的 Azimuth，表示水平旋转角度信息，每 个角度信息对应着 32 个的 channel data，包含 2 组完整的 16 通道信息。
{
  uint16_t header;  ///< UPPER_BANK or LOWER_BANK
  uint8_t rotation_1; 
  uint8_t rotation_2;  /// combine rotation1 and rotation2 together to get 0-35999, divide by 100 to get degrees
  uint8_t data[BLOCK_DATA_SIZE];  // 96 32 * 2(一个block出去标志位和水平转角信息的数据大小)
} raw_block_t;

/** used for unpacking the first two data bytes in a block
 *
 *  They are packed into the actual data stream misaligned.  I doubt
 *  this works on big endian machines.
 */
union two_bytes //用于解压块中的前两个数据字节 （无实际意义
{
  uint16_t uint;
  uint8_t bytes[2];
};

/**
 * @brief 自定义用于解析数据包中的时间信息
 * raw data
 * 
 */
struct raw_time
{
  uint8_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  uint16_t msecond;
  uint16_t usecond;
}

static const int PACKET_SIZE = 1248;
static const int BLOCKS_PER_PACKET = 12;
static const int PACKET_STATUS_SIZE = 4;
static const int SCANS_PER_PACKET = (SCANS_PER_BLOCK * BLOCKS_PER_PACKET); //32 * 12
//一个block，一共有32线次的扫描（2次 每次16根），一个packet中有12个block
//一次扫描的大小就是3byte

/** \brief Raw Rsldar packet.
 *
 *  revolution is described in the device manual as incrementing
 *    (mod 65536) for each physical turn of the device.  Our device
 *    seems to alternate between two different values every third
 *    packet.  One value increases, the other decreases.
 *
 *  \todo figure out if revolution is only present for one of the
 *  two types of status fields
 *
 *  status has either a temperature encoding or the microcode level
 */
typedef struct raw_packet
{
  raw_block_t blocks[BLOCKS_PER_PACKET]; //12个block 12*100
  uint16_t revolution; // ???//2
  uint8_t status[PACKET_STATUS_SIZE];//4 ???
} raw_packet_t;

/** \brief RSLIDAR data conversion class */
class RawData
{
public:
  RawData();

  ~RawData()
  {
  }

  /*load the cablibrated files: angle, distance, intensity 加载校准文件：角度、距离、强度*/
  void loadConfigFile(ros::NodeHandle node, ros::NodeHandle private_nh);

  /*unpack the RS16 UDP packet and opuput PCL PointXYZI type*/
  //重点
  void unpack(const rslidar_msgs::rslidarPacket& pkt, pcl::PointCloud<RsPointXYZIRT>::Ptr pointcloud, 
                     pcl::PointCloud<VelodynePointXYZIRT>::Ptr pointcloud_velodyne, bool is_first_packet);
  /*unpack the RS32 UDP packet and opuput PCL PointXYZI type*/
  // void unpack_RS32(const rslidar_msgs::rslidarPacket& pkt, pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud);

  /*compute temperature*/
  float computeTemperature(unsigned char bit1, unsigned char bit2);

  /*estimate temperature*/
  int estimateTemperature(float Temper);

  /*calibrated the disctance*/
  float pixelToDistance(int pixelValue, int passageway);

  /*calibrated the azimuth*/
  int correctAzimuth(float azimuth_f, int passageway);

  /*calibrated the intensity*/
  float calibrateIntensity(float inten, int calIdx, int distance);
  float calibrateIntensity_old(float inten, int calIdx, int distance);

  /*estimate the packet type*/
  int isABPacket(int distance);

  void processDifop(const rslidar_msgs::rslidarPacket::ConstPtr& difop_msg);
  ros::Subscriber difop_sub_;
  bool is_init_curve_;
  bool is_init_angle_;
  int block_num = 0;

  //记录起始时间戳
  double init_timestamp = 0;
};

float VERT_ANGLE[32];
float HORI_ANGLE[32];
float aIntensityCal[7][32];
float aIntensityCal_old[1600][32];
bool Curvesis_new = true;
int g_ChannelNum[32][51];
float CurvesRate[32];

float temper = 31.0;
int tempPacketNum = 0;
int numOfLasers = 16;
int TEMPERATURE_RANGE = 40;

/**
 * @brief 用于时间戳的计算
 * 根据雷达手册 head的21byte -- 30byte为时间戳
 * 考虑实际存储的时间戳，由4byte的sec和4byte的nsec组成（一共8byte）
 * nsec可以对应后4byte，但是前6byte无法直接对应
 * 所以打算舍弃前两个byte(记录年 月信息)
 * 理论上会有点小问题（跨月的时候）
 * 
 * 这里等明天再问问学长吧
 * 
 */
double get_timestamp(const raw_time raw_time_data) ;



}  // namespace rslidar_rawdata

#endif  // __RAWDATA_H

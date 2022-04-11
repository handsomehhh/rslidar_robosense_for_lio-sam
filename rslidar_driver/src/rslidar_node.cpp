/*
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *	Copyright (C) 2017 Robosense, Tony Zhang
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  ROS driver node for the Robosense 3D LIDARs.
 */
#include <ros/ros.h>
#include "rsdriver.h"
#include "std_msgs/String.h"

using namespace rslidar_driver;
volatile sig_atomic_t flag = 1;

static void my_handler(int sig)
{
  flag = 0;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rsdriver");
  ros::NodeHandle node;
  ros::NodeHandle private_nh("~");

  std::cout<<"param"<<std::endl;
  std::string temp;
  //使用launch文件的时候，是可以读取参数的（我是sb，单独启动的节点）
  private_nh.param("device_ip", temp, std::string(""));

  //linux的系统函数 捕捉中断信号、第二个参数相当于回调函数8应该
  signal(SIGINT, my_handler);

  // start the driver
  rslidar_driver::rslidarDriver dvr(node, private_nh);
  // loop until shut down or end of file
  while (ros::ok() && dvr.poll()) //这个poll ： 轮询设备
  {
    //https://blog.csdn.net/weixin_40215443/article/details/103793316
    ros::spinOnce();
  }

  return 0;
}

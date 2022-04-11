/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2015, Jack O'Quin
 *	Copyright (C) 2017, Robosense, Tony Zhang
 *
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  Input classes for the RSLIDAR RS-16 3D LIDAR:
 *
 *     Input -- base class used to access the data independently of
 *              its source 用于独立于数据源访问数据的基类
 *
 *     InputSocket -- derived class reads live data from the device
 *              via a UDP socket 派生类通过UDP套接字从设备读取实时数据
 *
 *     InputPCAP -- derived class provides a similar interface from a
 *              PCAP dump 派生类提供了来自PCAP转储的类似接口 
 */
#include "input.h"
#include <iostream>
using namespace std;

extern volatile sig_atomic_t flag;
namespace rslidar_driver
{
static const size_t packet_size = sizeof(rslidar_msgs::rslidarPacket().data);

////////////////////////////////////////////////////////////////////////
// Input base class implementation
////////////////////////////////////////////////////////////////////////

/** @brief constructor
 *
 *  @param private_nh ROS private handle for calling node.
 *  @param port UDP port number.
 */
Input::Input(ros::NodeHandle private_nh, uint16_t port) : private_nh_(private_nh), port_(port)
{
  private_nh.param("device_ip", devip_str_, std::string(""));
  cout<<"Input divice_ip "<<devip_str_<<endl;
  if (!devip_str_.empty())
    ROS_INFO_STREAM("Only accepting packets from IP address: " << devip_str_);
}

////////////////////////////////////////////////////////////////////////
// InputSocket class implementation
////////////////////////////////////////////////////////////////////////

/** @brief constructor
   *
   *  @param private_nh ROS private handle for calling node.
   *  @param port UDP port number
*/
InputSocket::InputSocket(ros::NodeHandle private_nh, uint16_t port) : Input(private_nh, port)
{
  sockfd_ = -1;

  if (!devip_str_.empty())
  {
    //extern int inet_aton (const char *__cp, struct in_addr *__inp) __THROW;
    //将Internet主机地址从CP中的数字和点符号转换为二进制数据，并将结果存储在结构INP中。
    inet_aton(devip_str_.c_str(), &devip_);
    cout<<"devip erjinzhi "<<devip_str_<<endl;
  }

  ROS_INFO_STREAM("Opening UDP socket: port " << port);
  //extern int socket (int __domain, int __type, int __protocol) __THROW;
  //使用协议在域中创建类型为的新套接字。如果协议为零，则自动选择一个。
  //返回新套接字的文件描述符，或返回错误的-1。
  //https://blog.csdn.net/bian_qing_quan11/article/details/71713647
  /**
   * @brief 
   * #define PF_INET		2	 //IP protocol family.  IPV4
   * SOCK_DGRAM 2 //UDP协议
   * 协议通常设置为0
   * 
   * 成功：返回一个套接字描述符
   * 失败：返回-1
   */
  sockfd_ = socket(PF_INET, SOCK_DGRAM, 0);
  if (sockfd_ == -1)
  {
    perror("socket");  // TODO: ROS_ERROR errno
    return;
  } else {
    cout<<"socket success "<<endl;
  }

  int opt = 1;
  //在协议级别将套接字FD的选项OPTNAME设置为*OPTVAL（长度为OPTLEN字节）。成功时返回0，错误时返回1。 
  //Set socket FD's option OPTNAME at protocol level LEVEL to *OPTVAL (which is OPTLEN bytes long).
   //Returns 0 on success, -1 for errors.
   // https://www.cnblogs.com/wanzaixiaoxin/p/5295233.html
  if (setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR, (const void*)&opt, sizeof(opt)))
  {
    perror("setsockopt error!\n");
    return;
  }

/*
/* Structure describing an Internet socket address.  
struct sockaddr_in
  {
    __SOCKADDR_COMMON (sin_);
    in_port_t sin_port;			/* Port number.  
    struct in_addr sin_addr;		/* Internet address.  

    /* Pad to size of `struct sockaddr'.  
    unsigned char sin_zero[sizeof (struct sockaddr) -
			   __SOCKADDR_COMMON_SIZE -
			   sizeof (in_port_t) -
			   sizeof (struct in_addr)];
  };
*/
//应该是定义接收端的socket
//https://blog.csdn.net/qingzhuyuxian/article/details/79736821
  sockaddr_in my_addr;                   // my address information
  memset(&my_addr, 0, sizeof(my_addr));  // initialize to zeros
  my_addr.sin_family = AF_INET;          // host byte order
  //static inline uint16_t htons(uint16_t hostvar) { return hostvar; }
  my_addr.sin_port = htons(port);        // port in network byte order
  //#define	INADDR_ANY		((in_addr_t) 0x00000000)
  //NADDR_ANY就是指定地址为0.0.0.0的地址，这个地址事实上表示不确定地址，或“所有地址”、“任意地址”。 一般来说，在各个系统中均定义成为0值。
  my_addr.sin_addr.s_addr = INADDR_ANY;  // automatically fill in my IP

  if (bind(sockfd_, (sockaddr*)&my_addr, sizeof(sockaddr)) == -1)
  {
    perror("bind");  // TODO: ROS_ERROR errno
    return;
  }

  if (fcntl(sockfd_, F_SETFL, O_NONBLOCK | FASYNC) < 0)
  {
    perror("non-block");
    return;
  }
}

/** @brief destructor */
InputSocket::~InputSocket(void)
{
  (void)close(sockfd_);
}

/** @brief Get one rslidar packet. */ //获取单个数据包 1248
int InputSocket::getPacket(rslidar_msgs::rslidarPacket* pkt, const double time_offset)
{
  double time1 = ros::Time::now().toSec(); //当前时间的时间戳
  struct pollfd fds[1]; //https://blog.csdn.net/wocjj/article/details/7612335
  fds[0].fd = sockfd_;
  fds[0].events = POLLIN; //#define POLLIN		0x001		/* There is data to read.  */
  static const int POLL_TIMEOUT = 1000;  // one second (in msec)

  sockaddr_in sender_address;
  socklen_t sender_address_len = sizeof(sender_address);
  //flag == 1
  while (flag == 1)
  {
    // Receive packets that should now be available from the
    // socket using a blocking read.
    // poll() until input available
    do
    {
      int retval = poll(fds, 1, POLL_TIMEOUT);
      if (retval < 0)  // poll() error?
      {
        if (errno != EINTR) //#define	EINTR		 4	/* Interrupted system call */
          ROS_ERROR("poll() error: %s", strerror(errno));
        return 1;
      }
      if (retval == 0)  // poll() timeout?
      {
        ROS_WARN("Rslidar poll() timeout");
//重新赋值
        char buffer_data[8] = "re-con";
        memset(&sender_address, 0, sender_address_len);          // initialize to zeros
        sender_address.sin_family = AF_INET;                     // host byte order
        sender_address.sin_port = htons(MSOP_DATA_PORT_NUMBER);  // port in network byte order, set any value
        sender_address.sin_addr.s_addr = devip_.s_addr;          // automatically fill in my IP
        //https://blog.csdn.net/radissh/article/details/98081032
        //sendto() 用来将数据由指定的socket传给对方主机
        sendto(sockfd_, &buffer_data, strlen(buffer_data), 0, (sockaddr*)&sender_address, sender_address_len);
        return 1;
      }
      if ((fds[0].revents & POLLERR) || (fds[0].revents & POLLHUP) || (fds[0].revents & POLLNVAL))  // device error?
      {
        
        ROS_ERROR("poll() reports Rslidar error");
        return 1;
      }
    } while ((fds[0].revents & POLLIN) == 0); //当实际发生的事情 与 期望发生的事情不同时，阻塞 直到相同
    //得到了数据
    //https://blog.csdn.net/keen_zuxwang/article/details/72872802
    ssize_t nbytes = recvfrom(sockfd_, &pkt->data[0], packet_size, 0, (sockaddr*)&sender_address, &sender_address_len);
///////////////////在这里对数据进行了存储 传输了指针
    if (nbytes < 0)
    {
      if (errno != EWOULDBLOCK)
      {
        perror("recvfail");
        ROS_INFO("recvfail");
        return 1;
      }
    }
    else if ((size_t)nbytes == packet_size)
    {// 到这里得到了长度匹配的数据
      if (devip_str_ != "" && sender_address.sin_addr.s_addr != devip_.s_addr) //从其他不知名的addr获取了数据？
        continue;
      else
        break;  // done
    }

    ROS_DEBUG_STREAM("incomplete rslidar packet read: " << nbytes << " bytes");
  }
  if (flag == 0)
  {
    abort();
  }
  // Average the times at which we begin and end reading.  Use that to
  // estimate when the scan occurred. Add the time offset. //取开始获取数据 和 真正获取时间 的平均值
  double time2 = ros::Time::now().toSec();
  pkt->stamp = ros::Time((time2 + time1) / 2.0 + time_offset);

  return 0;
}

////////////////////////////////////////////////////////////////////////
// InputPCAP class implementation
////////////////////////////////////////////////////////////////////////

/** @brief constructor
   *
   *  @param private_nh ROS private handle for calling node.
   *  @param port UDP port number
   *  @param packet_rate expected device packet frequency (Hz)
   *  @param filename PCAP dump file name
   */
InputPCAP::InputPCAP(ros::NodeHandle private_nh, uint16_t port, double packet_rate, std::string filename,
                     bool read_once, bool read_fast, double repeat_delay)
  : Input(private_nh, port), packet_rate_(packet_rate), filename_(filename)
{
  pcap_ = NULL;
  empty_ = true;

  // get parameters using private node handle
  private_nh.param("read_once", read_once_, false);
  private_nh.param("read_fast", read_fast_, false);
  private_nh.param("repeat_delay", repeat_delay_, 0.0);

  if (read_once_)
    ROS_INFO("Read input file only once.");
  if (read_fast_)
    ROS_INFO("Read input file as quickly as possible.");
  if (repeat_delay_ > 0.0)
    ROS_INFO("Delay %.3f seconds before repeating input file.", repeat_delay_);

  // Open the PCAP dump file
  // ROS_INFO("Opening PCAP file \"%s\"", filename_.c_str());
  ROS_INFO_STREAM("Opening PCAP file " << filename_);
  if ((pcap_ = pcap_open_offline(filename_.c_str(), errbuf_)) == NULL)
  {
    ROS_FATAL("Error opening rslidar socket dump file.");
    return;
  }

  std::stringstream filter;
  if (devip_str_ != "")  // using specific IP?
  {
    filter << "src host " << devip_str_ << " && ";
  }
  filter << "udp dst port " << port;
  pcap_compile(pcap_, &pcap_packet_filter_, filter.str().c_str(), 1, PCAP_NETMASK_UNKNOWN);
}

/** destructor */
InputPCAP::~InputPCAP(void)
{
  pcap_close(pcap_);
}

/** @brief Get one rslidar packet. */
int InputPCAP::getPacket(rslidar_msgs::rslidarPacket* pkt, const double time_offset)
{
  struct pcap_pkthdr* header;
  const u_char* pkt_data;

  while (flag == 1)
  {
    int res;
    if ((res = pcap_next_ex(pcap_, &header, &pkt_data)) >= 0)
    {
      // Skip packets not for the correct port and from the
      // selected IP address.
      if (!devip_str_.empty() && (0 == pcap_offline_filter(&pcap_packet_filter_, header, pkt_data)))
        continue;

      // Keep the reader from blowing through the file.
      if (read_fast_ == false)
        packet_rate_.sleep();

      memcpy(&pkt->data[0], pkt_data + 42, packet_size);
      pkt->stamp = ros::Time::now();  // time_offset not considered here, as no
                                      // synchronization required
      empty_ = false;
      return 0;  // success
    }

    if (empty_)  // no data in file?
    {
      ROS_WARN("Error %d reading rslidar packet: %s", res, pcap_geterr(pcap_));
      return -1;
    }

    if (read_once_)
    {
      ROS_INFO("end of file reached -- done reading.");
      return -1;
    }

    if (repeat_delay_ > 0.0)
    {
      ROS_INFO("end of file reached -- delaying %.3f seconds.", repeat_delay_);
      usleep(rint(repeat_delay_ * 1000000.0));
    }

    ROS_DEBUG("replaying rslidar dump file");

    // I can't figure out how to rewind the file, because it
    // starts with some kind of header.  So, close the file
    // and reopen it with pcap.
    pcap_close(pcap_);
    pcap_ = pcap_open_offline(filename_.c_str(), errbuf_);
    empty_ = true;  // maybe the file disappeared?
  }                 // loop back and try again

  if (flag == 0)
  {
    abort();
  }
}
}

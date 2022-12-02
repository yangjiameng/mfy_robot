#include "serial_demo.h"

#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <iomanip>
#include <iostream>
#include <sys/ioctl.h>
#include<sys/types.h>
#include<sys/stat.h>
#include<fcntl.h> 

#define DISTANCE 16384.
#define RADIUS 0.0655     // 车轮半径 m
#define RADIUS_CAR 0.290  // 轮距 m
#define IMP 4096          // 每转编码器脉冲数
#define CONVERSE 65536
#define ODOM_RECEIVED 1
#define BATTERY_LOW 2300

qy_robot_bringup::qy_robot_bringup() {
  odom_ = nh_.advertise<nav_msgs::Odometry>("odom_raw", 50);
  cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  stop_status_ = nh_.advertise<std_msgs::String>("stop_status", 1);
  setlocale(LC_ALL, "");
  serial_connect();
}

qy_robot_bringup::~qy_robot_bringup() {
  ROS_INFO("\033[32m节点已经关闭!\033[0m");

#if 1
  // uint8_t buffer_num[8] = {0xD5, 0x6C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x6C};
  // sp_.write(buffer_num, 8);
  sp_.close();
  ros::shutdown();
#endif
}

void qy_robot_bringup::cmd_callback(
    const geometry_msgs::Twist::ConstPtr& twist) {
  linear_speed_ = twist->linear.x;
  angular_speed_ = twist->angular.z;
  int linear = linear_speed_ * 1000;
  int angular = angular_speed_ * 1000;
  if (angular >= 0) {
    speed_pub(linear, -angular, false);
  } else {
    speed_pub(linear, -angular, true);
  }
}

int qy_robot_bringup::buff_return(uint8_t high, uint8_t low, std::string name) {
  uint16_t result_16;
  result_16 = high << 8 | low;
  int result = result_16;
  return result;
}

void qy_robot_bringup::serial_connect() {
  // 串口连接
  std::string serial_port;
  int baudrate;
  // change_cosole("/dev/tty1");
  nh_.param<std::string>("/robot_bringup/serial_port", serial_port,
                         "/dev/ttyS1");
  nh_.param<int>("/robot_bringup/baudrate", baudrate, 115200);
  serial::Timeout time_out = serial::Timeout::simpleTimeout(100);
  sp_.setPort(serial_port);
  sp_.setBaudrate(baudrate);
  sp_.setTimeout(time_out);

  try {
    sp_.open();
  } catch (serial::IOException& e) {
    std::cerr << e.what() << '\n';
    ROS_ERROR_STREAM("Unable to open port");
    ros::shutdown();
  }

  if (sp_.isOpen()) {
    ROS_INFO("\033[32m%s: 串口通信成功...\033[0m", serial_port.c_str());
    ROS_INFO("\033[32m%d: 波特率\033[0m", baudrate);
  } else {
    ros::shutdown();
  }
}

void qy_robot_bringup::node_bringup() {
  ros::Rate loop_rate(50);
  while (ros::ok()) {
    size_t n = sp_.available();
    std::cout << n << std::endl;
//     if (n != 0) {
//       uint8_t buffer[n];
//       n = sp_.read(buffer, n);
//       for(auto a : buffer)  {
//         std::cout << (a & 0xff) << "  ";
//       }
//       std::cout <<n<<std::endl;
// #if ODOM_RECEIVED
//       // 6A 编码器
//       if ((buffer[0] & 0xff) == 122 && n == 14) {
//         int a_num = buff_return(buffer[1], buffer[2], "A");
//         // 6B 编码器
//         int b_num = buff_return(buffer[3], buffer[4], "B");
//         // 编码值初始化
//         // std::cout << a_num << " , "<< b_num<<std::endl;
//         if (left_imp_ != -1) {
//           if (abs(a_num) <= 65536 && abs(b_num) <= 65536) {
//             odom_msg_pub(a_num, b_num);
//           }

//         } else {
//           left_imp_ = a_num;
//           right_imp_ = b_num;
//           last_time_ = ros::Time::now();
//         }
//       }
// #endif

//       if ((buffer[0] & 0xff) == 122 && n == 14) {
//         // 充电信号
//         signal_ = buff_return(0x00, buffer[5], "signal");

//       }
//     }
    loop_rate.sleep();
  }
  // sp_.close();
}

void qy_robot_bringup::speed_pub(int linear, int angular, bool flag) {
  // cmd控制解算
  uint8_t low_8_left;
  uint8_t high_8_left;
  uint8_t low_8_right;
  uint8_t high_8_right;
  if (flag) {
    if (linear >= 0) {
      up_down_flag_ = 0;
      low_8_left = uint8_t(linear);
      high_8_left = uint8_t(linear >> 8);
      low_8_right = uint8_t(angular);
      high_8_right = uint8_t(angular >> 8);
      uint8_t buffer_num[7] = {
          0xD5, low_8_left, high_8_left, low_8_right, high_8_right, 0x02, 0x00};
      sp_.write(buffer_num, 7);
    } else {
      up_down_flag_ = 1;
      low_8_left = uint8_t(linear + CONVERSE);
      high_8_left = uint8_t((linear + CONVERSE) >> 8);
      low_8_right = uint8_t(angular);
      high_8_right = uint8_t(angular >> 8);
      uint8_t buffer_num[7] = {
          0xD5, low_8_left, high_8_left, low_8_right, high_8_right, 0x02, 0x00};
      sp_.write(buffer_num, 7);
    }
  } else {
    if (linear >= 0) {
      up_down_flag_ = 0;
      low_8_left = uint8_t(linear);
      high_8_left = uint8_t(linear >> 8);
      low_8_right = uint8_t(angular + CONVERSE);
      high_8_right = uint8_t((angular + CONVERSE) >> 8);
      uint8_t buffer_num[7] = {
          0xD5, low_8_left, high_8_left, low_8_right, high_8_right, 0x02, 0x00};
      sp_.write(buffer_num, 7);
    } else {
      up_down_flag_ = 1;
      low_8_left = uint8_t(linear + CONVERSE);
      high_8_left = uint8_t((linear + CONVERSE) >> 8);
      low_8_right = uint8_t(angular + CONVERSE);
      high_8_right = uint8_t((angular + CONVERSE) >> 8);
      uint8_t buffer_num[7] = {
          0xD5, low_8_left, high_8_left, low_8_right, high_8_right, 0x02, 0x00};
      sp_.write(buffer_num, 7);
    }
  }
}

void qy_robot_bringup::odom_msg_pub(int num_a, int num_b) {
  current_time_ = ros::Time::now();
  int left_result = 0;
  int right_result = 0;

  left_result = num_a - left_imp_;
  right_result = num_b - right_imp_;
  if (left_result > 30000) {
    left_result -= IMP * 16;
  } else if (left_result < -30000) {
    left_result += IMP * 16;
  }
  if (right_result > 3000) {
    right_result -= IMP * 16;
  } else if (right_result < -3000) {
    right_result += IMP * 16;
  }

  float dx_letf =
      (left_result)*M_PI * 2 * RADIUS / IMP;  // 单位时间左轮移动距离
  float dx_right =
      (right_result)*M_PI * 2 * RADIUS / IMP;  // 单位时间右轮移动距离
  left_imp_ = num_a;
  right_imp_ = num_b;

  // odom_msg构建
  float dt = (current_time_ - last_time_).toSec();
  last_time_ = current_time_;
  float dxy_ave = (dx_right + dx_letf) / 2.0;           // 移动均值
  float dth = (dx_right - dx_letf) / (2 * RADIUS_CAR);  //中心点转动角度
  float vxy = dxy_ave / dt;                             // 中心点速度
  float vth = dth / dt;                                 // 角速度
  auto dx = cos(dth) * dxy_ave;
  auto dy = -sin(dth) * dxy_ave;
  odom_x_ += (cos(odom_th_) * dx - sin(odom_th_) * dy);
  odom_y_ += (sin(odom_th_) * dx + cos(odom_th_) * dy);
  if (dth != 0.) odom_th_ += dth;
  if (odom_th_ > M_PI) odom_th_ -= 2 * M_PI;
  if (odom_th_ < -M_PI) odom_th_ += 2 * M_PI;

  geometry_msgs::Quaternion odom_quat =
      tf::createQuaternionMsgFromYaw(odom_th_);
  static tf::TransformBroadcaster odom_to_base_link;

  // 发布odom to base_link 坐标变换
  geometry_msgs::TransformStamped odom_trans;
  auto time_now = ros::Time::now();
  odom_trans.header.stamp = time_now;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";
  odom_trans.transform.translation.x = odom_x_;
  odom_trans.transform.translation.y = odom_y_;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;
  // odom_to_base_link.sendTransform(odom_trans);

  // odom_msg_pub
  nav_msgs::Odometry odom;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";
  odom.header.stamp = time_now;
  odom.pose.pose.position.x = odom_x_;
  odom.pose.pose.position.y = odom_y_;
  odom.pose.pose.position.z = 0;
  odom.pose.pose.orientation = odom_quat;
  // 协方差矩阵pose
  odom.pose.covariance = {1e-3, 0, 0,   0, 0,   0, 0, 1e-3, 0, 0,   0, 0,
                          0,    0, 1e6, 0, 0,   0, 0, 0,    0, 1e6, 0, 0,
                          0,    0, 0,   0, 1e6, 0, 0, 0,    0, 0,   0, 1e3};

  odom.twist.twist.linear.x = vxy;
  odom.twist.twist.linear.y = 0;
  odom.twist.twist.angular.z = vth;
  // 协方差矩阵twist
  odom.twist.covariance = {1e-3, 0, 0,   0, 0,   0, 0, 1e-3, 0, 0,   0, 0,
                           0,    0, 1e6, 0, 0,   0, 0, 0,    0, 1e6, 0, 0,
                           0,    0, 0,   0, 1e6, 0, 0, 0,    0, 0,   0, 1e3};
  odom_.publish(odom);
}

void qy_robot_bringup::thread_sub() {
  cmd_msg_ =
      nh_.subscribe("cmd_vel", 10, &qy_robot_bringup::cmd_callback, this);
  ros::spin();
}

//将shell切换到其他串口终端
int qy_robot_bringup::change_cosole(char *tty)
{
    int fp = 0;

    fp = open(tty, O_RDONLY); 
    if(fp == -1)
    {
        printf("切换调试串口失败\n");
        return -1;
    }
    ioctl(fp,TIOCCONS);// 改变console到当前串口
    close(fp);
    return 0;
}


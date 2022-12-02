#ifndef _SERIAL_DEMO_H_
#define _SERIAL_DEMO_H_

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <serial/serial.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>


class qy_robot_bringup {
 private:
  ros::NodeHandle nh_;
  ros::Subscriber cmd_to_odom_sub_;
  ros::Publisher odom_;
  ros::Publisher cancel_nav_;
  ros::Publisher cmd_pub_;
  ros::Publisher stop_status_;
  ros::Subscriber cmd_msg_;
  ros::Subscriber status_sub_;
  ros::Subscriber door_;
  ros::Subscriber status_wheel_;
  ros::Time current_time_, last_time_;
  serial::Serial sp_;
  geometry_msgs::Quaternion imu_odom_;
  int left_imp_ = -1;
  int right_imp_ = -1;
  float odom_x_ = 0.;
  float odom_y_ = 0.;
  float odom_th_ = 0.;
  int signal_ = -1;
  bool signal_flag_ = true;
  int shade_ = -1;
  int up_down_flag_ = 0;  // 编码器计数标志位，0: 向前 1: 向后 2: 左转 3: 右转
  double linear_speed_ = 0.0;
  double angular_speed_ = 0.0;
  bool wheel_flag_ = false;

 public:
  qy_robot_bringup(/* args */);
  ~qy_robot_bringup();

  int buff_return(uint8_t high, uint8_t low, std::string name);
  void cmd_callback(const geometry_msgs::Twist::ConstPtr& twist);
  void serial_connect();
  void node_bringup();
  void speed_pub(int linear, int angular, bool flag);
  void odom_msg_pub(int num_a, int num_b);
  void thread_sub();
  int change_cosole(char *tty);
};

#endif
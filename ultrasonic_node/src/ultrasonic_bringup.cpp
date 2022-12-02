#include "ultrasonic_bringup.h"

#include <sensor_msgs/Range.h>

#include <iostream>

ultrasonic_bringup::ultrasonic_bringup() {
  ultrasonic_pub_ = nh_.advertise<sensor_msgs::Range>("/ultrasonic", 10);
  setlocale(LC_ALL, "");
  serial_connect();
}

ultrasonic_bringup::~ultrasonic_bringup() {}

void ultrasonic_bringup::serial_connect() {
  std::string serial_port;
  nh_.param<std::string>("/ultrasonic_node/serial_port", serial_port,
                         "/dev/ttyS3");

  serial::Timeout time_out = serial::Timeout::simpleTimeout(100);
  sp_.setPort(serial_port);
  sp_.setBaudrate(9600);
  sp_.setTimeout(time_out);

  try {
    sp_.open();
  } catch (serial::IOException& e) {
    std::cerr << e.what() << '\n';
    ROS_ERROR_STREAM("Unable to open port");
    ros::shutdown();
  }

  if (sp_.isOpen()) {
    ROS_INFO("\033[32m%s: 超声波串口通信成功...\033[0m", serial_port.c_str());

  } else {
    ros::shutdown();
  }
}

void ultrasonic_bringup::ultrasonic_go() {
  ros::Rate loop_rate(10);
  while (ros::ok()) {
    sensor_msgs::Range range;
    size_t n = sp_.available();
    if (n != 0) {
      uint8_t buffer[n];
      n = sp_.read(buffer, n);
      if ((buffer[0] & 0xff) == 255 && n == 10) {
        int data_left = buff_return(buffer[1], buffer[2], "left");
        int data_right = buff_return(buffer[3], buffer[4], "right");
        std::cout << "超声探测距离："<< data_left<< ", "<<data_right << std::endl;
        range.header.frame_id = "ultrasonic_link";
        range.header.stamp = ros::Time::now();
        range.field_of_view = 1;
        range.max_range = 0.5;
        range.min_range = 0.1;
        // if (data < 25) {
        //   range.range = 0.12;
        // } else {
        //   range.range = data / 100.;
        // }
        // ultrasonic_pub_.publish(range);
      }
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  sp_.close();
}

int ultrasonic_bringup::buff_return(uint8_t high, uint8_t low,
                                    std::string name) {
  uint16_t result_16;
  result_16 = high << 8 | low;
  int result = result_16;
  return result;
}

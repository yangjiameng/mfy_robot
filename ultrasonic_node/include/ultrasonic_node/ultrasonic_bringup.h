#include <ros/ros.h>
#include <serial/serial.h>

class ultrasonic_bringup {
 private:
  ros::NodeHandle nh_;
  ros::Publisher ultrasonic_pub_;
  serial::Serial sp_;

 public:
  ultrasonic_bringup(/* args */);
  ~ultrasonic_bringup();

  void serial_connect();
  void ultrasonic_go();
  int buff_return(uint8_t high, uint8_t low, std::string name);
};

#include "serial_demo.h"
#include <signal.h>
#include <thread>

qy_robot_bringup *qy_robot = nullptr;

void shutdown_signal(int sig) {
  if(qy_robot != nullptr) {
    delete qy_robot;
  }
}

int main(int argc, char** argv) {
    
  ros::init(argc, argv, "serial_node");
  qy_robot = new qy_robot_bringup();
  signal(SIGINT, shutdown_signal);
  std::thread thread_cmd(&qy_robot_bringup::thread_sub, qy_robot);
  qy_robot->node_bringup();
  thread_cmd.join();


  // qy_robot_bringup qy;
  // qy.node_bringup();

  return 0;
}

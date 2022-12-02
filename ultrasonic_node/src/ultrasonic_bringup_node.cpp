#include "ultrasonic_bringup.h"
#include "signal.h"


int main(int argc, char** argv) {

    ros::init(argc, argv, "ultrasonic_node");
    ultrasonic_bringup test;
    test.ultrasonic_go();
    

    return 0;

}
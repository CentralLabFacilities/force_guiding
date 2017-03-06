#include "ros/ros.h"
#include "MovementController.h"

int main(int argc, char **argv) {

    //init node
    ros::init(argc, argv, "meka_guiding");

    //create controller which then created modules
    MovementController mc;
    mc.start();

    ROS_INFO("... and we're spinning in the main thread!");
    ros::MultiThreadedSpinner().spin();
    
    return 0;
}
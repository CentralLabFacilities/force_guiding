#include "ros/ros.h"
#include "MovementController.h"



int main(int argc, char **argv) {

    //init node
    ros::init(argc, argv, "force_guiding");
    
    MovementController mc(ros::this_node::getName());
    
    ROS_INFO("... and we're spinning in the main thread!");
    ros::MultiThreadedSpinner().spin();
    
    return 0;
}
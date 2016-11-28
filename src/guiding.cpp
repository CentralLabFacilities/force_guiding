//
// Created by llach on 21.11.16.
//

#include "ros/ros.h"
#include "std_msgs/String.h"

/** global variables **/
std::string topic_sub, topic_pub, input_joint, controlled_joint;
void readParams(ros::NodeHandle nh);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "meka_guiding");

    ros::NodeHandle nh;

    readParams(nh);

    return 0;
}

void readParams(ros::NodeHandle nh){
    if(nh.getParam ("topic_sub", topic_sub)){
        ROS_INFO("Using given topic %s to subscribe to.", topic_sub.c_str());
    } else {
        topic_sub = "/joint_states";
        ROS_INFO("Using default topic %s to subscribe to.", topic_sub.c_str());
    }

    if(nh.getParam ("topic_pub", topic_pub)){
        ROS_INFO("Using given topic %s for publishing.", topic_pub.c_str());
    } else {
        topic_pub = "/joint_states";
        ROS_INFO("Using default topic %s for publishing.", topic_pub.c_str());
    }

    if(nh.getParam ("input_joint", input_joint)){
        ROS_INFO("Using given input joint %s.", input_joint.c_str());
    } else {
        input_joint = "right_arm_j0";
        ROS_INFO("Using default input joint %s.", input_joint.c_str());
    }

    if(nh.getParam ("controlled_joint", controlled_joint)){
        ROS_INFO("Using given joint %s to actuate.", controlled_joint.c_str());
    } else {
        controlled_joint = "zlift_j0";
        ROS_INFO("Using default joint %s to actuate.", controlled_joint.c_str());
    }
}
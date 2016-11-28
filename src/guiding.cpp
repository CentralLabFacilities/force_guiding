//
// Created by llach on 21.11.16.
//

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

/** global variables **/

//rosparams
std::string topic_sub, topic_pub, input_joint, controlled_joint;

//index of input joint
int joint_id = -1;
int cjoint_id = -1;

//position of joints
double joint_pos;
double cjoint_pos;

//input joint limits [atm hardcoded due to missing robot_description]
double joint_min = -1.40;
double jount_max = 3.49;
double cjoint_min = 0.0;
double cjount_max = 1.5;

/** function prototypes **/

//read rosparams or set default values
void readParams(ros::NodeHandle nh);

//subscriber callback function
void readState(const sensor_msgs::JointState::ConstPtr& js);


//takes care of all the node specific stuff
int main(int argc, char **argv)
{
    //init node with name
    ros::init(argc, argv, "meka_guiding");

    //create nodehandle
    ros::NodeHandle nh;

    //read necessary parameters
    readParams(nh);

    //add subscriber for joint_state topic
    ros::Subscriber sub = nh.subscribe(topic_sub, 500, readState);

    //do the spin
    ros::spin();

    return 0;
}

//reads the state of the joint that shall be actuated
void readState(const sensor_msgs::JointState::ConstPtr& js){

    //get joint indices
    for(int i = 0; i < js->name.size(); i++){

        if(js->name[i] == input_joint){
            joint_id = i;
            joint_pos = js->position[i];
            ROS_DEBUG("input_joint %s with id %d at position %f", input_joint.c_str(), joint_id, joint_pos);
        }

        if(js->name[i] == controlled_joint){
            cjoint_id = i;
            cjoint_pos = js->position[i];
            ROS_DEBUG("controlled_joint %s with id %d at position %f", controlled_joint.c_str(), cjoint_id, cjoint_pos);
        }
    }

    //shut down node, if input joint is unknown
    if(joint_id == -1){
        ROS_FATAL("Given input joint not found! Shutting down.");
        ros::shutdown();
    }

    //shut down node, if controlled joint is unknown
    if(cjoint_id == -1){
        ROS_FATAL("Given controlled joint not found! Shutting down.");
        ros::shutdown();
    }

}

//only looks for given parameters, otherwise uses hardcoded default values for sim
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
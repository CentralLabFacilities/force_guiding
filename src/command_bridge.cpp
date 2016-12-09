#include "ros/ros.h"
#include "control_msgs/JointTrajectoryControllerState.h"
#include "trajectory_msgs/JointTrajectory.h"

/** function prototypes **/
void callback(const trajectory_msgs::JointTrajectory::ConstPtr& msg);

/** global **/
ros::Publisher pub;

//takes care of all the node specific stuff
int main(int argc, char **argv)
{
    //init node with name
    ros::init(argc, argv, "command_bridge");

    //create nodehandle
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/meka_roscontrol/zlift_position_trajectory_controller/command", 500, callback);

    pub = nh.advertise<control_msgs::JointTrajectoryControllerState>("/meka_roscontrol/zlift_position_trajectory_controller/state", 500);
    //do the spin
    ros::spin();

    return 0;
}

void callback(const trajectory_msgs::JointTrajectory::ConstPtr& msg){
    double pos = msg->points.front().positions.front();
    control_msgs::JointTrajectoryControllerState msg_state;

    msg_state.actual.positions.push_back(pos);
    msg_state.desired.positions.push_back(pos);

    pub.publish(msg_state);
}

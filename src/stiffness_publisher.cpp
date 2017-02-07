#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"

//takes care of all the node specific stuff
int main(int argc, char **argv)
{
    //calibrate node with name
    ros::init(argc, argv, "stiffness_publisher");

    //create nodehandle
    ros::NodeHandle nh;

    std::string topic_stiff = "/meka_roscontrol/stiffness_controller/command";
    int jointcount = 29;
    int arm_count = 24; //first 24 joints are arms & hands
    double arm_stiff, torso_stiff;

    //get stiffness for arms and hands from rosparam; otherwise set default
    if(!nh.getParam ("astiff", arm_stiff)){
        arm_stiff = 0.3;  // default stiffness
    }

    //get stiffness for torso, head and zlift from rosparam; otherwise set default
    if(!nh.getParam ("tstiff", torso_stiff)){
        torso_stiff = 0.7;  // default stiffness
    }

    ros::Rate rate(10.0);

   //setup publisher and message
    ros::Publisher stiff_pub = nh.advertise<std_msgs::Float64MultiArray>(topic_stiff, 1000);
    std_msgs::Float64MultiArray stiff_array;

    //fill with desired stiffness
    for (int i = 0; i < arm_count; i++) {
        stiff_array.data.push_back(arm_stiff);
    }

    for (int i = arm_count; i < jointcount; i++) {
        stiff_array.data.push_back(torso_stiff);
    }

    //generate timeout
    ros::Time start_time = ros::Time::now();
    ros::Duration timeout(1.0);

    //wait until the publisher is initialized, but use a timeout to avoid locking if no subscriber is present
    while(stiff_pub.getNumSubscribers() == 0 && (ros::Time::now() - start_time < timeout)){
        ROS_INFO("publisher not yet ready");
        rate.sleep();       //sleep to avoid log bloating
    }

    //publish stiffnessarray
    ROS_INFO("Stiffness for arms & hands: %f; torso: %f", arm_stiff, torso_stiff);
    stiff_pub.publish(stiff_array);
    ros::spinOnce();

    ROS_INFO("DONE");

    return 0;
}

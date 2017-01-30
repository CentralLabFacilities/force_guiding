#include "ros/ros.h"
#include "BaseController.h"
#include "std_msgs/Float64MultiArray.h"

/**     function prototypes     **/
void readParams(ros::NodeHandle nh, std::string& topic_pub, std::string& topic_stiff, int& jointcount, double& stiffness);
void publishStiffness(ros::NodeHandle nh, std::string& topic_stiff, int jointcount, double stiffness);

//takes care of all the node specific stuff
int main(int argc, char **argv)
{
    //calibrate node with name
    ros::init(argc, argv, "meka_guiding");

    //create nodehandle
    ros::NodeHandle nh;

    //create base_controller
    //BaseController base_ctrl;

    std::string topic_pub;
    std::string topic_stiff;
    int jointcount;
    double stiffness;

    //read ros params
    readParams(nh, topic_pub, topic_stiff, jointcount, stiffness);
    
    publishStiffness(nh, topic_stiff, jointcount, stiffness);
    /*
    //initialze publisher
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>(topic_pub, 1);

    //set frequency to 10Hz
    ros::Rate rate(10.0);

    while(nh.ok()){
        pub.publish(base_ctrl.controlJoint());
	    rate.sleep();
        ros::spinOnce();
    }
    */
    return 0;
}

//reads parameter relevant to the main method
void readParams(ros::NodeHandle nh, std::string& topic_pub, std::string& topic_stiff, int& jointcount, double& stiffness){

    if(!nh.getParam ("topic_pub", topic_pub)){
        topic_pub = "/cmd_vel";
    }

    if(!nh.getParam ("topic_stiff", topic_stiff)){
        topic_stiff = "/meka_roscontrol/stiffness_controller/command"; //change to real topic
    }
    
    if(!nh.getParam ("jointcount", jointcount)){
        jointcount = 29; // fallback for meka's jointcount
    }
    
    if(!nh.getParam ("stiffness", stiffness)){
        stiffness = 0.3;  // default stiffness
    }

}

void publishStiffness(ros::NodeHandle nh, std::string& topic_stiff, int jointcount, double stiffness){
    ros::Publisher stiff_pub = nh.advertise<std_msgs::Float64MultiArray>(topic_stiff, 1000);
    std_msgs::Float64MultiArray stiff_array;

    ROS_INFO("Setting stiffness to %f for %d joints on topic %s", stiffness, jointcount, topic_stiff.c_str());

    for(int i = 0; i < jointcount; i++){
        stiff_array.data.push_back(stiffness);
    }
    
    stiff_pub.publish(stiff_array);
    
    ros::spinOnce();
}

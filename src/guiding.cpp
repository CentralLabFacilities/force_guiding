#include "ros/ros.h"
#include "BaseController.h"
#include "std_msgs/Float64MultiArray.h"

/**     function prototypes     **/
void readParams(ros::NodeHandle nh, std::string& topic_pub, std::string& topic_stiff, int& jointcount, double& stiffness);
void publishStiffness(ros::NodeHandle nh, ros::Rate rate, std::string& topic_stiff, int jointcount, double stiffness);

//takes care of all the node specific stuff
int main(int argc, char **argv)
{
    //calibrate node with name
    ros::init(argc, argv, "meka_guiding");

    //create nodehandle
    ros::NodeHandle nh;

    //create base_controller
    BaseController base_ctrl("test");
    BaseController base_ctrl2("tst2");

    std::string topic_pub;
    std::string topic_stiff;
    int jointcount;
    double stiffness;

    //read ros params
    readParams(nh, topic_pub, topic_stiff, jointcount, stiffness);

    //set frequency to 10Hz
    ros::Rate rate(10.0);

    //well ... publish stiffness!
    //publishStiffness(nh, rate, topic_stiff, jointcount, stiffness);
    
    //initialze publisher
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>(topic_pub, 1);

    while(nh.ok()){
        //pub.publish(base_ctrl.controlJoint());
	rate.sleep();
        ros::spinOnce();
    }
    
    return 0;
}

//reads parameter relevant to the main method; otherwise uses standard values
void readParams(ros::NodeHandle nh, std::string& topic_pub, std::string& topic_stiff, int& jointcount, double& stiffness){

    if(!nh.getParam ("topic_pub", topic_pub)){
        topic_pub = "/cmd_vel";
    }

    if(!nh.getParam ("topic_stiff", topic_stiff)){
        topic_stiff = "/meka_roscontrol/stiffness_controller/command";
    }
    
    if(!nh.getParam ("jointcount", jointcount)){
        jointcount = 29; // fallback for meka's jointcount
    }
    
    if(!nh.getParam ("stiffness", stiffness)){
        stiffness = 0.3;  // default stiffness
    }

    ROS_INFO("Parameters done.");

}

void publishStiffness(ros::NodeHandle nh, ros::Rate rate, std::string& topic_stiff, int jointcount, double stiffness) {

    //setup publisher and message
    ros::Publisher stiff_pub = nh.advertise<std_msgs::Float64MultiArray>(topic_stiff, 1000);
    std_msgs::Float64MultiArray stiff_array;

    //fill with desired stiffness
    for (int i = 0; i < jointcount; i++) {
        stiff_array.data.push_back(stiffness);
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
    ROS_INFO("Setting stiffness to %f for %d jjoints on topic %s", stiffness, jointcount, topic_stiff.c_str());
    stiff_pub.publish(stiff_array);
    ros::spinOnce();
}
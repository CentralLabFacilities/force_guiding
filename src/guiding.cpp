#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/Twist.h"
#include "EnumKeys.h"
#include "MovementModule.h"
#include <meka_guiding/ControllerConfig.h>

/**     function prototypes     **/
void readParams(ros::NodeHandle nh, std::string& topic_pub, std::string& topic_stiff, int& jointcount, double& stiffness);
void publishStiffness(ros::NodeHandle nh, ros::Rate rate, std::string& topic_stiff, int jointcount, double stiffness);
void parameterCallback(meka_guiding::ControllerConfig &config, uint32_t level);

//takes care of all the node specific stuff
int main(int argc, char **argv)
{
    //calibrate node with name
    ros::init(argc, argv, "meka_guiding");

    //create nodehandle
    ros::NodeHandle nh;

    //setup dynamic_reconfigure
    dynamic_reconfigure::Server<meka_guiding::ControllerConfig> dyn_reconf_server_;
    dynamic_reconfigure::Server<meka_guiding::ControllerConfig>::CallbackType f_;

    f_ = boost::bind(&parameterCallback, _1, _2);
    dyn_reconf_server_.setCallback(f_);

    std::string topic_pub;
    std::string topic_stiff;
    int jointcount;
    double stiffness;

    //read ros params
    readParams(nh, topic_pub, topic_stiff, jointcount, stiffness);

    // initialize modules
    MovementModule mv("hello", "base_link", "panplate", tf_key(1), cmd_key::LINEAR_Y, 0.2);

    //set frequency to 10Hz
    ros::Rate rate(10.0);

    //well ... publish stiffness!
    //publishStiffness(nh, rate, topic_stiff, jointcount, stiffness);
    
    //initialze publisher
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>(topic_pub, 1);

    while(nh.ok()){
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

void parameterCallback(meka_guiding::ControllerConfig &config, uint32_t level) {
    ROS_INFO_STREAM("ControllerReconfiguration");
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
    ROS_INFO("Setting stiffness to %f for %d joints on topic %s", stiffness, jointcount, topic_stiff.c_str());
    stiff_pub.publish(stiff_array);
    ros::spinOnce();
}
#include "ros/ros.h"
#include "Helper.h"

/**     function prototypes     **/
void readParams(ros::NodeHandle nh, std::string& topic_pub);

//takes care of all the node specific stuff
int main(int argc, char **argv)
{
    //calibrate node with name
    ros::init(argc, argv, "meka_guiding");

    //create nodehandle
    ros::NodeHandle nh;

    //create Helper
    Helper helper(nh);

    std::string topic_pub;

    //read ros params
    readParams(nh, topic_pub);

    //initialze publisher
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>(topic_pub, 10);

    //set frequency to 10Hz
    ros::Rate rate(10.0);

    while(nh.ok()){
        pub.publish(helper.controlJoint());
    }

    return 0;
}

//reads parameter relevant to the main method
void readParams(ros::NodeHandle nh, std::string& topic_pub){

    if(!nh.getParam ("topic_pub", topic_pub)){
        topic_pub = "/cmd_vel";
    }

}
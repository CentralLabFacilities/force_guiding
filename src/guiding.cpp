#include "ros/ros.h"
#include "Helper.h"

/**     variables       **/
Helper helper;
tf::TransformListener listener;
ros::Publisher pub;
std::string tf_src, tf_dst, topic_pub;

/**     function prototypes     **/
void readParams(ros::NodeHandle nh);


//takes care of all the node specific stuff
int main(int argc, char **argv)
{
    //std::cout << "start"  << std::endl;
    //calibrate node with name
    ros::init(argc, argv, "meka_guiding");

    //create nodehandle
    ros::NodeHandle nh;

    //read ros params
    readParams(nh);

    ROS_INFO("waiting for transform for .5s");
    listener.waitForTransform(tf_src, tf_dst, ros::Time::now(), ros::Duration(0.5));

    //initialze publisher
    pub = nh.advertise<geometry_msgs::Twist>(topic_pub, 10);

    //initialize helper
    helper.setup(tf_src, tf_dst);
    helper.calibrate();

    //RATE

    while(nh.ok()){
        pub.publish(helper.controlJoint());
    }

    return 0;
}

//checks is parameter were given; otherwise uses default values
void readParams(ros::NodeHandle nh){

    if(!nh.getParam ("tf_src", tf_src)){
        tf_src = "base_link";
    }

    if(!nh.getParam ("tf_dst", tf_dst)){
        tf_dst = "wrist_LEFT";
    }

    if(!nh.getParam ("topic_pub", topic_pub)){
        topic_pub = "/cmd_vel";
    }

}
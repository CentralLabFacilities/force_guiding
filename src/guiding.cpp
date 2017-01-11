#include "ros/ros.h"
#include "Helper.h"
#include "control_msgs/JointTrajectoryControllerState.h"

/** global variables **/

/**
 * using shared_ptr, so the constructor is later.
 * otherwise, the nh's used in Helper would be initialised before ros::init
 */
boost::shared_ptr<Helper> helper;

/** function prototypes **/
void callback(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg);

//takes care of all the node specific stuff
int main(int argc, char **argv)
{
    //init node with name
    ros::init(argc, argv, "meka_guiding");

    //create nodehandle
    ros::NodeHandle nh;

    //which service to wait for on meka?
    //ros::service::waitForService("spawn");

    //initialize helper
    helper.reset(new Helper());
    helper.get()->init(nh);

    //create subscriber
    ros::Subscriber sub = nh.subscribe(helper.get()->getTopicSub(), 500, callback);

    //do the spin
    ros::spin();

    return 0;
}

void callback(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg){

    //useful for debugging if msg has more then one joint
    for(float f : msg->actual.positions){
        ROS_DEBUG("val %f", f);
    }

    //get the acutal position
    helper.get()->setActZ(msg->actual.positions[0]);

    //control the joint
    helper.get()->controlJoint();
}
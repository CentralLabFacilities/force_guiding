#include "ros/ros.h"
#include "ZliftController.h"
#include "control_msgs/JointTrajectoryControllerState.h"

/** global variables **/

/**
 * using shared_ptr, so the constructor is called later.
 * otherwise, the nh's used in ZliftController would be initialised before ros::init
 */
boost::shared_ptr<ZliftController> zlift_ctrl;

/** function prototypes **/
void callback(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg);

//takes care of all the node specific stuff
int main(int argc, char **argv)
{
    //init node with name
    ros::init(argc, argv, "meka_guiding");

    //create nodehandle
    ros::NodeHandle nh;

    //initialize zlift_controller
    zlift_ctrl.reset(new ZliftController());
    zlift_ctrl.get()->init(nh);

    //create subscriber
    ros::Subscriber sub = nh.subscribe(zlift_ctrl.get()->getTopicSub(), 500, callback);

    //do the spin
    ros::spin();

    return 0;
}

void callback(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg){

    //get the acutal position
    zlift_ctrl.get()->setActZ(msg->actual.positions[0]);

    //control the joint
    zlift_ctrl.get()->controlJoint();
}
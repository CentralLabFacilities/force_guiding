#include "ros/ros.h"
#include "Helper.h"
#include "control_msgs/JointTrajectoryControllerState.h"

/** global variables **/

/**
 * using shared_ptr, so the constructor is later.
 * otherwise, the nh's used in Helper would be initialised before ros::init
 */
boost::shared_ptr<Helper> helper;


//takes care of all the node specific stuff
int main(int argc, char **argv)
{
    //init node with name
    ros::init(argc, argv, "meka_guiding");

    //create nodehandle
    ros::NodeHandle nh;

    //which service to wait for on meka? --> should be handled with waitForTransform in Helper::init();
    //ros::service::waitForService("spawn");

    //initialize helper
    helper.reset(new Helper());
    helper.get()->init(nh);

    //TODO call Helper::controlJoint after getting transform

    return 0;
}
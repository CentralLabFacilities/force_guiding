#include "Controller.h"

Controller::Controller(ros::NodeHandle nh) {
    ROS_DEBUG_STREAM("controller");

    dyn_reconf_server_ptr_.reset(new dynamic_reconfigure::Server<meka_guiding::ControllerConfig>(nh));
    f_ = boost::bind(&Controller::parameterCallback, this, _1, _2);
    dyn_reconf_server_ptr_.get()->setCallback(f_);

}

geometry_msgs::Twist Controller::controlJoint() {
    geometry_msgs::Twist twist;
    return twist;
}

void Controller::parameterCallback(meka_guiding::ControllerConfig &config, uint32_t level) {
    ROS_DEBUG_STREAM("ControllerReconfiguration");
}

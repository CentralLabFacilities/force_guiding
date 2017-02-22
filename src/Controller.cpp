#include "Controller.h"

Controller::Controller() {
    f_ = boost::bind(&Controller::parameterCallback, this, _1, _2);
    dyn_reconf_server_.setCallback(f_);
}

geometry_msgs::Twist Controller::controlJoint() {
    geometry_msgs::Twist twist;

    return twist;
}

void Controller::parameterCallback(meka_guiding::GuidingConfig &config, uint32_t level) {
}

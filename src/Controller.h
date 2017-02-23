#ifndef MEKA_GUIDING_CONTROLLER_H
#define MEKA_GUIDING_CONTROLLER_H


#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "dynamic_reconfigure/server.h"
#include "MovementModule.h"
#include <meka_guiding/ControllerConfig.h>

class Controller {

public:
    /**     constructor     **/
    Controller(ros::NodeHandle nh);

    /**     functions  **/
    geometry_msgs::Twist controlJoint();
    void parameterCallback(meka_guiding::ControllerConfig &config, uint32_t level);

private:
    /**     dynamic     **/
    boost::shared_ptr<dynamic_reconfigure::Server<meka_guiding::ControllerConfig> > dyn_reconf_server_ptr_;
    dynamic_reconfigure::Server<meka_guiding::ControllerConfig>::CallbackType f_;

    /**     constants   **/

    /**     variables   **/

    /**     functions  **/
};


#endif //MEKA_GUIDING_CONTROLLER_H

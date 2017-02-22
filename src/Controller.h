#ifndef MEKA_GUIDING_CONTROLLER_H
#define MEKA_GUIDING_CONTROLLER_H


#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "dynamic_reconfigure/server.h"
#include <meka_guiding/GuidingConfig.h>

class Controller {

public:
    /**     constructor     **/
    Controller();

    /**     functions  **/
    geometry_msgs::Twist controlJoint();
    void parameterCallback(meka_guiding::GuidingConfig &config, uint32_t level);

private:
    /**     dynamic     **/
    dynamic_reconfigure::Server<meka_guiding::GuidingConfig> dyn_reconf_server_;
    dynamic_reconfigure::Server<meka_guiding::GuidingConfig>::CallbackType f_;
    
    /**     constants   **/

    /**     variables   **/
    
    /**     functions  **/
};


#endif //MEKA_GUIDING_CONTROLLER_H

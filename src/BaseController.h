#ifndef MEKA_GUIDING_HELPER_H
#define MEKA_GUIDING_HELPER_H


#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "geometry_msgs/Twist.h"
#include "dynamic_reconfigure/server.h"
#include <meka_guiding/GuidingConfig.h>

class BaseController {

public:
    /**     constructor     **/
    BaseController();

    /**     functions  **/
    geometry_msgs::Twist controlJoint();
    void parameterCallback(meka_guiding::GuidingConfig &config, uint32_t level);

private:
    /**     constants   **/
    const double VELOCITY_LOWER = 0.0;
    const int MAX_CALIBRATION_TRIES = 5;




    /**     variables   **/
    tf::TransformListener listener;
    tf::StampedTransform transform;
    tf::Vector3 initial_translation, new_translation;

    double x_vel, y_vel;

    /**     dynamic     **/
    double LINEAR_VELOCITY_UPPER = 5.0;
    double ANGULAR_VELOCITY_UPPER = 10.0;
    double VELOCITY_FACTOR = 1.0;
    double DEADLOCK_SIZE = 0.1;

    std::string tf_src = "/base_link";
    std::string tf_dst = "/wrist_LEFT";

    dynamic_reconfigure::Server<meka_guiding::GuidingConfig> server;
    dynamic_reconfigure::Server<meka_guiding::GuidingConfig>::CallbackType f;

    /**     functions   **/
    void calibrate();
    void calcVelocity();
    bool lookupInitialTransform();

};


#endif //MEKA_GUIDING_HELPER_H

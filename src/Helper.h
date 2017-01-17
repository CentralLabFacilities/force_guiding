#ifndef MEKA_GUIDING_HELPER_H
#define MEKA_GUIDING_HELPER_H


#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "geometry_msgs/Twist.h"

class Helper {

public:
    /**     functions  **/
    void calibrate();
    geometry_msgs::Twist controlJoint();
    void setup(std::string tf_src, std::string tf_dst);

private:
    /**     constants   **/
    const double VELOCITY_MIN = 0.0;
    const double VELOCITY_MAX = 1.0;
    const double DEADLOCK_SIZE = 0.1;
    const double VELOCITY_FATOR = 1.0;
    const int MAX_CALIBRATION_TRIES = 5;

    /**     variables   **/
    tf::TransformListener listener;
    tf::StampedTransform transform;
    tf::Vector3 initial_translation, new_translation;
    std::string tf_src, tf_dst;
    double x_vel, y_vel;


    /**     functions   **/
    void calcVelocity();
    bool lookupInitialTransform();

};


#endif //MEKA_GUIDING_HELPER_H

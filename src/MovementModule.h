/* 
 * File:   MovementModule.h
 * Author: llach
 *
 * Created on February 22, 2017, 5:48 PM
 */

#ifndef MEKA_GUIDING_MOVEMENTMODULE_H
#define	MEKA_GUIDING_MOVEMENTMODULE_H

#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "dynamic_reconfigure/server.h"
#include <meka_guiding/GuidingConfig.h>

class MovementModule {

public:
    /**     constructor     **/
    MovementModule();

    /**     functions  **/
    void parameterCallback(meka_guiding::GuidingConfig &config, uint32_t level);

private:
        /**     dynamic     **/
    double LINEAR_VELOCITY_UPPER = 5.0;
    double ANGULAR_VELOCITY_UPPER = 10.0;
    double VELOCITY_FACTOR = 1.0;
    double DEADLOCK_SIZE = 0.1;
    bool ACTIVATION_TOGGLE = true;

    boost::shared_ptr<dynamic_reconfigure::Server<meka_guiding::GuidingConfig> > dyn_reconf_server_ptr_;
    dynamic_reconfigure::Server<meka_guiding::GuidingConfig>::CallbackType f;
    
    /**     constants   **/
    const double VELOCITY_LOWER = 0.0;
    const int MAX_CALIBRATION_TRIES = 5;

    /**     variables   **/
    std::string tf_src;
    std::string tf_dst;
    std::string nameprefix = "~";
    std::string nhname;
    
    tf::TransformListener listener;
    tf::StampedTransform transform;
    tf::Vector3 initial_translation, new_translation;

    double x_vel, y_vel;
    
    /**     functions   **/
    void calibrate();
    void calcVelocity();
    bool lookupInitialTransform();
};



#endif	/* MEKA_GUIDING_MOVEMENTMODULE_H */


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
#include <meka_guiding/ModuleConfig.h>

class MovementModule {

public:
    /**     constructor     **/
    MovementModule(std::string name);
    MovementModule(std::string name, std::string tf_src, std::string tf_dst, int tf_key, int cmd_key, float velocity_factor);

    /**     functions  **/
    void parameterCallback(meka_guiding::ModuleConfig &config, uint32_t level);

private:
        /**     dynamic     **/
    double LINEAR_VELOCITY_UPPER = 5.0;
    double ANGULAR_VELOCITY_UPPER = 10.0;
    double VELOCITY_FACTOR = 1.0;
    double DEADLOCK_SIZE = 0.1;
    bool ACTIVATION_TOGGLE = true;

    boost::shared_ptr<dynamic_reconfigure::Server<meka_guiding::ModuleConfig> > dyn_reconf_server_ptr_;
    dynamic_reconfigure::Server<meka_guiding::ModuleConfig>::CallbackType f_;
    
    /**     constants   **/
    const double VELOCITY_LOWER = 0.0;
    const int MAX_CALIBRATION_TRIES = 5;  //--> error handling

    /**     variables   **/
    std::string tf_src;
    std::string tf_dst;
    std::string nameprefix_ = "~";
    std::string nhname_;
    
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


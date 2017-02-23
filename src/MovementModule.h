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
    MovementModule(std::string name, std::string tf_src, std::string tf_dst, int tf_key, int dir_key, float velocity_factor);

    /**     functions  **/
    

private:
    /**     dynamic     **/
    std::string tf_src_;
    std::string tf_dst_;
    
    int tf_key_;
    int dir_key_;
    
    double velocity_upper_ = 5.0;
    double velocity_factor_ = 1.0;
    double deadzone_factor_ = 0.1;
    
    bool activation_toggle_ = true;
    bool reflection_toggle_ = false;

    boost::recursive_mutex dyn_reconfigure_mutex_;
    boost::shared_ptr<dynamic_reconfigure::Server<meka_guiding::ModuleConfig> > dyn_reconfigure_server_ptr_;
    dynamic_reconfigure::Server<meka_guiding::ModuleConfig>::CallbackType f_;
    
    /**     constants   **/
    const double VELOCITY_LOWER = 0.0;
    const int MAX_CALIBRATION_TRIES = 5;  //--> error handling

    /**     variables   **/
    std::string nameprefix_ = "~";
    std::string nhname_;
    
    tf::TransformListener listener;
    tf::StampedTransform transform;
    tf::Vector3 initial_translation, new_translation;

    double velocity_;
    
    /**     functions   **/
    void calibrate();
    void calcVelocity();
    bool lookupInitialTransform();
    void initializeDynamicReconfigure();
    
    void parameterCallback(meka_guiding::ModuleConfig &config, uint32_t level);
};



#endif	/* MEKA_GUIDING_MOVEMENTMODULE_H */


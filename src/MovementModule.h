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
#include "EnumKeys.h"
#include <meka_guiding/ModuleConfig.h>

class MovementModule {

public:
    /**     constructor     **/
    MovementModule(std::string name, std::string tf_src, std::string tf_dst, tf_key tf_key, dir_key dir, float velocity_factor);

    /**     functions  **/
    

private:
    /**     dynamic     **/
    std::string tf_src_;
    std::string tf_dst_;
    
    tf_key tf_key_;
    dir_key dir_key_;
    
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
    std::string name_;
    
    tf::TransformListener listener;
    tf::StampedTransform transform;

    double reference_position_;
    
    /**     functions   **/
    void calibrate();
    void calcVelocity();
    double getPositionByKey();
    void initializeDynamicReconfigure();
    void parameterCallback(meka_guiding::ModuleConfig &config, uint32_t level);
};



#endif	/* MEKA_GUIDING_MOVEMENTMODULE_H */


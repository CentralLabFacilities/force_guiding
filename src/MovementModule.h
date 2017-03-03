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
#include <XmlRpcException.h>
#include "dynamic_reconfigure/server.h"
#include "meka_guiding/Velocity.h"
#include "EnumKeys.h"
#include <meka_guiding/ModuleConfig.h>

class MovementModule {

public:
    /**     constructor     **/
    MovementModule(std::string name, cmd_key key, XmlRpc::XmlRpcValue params = new XmlRpc::XmlRpcValue);

    /**     functions   **/
private:
    /**     dynamic     **/
    std::string tf_src_;
    std::string tf_dst_;
    
    tf_key tf_key_;
    dir_key dir_key_;
    cmd_key cmd_key_;
    
    double velocity_upper_;
    double velocity_factor_;
    double deadzone_factor_;
    
    bool enable_toggle_;

    boost::recursive_mutex dyn_reconfigure_mutex_;
    boost::shared_ptr<dynamic_reconfigure::Server<meka_guiding::ModuleConfig> > dyn_reconfigure_server_ptr_;
    dynamic_reconfigure::Server<meka_guiding::ModuleConfig>::CallbackType f_;

    boost::shared_ptr<ros::ServiceServer> service_ptr_;

    /**     variables   **/
    std::string name_;
    
    tf::TransformListener listener_;
    tf::StampedTransform transform_;

    double reference_position_;
    double last_vel_ = 0;
    
    /**     functions   **/
    bool calcVelocity(meka_guiding::Velocity::Request &request, meka_guiding::Velocity::Response &response);
    double getPositionByKey(ros::Time  = ros::Time::now());
    void overrideDefaultParameter(XmlRpc::XmlRpcValue params);
    void readConfig(meka_guiding::ModuleConfig &config);
    void parameterCallback(meka_guiding::ModuleConfig &config, uint32_t level);
};



#endif	/* MEKA_GUIDING_MOVEMENTMODULE_H */


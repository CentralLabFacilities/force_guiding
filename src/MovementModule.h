/* 
 * File:   MovementModule.h
 * Author: llach
 *
 * Created on February 22, 2017, 5:48 PM
 */

#ifndef FORCE_GUIDING_MOVEMENTMODULE_H
#define	FORCE_GUIDING_MOVEMENTMODULE_H

#include "ros/ros.h"
#include "tf/transform_listener.h"
#include <XmlRpcException.h>
#include "dynamic_reconfigure/server.h"
#include "force_guiding/Velocity.h"
#include "EnumKeys.h"
#include <force_guiding/ModuleConfig.h>

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
    boost::shared_ptr<dynamic_reconfigure::Server<force_guiding::ModuleConfig> > dyn_reconfigure_server_ptr_;
    dynamic_reconfigure::Server<force_guiding::ModuleConfig>::CallbackType f_;

    boost::shared_ptr<ros::ServiceServer> service_ptr_;

    /**     constants   **/
    const std::map<std::string, tf_key> tf_string_map = {
        {"X_AXIS", tf_key::X_AXIS},
        {"Y_AXIS", tf_key::Y_AXIS},
        {"Z_AXIS", tf_key::Z_AXIS},
        {"ROLL", tf_key::ROLL},
        {"PITCH", tf_key::PITCH},
        {"YAW", tf_key::YAW}
    };
    
    const std::map<std::string, dir_key> dir_string_map = {
        {"POSITIVE", dir_key::POSITIVE},
        {"NEGATIVE", dir_key::NEGATIVE},
        {"BIDIRECTIONAL", dir_key::BIDIRECTIONAL}
    };
    
    /**     variables   **/
    std::string name_;
    
    tf::TransformListener listener_;
    tf::StampedTransform transform_;

    double reference_position_;
    double last_vel_ = 0;
    
    /**     functions   **/
    bool calcVelocity(force_guiding::Velocity::Request &request, force_guiding::Velocity::Response &response);
    double getPositionByKey(ros::Time  = ros::Time::now());
    void overrideDefaultParameter(XmlRpc::XmlRpcValue params);
    void readConfig(force_guiding::ModuleConfig &config);
    void parameterCallback(force_guiding::ModuleConfig &config, uint32_t level);
    
    bool matchTfKey(tf_key& key, std::string key_string);
    bool matchTfKey(tf_key& key, int key_int);
    bool matchDirKey(dir_key& key, std::string key_string);
    bool matchDirKey(dir_key& key, int key_int);
};



#endif	/* FORCE_GUIDING_MOVEMENTMODULE_H */


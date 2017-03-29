//
// Created by llach on 03.03.17.
//

#ifndef FORCE_GUIDING_MOVEMENTCONTROLLER_H
#define FORCE_GUIDING_MOVEMENTCONTROLLER_H

#include "geometry_msgs/Twist.h"
#include "MovementModule.h"
#include <force_guiding/ControllerConfig.h>
#include <dynamic_reconfigure/server.h>
#include <actionlib/server/simple_action_server.h>
#include <force_guiding/GuidingAction.h>
#include <thread>

class MovementController {

public:
    /**     constructor     **/
    MovementController(std::string name);

private:
    ros::NodeHandle nh;
    
    /**     dynamic     **/
    boost::recursive_mutex dyn_reconfigure_mutex_;
    boost::shared_ptr<dynamic_reconfigure::Server<force_guiding::ControllerConfig> > dyn_reconfigure_server_ptr_;
    dynamic_reconfigure::Server<force_guiding::ControllerConfig>::CallbackType f_;

    bool priority_;
    bool startup = true;
    bool kill_thread_ = false;

    /**     variables   **/
    std::map<cmd_key, std::string> cmd_map = {
            {cmd_key::LINEAR_X, ""},
            {cmd_key::LINEAR_Y, ""},
            {cmd_key::LINEAR_Z, ""},
            {cmd_key::ANGULAR_X, ""},
            {cmd_key::ANGULAR_Y, ""},
            {cmd_key::ANGULAR_Z, ""}
    };
    
    std::string topic_pub;
    boost::shared_ptr<ros::Publisher> pub_ptr_;
    
    /**     module management   **/
    boost::mutex mv_mutex;
    std::vector<boost::shared_ptr<MovementModule> > mv;
    std::vector<std::string> active_modules;
    std::vector<ros::ServiceClient> client_list;

    //actionlib
    actionlib::SimpleActionServer<force_guiding::GuidingAction> as_;
    bool running_ = false;
    
    /**     constants   **/
    const std::map<std::string, cmd_key> cmd_string_map = {
        {"LINEAR_X", cmd_key::LINEAR_X},
        {"LINEAR_Y", cmd_key::LINEAR_Y},
        {"LINEAR_Z", cmd_key::LINEAR_Z},
        {"ANGULAR_X", cmd_key::ANGULAR_X},
        {"ANGULAR_Y", cmd_key::ANGULAR_Y},
        {"ANGULAR_Z", cmd_key::ANGULAR_Z}
    };

    /**     functions    **/
    void parameterCallback(force_guiding::ControllerConfig &config, uint32_t level);
    bool configure(ros::NodeHandle nh);
    void setConfig();
    std::vector<std::string> split(std::string str, char delimiter);
    bool is_int(const std::string& s);
    bool addModule(std::string name, cmd_key key, XmlRpc::XmlRpcValue params = new XmlRpc::XmlRpcValue);
    void generateAndPublish();
    void setVelocityByKey(geometry_msgs::Twist& msg, double velocity, cmd_key key);
    
    bool matchCmdKey(cmd_key& key, std::string name, std::string key_string);
    bool matchCmdKey(cmd_key& key, std::string name, int key_int);
    
    void preemptCallback();
    void start(const force_guiding::GuidingGoalConstPtr &goal);

};


#endif //FORCE_GUIDING_MOVEMENTCONTROLLER_H

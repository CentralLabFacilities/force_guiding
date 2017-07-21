//
// Created by llach on 03.03.17.
//

#include "MovementController.h"

MovementController::MovementController(std::string name) : as_(nh, name, boost::bind(&MovementController::start, this, _1), false){

    //configure and load modules
    if (!configure(nh)) {
        ROS_FATAL("Configuration failed!");
        ros::shutdown();
    }

    //setup dynamic_reconfigure
    dyn_reconfigure_server_ptr_.reset(new dynamic_reconfigure::Server<force_guiding::ControllerConfig>(dyn_reconfigure_mutex_));

    f_ = boost::bind(&MovementController::parameterCallback, this, _1, _2);
    dyn_reconfigure_server_ptr_.get()->setCallback(f_);

    if(!nh.getParam("topic_pub", topic_pub)){
        ROS_ERROR("No topic to publish, using cmd_vel!");
        topic_pub = "/cmd_vel";
    }

    as_.start();
    
    //initialze publisher
    pub_ptr_.reset(new ros::Publisher(nh.advertise<geometry_msgs::Twist>(topic_pub, 1)));
    
}

void MovementController::start(const force_guiding::GuidingGoalConstPtr &goal) {
    ROS_INFO("calibrating modules ...");
    for(auto mod_ptr : mv){
        mod_ptr.get()->calibrate();
    }

    ROS_INFO("creating message generator thread ... ");
    generateAndPublish();
}

bool MovementController::configure(ros::NodeHandle nh = ros::NodeHandle()) {
    /* https://github.com/ros/filters/blob/hydro-devel/include/filters/filter_chain.h */

    XmlRpc::XmlRpcValue config;

    if (nh.getParam("ModuleList", config) && config.size() > 0) {
        ROS_INFO("Found module list containing %d module(s)", config.size());
    } else {
        ROS_ERROR("No modules found to configure!");
        return false;
    }


    //verify proper naming and structure
    if (config.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        ROS_ERROR("The module list specification must be a list. but is of of XmlRpcType %d", config.getType());
        ROS_ERROR("The xml passed in is formatted as follows:\n %s", config.toXml().c_str());
        return false;
    }

    bool valid = true;
    cmd_key key;

    //Iterate over all modules
    for (int i = 0; i < config.size(); ++i) {
        if (config[i].getType() != XmlRpc::XmlRpcValue::TypeStruct) {
            ROS_ERROR("Modules must be specified as maps, but they are XmlRpcType:%d", config[i].getType());
            valid = false;
        } else if (!config[i].hasMember("name") || config[i]["name"].getType() != XmlRpc::XmlRpcValue::TypeString) {
            ROS_ERROR("Could not add module %d because no valid name was given", i);
            valid = false;
        } else if (!config[i].hasMember("base_dof")) {
            ROS_INFO("%s no base_dof was given, using default", std::string(config[i]["name"]).c_str());
            key = cmd_key::LINEAR_X;
        }
        
        std::string module_name = std::string(config[i]["name"]);

        for (int j = 0; j < active_modules.size(); j++) {
            if (module_name == active_modules[j]) {
                ROS_ERROR("Module names have to be unique! %s violates that.", active_modules[j].c_str());
                valid = false;
            }
        }

        
        if (config[i]["base_dof"].getType() == XmlRpc::XmlRpcValue::TypeInt) {
            if(!matchCmdKey(key, module_name, int(config[i]["base_dof"])))
                key = cmd_key::LINEAR_X;
        } else if (config[i]["base_dof"].getType() == XmlRpc::XmlRpcValue::TypeString) {
            if(!matchCmdKey(key, module_name, std::string(config[i]["base_dof"])))
                key = cmd_key::LINEAR_X;
        } else {
            ROS_INFO("%s using default dof, because base_dof was not valid", module_name.c_str());
            key = cmd_key::LINEAR_X;
        }

        if (!valid)
            break;
        
        if (config[i].hasMember("params")) {
            valid = addModule(module_name, key, config[i]["params"]);
        } else {
            valid = addModule(module_name, key);
        }

        if (!valid)
            break;
    }

    return valid;
}


bool MovementController::addModule(std::string name, cmd_key key, XmlRpc::XmlRpcValue params) {

    //check for dubs
    for (int j = 0; j < active_modules.size(); j++) {
        if (active_modules[j] == name) {
            ROS_ERROR("Module names have to be unique! %s violates that.", active_modules[j].c_str());
            return false;
        }
    }

    boost::shared_ptr<MovementModule> mm(new MovementModule(name, key, params));

    mv_mutex.lock();
    mv.push_back(mm);
    mv_mutex.unlock();

    ROS_INFO("Created module %s", name.c_str());

    std::string service_name(name);
    service_name.append("/calculateVelocity");

    client_list.push_back(ros::NodeHandle("~").serviceClient<force_guiding::Velocity>(service_name));
    active_modules.push_back(name);
    ROS_DEBUG_STREAM("connected to: " << client_list[client_list.size() - 1].getService());

    return true;
}
void MovementController::generateAndPublish() {
    
    ros::Rate rate(5);
    ros::Time sync_stamp;
    geometry_msgs::Twist twist;
    
    running_ = true;
    
    while (ros::NodeHandle("~").ok()) {
        sync_stamp = ros::Time::now();
        twist = geometry_msgs::Twist();
        
        if(!as_.isActive() || as_.isPreemptRequested())  {
            ROS_INFO("Preempted");
            as_.setPreempted();
            break;
        }
        /** maybe call async clients in one for loop and collect in another one **/
        
        for (auto client : client_list) {
            force_guiding::Velocity srv;
            srv.request.stamp = sync_stamp;

            if (client.call(srv)) {
                if (srv.response.finished_movement) {
                    ROS_INFO_STREAM(srv.response.name << " finished movement");

                    for (auto& it : cmd_map) {
                        if(it.second == srv.response.name){
                            ROS_DEBUG("cmd key %d will be freed from %s", static_cast<int>(it.first), it.second.c_str());
                            it.second = "";
                        }
                    }

                    continue;
                    }
               
                if (priority_) {
                    if (srv.response.priority_flag) {
                        cmd_map[cmd_key(srv.response.base_dof)] = srv.response.name;
                        setVelocityByKey(twist, srv.response.vel, cmd_key(srv.response.base_dof));
                    } else if (cmd_map[cmd_key(srv.response.base_dof)] == srv.response.name || cmd_map[cmd_key(srv.response.base_dof)] == "") {
                        setVelocityByKey(twist, srv.response.vel, cmd_key(srv.response.base_dof));
                    } else {
                        continue;
                    }
                } else {
                    if (cmd_map[cmd_key(srv.response.base_dof)] == srv.response.name || cmd_map[cmd_key(srv.response.base_dof)] == "") {
                        cmd_map[cmd_key(srv.response.base_dof)] = srv.response.name;
                        setVelocityByKey(twist, srv.response.vel, cmd_key(srv.response.base_dof));
                    } else {
                        continue;
                    }
                }


            } else {
                ROS_ERROR_STREAM("Controller: unable to communicate with " << client.getService());
            }
        }

        pub_ptr_.get()->publish(twist);

        //obsolete due to sleeping
        //double dt = ros::Time::now().toSec() - sync_stamp.toSec();
        //ROS_DEBUG("controller: reading at took %.2f hZ", 1/dt);
        
        rate.sleep();
    }
    
    running_ = false;
   
}


void MovementController::setVelocityByKey(geometry_msgs::Twist& msg, double velocity, cmd_key key){
    switch (key){
        case cmd_key::LINEAR_X:
            msg.linear.x = velocity;
            break;
        case cmd_key::LINEAR_Y:
            msg.linear.y = velocity;
            break;
        case cmd_key::LINEAR_Z:
            msg.linear.z = velocity;
            break;
        case cmd_key::ANGULAR_X:
            msg.angular.x = velocity;
            break;
        case cmd_key::ANGULAR_Y:
            msg.angular.y = velocity;
            break;
        case cmd_key::ANGULAR_Z:
            msg.angular.z = velocity;
            break;
    }
}


void MovementController::parameterCallback(force_guiding::ControllerConfig &config, uint32_t level) {
    ROS_INFO_STREAM("ControllerReconfiguration");
    
    if (startup) {
        startup = false;
        return;
    }
    
    //set priority or fcfs mode
    priority_ = config.priority_mode;

    std::vector<std::string> new_module = split(config.new_module, ' ');
    
    ROS_INFO("adding module %s", new_module[0].c_str());
    addModule(new_module[0], cmd_key::LINEAR_X);
}


//maybe not needed as this was required for killing modules which kills reconfigure
void MovementController::setConfig() {
    //make sure all values are set
    force_guiding::ControllerConfig config;
    dyn_reconfigure_server_ptr_.get()->getConfigDefault(config);

    std::string module_list = "";

    for (int i = 0; i < active_modules.size(); i++) {
        module_list.append(active_modules[i]).append(" ");
    }

    ROS_DEBUG("module list: %s", module_list.c_str());

    //config.module_list = module_list; 

    boost::recursive_mutex::scoped_lock dyn_reconf_lock(dyn_reconfigure_mutex_);
    dyn_reconfigure_server_ptr_.get()->updateConfig(config);
    dyn_reconf_lock.unlock();
}


//split string by turning it into a stream and reading lines
std::vector<std::string> MovementController::split(std::string str, char delimiter) {
    std::vector<std::string> internal;
    std::stringstream ss(str);
    std::string tok;

    while (getline(ss, tok, delimiter)) {
        internal.push_back(tok);
    }

    return internal;
}


bool MovementController::is_int(const std::string& s) {
    return !s.empty() && std::find_if(s.begin(),
                                      s.end(), [](char c) {
                return !std::isdigit(c); }) == s.end();
}

bool MovementController::matchCmdKey(cmd_key& key, std::string name, std::string key_string) {
    bool found = false;

    for (auto& it_key : cmd_string_map) {
        if (it_key.first == key_string) {
            ROS_DEBUG_STREAM("found key by name " << key_string);
            key = it_key.second;
            found = true;
            break;
        }
    }
    
    if(!found)
        ROS_ERROR("Could not add %s because the given string %s does not map to a cmd_key", name.c_str(), key_string.c_str());

    return found;
}

bool MovementController::matchCmdKey(cmd_key& key, std::string name, int key_int) {
    if (key_int < 0 || key_int > static_cast<int> (cmd_key::CMD_KEY_MAX)) {
        ROS_ERROR("Could not add %s because the %d didn't match a cmd_key", name.c_str(), key_int);
        return false;
    } else {
        key = cmd_key(key_int);
    }
    
    return true;
}
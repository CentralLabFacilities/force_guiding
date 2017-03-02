#include <thread>
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/Twist.h"
#include "MovementModule.h"
#include <meka_guiding/ControllerConfig.h>

const std::map<std::string, cmd_key> cmd_map = {
        {"LINEAR_X", cmd_key::LINEAR_X},
        {"LINEAR_Y", cmd_key::LINEAR_Y},
        {"LINEAR_Z", cmd_key::LINEAR_Z},
        {"ANGULAR_X", cmd_key::ANGULAR_X},
        {"ANGULAR_Y", cmd_key::ANGULAR_Y},
        {"ANGULAR_Z", cmd_key::ANGULAR_Z}
};

boost::mutex mv_mutex;
std::vector<boost::shared_ptr<MovementModule> > mv;
std::vector<std::string> active_modules;
std::vector<ros::ServiceClient> client_list;

boost::recursive_mutex dyn_reconfigure_mutex_;
boost::shared_ptr<dynamic_reconfigure::Server<meka_guiding::ControllerConfig> > dyn_reconfigure_server_ptr_;
dynamic_reconfigure::Server<meka_guiding::ControllerConfig>::CallbackType f_;

ros::MultiThreadedSpinner spinner;

bool startup = true;

/**     function prototypes     **/
void parameterCallback(meka_guiding::ControllerConfig &config, uint32_t level);
bool configure(ros::NodeHandle nh);
void setConfig();
std::vector<std::string> split(std::string str, char delimiter);
bool is_int(const std::string& s);
bool addModule(std::string name, XmlRpc::XmlRpcValue params = new XmlRpc::XmlRpcValue);
void generateAndPublish();

//takes care of all the node specific stuff

int main(int argc, char **argv) {
    //calibrate node with name
    ros::init(argc, argv, "meka_guiding");

    //create nodehandle
    ros::NodeHandle nh;

    //configure and load modules
    if (!configure(nh)) {
        ROS_FATAL("Configuration failed!");
        ros::shutdown();
    }

    //setup dynamic_reconfigure
    dyn_reconfigure_server_ptr_.reset(new dynamic_reconfigure::Server<meka_guiding::ControllerConfig>(dyn_reconfigure_mutex_));

    setConfig();

    f_ = boost::bind(&parameterCallback, _1, _2);
    dyn_reconfigure_server_ptr_.get()->setCallback(f_);

    std::string topic_pub = "/cmd_vel";

    //set frequency to 10Hz
    ros::Rate rate(10.0);

    //initialze publisher
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>(topic_pub, 1);
    
    ROS_INFO("creating message generator thread ... ");

    std::thread t(generateAndPublish);
    
    ROS_INFO("... and we're spinning in the main thread!");

    spinner.spin();
    return 0;
}

void generateAndPublish() {
    while (ros::NodeHandle("~").ok()) {
        ros::Time sync_stamp = ros::Time::now();
        
        for (auto client : client_list) {
            meka_guiding::Velocity srv;
            srv.request.stamp = sync_stamp;
            
            if (client.call(srv)) {
                ROS_DEBUG("Controller: %s calculated %f", client.getService().c_str(), srv.response.vel);
            } else {
                ROS_ERROR_STREAM("Controller: unable to communicate with " << client.getService());
            }
        }
        
        double dt = ros::Time::now().toSec() - sync_stamp.toSec();
        ROS_DEBUG("controller: reading at %.2f hZ", 1/dt);
    }
}

void parameterCallback(meka_guiding::ControllerConfig &config, uint32_t level) {
    ROS_INFO_STREAM("ControllerReconfiguration");

    if (startup) {
        startup = false;
        return;
    }

    /*  broken bc it kills dynamic reconf
      
    bool removed = false;

    for (int i = 0; i < active_modules.size(); i++) {
        if (config.module_list.find(active_modules[i]) != std::string::npos) {
            ROS_INFO("Keeping module %s", active_modules[i].c_str());
        } else {
            ROS_INFO("Removing module %s", active_modules[i].c_str());
            active_modules.erase(active_modules.begin() + i);
            mv[i].reset();
            mv.erase(mv.begin() + i);
            removed = true;
        }
    }

    if (removed) {
        setConfig();
        return;
    }*/

    std::vector<std::string> new_module = split(config.add_module, ' ');
    if (static_cast<int> (new_module.size()) == 2 && is_int(new_module[1])) {
        addModule(new_module[0]);
    } else {
        ROS_ERROR("Controller: wrong arguments in add_module");
    }
}

void setConfig() {
    //make sure all values are set
    meka_guiding::ControllerConfig config;
    dyn_reconfigure_server_ptr_.get()->getConfigDefault(config);

    std::string module_list = "";

    for (int i = 0; i < active_modules.size(); i++) {
        module_list.append(active_modules[i]).append(" ");
    }

    ROS_DEBUG("module list: %s", module_list.c_str());

    config.module_list = module_list;

    boost::recursive_mutex::scoped_lock dyn_reconf_lock(dyn_reconfigure_mutex_);
    dyn_reconfigure_server_ptr_.get()->updateConfig(config);
    dyn_reconf_lock.unlock();
}

bool configure(ros::NodeHandle nh = ros::NodeHandle()) {
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

    //Iterate over all modules
    for (int i = 0; i < config.size(); ++i) {
        if (config[i].getType() != XmlRpc::XmlRpcValue::TypeStruct) {
            ROS_ERROR("Modules must be specified as maps, but they are XmlRpcType:%d", config[i].getType());
            valid = false;
        } else if (!config[i].hasMember("name") || config[i]["name"].getType() != XmlRpc::XmlRpcValue::TypeString) {
            ROS_ERROR("Could not add module %d because no valid name was given", i);
            valid = false;
        } else if (!config[i].hasMember("cmd_key") || config[i]["cmd_key"].getType() != XmlRpc::XmlRpcValue::TypeInt) {
            ROS_ERROR("Could not add because no valid cmd_key was given");
            valid = false;
        }

        for (int j = 0; j < active_modules.size(); j++) {
            if (std::string(config[i]["name"]) == active_modules[j]) {
                ROS_ERROR("Module names have to be unique! %s violates that.", active_modules[j].c_str());
                valid = false;
            }
        }

        if (!valid)
            break;


        if (config[i].hasMember("params")) {
            valid = addModule(std::string(config[i]["name"]), config[i]["params"]);
        } else {
            valid = addModule(std::string(config[i]["name"]));
        }

        if (!valid)
            break;
    }

    return valid;
}

//split string by turning it into a stream and reading lines

std::vector<std::string> split(std::string str, char delimiter) {
    std::vector<std::string> internal;
    std::stringstream ss(str);
    std::string tok;

    while (getline(ss, tok, delimiter)) {
        internal.push_back(tok);
    }

    return internal;
}

bool is_int(const std::string& s) {
    return !s.empty() && std::find_if(s.begin(),
            s.end(), [](char c) {
                return !std::isdigit(c); }) == s.end();
}

bool addModule(std::string name, XmlRpc::XmlRpcValue params) {

    //check for dubs
    for (int j = 0; j < active_modules.size(); j++) {
        if (active_modules[j] == name) {
            ROS_ERROR("Module names have to be unique! %s violates that.", active_modules[j].c_str());
            return false;
        }
    }
    
    /* TODO add cmdkey to list && check cmd key */
    boost::shared_ptr<MovementModule> mm(new MovementModule(name, params));

    mv_mutex.lock();
    mv.push_back(mm);
    mv_mutex.unlock();
    
    ROS_INFO("Created module %s", name.c_str());

    std::string service_name(name);
    service_name.append("/calculateVelocity");

    client_list.push_back(ros::NodeHandle("~").serviceClient<meka_guiding::Velocity>(service_name));
    active_modules.push_back(name);
    ROS_DEBUG_STREAM("connected to: " << client_list[client_list.size() - 1].getService());
    
    return true;
}

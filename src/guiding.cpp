#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/Twist.h"
#include "MovementModule.h"
#include <meka_guiding/ControllerConfig.h>
#include <c++/5/bits/stl_vector.h>

boost::mutex mv_mutex;
std::vector<boost::shared_ptr<MovementModule> > mv;
std::vector<std::string> active_modules;

boost::recursive_mutex dyn_reconfigure_mutex_;
boost::shared_ptr<dynamic_reconfigure::Server<meka_guiding::ControllerConfig> > dyn_reconfigure_server_ptr_;
dynamic_reconfigure::Server<meka_guiding::ControllerConfig>::CallbackType f_;

bool startup = true;

/**     function prototypes     **/
void parameterCallback(meka_guiding::ControllerConfig &config, uint32_t level);
bool configure(ros::NodeHandle nh);
void setConfig();
std::vector<std::string> split(std::string str, char delimiter);
bool is_int(const std::string& s);

//takes care of all the node specific stuff
int main(int argc, char **argv)
{
    //calibrate node with name
    ros::init(argc, argv, "meka_guiding");

    //create nodehandle
    ros::NodeHandle nh;
    
    //configure and load modules
    if(!configure(nh)){
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

    while(nh.ok()){
	rate.sleep();
        ros::spinOnce();
    }
    
    return 0;
}

void parameterCallback(meka_guiding::ControllerConfig &config, uint32_t level) {
    ROS_INFO_STREAM("ControllerReconfiguration");

    if(startup){
        startup = false;
        return;
    }

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
    
    if(removed) {
        setConfig();
        return;
    }

    /* TODO add cmdkey to list && check cmd key */
    /* TODO checks dubs */
    std::vector<std::string> new_module = split(config.add_module, ' ');
    if (static_cast<int> (new_module.size()) == 2 && is_int(new_module[1])) {
        
        for (int j = 0; j < active_modules.size(); j++) {
            if (active_modules[j] == new_module[0]) {
                ROS_ERROR("Module names have to be unique! %s violates that.", active_modules[j].c_str());
                setConfig();
                return;
            }
        }
        
        boost::shared_ptr<MovementModule> mm(new MovementModule(new_module[0]));

        mv_mutex.lock();
        mv.push_back(mm);
        mv_mutex.unlock();

        active_modules.push_back(new_module[0]);

        ROS_INFO("Created module %s", new_module[0].c_str());
    } else {
        ROS_ERROR("Controller: wrong arguments in add_module");
    }
}

void setConfig(){
    //make sure all values are set
    meka_guiding::ControllerConfig config;
    dyn_reconfigure_server_ptr_.get()->getConfigDefault(config);

    std::string module_list = "";
    
    for(int i = 0; i < active_modules.size(); i++){
        module_list.append(active_modules[i]).append(" ");
        ROS_WARN("%s", active_modules[i].c_str());
    }
    
    ROS_DEBUG("module list: %s", module_list.c_str());
    
    config.module_list = module_list;
    
    boost::recursive_mutex::scoped_lock dyn_reconf_lock(dyn_reconfigure_mutex_);
    dyn_reconfigure_server_ptr_.get()->updateConfig(config);
    dyn_reconf_lock.unlock();
}

bool configure(ros::NodeHandle nh = ros::NodeHandle()){
    /* https://github.com/ros/filters/blob/hydro-devel/include/filters/filter_chain.h */
    
    XmlRpc::XmlRpcValue config;

    if (nh.getParam("ModuleList", config) && config.size() > 0) {
        ROS_INFO("Found module list containing %d module(s)", config.size());
    } else {
        ROS_ERROR("No modules found to configure!");
        return false;
    }
    

    //verify proper naming and structure    
    if (config.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_ERROR("The module list specification must be a list. but is of of XmlRpcType %d", config.getType());
      ROS_ERROR("The xml passed in is formatted as follows:\n %s", config.toXml().c_str());
      return false;
    }

    //Iterate over all modules
    for (int i = 0; i < config.size(); ++i) {
        if (config[i].getType() != XmlRpc::XmlRpcValue::TypeStruct) {
            ROS_ERROR("Modules must be specified as maps, but they are XmlRpcType:%d", config[i].getType());
            return false;
        } else if (!config[i].hasMember("name") || config[i]["name"].getType() != XmlRpc::XmlRpcValue::TypeString) {
            ROS_ERROR("Could not add module %d because no valid name was given", i);
            return false;
        } else if (!config[i].hasMember("cmd_key") || config[i]["cmd_key"].getType() != XmlRpc::XmlRpcValue::TypeInt) {
            ROS_ERROR("Could not add because no valid cmd_key was given");
            return false;
        }

        for (int j = 0; j < active_modules.size(); j++) {
            if (std::string(config[i]["name"]) == active_modules[j]) {
                ROS_ERROR("Module names have to be unique! %s violates that.", active_modules[j].c_str());
                return false;
            }
        }

        /* TODO add cmdkey to list && check cmd key */
        boost::shared_ptr<MovementModule> mm;



        if (config[i].hasMember("params")) {
            mm.reset(new MovementModule(std::string(config[i]["name"]), config[i]["params"]));
        } else {
            mm.reset(new MovementModule(std::string(config[i]["name"])));
        }

        mv_mutex.lock();
        mv.push_back(mm);
        mv_mutex.unlock();

        active_modules.push_back(std::string(config[i]["name"]));

        ROS_DEBUG("Created module %s", std::string(config[i]["name"]).c_str());

    } 
    
    return true;
}

//split string by turning it into a stream and reading lines
std::vector<std::string> split(std::string str, char delimiter) {
  std::vector<std::string> internal;
  std::stringstream ss(str);
  std::string tok;
  
  while(getline(ss, tok, delimiter)) {
    internal.push_back(tok);
  }
  
  return internal;
}

bool is_int(const std::string& s) {
    return !s.empty() && std::find_if(s.begin(), 
        s.end(), [](char c) { return !std::isdigit(c); }) == s.end();
}

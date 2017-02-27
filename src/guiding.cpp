#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/Twist.h"
#include "MovementModule.h"
#include <meka_guiding/ControllerConfig.h>

boost::mutex mv_mutex;
std::vector<boost::shared_ptr<MovementModule> > mv;

/**     function prototypes     **/
void parameterCallback(meka_guiding::ControllerConfig &config, uint32_t level);
bool configure(ros::NodeHandle nh);

//takes care of all the node specific stuff
int main(int argc, char **argv)
{
    //calibrate node with name
    ros::init(argc, argv, "meka_guiding");

    //create nodehandle
    ros::NodeHandle nh;
    
    //setup dynamic_reconfigure
    dynamic_reconfigure::Server<meka_guiding::ControllerConfig> dyn_reconf_server_;
    dynamic_reconfigure::Server<meka_guiding::ControllerConfig>::CallbackType f_;

    f_ = boost::bind(&parameterCallback, _1, _2);
    dyn_reconf_server_.setCallback(f_);
    
    //configure and load modules
    if(!configure(nh)){
        ROS_FATAL("Configuration failed!");
        ros::shutdown();
    }

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
        } else {
            boost::shared_ptr<MovementModule> mm;
            
            if (config[i].hasMember("params")) {
                mm.reset(new MovementModule(std::string(config[i]["name"]), config[i]["params"]));
            } else {
                mm.reset(new MovementModule(std::string(config[i]["name"])));
            }

            mv_mutex.lock();
            mv.push_back(mm);
            mv_mutex.unlock();
            
            ROS_INFO("Created module %s", std::string(config[i]["name"]).c_str());
        }
    } 
    
    return true;
}
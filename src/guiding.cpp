#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/Twist.h"
#include "MovementModule.h"
#include <meka_guiding/ControllerConfig.h>

boost::mutex mv_mutex;
std::vector<boost::shared_ptr<MovementModule> > mv;

/**     function prototypes     **/
void readParams(ros::NodeHandle nh, std::string& topic_pub, std::string& topic_stiff, int& jointcount, double& stiffness);
void publishStiffness(ros::NodeHandle nh, ros::Rate rate, std::string& topic_stiff, int jointcount, double stiffness);
void parameterCallback(meka_guiding::ControllerConfig &config, uint32_t level);
bool configure(ros::NodeHandle nh);
void createModule(std::string name, std::string tf_src, std::string tf_dst, tf_key tf_key, dir_key dir, cmd_key cmd_key, float velocity_factor);

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
    if(!configure(nh))
        ros::shutdown();

    std::string topic_pub;
    std::string topic_stiff;
    int jointcount;
    double stiffness;

    //read ros params
    //readParams(nh, topic_pub, topic_stiff, jointcount, stiffness);

    //set frequency to 10Hz
    ros::Rate rate(10.0);

    //well ... publish stiffness!
    //publishStiffness(nh, rate, topic_stiff, jointcount, stiffness);
    
    //initialze publisher
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>(topic_pub, 1);

    while(nh.ok()){
	    rate.sleep();
        ros::spinOnce();
    }
    
    return 0;
}

//reads parameter relevant to the main method; otherwise uses standard values
void readParams(ros::NodeHandle nh, std::string& topic_pub, std::string& topic_stiff, int& jointcount, double& stiffness){

    if(!nh.getParam ("topic_pub", topic_pub)){
        topic_pub = "/cmd_vel";
    }

    if(!nh.getParam ("topic_stiff", topic_stiff)){
        topic_stiff = "/meka_roscontrol/stiffness_controller/command";
    }
    
    if(!nh.getParam ("jointcount", jointcount)){
        jointcount = 29; // fallback for meka's jointcount
    }
    
    if(!nh.getParam ("stiffness", stiffness)){
        stiffness = 0.3;  // default stiffness
    }

    ROS_INFO("Parameters done.");

}

void parameterCallback(meka_guiding::ControllerConfig &config, uint32_t level) {
    ROS_INFO_STREAM("ControllerReconfiguration");
}

void publishStiffness(ros::NodeHandle nh, ros::Rate rate, std::string& topic_stiff, int jointcount, double stiffness) {

    //setup publisher and message
    ros::Publisher stiff_pub = nh.advertise<std_msgs::Float64MultiArray>(topic_stiff, 1000);
    std_msgs::Float64MultiArray stiff_array;

    //fill with desired stiffness
    for (int i = 0; i < jointcount; i++) {
        stiff_array.data.push_back(stiffness);
    }

    //generate timeout
    ros::Time start_time = ros::Time::now();
    ros::Duration timeout(1.0);

    //wait until the publisher is initialized, but use a timeout to avoid locking if no subscriber is present
    while(stiff_pub.getNumSubscribers() == 0 && (ros::Time::now() - start_time < timeout)){
        ROS_INFO("publisher not yet ready");
        rate.sleep();       //sleep to avoid log bloating
    }

    //publish stiffnessarray
    ROS_INFO("Setting stiffness to %f for %d joints on topic %s", stiffness, jointcount, topic_stiff.c_str());
    stiff_pub.publish(stiff_array);
    ros::spinOnce();
}

bool configure(ros::NodeHandle nh = ros::NodeHandle()){
    /* https://github.com/ros/filters/blob/hydro-devel/include/filters/filter_chain.h */
    
    XmlRpc::XmlRpcValue config;

    if (nh.getParam("ModuleList", config)) {
        ROS_INFO("Found module list containing %d module(s)", config.size());
    } else {
        ROS_ERROR("No modules found to configure!");
        return false;
    }
    
    
    /*************************** Parse the XmlRpcValue ***********************************/
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
        } else if (!config[i].hasMember("name") ||config[i]["name"].getType() != XmlRpc::XmlRpcValue::TypeString) {
            ROS_ERROR("Could not add a module because no valid name was given");
            return false;
        } else if (!config[i].hasMember("params") ||config[i]["params"].getType() != XmlRpc::XmlRpcValue::TypeStruct) {
            ROS_ERROR("Could not add module %s because the parameter were in the wrong format or not given at all", std::string(config[i]["name"]).c_str());
            return false;
        } else if (false) /* add struct size */ {
            
            return false;
        } else {
            if(config[i]["params"]["source_frame"].getType() != XmlRpc::XmlRpcValue::TypeString){
                ROS_ERROR("Could not add module %s because the source_frame has to be a sting, but is a %d", std::string(config[i]["name"]).c_str(), config[i]["params"]["source_frame"].getType());
                return false;
            } else if(config[i]["params"]["target_frame"].getType() != XmlRpc::XmlRpcValue::TypeString){
                ROS_ERROR("Could not add module %s because the target_frame has to be a sting, but is a %d", std::string(config[i]["name"]).c_str(), config[i]["params"]["target_frame"].getType());
                return false;
            } else if(config[i]["params"]["transform_key"].getType() != XmlRpc::XmlRpcValue::TypeInt){
                ROS_ERROR("Could not add module %s because the transform_key has to be an int, but is a %d", std::string(config[i]["name"]).c_str(), config[i]["params"]["transform_key"].getType());
                return false;
            } else if(config[i]["params"]["direction_key"].getType() != XmlRpc::XmlRpcValue::TypeInt){
                ROS_ERROR("Could not add module %s because the direction_key has to be an int, but is a %d", std::string(config[i]["name"]).c_str(), config[i]["params"]["direction_key"].getType());
                return false;
            } else if(config[i]["params"]["cmd_vel_key"].getType() != XmlRpc::XmlRpcValue::TypeInt){
                ROS_ERROR("Could not add module %s because the cmd_vel_key has to be an int, but is a %d", std::string(config[i]["name"]).c_str(), config[i]["params"]["cmd_vel_key"].getType());
                return false;
            }  else if(config[i]["params"]["velocity_factor"].getType() != XmlRpc::XmlRpcValue::TypeDouble){
                ROS_ERROR("Could not add module %s because the velocity_factor has to be an double, but is a %d", std::string(config[i]["name"]).c_str(), config[i]["params"]["velocity_factor"].getType());
            
            } else {
                /* do the parsing stuff */
                createModule(std::string(config[i]["name"]),
                             std::string(config[i]["params"]["source_frame"]),
                             std::string(config[i]["params"]["target_frame"]),
                             tf_key(static_cast<int>(config[i]["params"]["transform_key"])),
                             dir_key(static_cast<int>(config[i]["params"]["direction_key"])),
                             cmd_key(static_cast<int>(config[i]["params"]["cmd_vel_key"])),
                             static_cast<double>(config[i]["params"]["velocity_factor"]));
            }
        }
    }
    
    
    return true;
}

void createModule(std::string name, std::string tf_src, std::string tf_dst, tf_key tf_key, dir_key dir_key, cmd_key cmd_key, float velocity_factor){
    boost::shared_ptr<MovementModule> mm(new MovementModule( name, tf_src, tf_dst, tf_key, dir_key, velocity_factor));
    
    mv_mutex.lock();
    mv.push_back(mm);
    mv_mutex.unlock();
    
    ROS_INFO("Successfully created module %s", name.c_str());
}
#include "MovementModule.h"

MovementModule::MovementModule(std::string name, cmd_key key, XmlRpc::XmlRpcValue params) : 
    name_(name), 
    cmd_key_(key) {
    ROS_DEBUG("creating module for cmd_key %d", static_cast<int>(key));
    
    //new name for private
    std::string nhname_ = std::string("~").append(name);
    
    //create dyn_reconf server with private node handle
    ros::NodeHandle nh(nhname_);
    dyn_reconfigure_server_ptr_.reset(new dynamic_reconfigure::Server<meka_guiding::ModuleConfig>(dyn_reconfigure_mutex_, nh));

    try {
        if(params.size() != 0 ){
            ROS_INFO("Module %s is overriding defaults", name.c_str());
            overrideDefaultParameter(params);
        } else {
            ROS_WARN("Module %s is using defaults", name.c_str());
        }
    }
    catch (XmlRpc::XmlRpcException exception) {
        ROS_WARN("Module %s is using defaults, XmlRpc says: %s", name.c_str(), exception.getMessage().c_str());
    }

    //create server
    f_ = boost::bind(&MovementModule::parameterCallback, this, _1, _2);
    dyn_reconfigure_server_ptr_.get()->setCallback(f_);
    
    //waitin, otherwise the first tf would always fail
    ROS_INFO("waiting for transform for .5s");
    listener_.waitForTransform(tf_src_, tf_dst_, ros::Time::now(), ros::Duration(0.5));
    
    //get initial position
    reference_position_ = getPositionByKey();

    service_ptr_.reset(new ros::ServiceServer(nh.advertiseService("calculateVelocity", &MovementModule::calcVelocity, this)));
}

void MovementModule::overrideDefaultParameter(XmlRpc::XmlRpcValue params){
    /* https://github.com/felix-kolbe/scitos_metralabs/blob/master/metralabs_ros/src/ScitosBase.cpp#L245 */
    
    //initially set config with module parameter 
    meka_guiding::ModuleConfig config;
    dyn_reconfigure_server_ptr_.get()->getConfigDefault(config);


    if (params.hasMember("tf_src") && params["tf_src"].getType() == XmlRpc::XmlRpcValue::TypeString) {
        config.tf_src = std::string(params["tf_src"]);
        ROS_INFO("Setting tf_src for module %s", name_.c_str());
    }
    if (params.hasMember("tf_dst") && params["tf_dst"].getType() == XmlRpc::XmlRpcValue::TypeString) {
        config.tf_dst = std::string(params["tf_dst"]);
        ROS_INFO("Setting tf_dst for module %s", name_.c_str());
    }
    if (params.hasMember("tf_key") && params["tf_key"].getType() == XmlRpc::XmlRpcValue::TypeInt) {
        config.tf_key = static_cast<int> (params["tf_key"]);
        ROS_INFO("Setting tf_key for module %s", name_.c_str());
    }
    if (params.hasMember("dir_key") && params["dir_key"].getType() == XmlRpc::XmlRpcValue::TypeInt) {
        config.dir_key = static_cast<int> (params["dir_key"]);
        ROS_INFO("Setting dir_key for module %s", name_.c_str());
    }
    if (params.hasMember("deadzone_factor") && params["deadzone_factor"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
        config.deadzone_factor = static_cast<double> (params["deadzone_factor"]);
        ROS_INFO("Setting deadzone_factor for module %s", name_.c_str());
    }
    if (params.hasMember("velocity_factor") && params["velocity_factor"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
        config.velocity_factor = static_cast<double> (params["velocity_factor"]);
        ROS_INFO("Setting velocity_factor for module %s", name_.c_str());
    }
    if (params.hasMember("velocity_upper") && params["velocity_upper"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
        config.velocity_upper = static_cast<double> (params["velocity_upper"]);
        ROS_INFO("Setting velocity_upper for module %s", name_.c_str());
    }

    boost::recursive_mutex::scoped_lock dyn_reconf_lock(dyn_reconfigure_mutex_);
    dyn_reconfigure_server_ptr_.get()->updateConfig(config);
    dyn_reconf_lock.unlock();
}

//calculates new velocities to set depending on the deflections of the input joint
bool MovementModule::calcVelocity(meka_guiding::Velocity::Request &request, meka_guiding::Velocity::Response &response){
    double dist, velocity, actual_position;

    if (!enable_toggle_) {
        response.name = name_;
        response.vel = 0;
        response.cmd_key = static_cast<int> (cmd_key_);
        response.priority_flag = 0;

        if (last_vel_ != 0 && velocity == 0) {
            response.finished_movement = true;
        } else {
            response.finished_movement = false;
        }
        last_vel_ = 0;

        return true;

    }
    
    //calculate distance depending ont he actual position
    actual_position = getPositionByKey(request.stamp);
    dist = std::fabs(reference_position_ - actual_position);

    //calculate velocity depending on dir key
    if(dir_key_ == dir_key::POSITIVE || dir_key_ == dir_key::BIDIRECTIONAL){
        if(reference_position_ < 0 && (actual_position > (reference_position_ * (1 - deadzone_factor_)))){
            velocity = dist * velocity_factor_;
        } else if(reference_position_ > 0 && (actual_position > (reference_position_ * (1 + deadzone_factor_)))){
            velocity = dist * velocity_factor_;
        } else if(reference_position_ == 0 && (actual_position > deadzone_factor_)) {
            velocity = dist * velocity_factor_;
        }
    } else if (dir_key_ == dir_key::NEGATIVE || dir_key_ == dir_key::BIDIRECTIONAL){
        if(reference_position_ < 0 && (actual_position < (reference_position_ * (1 + deadzone_factor_)))){
            velocity = -(dist * velocity_factor_);
        } else if(reference_position_ > 0 && (actual_position < (reference_position_ * (1 - deadzone_factor_)))){
            velocity = -(dist * velocity_factor_);
        } else if(reference_position_ == 0 && (actual_position < -(deadzone_factor_))) {
            velocity = -(dist * velocity_factor_);
        }
    } else {
        velocity = 0;
    }

    if(std::fabs(velocity) > velocity_upper_){
        velocity = 0;
    }

    ROS_DEBUG_STREAM( "Module" << name_ <<  "[" << reference_position_ << ", " << actual_position << ", " << velocity << "]");
    
    response.name = name_;
    response.vel = velocity;
    response.cmd_key = static_cast<int>(cmd_key_);
    
    if(last_vel_ == 0 && velocity != 0){
        response.priority_flag = true;
    } else {
        response.priority_flag = false;
    }
    
    if(last_vel_ != 0 && velocity == 0){
        response.finished_movement = true;
    } else {
        response.finished_movement = false;
    }
    
    last_vel_ = velocity;
    
    return true;
    
}

//gets transform and returns value depending on tf key of the module
double MovementModule::getPositionByKey(ros::Time time){
    ROS_DEBUG("%s trying to get position at %d seconds", name_.c_str(), time.sec);

    //get transform, on error return 0
    try{
        listener_.lookupTransform(tf_src_, tf_dst_, ros::Time(0), transform_);
        ROS_DEBUG("got transform!");
    }
    catch (tf::TransformException ex){
        ROS_ERROR("Module %s couldn't get transform: %s", name_.c_str(), ex.what());
        ros::Duration(1.0).sleep();
        return 0;
    }

    //switch translation as it can be returned directly
    switch(tf_key_){
        case tf_key::X_AXIS:
            return transform_.getOrigin().getX();
        case tf_key::Y_AXIS:
            return transform_.getOrigin().getY();
        case tf_key::Z_AXIS:
            return transform_.getOrigin().getZ();
    }
    
    //get RPY as these values are not directly returnable
    double roll, pitch, yaw;
    transform_.getBasis().getRPY(roll, pitch, yaw);

    //switch rotation
    switch(tf_key_){
        case tf_key::ROLL:
            return roll;
        case tf_key::PITCH:
            return pitch;
        case tf_key::YAW:
            return yaw;
    }
}

//callback for dyn_reconfigure
void MovementModule::parameterCallback(meka_guiding::ModuleConfig &config, uint32_t level) {
    ROS_INFO("ParameterCallback %s", name_.c_str());
    readConfig(config);
}

//getting all values from the dyn_reconfigure config
void MovementModule::readConfig(meka_guiding::ModuleConfig &config){
    tf_src_ = config.tf_src.c_str();
    tf_dst_ = config.tf_dst.c_str();

    tf_key_ = tf_key(config.tf_key);
    dir_key_ = dir_key(config.dir_key);

    velocity_upper_ = config.velocity_upper;
    velocity_factor_ = config.velocity_factor;
    deadzone_factor_ = config.deadzone_factor;
    
    enable_toggle_ = config.enable_toggle;
} 

bool MovementModule::matchTfKey(tf_key& key, std::string key_string) {
    bool found = false;

    for (auto& it_key : tf_string_map) {
        if (it_key.first == key_string) {
            ROS_DEBUG_STREAM("found key by name " << key_string);
            key = it_key.second;
            found = true;
            break;
        }
    }
    
    if(!found)
        ROS_ERROR("the given string %s does not map to a tf_key", key_string.c_str());

    return found;
}

bool MovementModule::matchTfKey(tf_key& key, int key_int) {
    if (key_int < 0 || key_int > static_cast<int> (tf_key::TF_KEY_MAX)) {
        ROS_ERROR("%d didn't match a tf_key", key_int);
        return false;
    } else {
        key = tf_key(key_int);
    }
    
    return true;
}

bool MovementModule::matchDirKey(dir_key& key, std::string key_string) {
    bool found = false;

    for (auto& it_key : dir_string_map) {
        if (it_key.first == key_string) {
            ROS_DEBUG_STREAM("found key by name " << key_string);
            key = it_key.second;
            found = true;
            break;
        }
    }
    
    if(!found)
        ROS_ERROR("the given string %s does not map to a dir_key", key_string.c_str());

    return found;
}

bool MovementModule::matchDirKey(dir_key& key, int key_int) {
    if (key_int < 0 || key_int > static_cast<int> (dir_key::DIR_KEY_MAX)) {
        ROS_ERROR("%d didn't match a dir_key", key_int);
        return false;
    } else {
        key = dir_key(key_int);
    }
    
    return true;
}

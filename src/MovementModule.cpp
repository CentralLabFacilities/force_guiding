#include "MovementModule.h"

MovementModule::MovementModule(std::string name, XmlRpc::XmlRpcValue params) {
    //new name for private
    name_ = std::string("~").append(name);
    
    //create dyn_reconf server with private node handle
    ros::NodeHandle private_nh(name_);
    dyn_reconfigure_server_ptr_.reset(new dynamic_reconfigure::Server<meka_guiding::ModuleConfig>(dyn_reconfigure_mutex_, private_nh));

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
    
    //get initial position
    reference_position_ = getPositionByKey();

    ros::ServiceServer service = private_nh.advertiseService(name, &MovementModule::calcVelocity, this);
    ROS_INFO("%s ready to calculate velocity", name.c_str());
    ros::spin();
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

    //respect upper velocity limit TODO set to last value to know when priority mode is engaged
    if(std::fabs(velocity) > velocity_upper_){
        velocity = 0;
    }

    ROS_DEBUG_STREAM( "Module" << name_ <<  "[" << reference_position_ << ", " << actual_position << ", " << velocity << "]");

    response.vel = velocity;
    return true;
    
}

//gets transform and returns value depending on tf key of the module
double MovementModule::getPositionByKey(ros::Time time){
    ROS_DEBUG("%s trying to get position at %d seconds", name_.c_str(), time.sec);

    //get transform, on error return 0
    try{
        listener.lookupTransform(tf_src_, tf_dst_, ros::Time(0), transform);
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
            return transform.getOrigin().getX();
        case tf_key::Y_AXIS:
            return transform.getOrigin().getY();
        case tf_key::Z_AXIS:
            return transform.getOrigin().getZ();
    }
    
    //get RPY as these values are not directly returnable
    double roll, pitch, yaw;
    transform.getBasis().getRPY(roll, pitch, yaw);

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
} 
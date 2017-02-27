#include "MovementModule.h"

MovementModule::MovementModule(std::string name, std::string tf_src, std::string tf_dst, tf_key tf_key, dir_key dir_key, float velocity_factor){
    ROS_DEBUG_STREAM("creating MovementModule " << name);

    name_ = std::string("~").append(name);
    
    this->tf_src_ = tf_src;
    this->tf_dst_ = tf_dst;
    this->tf_key_ = tf_key;
    this->dir_key_ = dir_key;
    this->velocity_factor_ = velocity_factor;
    
    initializeDynamicReconfigure();

    reference_position_ = getPositionByKey();
}

void MovementModule::initializeDynamicReconfigure(){
    
    /* https://github.com/felix-kolbe/scitos_metralabs/blob/master/metralabs_ros/src/ScitosBase.cpp#L245 */
    
    //create dyn_reconf server with private node handle
    ros::NodeHandle private_nh(name_);
    dyn_reconfigure_server_ptr_.reset(new dynamic_reconfigure::Server<meka_guiding::ModuleConfig>(dyn_reconfigure_mutex_, private_nh));
    
    //initially set config with module parameter 
    meka_guiding::ModuleConfig config;
    dyn_reconfigure_server_ptr_.get()->getConfigDefault(config);
    
    config.tf_src = this->tf_src_;
    config.tf_dst = this->tf_dst_;
    config.tf_key = static_cast<int >(this->tf_key_);
    config.dir_key = static_cast<int>(this->dir_key_);
    config.velocity_factor = this->velocity_factor_;
    
    boost::recursive_mutex::scoped_lock dyn_reconf_lock(dyn_reconfigure_mutex_);
    dyn_reconfigure_server_ptr_.get()->updateConfig(config);
    dyn_reconf_lock.unlock();
    
    //create server
    f_ = boost::bind(&MovementModule::parameterCallback, this, _1, _2);
    dyn_reconfigure_server_ptr_.get()->setCallback(f_);
}

//calculates new velocities to set depending on the deflections of the input joint
double MovementModule::calcVelocity(){
    double dist, velocity, actual_position;
    
    actual_position = getPositionByKey();
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

    //respect upper velocity limit TODO set to last value
    if(std::fabs(velocity) > velocity_upper_){
        velocity = 0;
    }

    ROS_DEBUG_STREAM( "Module" << name_ <<  "[" << reference_position_ << ", " << actual_position << ", " << velocity << "]");

    return velocity;
    
}

//gets transform and returns value depending on tf key of the module
double MovementModule::getPositionByKey(){

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
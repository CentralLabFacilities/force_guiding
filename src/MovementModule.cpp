#include "MovementModule.h"

MovementModule::MovementModule(std::string name, cmd_key key, XmlRpc::XmlRpcValue params) :
    name_(name),
    cmd_key_(key) {
    ROS_DEBUG("creating module for cmd_key %d", static_cast<int>(key));

    //new name for private
    std::string nhname_ = std::string("~").append(name);

    //create dyn_reconf server with private node handle
    ros::NodeHandle nh(nhname_);
    dyn_reconfigure_server_ptr_.reset(new dynamic_reconfigure::Server<force_guiding::ModuleConfig>(dyn_reconfigure_mutex_, nh));

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

    //waitin, otherwise the first tf would always fail
    listener_ptr_.reset(new tf::TransformListener);
    ROS_INFO("%s waiting for transform for 1s", name.c_str());
    listener_ptr_.get()->waitForTransform(source_frame_, target_frame_, ros::Time::now(), ros::Duration(1.0));
    
    //create server
    f_ = boost::bind(&MovementModule::parameterCallback, this, _1, _2);
    dyn_reconfigure_server_ptr_.get()->setCallback(f_);

    

    //get initial position
    reference_position_ = getPositionByKey();

    service_ptr_.reset(new ros::ServiceServer(nh.advertiseService("calculateVelocity", &MovementModule::calcVelocity, this)));
}

void MovementModule::overrideDefaultParameter(XmlRpc::XmlRpcValue params){
    /* https://github.com/felix-kolbe/scitos_metralabs/blob/master/metralabs_ros/src/ScitosBase.cpp#L245 */

    //initially set config with module parameter
    force_guiding::ModuleConfig config;
    dyn_reconfigure_server_ptr_.get()->getConfigDefault(config);

    config.base_dof = static_cast<int>(cmd_key_);


    if (params.hasMember("source_frame") && params["source_frame"].getType() == XmlRpc::XmlRpcValue::TypeString) {
        config.source_frame = std::string(params["source_frame"]);
        ROS_INFO("Setting source_frame %s for module %s", config.source_frame.c_str(), name_.c_str());
    }
    if (params.hasMember("target_frame") && params["target_frame"].getType() == XmlRpc::XmlRpcValue::TypeString) {
        config.target_frame = std::string(params["target_frame"]);
        ROS_INFO("Setting target_frame %s for module %s", config.target_frame.c_str(), name_.c_str());
    }
    if (params.hasMember("tf_key")) {

        tf_key key;

        if(params["tf_key"].getType() == XmlRpc::XmlRpcValue::TypeInt){
            if(matchTfKey(key, int(params["tf_key"])))
                tf_key_ = key;
        } else if(params["tf_key"].getType() == XmlRpc::XmlRpcValue::TypeString){
            if(matchTfKey(key, std::string(params["tf_key"])))
                tf_key_ = key;
        } else {
            ROS_ERROR("%s: tf_key has wrong type, ignoring", name_.c_str());
        }

        config.transform_dof = static_cast<int>(tf_key_);

        ROS_INFO("Setting transform_dof %d for module %s", config.transform_dof, name_.c_str());
    }
    if (params.hasMember("dir_key")) {


        dir_key key;

        if(params["dir_key"].getType() == XmlRpc::XmlRpcValue::TypeInt){
            if(matchDirKey(key, int(params["dir_key"])))
                dir_key_ = key;
        } else if(params["dir_key"].getType() == XmlRpc::XmlRpcValue::TypeString){
            if(matchDirKey(key, std::string(params["dir_key"])))
                dir_key_ = key;
        } else {
            ROS_ERROR("%s: dir_key has wrong type, ignoring", name_.c_str());
        }

        config.direction = static_cast<int>(dir_key_);

        ROS_INFO("Setting direction %d for module %s", config.direction, name_.c_str());
    }
    if (params.hasMember("deadzone_factor") && params["deadzone_factor"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
        config.deadzone_factor = static_cast<double> (params["deadzone_factor"]);
        ROS_INFO("Setting deadzone_factor %f for module %s", config.deadzone_factor, name_.c_str());
    }
    if (params.hasMember("velocity_factor") && params["velocity_factor"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
        config.velocity_factor = static_cast<double> (params["velocity_factor"]);
        ROS_INFO("Setting velocity_factor %f for module %s", config.velocity_factor, name_.c_str());
    }
    if (params.hasMember("max_velocity") && params["max_velocity"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
        config.max_velocity = static_cast<double> (params["max_velocity"]);
        ROS_INFO("Setting max_velocity %f for module %s", config.max_velocity, name_.c_str());
    }
    if (params.hasMember("enabled") && params["enabled"].getType() == XmlRpc::XmlRpcValue::TypeBoolean) {
        config.enabled = static_cast<bool> (params["enabled"]);
        ROS_INFO("Setting enabled flag for module %s", name_.c_str());
    }

    boost::recursive_mutex::scoped_lock dyn_reconf_lock(dyn_reconfigure_mutex_);
    dyn_reconfigure_server_ptr_.get()->updateConfig(config);
    dyn_reconf_lock.unlock();
}

//calculates new velocities to set depending on the deflections of the input joint
bool MovementModule::calcVelocity(force_guiding::Velocity::Request &request, force_guiding::Velocity::Response &response){
    double dist, velocity, actual_position;

    if (!enable_toggle_) {
        response.name = name_;
        response.vel = 0;
        response.base_dof = static_cast<int> (cmd_key_);
        response.priority_flag = 0;

        if (last_vel_ != 0 && velocity == 0) {
            response.finished_movement = true;
        } else {
            response.finished_movement = false;
        }
        last_vel_ = 0;

        return true;

    }  else { //set default velocity
        velocity = 0;
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
    }

    if (dir_key_ == dir_key::NEGATIVE || dir_key_ == dir_key::BIDIRECTIONAL){
        if(reference_position_ < 0 && (actual_position < (reference_position_ * (1 + deadzone_factor_)))){
            velocity = -(dist * velocity_factor_);
        } else if(reference_position_ > 0 && (actual_position < (reference_position_ * (1 - deadzone_factor_)))){
            velocity = -(dist * velocity_factor_);
        } else if(reference_position_ == 0 && (actual_position < -(deadzone_factor_))) {
            velocity = -(dist * velocity_factor_);
        }
    }

    if(std::fabs(velocity) > max_velocity_){
        velocity = max_velocity_;
    }

    ROS_DEBUG("Module(%d) %s: [%f, %f, %f]", static_cast<int>(dir_key_), name_.c_str(), reference_position_, actual_position, velocity);

    response.name = name_;
    response.vel = velocity;
    response.base_dof = static_cast<int>(cmd_key_);

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
    ROS_DEBUG("%s trying to get position by key %d at %d seconds", name_.c_str(), static_cast<int>(tf_key_), time.sec);

    //get transform, on error return 0
    try{
        listener_ptr_.get()->lookupTransform(source_frame_, target_frame_, ros::Time(0), transform_);
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
void MovementModule::parameterCallback(force_guiding::ModuleConfig &config, uint32_t level) {
    ROS_INFO("ParameterCallback %s", name_.c_str());
    readConfig(config);
}

//getting all values from the dyn_reconfigure config
void MovementModule::readConfig(force_guiding::ModuleConfig &config){

    if(source_frame_ != config.source_frame){
        ROS_INFO("%s setting new source_frame %s", name_.c_str(), config.source_frame.c_str());
        source_frame_ = config.source_frame.c_str();
        reference_position_ = getPositionByKey();
    }

    if(target_frame_ != config.target_frame){
        ROS_INFO("%s setting new target_frame %s", name_.c_str(), config.target_frame.c_str());
        target_frame_ = config.target_frame.c_str();
        reference_position_ = getPositionByKey();
    }

    if(tf_key_ != tf_key(config.transform_dof)){
        ROS_INFO("%s setting new transform_dof %d", name_.c_str(), config.transform_dof);
        tf_key_ = tf_key(config.transform_dof);
        reference_position_ = getPositionByKey();
    }

    cmd_key_ = cmd_key(config.base_dof);

    dir_key_ = dir_key(config.direction);

    max_velocity_ = config.max_velocity;
    velocity_factor_ = config.velocity_factor;
    deadzone_factor_ = config.deadzone_factor;

    enable_toggle_ = config.enabled;
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

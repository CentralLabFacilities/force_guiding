#include "MovementModule.h"

MovementModule::MovementModule(std::string name, std::string tf_src, std::string tf_dst, tf_key tf_key, dir_key dir_key, float velocity_factor){
    ROS_DEBUG_STREAM("creating MovementModule " << name);

    nhname_ = nameprefix_.append(name);
    
    this->tf_src_ = tf_src;
    this->tf_dst_ = tf_dst;
    this->tf_key_ = tf_key;
    this->dir_key_ = dir_key;
    this->velocity_factor_ = velocity_factor;
    
    initializeDynamicReconfigure();
    
    //calibrate();
}

void MovementModule::initializeDynamicReconfigure(){
    
    /* https://github.com/felix-kolbe/scitos_metralabs/blob/master/metralabs_ros/src/ScitosBase.cpp#L245 */
    
    //create dyn_reconf server with private node handle
    ros::NodeHandle private_nh(nhname_);
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
    
    //creating server
    
    f_ = boost::bind(&MovementModule::parameterCallback, this, _1, _2);
    dyn_reconfigure_server_ptr_.get()->setCallback(f_);
}

//calculates new velocities to set depending on the deflections of the input joint
void MovementModule::calcVelocity(){
    /*
    //deflection using the distance of the vectorelements
    double x_dist = new_translation.getX() - initial_translation.getX();
    double y_dist = new_translation.getY() - initial_translation.getY();

    //for precise determination of the deadzone
    double dist = initial_translation.distance(new_translation);

    //calculation of x_vel
    if((new_translation.getX() > (initial_translation.getX() * (1 + DEADLOCK_SIZE))) ||
       (new_translation.getX() < (initial_translation.getX() * (1 - DEADLOCK_SIZE)))){

        x_vel = x_dist * VELOCITY_FACTOR;
    } else {
        x_vel = 0.0;
    }

    //calculation of y_vel
    if((new_translation.getY() > (initial_translation.getY() * (1 + DEADLOCK_SIZE))) ||
       (new_translation.getY() < (initial_translation.getY() * (1 - DEADLOCK_SIZE)))){

        y_vel = y_dist * VELOCITY_FACTOR;
    } else {
        y_vel = 0.0;
    }

    ROS_DEBUG_STREAM( "VelocityX (dist, vel):[" << x_dist << ", " << x_vel << "]");
    ROS_DEBUG_STREAM( "VelocityY (dist, vel):[" << y_dist << ", " << y_vel << "]");
    ROS_DEBUG_STREAM( "Translation{initial}: [" << initial_translation.getX() << ", " << initial_translation.getY() << ", " << initial_translation.getZ() << "]");
    ROS_DEBUG_STREAM( "Translation{actual}:  [" << new_translation.getX() << ", " << new_translation.getY() << ", " << new_translation.getZ() << "]"); */

}

//calibration ... what else?
void MovementModule::calibrate(){

    //give calibration additional tries to avoid first tf lookup error
    for(int i = 1; i <= MAX_CALIBRATION_TRIES; i++){
        ROS_INFO("calibration try %i", i);
        if(lookupInitialTransform()){
            ROS_INFO("successfully calibrated");
            break;
        } else if( i == MAX_CALIBRATION_TRIES){
            ROS_FATAL("failed to lookupInitialTransform after %i tries", MAX_CALIBRATION_TRIES);
            ros::shutdown();
        }
    }
    
}

//sets initial position
bool MovementModule::lookupInitialTransform() {

    ROS_DEBUG("calibrating...");

    try{
        listener.lookupTransform(tf_src_, tf_dst_, ros::Time(0), transform);
        ROS_DEBUG("got transform!");
    }
    catch (tf::TransformException ex){
        ROS_ERROR("error while calibrating: %s", ex.what());
        ros::Duration(1.0).sleep();
        return false;
    }

    initial_translation = transform.getOrigin();

    return true;
}

void MovementModule::parameterCallback(meka_guiding::ModuleConfig &config, uint32_t level) {
    ROS_INFO_STREAM("ModuleCallback " << nhname_);

    tf_src_ = config.tf_src.c_str();
    tf_dst_ = config.tf_dst.c_str();

    /*ROS_INFO(
            "\n#######ParameterCallback\nLINEAR_VELOCITY_UPPER:     %f \nANGULAR_VELOCITY_UPPER:    %f \nVELOCITY_FACTOR:           %f \nDEADLOCK_SIZE:             %f \nSOURCE_FRAME:              %s \nTARGET_FRAME:              %s\n#######",
            LINEAR_VELOCITY_UPPER, ANGULAR_VELOCITY_UPPER, VELOCITY_FACTOR, DEADLOCK_SIZE, tf_src.c_str(),
            tf_dst.c_str());
*/
}
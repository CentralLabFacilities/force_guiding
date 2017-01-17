#include "Helper.h"

//gets transform and sets the new position
geometry_msgs::Twist Helper::controlJoint() {
    geometry_msgs::Twist twist;

    try{
        listener.lookupTransform(tf_src, tf_dst, ros::Time(0), transform);
        ROS_DEBUG("Got Transform!");
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    new_translation = transform.getOrigin();

    calcVelocity();

    twist.linear.x = x_vel;
    twist.linear.y = y_vel;

    return twist;
}

//calculates new velocities to set depending on the deflections of the input joint
void Helper::calcVelocity(){

    // deflection using the distance of the vectors
    double dist = initial_translation.distance(new_translation);

    // check for "negative defelction" for calculating negative velocites
    if(initial_translation.getX() > new_translation.getX()){
        dist = (-1) * VELOCITY_FATOR * dist;
    } else {
        dist = VELOCITY_FATOR * dist;
    }

    //TODO actually calculate velocity

    ROS_DEBUG_STREAM( "Translation{initial}: [" << initial_translation.getX() << ", " << initial_translation.getY() << ", " << initial_translation.getZ() << "]");
    ROS_DEBUG_STREAM( "Translation{actual}:  [" << new_translation.getX() << ", " << new_translation.getY() << ", " << new_translation.getZ() << "]");

}

//calibration ... what else?
void Helper::calibrate(){

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
bool Helper::lookupInitialTransform() {
    ROS_DEBUG("calibrating...");

    try{
        listener.lookupTransform(tf_src, tf_dst, ros::Time(0), transform);
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

void Helper::setup(std::string tf_src, std::string tf_dst) {
    this->tf_src = tf_src;
    this->tf_dst = tf_dst;
    x_vel = 0.0;
    y_vel = 0.0;
}

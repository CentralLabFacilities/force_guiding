#include "Helper.h"

Helper::Helper(ros::NodeHandle nh) {

    //read classrelevant parameter
    readParams(nh);

    //waitin, otherwise the first tf would always fail
    ROS_INFO("waiting for transform for .5s");
    listener.waitForTransform(tf_src, tf_dst, ros::Time::now(), ros::Duration(0.5));

    //initialize velocity variables
    x_vel = 0.0;
    y_vel = 0.0;

    calibrate();

}

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

//checks is parameter were given; otherwise uses default values
void Helper::readParams(ros::NodeHandle nh){

    if(!nh.getParam ("tf_src", tf_src)){
        tf_src = "base_link";
    }

    if(!nh.getParam ("tf_dst", tf_dst)){
        tf_dst = "wrist_LEFT";
    }

}

//calibration ... what else?
void Helper::calibrate(){

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
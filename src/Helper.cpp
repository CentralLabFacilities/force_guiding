#include "Helper.h"

//prep-work
void Helper::init(ros::NodeHandle nh){

    readParams(nh);

    ROS_INFO("waiting for transform");
    listener.waitForTransform(tf_src, tf_dst, ros::Time::now(), ros::Duration(0.5));

    for(int i = 1; i <= MAX_CALIBRATION_TRIES; i++){
        ROS_INFO("calibration try %i", i);
        if(calibrate()){
            ROS_INFO("successfully calibrated");
            break;
        } else if( i == MAX_CALIBRATION_TRIES){
            ROS_FATAL("failed to calibrate after %i tries", MAX_CALIBRATION_TRIES);
            ros::shutdown();
        }
    }

    pub = nh.advertise<geometry_msgs::Twist>(topic_pub, 500);
}

//gets transform and sets the new position
void Helper::controlJoint() {

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

    //TODO create message here

    //pub.publish(msg_twist);
}

//calculates new position to set depending on the deflections of the input joints
void Helper::calcVelocity(){

    // deflection using the distance of the vectors
    double dist = initial_translation.distance(new_translation);

    // check for "negative defelction" to be able to decrease the z-Position
    if(initial_translation.getX() > new_translation.getX()){
        dist = (-1) * VELOCITY_FATOR * dist;
    } else {
        dist = VELOCITY_FATOR * dist;
    }

    //TODO actually calculate velocity

    ROS_DEBUG_STREAM( "Translation{initial}: [" << initial_translation.getX() << ", " << initial_translation.getY() << ", " << initial_translation.getZ() << "]");
    ROS_DEBUG_STREAM( "Translation{actual}:  [" << new_translation.getX() << ", " << new_translation.getY() << ", " << new_translation.getZ() << "]");

}

//sets initial position
bool Helper::calibrate() {
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

//checks is parameter were given; otherwise uses default values
void Helper::readParams(ros::NodeHandle nh){

    if(!nh.getParam ("tf_src", tf_src)){
         tf_src = "base_link";
    }

    if(!nh.getParam ("tf_dst", tf_dst)){
        tf_dst = "wrist_LEFT";
    }

    if(!nh.getParam ("topic_pub", topic_pub)){
        topic_pub = "/cmd_vel";
    }

}
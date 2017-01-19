#include "ZliftController.h"

//prep-work
void ZliftController::init(ros::NodeHandle nh){

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

    pub = nh.advertise<trajectory_msgs::JointTrajectory>(topic_pub, 500);
}

//gets transform and sets the new position
void ZliftController::controlJoint() {

    trajectory_msgs::JointTrajectoryPoint msg_point;
    trajectory_msgs::JointTrajectory msg_tra;

    try{
        listener.lookupTransform(tf_src, tf_dst, ros::Time(0), transform);
        ROS_DEBUG("Got Transform!");
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    new_translation = transform.getOrigin();

    calcPos();

    msg_point.positions.push_back(new_zpos);
    msg_point.time_from_start.nsec = 25000000; // time to complete -> 40Hz
    msg_tra.points.push_back(msg_point);
    msg_tra.joint_names.push_back(controlled_joint);
    //msg_tra.header.stamp = ros::Time::now();

    pub.publish(msg_tra);
}

//calculates new position to set depending on the deflections of the input joints
void ZliftController::calcPos(){
    double actual_zpos_;

    // deflection along the x-axis
    //double dist = -(initial_translation.getX() - new_translation.getX());

    // deflection using the distance of the vectors
    double dist = initial_translation.distance(new_translation);

    // check for "negative defelction" to be able to decrease the z-Position
    if(initial_translation.getX() > new_translation.getX()){
        dist = (-1) * DISTANCE_FACTOR * dist;
    } else {
        dist = DISTANCE_FACTOR * dist;
    }

    /** check old_pos method and document it for writing  NOT USEFUL **/
    // deflection using the distance of the vectors
    //double dist = old_pos.distance(new_translation);

    // check for "negative defelction" to be able to decrease the z-Position
    //if(old_pos.getX() > new_translation.getX()){
    //    dist = -dist;
    //}


    //get actual zlift position
    actual_zpos_mutex.lock();
    actual_zpos_ = actual_zpos;
    actual_zpos_mutex.unlock();

    //calculation of position
    if(new_translation.getX() > (initial_translation.getX() * (1 + DEADLOCK_SIZE))){
        if((actual_zpos_ + dist) < ZLIFT_MAX) {
            new_zpos = actual_zpos_ + dist;
        } else {
            new_zpos = ZLIFT_MAX;
        }
    } else if(new_translation.getX() < (initial_translation.getX() * (1 - DEADLOCK_SIZE))){
        if((actual_zpos_ + dist) > ZLIFT_MIN){
            new_zpos = actual_zpos_ + dist;
        } else {
            new_zpos = ZLIFT_MIN;
        }
    } else {
        new_zpos = actual_zpos_; //could lead to suboptimal behavior -> dont publish / old position
    }

    //new_zpos = (double)((int) (new_zpos * 100)) / 100;

    ROS_DEBUG_STREAM( "Translation{initial}: [" << initial_translation.getX() << ", " << initial_translation.getY() << ", " << initial_translation.getZ() << "]");
    ROS_DEBUG_STREAM( "Translation{actual}:  [" << new_translation.getX() << ", " << new_translation.getY() << ", " << new_translation.getZ() << "]");

    ROS_DEBUG("ActualZ: %f; NewZ: %f; DistanceStep: %f", actual_zpos_, new_zpos, dist);
}

//sets initial position
bool ZliftController::calibrate() {
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
void ZliftController::readParams(ros::NodeHandle nh){

    if(!nh.getParam ("tf_src", tf_src)){
         tf_src = "base_link";
    }

    if(!nh.getParam ("tf_dst", tf_dst)){
        tf_dst = "wrist_LEFT";
    }

    if(!nh.getParam ("topic_sub", topic_sub)){
        topic_sub = "/meka_roscontrol/zlift_position_trajectory_controller/state";
    }

    if(!nh.getParam ("topic_pub", topic_pub)){
        topic_pub = "/meka_roscontrol/zlift_position_trajectory_controller/command";
    }

    if(!nh.getParam ("controlled_joint", controlled_joint)){
        controlled_joint = "zlift_j0";
    }
}

//getter for topic_sub
std::string ZliftController::getTopicSub() {
    return topic_sub;
}

//setter for actual z position
void ZliftController::setActZ(double act_z) {
    actual_zpos_mutex.lock();
    this->actual_zpos = act_z;
    actual_zpos_mutex.unlock();
}

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

    pub = nh.advertise<trajectory_msgs::JointTrajectory>(topic_pub, 500);
}

//gets transform and sets the new position
void Helper::controlJoint() {

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

    new_pos = transform.getOrigin();

    calcNewPos();

    msg_point.positions.push_back(new_z);
    msg_point.time_from_start.sec = 1; //necessary for this message, otherwise it would be rejected
    msg_tra.points.push_back(msg_point);
    msg_tra.joint_names.push_back(controlled_joint);
    //msg_tra.header.stamp = ros::Time::now();

    pub.publish(msg_tra);

    old_pos = new_pos;
}

//calculates new position to set depending on the deflections of the input joints
void Helper::calcNewPos(){
    double act_z_;

    // deflection along the x-axis
    //double dist = -(init_pos.getX() - new_pos.getX());

    // deflection using the distance of the vectors
    double dist = init_pos.distance(new_pos);

    // check for "negative defelction" to be able to decrease the z-Position
    if(init_pos.getX() > new_pos.getX()){
        dist = -dist;
    }

    /** check old_pos method and document it for writing  NOT USEFUL **/
    // deflection using the distance of the vectors
    //double dist = old_pos.distance(new_pos);

    // check for "negative defelction" to be able to decrease the z-Position
    //if(old_pos.getX() > new_pos.getX()){
    //    dist = -dist;
    //}


    //ROS_DEBUG("locking...");
    act_z_mutex.lock();
    act_z_ = act_z;
    act_z_mutex.unlock();
    //ROS_DEBUG("unlocked...");

    //calculation of position
    if(new_pos.getX() > (init_pos.getX() * (1 + DEADLOCK_SIZE))){
        if((act_z_ + dist) < ZLIFT_MAX) {
            new_z = act_z_ + dist;
        } else {
            new_z = ZLIFT_MAX;
        }
    } else if(new_pos.getX() < (init_pos.getX() * (1 - DEADLOCK_SIZE))){
        if((act_z_ + dist) > ZLIFT_MIN){
            new_z = act_z_ + dist;
        } else {
            new_z = ZLIFT_MIN;
        }
    } else {
        new_z = act_z_; //could lead to suboptimal behavior -> buffer latest new_z-value
    }

    new_z = (double)((int) (new_z * 100)) / 100;

    ROS_DEBUG_STREAM( "Translation{initial}: [" << init_pos.getX() << ", " << init_pos.getY() << ", " << init_pos.getZ() << "]");
    ROS_DEBUG_STREAM( "Translation{actual}:  [" << new_pos.getX() << ", " << new_pos.getY() << ", " << new_pos.getZ() << "]");

    ROS_DEBUG("ActualZ: %f; NewZ: %f; StepDistance: %f", act_z_, new_z, dist);
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

    init_pos = transform.getOrigin();
    old_pos = init_pos;

    return true;
}

//only looks for given parameters, otherwise uses hardcoded default values for sim
void Helper::readParams(ros::NodeHandle nh){

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

    if(!nh.getParam ("sim", sim)){
        sim = false;
    }
}

//getter for topic_sub
std::string Helper::getTopicSub() {
    return topic_sub;
}

//setter for actual z position
void Helper::setActZ(double act_z) {
    //ROS_DEBUG("LOCKING:::");
    act_z_mutex.lock();
    this->act_z = act_z;
    act_z_mutex.unlock();
    //ROS_DEBUG("UNLOCKED:::");
}

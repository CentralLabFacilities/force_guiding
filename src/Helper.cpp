#include "Helper.h"

//prep-work
void Helper::init(ros::NodeHandle nh){

    readParams(nh);
    calibrate();

    pub = nh.advertise<trajectory_msgs::JointTrajectory>(topic_pub, 500);
}

//gets transform and sets the new position
void Helper::controlJoint() {
    ROS_INFO("controlJoint START");

    trajectory_msgs::JointTrajectoryPoint msg_point;
    trajectory_msgs::JointTrajectory msg_tra;

    try{
        listener.lookupTransform(tf_src, tf_dst, ros::Time(0), transform);
        ROS_INFO("got transform!");
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    new_pos = transform.getOrigin();

    calcNewPos();

    msg_point.positions.push_back(new_z);
    msg_tra.points.push_back(msg_point);
    msg_tra.joint_names.push_back(controlled_joint);

    ROS_INFO("publishing...");
    pub.publish(msg_tra);

    old_pos = new_pos;
    ROS_INFO("controlJoint END");
}

//calculates new position to set depending on the deflections of the input joints
void Helper::calcNewPos(){
    double act_z_;
    double dist = old_pos.distance(new_pos);

    ROS_INFO("locking...");
    act_z_mutex.lock();
    act_z_ = act_z;
    act_z_mutex.unlock();
    ROS_INFO("unlocked...");

    if(new_pos.getX() > (init_pos.getX() * 1.05) && (act_z_ + dist) < zlift_max){
        new_z = act_z_ + dist;
    } else if(new_pos.getX() < (init_pos.getX() * 0.95) && (act_z_ - dist) > zlift_min){
        new_z = act_z_ - dist;
    }

    ROS_INFO("new z: %f; act z: %f", new_z, act_z_);
}

//sets initial position
void Helper::calibrate() {
    ROS_INFO("calibrating...");

    try{
        listener.lookupTransform(tf_src, tf_dst, ros::Time(0), transform);
        ROS_INFO("got transform!");
    }
    catch (tf::TransformException ex){
        ROS_ERROR("error while calibrating: %s", ex.what());
        ros::Duration(1.0).sleep();
    }

    init_pos = transform.getOrigin();
    old_pos = init_pos;
}

//only looks for given parameters, otherwise uses hardcoded default values for sim
void Helper::readParams(ros::NodeHandle nh){

    if(!nh.getParam ("tf_src", tf_src)){
         tf_src = "base_link";
    }

    if(!nh.getParam ("tf_dst", tf_dst)){
        tf_dst = "wrist_RIGHT";
    }

    if(!nh.getParam ("topic_sub", topic_sub)){
        topic_sub = "/meka_roscontrol/zlift_position_trajectory_controller/state";
    }

    if(!nh.getParam ("topic_pub", topic_pub)){
        topic_pub = "/meka_roscontrol/zlift_position_trajectory_controller/command";
    }

    if(!nh.getParam ("input_joint", input_joint)){
        input_joint = "right_arm_j0";
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
    ROS_INFO("LOCKING:::");
    act_z_mutex.lock();
    this->act_z = act_z;
    act_z_mutex.unlock();
    ROS_INFO("UNLOCKED:::");
}
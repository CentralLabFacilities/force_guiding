#ifndef MEKA_GUIDING_HELPER_H
#define MEKA_GUIDING_HELPER_H


#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "trajectory_msgs/JointTrajectory.h"
#include <mutex>

class Helper {

public:
    /**     functions  **/
    void init(ros::NodeHandle nh);
    void setActZ(double act_z);
    void controlJoint();
    std::string getTopicSub();

private:

    /**     constants   **/
    const double ZLIFT_MAX = 0.55;
    const double ZLIFT_MIN = 0.20;
    const double DEADLOCK_SIZE = 0.1;
    const double DISTANCE_FACTOR = 0.5;
    const double VELOCITY_FATOR = 1.0;
    const int MAX_CALIBRATION_TRIES = 5;

    /**     variables   **/
    ros::Publisher pub;
    tf::TransformListener listener;
    tf::StampedTransform transform;
    std::mutex actual_zpos_mutex;
    tf::Vector3 initial_translation, new_translation;
    std::string topic_pub, topic_sub, controlled_joint, tf_src, tf_dst;
    double actual_zpos, new_zpos;


    /**     functions   **/
    void readParams(ros::NodeHandle nh);
    void calcPos();
    bool calibrate();

};


#endif //MEKA_GUIDING_HELPER_H

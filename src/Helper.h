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
    /**     variables   **/
    ros::Publisher pub;
    tf::TransformListener listener;
    tf::StampedTransform transform;
    std::mutex act_z_mutex;
    tf::Vector3 init_pos, old_pos, new_pos;
    std::string topic_pub, topic_sub, input_joint, controlled_joint, tf_src, tf_dst;
    bool sim;
    double act_z, new_z;
    const double zlift_max = 0.80;
    const double zlift_min = 0.00;

    /**     functions   **/
    void readParams(ros::NodeHandle nh);
    void calcNewPos();

};


#endif //MEKA_GUIDING_HELPER_H

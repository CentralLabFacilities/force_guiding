#include "ros/ros.h"
#include "MovementController.h"
#include <actionlib/server/simple_action_server.h>
#include <force_guiding/GuidingAction.h>

class GuidingAction {
public:

    GuidingAction(std::string name) :
    as_(nh_, name, false),
    action_name_(name) {

        //create controller which then created modules
        mc_ptr_.reset(new MovementController());
        
        //register the goal and feeback callbacks
        as_.registerGoalCallback(boost::bind(&GuidingAction::goalCB, this));
        as_.registerPreemptCallback(boost::bind(&GuidingAction::preemptCB, this));

        //subscribe to the data topic of interest
        //sub_ = nh_.subscribe("/force_guiding", 1, &GuidingAction::analysisCB, this);
        as_.start();
    }

    ~GuidingAction(void) {
    }

    void goalCB() {
        mc_ptr_.get()->start();
    }

    void preemptCB() {
        
    }

    void analysisCB() {
        // make sure that the action hasn't been canceled
        if (!as_.isActive())
            return;

    }

protected:

    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<force_guiding::GuidingAction> as_;
    std::string action_name_;
    force_guiding::GuidingFeedback feedback_;
    force_guiding::GuidingResult result_;
    ros::Subscriber sub_;
    boost::shared_ptr<MovementController> mc_ptr_; 
};


int main(int argc, char **argv) {

    //init node
    ros::init(argc, argv, "force_guiding");

    GuidingAction guiding(ros::this_node::getName());
    
    ROS_INFO("... and we're spinning in the main thread!");
    ros::MultiThreadedSpinner().spin();
    
    return 0;
}
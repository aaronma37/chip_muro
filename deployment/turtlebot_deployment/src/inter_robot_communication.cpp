#include <ros/ros.h>

#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <turtlebot_deployment/PoseWithName.h>

double on;
class Communication
{
public:
    // Constructor
    Communication();

private:
    // Functions
    void positionCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& position);

    // ROS stuff
    ros::NodeHandle ph_, nh_, gnh_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    ros::Timer timer_;

    // Other variables
    std::string name_;
    turtlebot_deployment::PoseWithName pose_with_name_;
};

Communication::Communication():
    ph_("~"),
    name_("no_name")
{
    ROS_DEBUG("Communication class created");
    pose_with_name_.pose;
    ph_.param("robot_name", name_, name_);

    if (name_ == "no_name") {
        ROS_ERROR("Communication: Robot name not set");
    }
    else {
    	if (on==1){
        pub_ = nh_.advertise<turtlebot_deployment::PoseWithName>("toKalmanfilter", 1, true);}
        else{
	pub_ = gnh_.advertise<turtlebot_deployment::PoseWithName>("/all_positions", 1, true);}
        sub_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 10, &Communication::positionCallback, this);
    }
}

void Communication::positionCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& position)
{
    pose_with_name_.pose = position->pose.pose;
    pose_with_name_.name = name_;
    pub_.publish(pose_with_name_);
    std::cout<<"yes"<<"\n";
}

int main(int argc, char** argv)
{
	on=1;
	
    ros::init(argc, argv, "inter_robot_communication");
    Communication inter_robot_communication;
    ros::NodeHandle ph;
    ph.getParam("EKF_switch", on);

    ros::spin();
}

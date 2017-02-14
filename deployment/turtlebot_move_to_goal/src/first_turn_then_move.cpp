// subscribe to your position
// calculate how to move to center
// publish to the cmd_vel

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <tf/tf.h>

const double PI = 3.14159265;

class FirstTurnThenMove
{
public:
    FirstTurnThenMove();
private:

    // Methods
    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&);
    void goalCallback(const geometry_msgs::PoseStampedConstPtr&);
    void odomCallback(const nav_msgs::OdometryConstPtr&);

    double square(double);

    // ROS stuff
    ros::NodeHandle ph_, nh_;
    ros::Subscriber pos_sub_;
    ros::Subscriber goal_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher vel_pub_;

    // Other member variables
    nav_msgs::Odometry odom_;
    geometry_msgs::PoseStamped goalPose_;
    geometry_msgs::TwistPtr cmd_vel_;

    double max_drive_speed_,
    max_turn_speed_,
    angle_tolerance_,
    distance_tolerance_;
    bool got_goal_;

};

FirstTurnThenMove::FirstTurnThenMove():
    cmd_vel_(new geometry_msgs::Twist),
    max_turn_speed_(0.2),
    max_drive_speed_(0.2),
    angle_tolerance_(0.5),
    distance_tolerance_(0.05),
    got_goal_(false)
{
    ph_.param("turn_speed", max_turn_speed_, max_turn_speed_ );     // Turning speed of the robot
    ph_.param("drive_speed", max_drive_speed_, max_drive_speed_);   // Driving speed of the robot
    ph_.param("angle_tolerance", angle_tolerance_, angle_tolerance_);
    ph_.param("distance_tolerance", distance_tolerance_, distance_tolerance_);

    pos_sub_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 1, &FirstTurnThenMove::poseCallback, this);
    goal_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("move_base_simple/goal",1, &FirstTurnThenMove::goalCallback, this);
    //    odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("odom",1, &FirstTurnThenMove::odomCallback, this);

    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 1, true);

    //    timer_ = nh_.createTimer(ros::Duration(0.05), boost::bind(&FirstTurnThenMove::update, this));
}

double FirstTurnThenMove::square(double x){
    return x*x;
}

void FirstTurnThenMove::goalCallback(const geometry_msgs::PoseStampedConstPtr& goalPose){
    goalPose_ = *goalPose;
    got_goal_ = true;
}

void FirstTurnThenMove::odomCallback(const nav_msgs::OdometryConstPtr & odom){
    odom_ = *odom;
}

void FirstTurnThenMove::poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose)
{
    if ( got_goal_ ){
        // Calculate yaw angle
        double orientation = tf::getYaw(pose->pose.pose.orientation);
        //std::cout << "heading = " << orientation << "\n";
	std::cout << "my_x = " << pose->pose.pose.position.x << " ;my_y = " << pose->pose.pose.position.y << "\n";
	std::cout << "goal_x = " << goalPose_.pose.position.x << " ;goal_y = " << goalPose_.pose.position.y << "\n";

        // Calculate direction from location to goal position
        double dy = goalPose_.pose.position.y - pose->pose.pose.position.y;
        double dx = goalPose_.pose.position.x - pose->pose.pose.position.x;
        double goalDirection = atan2(dy,dx);

        double distance_to_goal = sqrt(square(dx)+square(dy));
        //std::cout << "distance to goal = " << distance_to_goal << "\n";

        // Calculate angle from orientation to goal
        double dth = goalDirection - orientation;
	std::cout << "my_orientation = " << orientation << " ;goal_orientation= " << goalDirection << "\n";

        // Make sure that the robot always turns over the smallest angle
        if ( dth > PI ) {
            dth = dth - 2*PI;
        } else if ( dth < -PI ) {
            dth = dth + 2*PI;
        }

        std::cout << "angle to goal = " << dth << "\n\n";

        if ( dth < - angle_tolerance_ ) {
            cmd_vel_->angular.z = -max_turn_speed_;
            cmd_vel_->linear.x = 0.0;
        }
        else if ( dth > angle_tolerance_ ) {
            cmd_vel_->angular.z = max_turn_speed_;
            cmd_vel_->linear.x = 0.0;
        }
        else if ( distance_to_goal > distance_tolerance_ ){
            cmd_vel_->angular.z = 0.0;
            cmd_vel_->linear.x = -max_drive_speed_;
        }
        else {
            cmd_vel_->angular.z = 0.0;
            cmd_vel_->linear.x = 0.0;
        }
    }
    else {
        ROS_WARN("No goal received yet");
        cmd_vel_->angular.z = 0.0;
        cmd_vel_->linear.x = 0.0;
    }

    vel_pub_.publish(cmd_vel_);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "FirstTurnThenMove");
    FirstTurnThenMove firstTurnThenMove;

    ros::spin();
}



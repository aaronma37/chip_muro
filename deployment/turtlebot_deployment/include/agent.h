#ifndef AGENT_H
#define AGENT_H

#include <geometry_msgs/Pose.h>
#include <turtlebot_deployment/PoseWithName.h>
#include <tf/tf.h>

class Agent
{
public:
    Agent();
    Agent( std::string );
    Agent( int );
    Agent( int, turtlebot_deployment::PoseWithName, double );
    Agent( int, std::string, geometry_msgs::Pose, double );

    void setId( int );
    void setName( std::string );
    void setPose( geometry_msgs::Pose );
    void setDistance( double );

    int getId() const;
    std::string getName() const;
    geometry_msgs::Pose getPose() const;
//    geometry_msgs::Pose::ConstPtr getPosePtr() const;
    double getDistance() const;
    ros::Duration getAge() const;

private:
    ros::Time timestamp_;
    int id_;
    std::string name_;
    geometry_msgs::Pose pose_;
    double distance_;
};

#endif

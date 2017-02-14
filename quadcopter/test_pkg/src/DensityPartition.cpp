#include <ros/ros.h>
#include <iostream>

double T = 50; // ROS loop rate

int main(int argc, char **argv)
{
    // ROS initialization
    ros::init(argc, argv, "Density Partition"); //Ros Initialize
    ros::start();
    ros::Rate loop_rate(T); //Set Ros frequency to 50/s (fast)
    ros::NodeHandle n;
    
    // Subscribers
    ros::Subscriber sites_sub; // retrieves array of sites (agent positions)
    ros::Subscriber std_devs_sub; // retrieves array of Gaussian standard deviations
    sites_sub = n.subscribe</*FIXME: Define*/>("/", 1, sitesCallback);
    std_devs_sub = n.subscribe</*FIXME: Define*/>("/", 1, stdDevsCallback);
    
    // Publishers
    ros::Publisher sites_pub;
    velPub = n.advertise</*FIXME: Define*/>("/", 1000, true);

    // Initialize msgs


    while (ros::ok()) 
    {
        ros::spinOnce();
        
        loop_rate.sleep();
    }
}

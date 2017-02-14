// This node is a simple way to approximate ros units to camera pixel conversion.
// To do this, note initial robot position (camera), then run this node. note final robot position (camera)

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <tf/tf.h>
#include <stdlib.h> 
#include <time.h>   

int main(int argc, char **argv)
{
  
ros::init(argc, argv, "rosUnitsMove");
time_t timer,begin,end;

std::cout<<"Checkpoint";
geometry_msgs::Twist cmd_vel_;
  ros::NodeHandle ph_, nh_;
  ros::Publisher vel_pub_;
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 1, true);
  //cmd_vel_.angular.z = 0.0;
  //cmd_vel_.linear.x = .1;
  double speed=.1 ;
  cmd_vel_.angular.z = speed;
  cmd_vel_.linear.x = 0;
  int x=1;
  double z=0;
  int flip=1;
  time(&end);
  while(1==1){
    time(&begin);
          while (difftime(time(&timer),begin)<200){
            
           // z=(x%20);
            //z=z/100+.1;
          z=1;
            //z=abs(cos(x/100));
            std::cout<<"normalizer"<<z<<"\n";
          cmd_vel_.angular.z = speed*z;
        vel_pub_.publish(cmd_vel_);
        //usleep(5000);
        usleep(100000);
        if (x==20){flip=-1;}
        if (x==0){flip=1;}
        x=x+flip;
          } 
    sleep(10);
  }
}

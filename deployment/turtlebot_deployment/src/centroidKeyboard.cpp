#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "PoseWithName.h"
#include <tf/tf.h>
#include <math.h>
#include <termios.h>
#include <time.h> 

//Declare Variables
double x2, x1, r;
double orientation;
double robVel_;
double OmegaC;
double OmegaD;
double cenx, ceny;
// Construct Node Class

int getch()
{
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering      
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

  int c = getchar();  // read character (non-blocking)

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
}

int main(int argc, char **argv)
{
ros::init(argc, argv, "centroidKeyboard");


ros::NodeHandle ph_("~"), nh_;
ros::Publisher cmd0,cmd1,cmd2,cmd3,cmd4,cmd5;



cmd0 = nh_.advertise<std_msgs::Bool>("/cmd0", 5, true);
cmd1 = nh_.advertise<std_msgs::Bool>("/cmd1", 5, true);
cmd2 = nh_.advertise<std_msgs::Bool>("/cmd2", 5, true);
cmd3 = nh_.advertise<std_msgs::Bool>("/cmd3", 5, true);
cmd4 = nh_.advertise<std_msgs::Bool>("/cmd4", 5, true);
cmd5 = nh_.advertise<std_msgs::Bool>("/cmd5", 5, true);

double k=1.75;
double u1=robVel_;
double u2=robVel_/r;
OmegaC=2;
OmegaD=1;
std::cout<<"Exiting Main Sequence: \n";
while(1==1){

	int c = getch();   // call your non-blocking input function
  if (c == 'a'){
	cmd0.publish(1);
	cmd1.publish(0);
	cmd2.publish(1);
	cmd3.publish(0);
	cmd4.publish(1);
	cmd5.publish(0);
}

  else if (c == 'd'){
	cmd0.publish(0);
	cmd1.publish(1);
	cmd2.publish(0);
	cmd3.publish(1);
	cmd4.publish(0);
	cmd5.publish(1);
}
    else if (c == 's'){
	cmd0.publish(0);
	cmd1.publish(0);
	cmd2.publish(0);
	cmd3.publish(0);
	cmd4.publish(0);
	cmd5.publish(0);
}
    else if (c == 'w'){
	cmd0.publish(1);
	cmd1.publish(0);
	cmd2.publish(1);
	cmd3.publish(0);
	cmd4.publish(0);
	cmd5.publish(0);
}



    usleep(100000);
}	
}

    


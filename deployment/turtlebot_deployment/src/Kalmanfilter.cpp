#include <stdio.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <math.h>
#include "agent.h"
#include <turtlebot_deployment/PoseWithName.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

const double T = 1;
const float H[3][3]={{1,0,0},{0,1,0},{0,0,1}};
const float Q[3][3]={{1,0,0},{0,1,0},{0,0,1}};
const float R[3][3]={{1,0,0},{0,1,0},{0,0,1}};
float X0[3][1]={{1},{1},{1}};
float P0[3][3]={{1,1,1},{1,1,1},{1,1,1}};
float X1[3][1];
float X2[3][1];
float U0[2][1];
float A[3][3];
float AT[3][3];
float P1[3][3];
float P2[3][3];
float temp1[3][3];
float temp[3][3];
float K[3][3];
float temp2[3][1];
float Z[3][1];
int a1,b1,c1,d1;
double abst;
bool flag=false  ;

class Kalmanfilter
{
public:
    Kalmanfilter();
private:
    // Methods
    void poseCallback(const turtlebot_deployment::PoseWithName::ConstPtr& pose);
    void iptCallback(const geometry_msgs::Twist::ConstPtr&);
    // ROS stuff
    ros::NodeHandle nh_ ;
    ros::Subscriber pos_sub_ ;
    ros::Subscriber ipt_sub_ ;
    ros::Publisher gl_pub_ ;
    ros::Publisher sf_pub_;
    // Other member variables
    turtlebot_deployment::PoseWithNamePtr newPose_;
    bool got_pose_;
    double theta,
    x,
    y;

}  ;

Kalmanfilter::Kalmanfilter():
got_pose_(false) ,
newPose_(new turtlebot_deployment::PoseWithName),
theta(0.0),
x(0.0),
y(0.0)
{

    pos_sub_= nh_.subscribe<turtlebot_deployment::PoseWithName>("toKalmanfilter", 10, &Kalmanfilter::poseCallback, this);
    ipt_sub_=nh_.subscribe<geometry_msgs::Twist>("mobile_base/commands/velocity",10,&Kalmanfilter::iptCallback, this); 

    gl_pub_ = nh_.advertise<turtlebot_deployment::PoseWithName>("/all_positions", 10, true);
    sf_pub_=nh_.advertise<turtlebot_deployment::PoseWithName>("afterKalman",10,true);
}

void Kalmanfilter::poseCallback(const turtlebot_deployment::PoseWithName::ConstPtr& posePtr)
{
       x=posePtr->pose.position.x;
       y=posePtr->pose.position.y;
       theta = tf::getYaw(posePtr->pose.orientation);
       newPose_->name=posePtr->name;
       got_pose_=true;

}


void Kalmanfilter::iptCallback(const geometry_msgs::Twist::ConstPtr& ipt)
{
int i,j,k,ad,i1,j1;
    if (flag){
                //calculate for X2=X1+K*(Z-h(X1,0))
       Z[0][0]=x;
       Z[1][0]=y;
       Z[2][0]=theta;
        for ( i=0;i<=2;i++){
            temp2[i][0]=Z[i][0]-X1[i][0];
        }

        for (i=0;i<=2;i++){
             X2[i][0]=X1[i][0]+K[i][0]*temp2[0][0] + K[i][1]*temp2[1][0]+K[i][2]*temp2[2][0];
        }

        //calculate for P2=(I-K*H)*P1
        for (i=0;i<=2;i++){
            for (j=0;j<=2;j++){
                if (i==j){
                    temp1[i][j]=1-K[i][j];
                }
                else{
                    temp1[i][j]=-K[i][j];
                }
            }
        }

        for (i=0;i<=2;i++){
                for ( j=0;j<=2;j++){
                    P2[i][j]=0;
                    for (k=0;k<=2;k++){
                       P2[i][j]=P2[i][j]+temp1[i][k]*P1[k][j];
                    }
                }
            }

       for(i=0;i<=2;i++){
        X0[i][0]=X2[i][0];
        for(j=0;j<=2;j++){
            P0[i][j]=P2[i][j];
        }
       }
//publish

        geometry_msgs::Pose goalPose;
        goalPose.position.x = X2[0][0];
        goalPose.position.y = X2[1][0];
        goalPose.orientation =tf::createQuaternionMsgFromYaw(X2[2][0]); 


        newPose_->pose = goalPose;
        gl_pub_.publish(newPose_);
	sf_pub_.publish(newPose_);
         }
         
    if (got_pose_){
        double velo=ipt->linear.x ;
        double omega=ipt->angular.z ;
//this is the begining of the calculation of Kalman Filter
       //refresh the data of  X1
      
  X1[0][0]=X0[1][1]+velo*cos(theta)*T;
        X1[1][0]=X0[2][1]+velo*sin(theta)*T;
        X1[2][0]=X0[3][1]+omega*T;
        
        //calculate for A
        A[0][0]=1;
        A[0][1]=0;
        A[0][2]=(-velo)*T*sin(theta);
        A[1][0]=0;
        A[1][1]=1;
        A[1][2]=velo*T*cos(theta);
        A[2][0]=0;
        A[2][1]=0;
        A[2][2]=1;
        //calculate for AT
        for ( i=0;i<=2;i++){
        for ( j=0;j<i;j++){
                AT[i][j]=A[j][i];
            }
        }
        
        //calculate for A*P*AT+W*Q*WT
            for (i=0;i<=2;i++){
                for (j=0;j<=2;j++){
                    temp[i][j]=0;
                    for ( k=0;k<=2;k++){
                        temp[i][j]=temp[i][j]+A[i][k]*P0[k][j];
                    }
                }
            }

            for (i=0;i<=2;i++){
                for (j=0;j<=2;j++){
                    temp1[i][j]=0;
                    for (k=0;k<=2;k++){
                        temp1[i][j]=temp1[i][j]+temp[i][k]*AT[k][j];
                    }
                }
            }
        
         for (i=0;i<=2;i++){
            for (j=0;j<=2;j++){
                P1[i][j]=Q[i][j]+temp1[i][j];
            }
        }

        //calculate for P*H/(H*P*HT+V*R*VT)
        for (i=0;i<=2;i++){
            for (j=0;j<=2;j++){
                temp1[i][j]=R[i][j]+P1[i][j];
            }
        }

          //calculate the inverse matrix of  temp1
        
        abst=abs(temp1[0][0]*temp1[1][1]*temp1[2][2]+temp1[0][1]*temp1[1][2]*temp1[2][0]+temp1[0][2]*temp1[1][0]*temp1[2][1]-temp1[0][2]*temp1[1][1]*temp1[2][0]-temp1[0][1]*temp1[1][0]*temp1[2][2]-temp1[0][0]*temp1[2][1]*temp1[1][2]);

        for (i=0;i<=2;i++){
            for (j=0;j<=2;j++){
                 ad=0;
                for ( i1=0;i1<=2;i1++){
                    for ( j1=0;j1<=2;j1++){
                        if((i1!=j)&&(j1!=i)){
                            if (ad==0){
                                a1=temp1[i1][j1];
                               
                            }
                            if (ad==1){
                                b1=temp1[i1][j1];
                                
                            }
                            if (ad==2){
                                c1=temp1[i1][j1];
                               
                            }
                              if (ad==3){
                                d1=temp1[i1][j1];
                               
			      }
			      ad++;
                        }
                    }
                }
        temp[i][j]=((-1)^(i+j))*(a1*d1-c1*b1)/abst;

            }
        }
        //rest of this procedure
            for (i=0;i<=2;i++){
                for (j=0;j<=2;j++){
                    K[i][j]=0;
                    for (k=0;k<=2;k++){
                       K[i][j]=K[i][j]+P1[i][k]*temp[k][j];
                    }
                }
            }

            flag=true;
     }
      
    else{
            ROS_WARN("can't get pose information");
        }
    }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Kalmanfilter");
    Kalmanfilter kalmanfilter;
    //ros::Rate loop_rate(1);
    //ros::spinOnce();
    //loop_rate.sleep();

ros::spin();
}

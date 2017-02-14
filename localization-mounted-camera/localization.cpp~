//Katherine Liu
//UCSD, Dynamics Systems and Controls
//objectTrackingTutorial.cpp

//Modifed from objectTrackingTutorial.cpp written by  Kyle Hounslow 2013

//Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software")
//, to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, 
//and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

//The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
//IN THE SOFTWARE.

//Based in part off of Siddhant Ahuja's Working with ROS and OpenCV tutorial, found here:http://siddhantahuja.wordpress.com/2011/07/20/working-with-ros-and-opencv-draft/


//Includes all the headers necessary to use the most common public pieces of the ROS system.
#include <ros/ros.h>
//Use image_transport for publishing and subscribing to images in ROS
#include <image_transport/image_transport.h>
//Use cv_bridge to convert between ROS and OpenCV Image formats
#include <cv_bridge/cv_bridge.h>
//Include some useful constants for image encoding. Refer to: http://www.ros.org/doc/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html for more info.
#include <sensor_msgs/image_encodings.h>
//Include headers for OpenCV Image processing
#include <opencv2/imgproc/imgproc.hpp>
//Include headers for OpenCV GUI handling
#include <opencv2/highgui/highgui.hpp>
#include "std_msgs/Int32.h"
#include "nav_msgs/Odometry.h"
#include <sstream>
#include <fstream>
#include <iostream>
#include <string>
#include <tf/transform_broadcaster.h>

//Use method of ImageTransport to create image publisher
image_transport::Publisher pub;
//Create the publishers for the location data
ros::Publisher x_pub;
ros::Publisher y_pub;
ros::Publisher theta_pub;
ros::Publisher odom_pub;
 
//Store all constants for image encodings in the enc namespace to be used later.
namespace enc = sensor_msgs::image_encodings;

//Declare a string with the name of the window that we will create using OpenCV where processed images will be displayed.
static const char WINDOW[] = "HSV";
static const char WINDOW2[] = "THRESHOLDED IMAGE (MAIN)";
static const char WINDOW3[] = "LOCATED IMAGE";
static const char WINDOW4[] = "THRESHOLDED IMAGE (POINTER)";
static const char trackbarWindowNameM[] = "Main Pointer Trackbars";
static const char trackbarWindowNameP[] = "Pointer Pointer Trackbars";

//for now, use global variables to return information - this should be changed, but this allows for the robot to hold it's original position if no object is found, rather than appearing to travel through time and space

//makes an myLocs array to save the "main" data to
double myLocs[3];
//make an myLocs2 array to save the "pointer" data to
double myLocs2[2];
//initalize the messages that will be published
std_msgs::Int32 xLocation;
std_msgs::Int32 yLocation;
std_msgs::Int32 heading;
nav_msgs::Odometry odom;
geometry_msgs::Quaternion odom_quat;

//HSV Filter Values, set one (main target)
int H_MIN = 95;
int H_MAX = 135;
int S_MIN = 51;
int S_MAX = 256;
int V_MIN = 0;
int V_MAX = 137;

//HSV Filter Values, set two (pointer)
//int H_MIN2 = 95;
//int H_MAX2 = 135;
//int S_MIN2 = 51;
//int S_MAX2 = 256;
//int V_MIN2 = 90;
//int V_MAX2 = 150;

int H_MIN2 = 0;
int H_MAX2 = 256;
int S_MIN2 = 0;
int S_MAX2 = 256;
int V_MIN2 = 0;
int V_MAX2 = 256;

//size values
//minimum and maximum object area
const int MIN_OBJECT_AREA = 20*20;
//const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH/1.5;
const int MAX_OBJECT_AREA = 200*200;
//maximum number of objects before declaring noise
const int MAX_NUM_OBJECTS = 100;
ros::Time current_time;

std::string intToString(int number){
    std::stringstream ss;
    ss << number;
    return ss.str();
}

void on_trackbar( int, void* )
{//This function gets called whenever a
// trackbar position is changed
}

void createTrackbarsMain(){
//create window for trackbars
	cv::namedWindow(trackbarWindowNameM,0);
//create memory to store trackbar name on window
	char TrackbarName[50];
	sprintf( TrackbarName, "H_MIN", H_MIN);
	sprintf( TrackbarName, "H_MAX", H_MAX);
	sprintf( TrackbarName, "S_MIN", S_MIN);
	sprintf( TrackbarName, "S_MAX", S_MAX);
	sprintf( TrackbarName, "V_MIN", V_MIN);
	sprintf( TrackbarName, "V_MAX", V_MAX);

//create trackbars and insert them into window
//3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
//the max value the trackbar can move (eg. H_HIGH),
//and the function that is called whenever the trackbar is moved(eg. on_trackbar)
// ----> ----> ---->
	cv::createTrackbar( "H_MIN", trackbarWindowNameM, &H_MIN, H_MAX, on_trackbar );
	cv::createTrackbar( "H_MAX", trackbarWindowNameM, &H_MAX, H_MAX, on_trackbar );
	cv::createTrackbar( "S_MIN", trackbarWindowNameM, &S_MIN, S_MAX, on_trackbar );
	cv::createTrackbar( "S_MAX", trackbarWindowNameM, &S_MAX, S_MAX, on_trackbar );
	cv::createTrackbar( "V_MIN", trackbarWindowNameM, &V_MIN, V_MAX, on_trackbar );
	cv::createTrackbar( "V_MAX", trackbarWindowNameM, &V_MAX, V_MAX, on_trackbar );
}

void createTrackbarsPointer(){
//create window for trackbars
	cv::namedWindow(trackbarWindowNameP,0);
//create memory to store trackbar name on window
	char TrackbarName2[50];
	sprintf( TrackbarName2, "H_MIN", H_MIN2);
	sprintf( TrackbarName2, "H_MAX", H_MAX2);
	sprintf( TrackbarName2, "S_MIN", S_MIN2);
	sprintf( TrackbarName2, "S_MAX", S_MAX2);
	sprintf( TrackbarName2, "V_MIN", V_MIN2);
	sprintf( TrackbarName2, "V_MAX", V_MAX2);

//create trackbars and insert them into window
//3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
//the max value the trackbar can move (eg. H_HIGH),
//and the function that is called whenever the trackbar is moved(eg. on_trackbar)
// ----> ----> ---->
	cv::createTrackbar( "H_MIN", trackbarWindowNameP, &H_MIN2, H_MAX2, on_trackbar );
	cv::createTrackbar( "H_MAX", trackbarWindowNameP, &H_MAX2, H_MAX2, on_trackbar );
	cv::createTrackbar( "S_MIN", trackbarWindowNameP, &S_MIN2, S_MAX2, on_trackbar );
	cv::createTrackbar( "S_MAX", trackbarWindowNameP, &S_MAX2, S_MAX2, on_trackbar );
	cv::createTrackbar( "V_MIN", trackbarWindowNameP, &V_MIN2, V_MAX2, on_trackbar );
	cv::createTrackbar( "V_MAX", trackbarWindowNameP, &V_MAX2, V_MAX2, on_trackbar );
}

void drawObject(int x,int y, cv::Mat &frame){
    cv::circle(frame,cv::Point(x,y),10,cv::Scalar(0,0,255));
    cv::putText(frame,intToString(x)+ " , " + intToString(y),cv::Point(x,y+20),1,1, cv::Scalar(0,255,0));
}
 
//This function dilates and errodes the image
void morphOps(cv::Mat &thresh){
    //create structuring element that will be used to "dilate" and "erode" image.
    //the element chosen here is a 3px by 3px rectangle
    cv::Mat erodeElement = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3));
    //dilate with larger element so make sure object is nicely visible
    cv::Mat dilateElement = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(8,8));
    cv::erode(thresh,thresh,erodeElement);
    cv::erode(thresh,thresh,erodeElement);
    cv::dilate(thresh,thresh,dilateElement);
    cv::dilate(thresh,thresh,dilateElement);  
}
 
void trackFilteredObject(double myLocs[], cv::Mat threshold, cv::Mat HSV, cv::Mat &cameraFeed, int mode)
{
    //Note that mode 1 indicates "main" position is desired; mode 2 indicates "pointer" position is desired
    cv::Mat temp;
    threshold.copyTo(temp);
    //these two vectors needed for output of findContours
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    //find contours of filtered image using openCV findContours function
    findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE);
    //use moments method to find our filtered object
    double refArea = 0;
    bool objectFound = false;
    if (hierarchy.size() > 0)
    {
        int numObjects = hierarchy.size();
        //if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
        if(numObjects<MAX_NUM_OBJECTS)
            {
                for (int index = 0; index >= 0; index = hierarchy[index][0]) 
                {
                    //int index = 0;    
                    cv::Moments moment = moments((cv::Mat)contours[index]);
                    double area = moment.m00;
                    //if the area is less than 20 px by 20px then it is probably just noise
                    //if the area is the same as the 3/2 of the image size, probably just a bad filter
                    //we only want the object with the largest area so we safe a reference area each
                    //iteration and compare it to the area in the next iteration.
                    if(area>MIN_OBJECT_AREA && area<MAX_OBJECT_AREA && area>refArea)
                        {
                            if (mode==1)
                            {
                                //myLocs[0] = (moment.m10/area);
                				xLocation.data = (int)(moment.m10/area);
                                //myLocs[1] = (moment.m01/area);
                				yLocation.data = (int)(moment.m01/area);
                                objectFound = true;
                                refArea = area;
                			}
                            else
                            {
        				        myLocs2[0] = (moment.m10/area);
        		                myLocs2[1] = (moment.m01/area);
        		                objectFound = true;
                                refArea = area;
    			             }
                        } //else objectFound = false;
                }
            }

        if(objectFound ==true)
            {
                //draw object location on screen, if you're looking for the main pointer
                if (mode==1)
                {
                    drawObject(xLocation.data,yLocation.data,cameraFeed);
                }
                else
                {
                    //calculate the heading
                    myLocs[2] = atan(myLocs2[0]/myLocs2[1]);
                    //probably want to change this to a double or something
                    heading.data = (int) atan(myLocs2[0]/myLocs2[1]);
                    drawObject(myLocs2[0],myLocs2[1],cameraFeed);
                }
            }
        } else putText(cameraFeed,"TOO MUCH NOISE! ADJUST FILTER",cv::Point(0,50),1,2,cv::Scalar(0,0,255),2);
}


//This function is called everytime a new image is published
void imageCallback(const sensor_msgs::ImageConstPtr& original_image)
{
    current_time = ros::Time::now();
    
    //Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        //Always copy, returning a mutable CvImage
        //OpenCV expects color images to use BGR channel order.
        cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        //if there is an error during conversion, display it
        ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
        return;
    }
 
    //translate to HSV plane
    cv::Mat HSV;
    cv::Mat HSV2;
    cv::Mat threshold;
    cv::Mat threshold2;
    cvtColor(cv_ptr->image, HSV, cv::COLOR_BGR2HSV);
    cv::inRange(HSV, cv::Scalar(H_MIN, S_MIN, V_MIN), cv::Scalar(H_MAX, S_MAX, V_MAX), threshold);
    cv::inRange(HSV, cv::Scalar(H_MIN2, S_MIN2, V_MIN2), cv::Scalar(H_MAX2, S_MAX2, V_MAX2), threshold2);
    morphOps(threshold);
    morphOps(threshold2);
     
    //Display the image using OpenCV
    //cv::imshow(WINDOW, cv_ptr->image);

    //DISPLAY THE HSV IMAGE
    cv::imshow(WINDOW, HSV);
    //DISPLAY THE THRESHOLDED IMAGE (main)
    cv::imshow(WINDOW2, threshold);
    //DISPLAY THE THRESHOLDED IMAGE (pointer)
    cv::imshow(WINDOW4, threshold2);

    //TRACK THE FILTERED OBJECT!!! (EEP!)
    trackFilteredObject(myLocs, threshold,HSV, cv_ptr->image, 1);
    trackFilteredObject(myLocs, threshold2, HSV2, cv_ptr->image, 2);

    //DISPLAY THE TRACKED OBJECT(???)
    cv::imshow(WINDOW3, cv_ptr->image);

    //Add some delay in miliseconds. The function only works if there is at least one HighGUI window created and the window is active. If there are several HighGUI windows, any of them can be active.
    cv::waitKey(3);
    /**
    * The publish() function is how you send messages. The parameter
    * is the message object. The type of this object must agree with the type
    * given as a template parameter to the advertise<>() call, as was done
    * in the constructor in main().
    */
    //Convert the CvImage to a ROS image message and publish it on the "camera/image_processed" topic.
        pub.publish(cv_ptr->toImageMsg());

        //Publish all the relevant information
    	x_pub.publish(xLocation);
    	y_pub.publish(yLocation);
    	theta_pub.publish(heading);

//Start making the odom message
	odom_quat = tf::createQuaternionMsgFromYaw((double)myLocs[2]);	
	ros::Time current_time;
  	current_time = ros::Time::now();	
	odom.header.stamp = current_time;
	//set the position
	odom.pose.pose.position.x = xLocation.data;
	odom.pose.pose.position.y = yLocation.data;
	odom.pose.pose.position.z = 0.0;
	//odom.pose.pose.orientation = odom_quat;
	odom.header.frame_id = "odom";

	/*odom.pose.pose.orientation.x = 1;               // identity quaternion
	odom.pose.pose.orientation.y = 0;              // identity quaternion
	odom.pose.pose.orientation.z = 0;              // identity quaternion
	odom.pose.pose.orientation.w = 0;              // identity quaternion
	odom.pose.covariance = {1000, 0, 0, 0, 0, 0,  // covariance on gps_x
		               0, 1000, 0, 0, 0, 0,  // covariance on gps_y
		               0, 0, 1000 0, 0, 0,  // covariance on gps_z
		               0, 0, 0, 99999, 0, 0,  // large covariance on rot x
		               0, 0, 0, 0, 99999, 0,  // large covariance on rot y
		               0, 0, 0, 0, 0, 99999};  // large covariance on rot z */
	odom_pub.publish(odom);
}
 
/**
* This tutorial demonstrates simple image conversion between ROS image message and OpenCV formats and image processing
*/
int main(int argc, char **argv)
{
	createTrackbarsMain();
	createTrackbarsPointer();

        ros::init(argc, argv, "image_processor");

        ros::NodeHandle nh;

        image_transport::ImageTransport it(nh);

        cv::namedWindow(WINDOW, CV_WINDOW_AUTOSIZE);
 
        image_transport::Subscriber sub = it.subscribe("camera/image_raw", 1, imageCallback);
    
        cv::destroyWindow(WINDOW);
     
	x_pub = nh.advertise<std_msgs::Int32>("xLocation", 10);
	y_pub = nh.advertise<std_msgs::Int32>("yLocation", 10);
	theta_pub = nh.advertise<std_msgs::Int32>("heading", 10);
	odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
	
   
        pub = it.advertise("camera/image_processed", 1);

        ros::spin();
   
    	ROS_INFO("tutorialROSOpenCV::main.cpp::No error.");
 
}


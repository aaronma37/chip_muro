#include <iostream>
#include <stdio.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <math.h>
#include <vector>
#include <stdlib.h>
#include "std_msgs/String.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <sstream>

using namespace std;


// Variables
double originalArray[15] = {1,2,3,4,5,6,7,8,9,10,11,0,0,0,0};
double placeHolder[15] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int chosenIndex = 5;
int main()
{
    for (int i=0; i<15; i++)
    {
        placeHolder[i] = originalArray[i];
    }
    
    originalArray[0] = originalArray[chosenIndex];
    int j = 1;
    int z = 0;
    while (originalArray[chosenIndex+j] != 0 )
    {
        originalArray[j] = originalArray[chosenIndex+j];
        j++;
    }
    for (z=0 ; z< chosenIndex; z++)
    {
        originalArray[j] = placeHolder[z];
        j++;
    }
    
    for (int w=0; w <15; w++)
    {
    cout << originalArray[w] << " "; 
    }
    return 0;
    
}

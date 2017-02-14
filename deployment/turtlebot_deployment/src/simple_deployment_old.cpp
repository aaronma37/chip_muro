#include <ros/ros.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
//#include <move_base_msgs/MoveBaseAction.h>
//#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>

#include <turtlebot_deployment/PoseWithName.h>

#include "agent.h"
#include "VoronoiDiagramGenerator.h"

//typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
const cv::Scalar  WHITE = cv::Scalar(255);
const cv::Scalar BLACK = cv::Scalar(0);

/// Functor definitions
struct MatchString
{
    MatchString(const std::string& s) : s_(s) {}
    bool operator()(const Agent& obj) const
    {
        return obj.getName() == s_;
    }
private:
    const std::string& s_;
};

struct SortAgentsOnDistance
{
    bool operator()( const Agent& agent1, const Agent& agent2 ) const {
        return agent1.getDistance() < agent2.getDistance();
    }
};  

/// Class definition
class SimpleDeployment
{
public:
    // Constructor and destructor
    SimpleDeployment();

private:

    // Methods
    void positionsCallback(const turtlebot_deployment::PoseWithName::ConstPtr& pose);
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map);
    void publish();
    cv::Mat drawMap();
    double distance(geometry_msgs::Pose,geometry_msgs::Pose);
    void removeOldAgents();

    // ROS stuff
    ros::NodeHandle ph_, nh_;
    ros::Subscriber pos_sub_;
    ros::Subscriber map_sub_;
    ros::Publisher goal_pub_;
    ros::Timer timer_;
//    MoveBaseClient ac_;

    // Message variables
    nav_msgs::OccupancyGrid map_;
    bool got_map_;
    bool got_me_;
    geometry_msgs::PoseStamped goal_;

    // Other member variables
    double hold_time_;
    Agent this_agent_;
    double resolution_;

    std::vector<Agent> agent_catalog_;
};

/// Method bodies
SimpleDeployment::SimpleDeployment():
    ph_("~"),
    this_agent_(),
    got_map_(false),
    got_me_(false),
    hold_time_(0.0),
    resolution_(0.0)
//    ac_("move_base",true)
{
    ROS_DEBUG("SimpleDeployment: Constructing SimpleDeployment object");
    std::string temp_name;
    ph_.param("robot_name", temp_name, this_agent_.getName() );     // Name of this agent
    ph_.param("hold_time", hold_time_, hold_time_ );                // Time to hold peer agents in agent catalog
    ph_.param("resolution", resolution_, resolution_ );             // Resolution of the voronoi image (in m/px)
    this_agent_.setName(temp_name);

    if (this_agent_.getName() == "not defined") {
        ROS_ERROR("SimpleDeployment: Robot name not set");
    }
    else if (hold_time_ <= 0.0) {
        ROS_ERROR("SimpleDeployment: Hold time invalid or not set");
    }
    else if (resolution_ <= 0.0) {
        ROS_ERROR("SimpleDeployment: Resolution invalid or not set");
    }
    else {

            pos_sub_ = nh_.subscribe<turtlebot_deployment::PoseWithName>("/all_positions", 1, &SimpleDeployment::positionsCallback, this);
            map_sub_ = nh_.subscribe<nav_msgs::OccupancyGrid>("/map",1, &SimpleDeployment::mapCallback, this);
            goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1, true);

        timer_ = nh_.createTimer(ros::Duration(2.0), boost::bind(&SimpleDeployment::publish, this));

        cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );
    }
    ROS_WARN("SimpleDeployment: SimpleDeployment object constructed");
}

void SimpleDeployment::positionsCallback(const turtlebot_deployment::PoseWithName::ConstPtr& posePtr)
{
    ROS_ERROR("SimpleDeployment: Positions received, finding robot in database");	//ROS_DEBUG
    // Search for agent in the catalog using functor that compares the name to an input string
    std::vector<Agent>::iterator it = std::find_if(agent_catalog_.begin(), agent_catalog_.end(), MatchString(posePtr->name) );

    // If the agent is already in the catalog, update the position and recalculate the distance.
    if ( it != agent_catalog_.end() ) {
        ROS_DEBUG("SimpleDeployment: Robot found, updating pose and distance");

        it->setPose(posePtr->pose);
        it->setDistance(distance(this_agent_.getPose(),posePtr->pose));
    }

    // If the agent is not yet in the catalog, create an Agent object and push it into the catalog vector (if it is not this agent)
    else {
        ROS_DEBUG("SimpleDeployment: Robot not found in database");

        if ( posePtr->name != this_agent_.getName() ) {
            ROS_DEBUG("SimpleDeployment: Robot is not me. Adding it to database");

            // This initializes an object called "agent" with id = 1, the pose of the incoming message, and the distance from this agent to the agent that published the message
            Agent agent( 1, *posePtr, distance(this_agent_.getPose(), posePtr->pose) );
            agent_catalog_.push_back( agent );
        }
        else {
            ROS_DEBUG("SimpleDeployment: Robot is me! Updating position and adding to database");
            this_agent_.setPose(posePtr->pose);
            got_me_ = true;

            // This initializes an object called "agent" with id = 0, the pose of the incoming message, and a distance of 0.0;
            Agent agent( 0, *posePtr, 0.0 );
            agent_catalog_.push_back( agent );
        }
    }

    // Sort agent list on distance (using functor)
    std::sort( agent_catalog_.begin(), agent_catalog_.end(), SortAgentsOnDistance() );
    ROS_WARN("SimpleDeployment: Positions processed");
}

// Function to retreive and store map information.
void SimpleDeployment::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& mapPtr)
{
    map_ = *mapPtr;
    got_map_ = true;
}

// In this function, the Voronoi cell is calculated, integrated and the new goal point is calculated and published.
void SimpleDeployment::publish()
{
    if ( got_map_ && got_me_ ) {
        double factor = map_.info.resolution / resolution_; // zoom factor for openCV visualization

        ROS_ERROR("SimpleDeployment: Map received, determining Voronoi cells and publishing goal"); 	//ROS_WARN

        removeOldAgents();

        // Create variables for x-values, y-values and the maximum and minimum of these, needed for VoronoiDiagramGenerator
        float xvalues[agent_catalog_.size()];
        float yvalues[agent_catalog_.size()];
        double xmin = 0.0, xmax = 0.0, ymin = 0.0, ymax = 0.0;
        cv::Point seedPt = cv::Point(1,1);

        // Create empty image with the size of the map to draw points and voronoi diagram in
        cv::Mat vorImg = cv::Mat::zeros(map_.info.height*factor,map_.info.width*factor,CV_8UC1);

        for ( uint i = 0; i < agent_catalog_.size(); i++ ) {
            geometry_msgs::Pose pose = agent_catalog_[i].getPose();

            // Keep track of x and y values
            xvalues[i] = pose.position.x;
            yvalues[i] = pose.position.y;

            // Keep track of min and max x
            if ( pose.position.x < xmin ) {
                xmin = pose.position.x;
            } else if ( pose.position.x > xmax ) {
                xmax = pose.position.x;
            }

            // Keep track of min and max y
            if ( pose.position.y < ymin ) {
                ymin = pose.position.y;
            } else if ( pose.position.y > ymax ) {
                ymax = pose.position.y;
            }

            // Store point as seed point if it represents this agent
            if ( agent_catalog_[i].getName() == this_agent_.getName() ){
                // Scale positions in metric system column and row numbers in image
                int c = ( pose.position.x - map_.info.origin.position.x ) * factor / map_.info.resolution;
                int r = map_.info.height * factor - ( pose.position.y - map_.info.origin.position.y ) * factor / map_.info.resolution;
                cv::Point pt =  cv::Point(c,r);

                seedPt = pt;
            }
            // Draw point on image
//            cv::circle( vorImg, pt, 3, WHITE, -1, 8);
        }

        ROS_DEBUG("SimpleDeployment: creating a VDG object and generating Voronoi diagram");
        // Construct a VoronoiDiagramGenerator (VoronoiDiagramGenerator.h)
        VoronoiDiagramGenerator VDG;

        xmin = map_.info.origin.position.x; xmax = map_.info.width  * map_.info.resolution + map_.info.origin.position.x;
        ymin = map_.info.origin.position.y; ymax = map_.info.height * map_.info.resolution + map_.info.origin.position.y;

        // Generate the Voronoi diagram using the collected x and y values, the number of points, and the min and max x and y values
        VDG.generateVoronoi(xvalues,yvalues,agent_catalog_.size(),float(xmin),float(xmax),float(ymin),float(ymax));

        float x1,y1,x2,y2;

        ROS_DEBUG("SimpleDeployment: collecting line segments from the VDG object");
        // Collect the generated line segments from the VDG object
        while(VDG.getNext(x1,y1,x2,y2))
        {
            // Scale the line segment end-point coordinates to column and row numbers in image
            int c1 = ( x1 - map_.info.origin.position.x ) * factor / map_.info.resolution;
            int r1 = vorImg.rows - ( y1 - map_.info.origin.position.y ) * factor / map_.info.resolution;
            int c2 = ( x2 - map_.info.origin.position.x ) * factor / map_.info.resolution;
            int r2 = vorImg.rows - ( y2 - map_.info.origin.position.y ) * factor / map_.info.resolution;

            // Draw line segment
            cv::Point pt1 = cv::Point(c1,r1),
                      pt2 = cv::Point(c2,r2);
            cv::line(vorImg,pt1,pt2,WHITE);
        }

        ROS_DEBUG("SimpleDeployment: drawing map occupancygrid and resizing to voronoi image size");
        // Create cv image from map data and resize it to the same size as voronoi image
        cv::Mat mapImg = drawMap();
        cv::Mat viewImg(vorImg.size(),CV_8UC1);
        cv::resize(mapImg, viewImg, vorImg.size(), 0.0, 0.0, cv::INTER_NEAREST );

        // Add images together to make the total image
        cv::Mat totalImg(vorImg.size(),CV_8UC1);
        cv::bitwise_or(viewImg,vorImg,totalImg);

        cv::Mat celImg = cv::Mat::zeros(vorImg.rows+2, vorImg.cols+2, CV_8UC1);
        cv::Scalar newVal = cv::Scalar(1), upDiff = cv::Scalar(100), loDiff = cv::Scalar(256);
        cv::Rect rect;

        cv::floodFill(totalImg,celImg,seedPt,newVal,&rect,loDiff,upDiff,4 + (255 << 8) + cv::FLOODFILL_MASK_ONLY);

        // Compute the center of mass of the Voronoi cell
        cv::Moments m = moments(celImg, true);
        cv::Point centroid(m.m10/m.m00, m.m01/m.m00);

        // Convert seed point to celImg coordinate system (totalImg(x,y) = celImg(x+1,y+1)
        cv::Point onePt(1,1);
        centroid = centroid - onePt;

        cv::circle( totalImg, seedPt, 3, WHITE, -1, 8);
        cv::circle( totalImg, centroid, 2, WHITE, -1, 8);

        cv::imshow( "Display window", totalImg );
        cv::waitKey(3);

        // Scale goal position in map back to true goal position
        geometry_msgs::Pose goalPose;
        goalPose.position.x = centroid.x * map_.info.resolution / factor + map_.info.origin.position.x;
        goalPose.position.y = (map_.info.height - centroid.y / factor) * map_.info.resolution + map_.info.origin.position.y;
        double phi = atan2( seedPt.y - centroid.y, centroid.x - seedPt.x );
        goalPose.orientation = tf::createQuaternionMsgFromYaw(phi);

        goal_.pose = goalPose;
        goal_.header.frame_id = "map";
        goal_.header.stamp = ros::Time::now();

        goal_pub_.publish(goal_);

//        move_base_msgs::MoveBaseGoal goal;

//        goal.target_pose.header.frame_id = "map";
//        goal.target_pose.header.stamp = ros::Time::now();

//        goal.target_pose.pose = goalPose;

//        ac_.sendGoal(goal);
    }
    else {
        ROS_DEBUG("SimpleDeployment: No map received");
    }

}


/// Function definitions

cv::Mat SimpleDeployment::drawMap()
{
    cv::Mat src = cv::Mat::zeros(map_.info.height,map_.info.width,CV_8UC1);

    // Copy map values to image
    for ( int i = 0; i < map_.info.height; i++ ) {
        for ( uint j = 0; j < map_.info.width; j++ ) {
            signed char* pxPtr = &map_.data[i*map_.info.width + j];
            if ( *pxPtr == -1 ) {
                src.at<uchar>(i,j) = 0;
            }
            else {
                src.at<uchar>(i,j) = round(map_.data[i*map_.info.width + j]*2.55);
            }
        }
    }
    return src;
}

double SimpleDeployment::distance(geometry_msgs::Pose pose2, geometry_msgs::Pose pose1)
{
    double dx = pose2.position.x - pose1.position.x;
    dx = dx*dx;

    double dy = pose2.position.y - pose1.position.y;
    dy = dy*dy;

    double dz = pose2.position.z - pose1.position.z;
    dz = dz*dz;

    // Using squared Euclidean distance
    return /*sqrt(dx+dy+dz);*/ dx+dy+dz;
}

void SimpleDeployment::removeOldAgents(){
    for( std::vector<Agent>::iterator it = agent_catalog_.begin(); it != agent_catalog_.end(); ++it) {
        if ( it->getAge().toSec() > hold_time_ ) {
            agent_catalog_.erase( it );
            it--;
        }
        else {
        }
    }
}

/// Main
int main(int argc, char** argv)
{
    ros::init(argc, argv, "simple_deployment");
    SimpleDeployment simple_deployment;

    ros::spin();
}

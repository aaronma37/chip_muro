#include <iostream>
#include <stdio.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/tf.h>
#include <fstream>
#include <math.h>
#include "PoseWithName.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "eigen/Eigen/Dense"
#include <vector>
#include <stdlib.h>
#include "std_msgs/String.h"
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <sstream>
#include <search.h>
#include <stdlib.h>
#include "VoronoiDiagramGenerator.h"
#include <tf2_msgs/TFMessage.h>
#include "CoMGenerator.h"
#include <set>


using namespace std;
double T=25;
bool gotPose=false;
bool freeBoundaryFlag=false;
bool trackingFlag=false;
const int maxNum=50;
int countD;
float xValues[maxNum];
float yValues[maxNum];
int selectedIndices[maxNum]; 

float sigma=.15;
float lastX[5][maxNum];
float lastY[5][maxNum];
float delayX[5][maxNum];
float delayY[5][maxNum];
float muX=0;
float muY=0;
float turtleX;
float turtleY;
int currentIndex;


geometry_msgs::PoseStamped androidTest;


class vertexArray {
		    public:

		 float x[100];
		 float y[100];
		 int size;
		 float dist[100];
		 float errorBound;
		 float angle[100];
		 bool used[100];

		vertexArray() {
			size=0;
			errorBound=.01;
			for (int i = 0 ; i<100;i++){
				x[i]=0;
				y[i]=0;
				dist[i]=100;
				angle[i]=0;
				used[i]=false;
			}
		}
};

class pointXY {
		    public:

			float x;
			float y;
			int index1;
			int index2;
			int count;
			bool targeted;
			bool boundary;
			int linkSize;
			float uniqueID;
			float angle[5];

			int obstacleLineIndex[5];
			int part[5];

			pointXY(){
				x=0;
				y=0;
				index1=0;
				index2=0;
				count=0;
				linkSize=0;
				uniqueID=-100;
				targeted=false;
				boundary=false;

				for (int i=0; i<5;i++){
					angle[i]=0;
				}
			}

			void findUniqueID(){
				uniqueID=.5*(x+y)*(x+y+1)+y;
			}

			void addLinkAngle(float linkAngle, int k, int t){
				angle[linkSize]= linkAngle;
				obstacleLineIndex[linkSize]=k;
				part[linkSize]= t;
				linkSize++;
			}
};

class pointXYList{
//A unique list of pointXY ~

	public:

	pointXY points[100];
	bool active[100];
	int size;



		pointXYList(){
			size=0;
		}

		void clearList(){
			size=0;
			for (int i=0;i<100;i++){
			active[i]=false;}
		}
};

class bEquation {

public:
float m[100];
float x[100];
float y[100];
float x2[100];
float y2[100];
bool active[100];

			bEquation(){
				for (int i=0;i<100;i++){
					m[i]=0;
					x[i]=0;
					y[i]=0;
					x2[i]=0;
					y2[i]=0;
					active[i]=false;
				}
			}
};

class obstacleLine {
    public:
float x[2];
float y[2];
	obstacleLine(){
	x[0]=0;
	x[1]=0;
	y[0]=0;
	y[1]=0;
	}
};







































		const int maxBots=50;
		bool flag=false;
		pointXY intersections[50];
		pointXY cellBoundaries[50];
		vertexArray uniqueVoronoiVertices;
		vertexArray voronoiCellBoundaries[maxBots];
		vertexArray geodesicPath[maxBots];
		vertexArray tempArrayInsertion;
		vertexArray tempArray;
		float insertionX=0;
		float insertionY=0;
		float insertionAngle=0;
		float tempVX;
		int tempMax;
		float tempVY;
		obstacleLine obstacleLineList[50];
		int obstacleLineSize=0;
		bEquation bEquations[50];
		bEquation oEquations[50];
		float xValuesT[50];
		float yValuesT[50];
		pointXYList obstacleMapPoints;
		pointXYList pathLine[maxBots];
		pointXYList previousPathLine[maxBots];
		int adjacencyMatrix[maxBots][102][102];
		obstacleLine lastObstacle;
		int lastNZ;
		int lastNK;
		int lastSign;
		int path[100];





float minX = -1, maxX = 1;    
float minY = -1, maxY = 1;

float StandardminX = -1, StandardmaxX = 1;    
float StandardminY = -1, StandardmaxY = 1;

//float minX = -.335, maxX = .335;    
//float minY = -.2, maxY = .9;
float filterX[maxNum];
float filterY[maxNum];
int weirdReadings[maxNum];
//geometry_msgs::PoseArray centroidPositions;

// Turtlebot indexes
const int MICHELANGELO_INDEX = 2;
const int DONATELLO_INDEX = 3;
const int RAPHAEL_INDEX = 4;
const int LEONARDO_INDEX = 5;
const int BOTICELLI_INDEX = 6;
const int GIOTTO_INDEX = 7;
const int BELLINI_INDEX = 8;
const int GHIBERTI_INDEX = 9;
const int MASACCIO_INDEX = 10;
const int TITIAN_INDEX = 1;


const int PICASSO_INDEX = 41;
const int DALI_INDEX = 42;
const int GOYA_INDEX = 43;

// This message is an array of PoseStamped that contains the centroids
// to be published to each individual turtlebot topic
geometry_msgs::PoseStamped centroidPosesStamped[GOYA_INDEX + 1];
geometry_msgs::Twist cDotTurtles[GOYA_INDEX+1];

void boundaryCallback(const std_msgs::Bool::ConstPtr& bPtr){
	if (bPtr->data==false){
		freeBoundaryFlag=false;
	}
	else if(bPtr->data==true){
		freeBoundaryFlag=true;
	}
}




	float returnTwoVectorAngle(float vx, float vy, float x1, float y1, float x2, float y2){
		float X1=(x1-vx);
		float X2=(x2-vx);
		float Y1=(y1-vy);
		float Y2=(y2-vy);


		float dot = (X1*X2+Y1*Y2);
		float det = (X1*Y2-Y1*X2);
		return atan2(det,dot);
	}

float dist(float x1,float x2,float y1,float y2){
return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
}

	float returnDotAngle(float vx, float vy, float x1, float y1, float x2, float y2){

		float dot = ((x1-vx)*(x2-vx)+(y1-vy)*(y2-vy));
		return acos(dot/(dist(x1,vx,y1,vy)*dist(x2,vx,y2,vy)));
}





	float findXIntercept(float m1, float m2, float x1, float x2, float y1, float y2){
		return (m1*x1-m2*x2-y1+y2)/(m1-m2);
	}


	float findYIntercept(float m1, float x, float x1, float y1){
		return m1*(x-x1)+y1;
	}


bool segmentIntersection(float x1, float x2, float y1,float y2, float x3, float x4, float y3, float y4){
        float Xa=0;

        Xa=1/(x3-x4);
        if (Xa!=Xa){
            Xa=x3;
            if(findYIntercept((y1-y2)/(x1-x2),Xa,x1,y1)<min(y3,y4) || findYIntercept((y1-y2)/(x1-x2),Xa,x1,y1)>max(y3,y4) ){
                return false;
            }
        }else{
            if (max(x1,x2)<min(x3,x4)){
                return false;
            }
            if ((y1-y2)/(x1-x2)==(y3-y4)/(x3-x4)){
                return false;
            }

            Xa=findXIntercept((y1-y2)/(x1-x2), (y3-y4)/(x3-x4), x1, x3,y1,y3);

            if (Xa<(max(min(x1,x2),min(x3, x4))) || Xa > min(max(x1,x2), max(x3,x4))){
                return false;
            }
        }

        return true;
}


	float findSlope(float x1, float x2, float y1, float y2){
		return (y1-y2)/(x1-x2);
	}

float segmentLineintersection(float m1, float x1, float y1, float x2, float x3, float y2, float y3){
float Xa=0;
Xa=findXIntercept(m1,(y2-y3)/(x2-x3),x1,x2,y1,y2);

	if (Xa!=Xa){
		Xa=x2;
			if(findYIntercept(m1,Xa,x1,y1)<min(y2,y3) || findYIntercept(m1,Xa,x1,y1)>max(y2,y3) ){
				return -100;
			}
	}else{
		if (Xa > max(x2, x3) || Xa < min(x2, x3)){
			return -100;
		}
	}

return Xa;
}

float segmentIntersectionDistance(float x1, float x2, float y1,float y2, float x3, float x4, float y3, float y4, float sourceX, float sourceY){
        float Xa=0;
	float Ya=0;

        Xa=1/(x3-x4);


        if (Xa!=Xa){
            Xa=x3;
	Ya=findYIntercept((y1-y2)/(x1-x2),Xa,x1,y1);
	return dist(sourceX, Xa, sourceY, Ya);
        }else{
		Xa=findXIntercept((y1-y2)/(x1-x2), (y3-y4)/(x3-x4), x1, x3,y1,y3);
		Ya=findYIntercept((y1-y2)/(x1-x2),Xa,x1,y1);
	return dist(sourceX, Xa, sourceY, Ya);
        }

        return -1;
}

int containsPointXY(pointXYList pList, pointXY p){
	for (int i=0;i<pList.size;i++){
		if (p.uniqueID==-100){
			cout << "unique ID a point is not set!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!";
		}
		if (p.uniqueID==pList.points[i].uniqueID){
			return i;
		}
	}

	return -1;
}



float calculateDistance2(float x1, float x2, float y1, float y2, float turtleX, float turtleY, pointXY obstruction, pointXY goal){

	float x33= goal.x;
	float dist1=0;
	float y33= goal.y;

	float d1=.75;
	float d2;
if (obstacleLineSize>0){
	int nz;
	int nk;
	int sign;
	int sign2;




	//Find the right obstacleCallback/ should find a quicker way to do this
int temp1;
int temp2;
int index=0;
float minDist=1000;
obstacleLine bObst;
int minIndex=-1;

if (obstruction.linkSize>0){

	for (int i=0;i<obstruction.linkSize;i++){
		temp1=obstruction.part[i];
		index=obstruction.obstacleLineIndex[i];
		if (temp1==1){
			temp2=0;
		}else{
			temp2=1;
		}

		if (segmentIntersectionDistance(turtleX, goal.x, turtleY, goal.y, obstacleLineList[index].x[temp1],obstacleLineList[index].x[temp2],obstacleLineList[index].y[temp1],obstacleLineList[index].y[temp2],turtleX,turtleY)<minDist){
			minDist=segmentIntersectionDistance(turtleX, goal.x, turtleY, goal.y, obstacleLineList[index].x[temp1],obstacleLineList[index].x[temp2],obstacleLineList[index].y[temp1],obstacleLineList[index].y[temp2],turtleX,turtleY);
			minIndex=i;
		}
	}


	bObst=obstacleLineList[obstruction.obstacleLineIndex[minIndex]];
	lastObstacle=bObst;
	nz=obstruction.part[minIndex];
	lastNZ=nz;
	cout << "NZ: "  << nz << "\n";
		if (nz==1){
			nk=0;
			sign=-1;
			sign=-1;
		}else{
			nk=1;
			sign=1;
			sign2=1;
		}
	lastNK=nk;
	lastSign=sign;

}else{

bObst=lastObstacle;
nz=lastNZ;
nk=lastNK;
sign=lastSign;
}

	d2=dist(bObst.x[nz], x33, bObst.y[nz],y33 );

	int sx;
	int sy;
	float x3,y3;
	float m=(bObst.y[nz]-y33)/(bObst.x[nz]-x33);	//THE SLOPE BETWEEN THE OBSTACLE POINT AND GOAL

	if (dist(turtleX,bObst.x[nz],turtleY,bObst.y[nz])>sigma){
		d1=0;
	}
	float m3=(bObst.y[1]-bObst.y[0])/(bObst.x[1]-bObst.x[0]);
	float m2;															//THE SLOPE BETWEEN THE POINT AND OBSTACLE POINT

	float alpha;
	float ang1;
	float ang2;
	float eD;


//BEHIND POINTS
//bool segmentIntersection(float x1, float x2, float y1,float y2, float x3, float x4, float y3, float y4){ 	float returnTwoVectorAngle(float vx, float vy, float x1, float y1, float x2, float y2){
//calculateDistance2(float x1, float x2, float y1, float y2, float turtleX, float turtleY, pointXY obstruction, pointXY goal){
/*
for (int i=0;i<obstacleLineSize;i++){

	if(	bObst.x[nz] != obstacleLineList[i].x[0]	&&	bObst.x[nz]!=obstacleLineList[i].x[1] && bObst.y[nz]!=obstacleLineList[i].y[0] && bObst.y[nz]!=obstacleLineList[i].y[1]){
		if (segmentIntersection(x1,bObst.x[nz],y1,bObst.y[nz],obstacleLineList[i].x[0],obstacleLineList[i].x[1],obstacleLineList[i].y[0],obstacleLineList[i].y[1])){
			
			cout << "FOUND: " << i << "\n";
			if(abs(returnTwoVectorAngle(bObst.x[nz],bObst.y[nz],turtleX,turtleY,obstacleLineList[i].x[1],obstacleLineList[i].y[1]))>abs(returnTwoVectorAngle(bObst.x[nz],bObst.y[nz],turtleX,turtleY,obstacleLineList[i].x[0],obstacleLineList[i].y[0]))){
//use 0
				pointXY tempPoint;
				tempPoint.x=obstacleLineList[i].x[0];
				tempPoint.y=obstacleLineList[i].y[0];
				//tempPoint=obstacleMapPoints.points[containsPointXY(obstacleMapPoints,tempPoint)];
				//dist1=calculateDistance2(x1,0,y1,0,turtleX,turtleY,tempPoint,obstruction);
//				dist1=1000;
				x1=obstacleLineList[i].x[0];
				y1=obstacleLineList[i].y[0];
			}else{
//use 1
				pointXY tempPoint;
				tempPoint.x=obstacleLineList[i].x[1];
				tempPoint.y=obstacleLineList[i].y[1];
				//tempPoint=obstacleMapPoints.points[containsPointXY(obstacleMapPoints,tempPoint)];
				
				//dist1=calculateDistance2(x1,0,y1,0,turtleX,turtleY,tempPoint,obstruction);
//				dist1=1000;
				x1=obstacleLineList[i].x[1];
				y1=obstacleLineList[i].y[1];
			}
			break;
		}
	}
}
*/

				if (findYIntercept(m,x1,bObst.x[nz],bObst.y[nz])<y1){

					if (findYIntercept(m,(bObst.x[0]+bObst.x[1])/2,(bObst.x[nz]),(bObst.y[nz]))<(bObst.y[0]+bObst.y[1])/2){

					}
					else{
						return dist1+sqrt((x1-x33)*(x1-x33)+(y1-y33)*(y1-y33));
					}
				}
				else{

					if (findYIntercept(m,(bObst.x[0]+bObst.x[1])/2,(bObst.x[nz]),(bObst.y[nz]))>(bObst.y[0]+bObst.y[1])/2){

					}
					else{
						return dist1+sqrt((x1-x33)*(x1-x33)+(y1-y33)*(y1-y33));
					}

				}



if ((findYIntercept(m3,x1,bObst.x[nz],bObst.y[nz])<y1  && findYIntercept(m3,x33,bObst.x[nz],bObst.y[nz])>y33) || (findYIntercept(m3,x1,bObst.x[nz],bObst.y[nz])>y1 
&&findYIntercept(m3,x33,bObst.x[nz],bObst.y[nz])<y33)){


					m2=(bObst.y[nz]-y1)/(bObst.x[nz]-x1);



					if (bObst.y[nz]>y1){
						sy=1;
					}else{sy=-1;}

					if (bObst.x[nz]>x1){
						sx=1;
					}else{sx=-1;}


										ang1=(180-57.2958*abs(returnTwoVectorAngle(-bObst.x[nz],bObst.y[nz],-bObst.x[nk],bObst.y[nk],-x33,y33)));
										ang2=(180-57.2958*abs(returnTwoVectorAngle(-bObst.x[nz],bObst.y[nz],-x1,y1,-x33,y33)));
										alpha =1-(ang1-ang2)/ang1;
										eD=(d2+(d1-d2)*alpha);

/*
					cout << "ang1: "<< ang1<<"\n";
					cout << "ang2: "<< ang2<<"\n";
					cout << "x: " << x1 << "y: " << y1 << "\n";
					cout << "nz: " << nz << "\n";
					cout << "xz: " << obstacleLineList[0].x[nz] << "yz: " << obstacleLineList[0].y[nz] << "\n";
					cout << "goal x: " << x2 << "goal y: " << y2 << "\n";

					cout << "effective Distance "<<  eD<<"\n\n";
*/



							
										x3=bObst.x[nz]+sx*abs(eD*cos(atan(m2)));
										y3=bObst.y[nz]+sy*abs(eD*sin(atan(m2)));

//cout << "x3: " << x3 << " y3: " << y3 << " m"<< m2 <<"\n";


										dist1=dist1+sqrt((x3-x33)*(x3-x33)+(y3-y33)*(y3-y33));
										dist1=dist1+sqrt((x3-x1)*(x3-x1)+(y3-y1)*(y3-y1));

								return dist1;

}
else{
//cout << "FAIL \n";
}



}
return dist1+sqrt((x1-x33)*(x1-x33)+(y1-y33)*(y1-y33));

}


/*


float calculateDistance(float x1, float x2, float y1, float y2, float turtleX, float turtleY){

	if (obstacleLineSize>0){
int nz;
int sign;
int sign2;
if (sqrt((turtleX-obstacleLineList[0].x[1])*(turtleX-obstacleLineList[0].x[1])+(turtleY-obstacleLineList[0].y[1])*(turtleY-obstacleLineList[0].y[1]))<sqrt((turtleX-obstacleLineList[0].x[0])*(turtleX-obstacleLineList[0].x[0])+(turtleY-obstacleLineList[0].y[0])*(turtleY-obstacleLineList[0].y[0]))){
nz=1;
sign=-1;
sign=-1;
}else{
nz=0;
sign=1;
sign2=1;
}

float dist1;
float dist2;
float x3;
float y3;
float swing=.6;
										androidTest.pose.position.x=x3;
										androidTest.pose.position.y=y3;
										androidTest.pose.position.z=-153;

	float m=(obstacleLineList[0].y[1]-obstacleLineList[0].y[0])/(obstacleLineList[0].x[1]-obstacleLineList[0].x[0]);

				if (findYIntercept(m,x1,obstacleLineList[0].x[1],obstacleLineList[0].y[1])<y1){

					if (findYIntercept(m,x2,obstacleLineList[0].x[1],obstacleLineList[0].y[1])>y2){
										x3=obstacleLineList[0].x[nz]+sign*swing/3*cos(atan(m))-swing*cos(atan(-1/m));
										y3=obstacleLineList[0].y[nz]+swing/3*sin(atan(m))-swing*sin(atan(-1/m));
										androidTest.pose.position.x=x3;
										androidTest.pose.position.y=y3;

										dist1=sqrt((x3-x2)*(x3-x2)+(y3-y2)*(y3-y2));
										dist1=dist1+sqrt((x3-x1)*(x3-x1)+(y3-y1)*(y3-y1));

										dist2=sqrt((obstacleLineList[0].x[1]-x2)*(obstacleLineList[0].x[1]-x2)+(obstacleLineList[0].y[1]-y2)*(obstacleLineList[0].y[1]-y2));
										dist2=dist2+sqrt((obstacleLineList[0].x[1]-x1)*(obstacleLineList[0].x[1]-x1)+(obstacleLineList[0].y[1]-y1)*(obstacleLineList[0].y[1]-y1));


										if (dist1<dist2 || 1==1){

										cout << "DISTANCE 1 CHOSEN";
										return dist1;
										}
										else{

										cout << "DISTANCE 2";
										return dist2;
										}
					}
					else{
						return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
					}



			}else{
					if (findYIntercept(m,x2,obstacleLineList[0].x[1],obstacleLineList[0].y[1])<y2){
										x3=obstacleLineList[0].x[nz]+sign*swing/3*cos(atan(m))+swing*cos(atan(-1/m));;
										y3=obstacleLineList[0].y[nz]+swing/3*sin(atan(m))+swing*sin(atan(-1/m));;
										androidTest.pose.position.x=x3;
										androidTest.pose.position.y=y3;

										dist1=sqrt((x3-x2)*(x3-x2)+(y3-y2)*(y3-y2));
										dist1=dist1+sqrt((x3-x1)*(x3-x1)+(y3-y1)*(y3-y1));

										dist2=sqrt((obstacleLineList[0].x[1]-x2)*(obstacleLineList[0].x[1]-x2)+(obstacleLineList[0].y[1]-y2)*(obstacleLineList[0].y[1]-y2));
										dist2=dist2+sqrt((obstacleLineList[0].x[1]-x1)*(obstacleLineList[0].x[1]-x1)+(obstacleLineList[0].y[1]-y1)*(obstacleLineList[0].y[1]-y1));


										if (dist1<dist2 || 1==1){

										cout << "DISTANCE 1 CHOSEN";
										return dist1;
										}
										else{

										cout << "DISTANCE 2";
										return dist2;
										}
					}
					else{
						return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
					}


				}


	}

return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
}

*/

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////Map points and Dikjstra




void toMapPoints(){
//THIS FUNCTION GENERATES OBSTACLEMAPPOINTS~~~

	for (int i=0;i<obstacleLineSize;i++){

		pointXY temp;
		pointXY temp2;
		pointXY temp3;

		temp.x=obstacleLineList[i].x[0];
		temp.y=obstacleLineList[i].y[0];
		temp.findUniqueID();

		if (containsPointXY(obstacleMapPoints, temp)==-1){

			obstacleMapPoints.points[obstacleMapPoints.size]=temp;
			obstacleMapPoints.points[obstacleMapPoints.size].addLinkAngle(returnTwoVectorAngle(temp.x, temp.y, temp.x + 1, temp.y, obstacleLineList[i].x[1], obstacleLineList[i].y[1]),i,0);
			obstacleMapPoints.size++;

				for (int k=i+1; k<obstacleLineSize;k++){
					temp3.x=obstacleLineList[k].x[0];
					temp3.y=obstacleLineList[k].y[0];
					temp3.findUniqueID();
	
					if (containsPointXY(obstacleMapPoints, temp3)!=-1){
						obstacleMapPoints.points[obstacleMapPoints.size-1].addLinkAngle(returnTwoVectorAngle(temp3.x, temp3.y, temp3.x + 1, temp3.y, obstacleLineList[k].x[1], obstacleLineList[k].y[1]),k,0);
					}

					temp3.x=obstacleLineList[k].x[1];
					temp3.y=obstacleLineList[k].y[1];
					temp3.findUniqueID();

					if (containsPointXY(obstacleMapPoints, temp3)!=-1){
						obstacleMapPoints.points[obstacleMapPoints.size-1].addLinkAngle(returnTwoVectorAngle(temp3.x, temp3.y, temp3.x + 1, temp3.y, obstacleLineList[k].x[0], obstacleLineList[k].y[0]),k ,1);
					}

				}
		}


		temp2.x=obstacleLineList[i].x[1];
		temp2.y = obstacleLineList[i].y[1];
		temp2.findUniqueID();

		if (containsPointXY(obstacleMapPoints, temp2)==-1){


			obstacleMapPoints.points[obstacleMapPoints.size]=temp2;
			obstacleMapPoints.points[obstacleMapPoints.size].addLinkAngle(returnTwoVectorAngle(temp2.x, temp2.y, temp2.x + 1, temp2.y, obstacleLineList[i].x[0], obstacleLineList[i].y[0]),i,1);
			obstacleMapPoints.size++;

			for (int k=i+1; k<obstacleLineSize;k++){

				temp3.x=obstacleLineList[k].x[0];
				temp3.y=obstacleLineList[k].y[0];
				temp3.findUniqueID();

				if (containsPointXY(obstacleMapPoints, temp3)!=-1){
					obstacleMapPoints.points[obstacleMapPoints.size-1].addLinkAngle(returnTwoVectorAngle(temp3.x, temp3.y, temp3.x + 1, temp3.y, obstacleLineList[k].x[1], obstacleLineList[k].y[1]),k,0);
				}

				temp3.x=obstacleLineList[k].x[1];
				temp3.y=obstacleLineList[k].y[1];
				temp3.findUniqueID();

				if (containsPointXY(obstacleMapPoints, temp3)!=-1){
					obstacleMapPoints.points[obstacleMapPoints.size-1].addLinkAngle(returnTwoVectorAngle(temp3.x, temp3.y, temp3.x + 1, temp3.y, obstacleLineList[k].x[0], obstacleLineList[k].y[0]),k,1);
				}
			}

		}
	}

}

bool thresholdDist(float x,float y){
        float threshold =.01f;
        if (x>=y-threshold && x<=y+threshold){
            return true;
        }
        return false;
}

bool concaveTest(float angle1, float angle2, float angle3){
        //TEST TO SEE IF ANGLE 3 IS WITHIN ACUTE PORTION OF ANGLE 1 AND ANGLE 2

        if (max(angle1,angle2)-min(angle1,angle2) > 3.14){
            if (angle3 > max(angle1,angle2) || angle3<min(angle1,angle2)){
                return true;
            }
        }else{
            if (angle3 < max(angle1,angle2) && angle3 > min(angle1,angle2)){
                return true;
            }
        }

        return false;
}




void calculateAdjacencyMatrix(){

	int uniqueVertices=obstacleMapPoints.size;

	float xR1, xR2, yR1, yR2;  //Obstacle points trying to determine connection
	float xO1, xO2, yO1, yO2;  //Possible obstructions
	bool flag;

	for(int i=0;i<uniqueVertices;i++){
		for(int j=0;j<uniqueVertices;j++){
			if(i!=j){

				xR1=obstacleMapPoints.points[i].x;
				xR2=obstacleMapPoints.points[j].x;
				yR1=obstacleMapPoints.points[i].y;
				yR2=obstacleMapPoints.points[j].y;
				flag=false; //?

				for (int k=0;k<obstacleLineSize;k++){
					xO1=obstacleLineList[k].x[0];
					yO1=obstacleLineList[k].y[0];
					xO2=obstacleLineList[k].x[1];
					yO2=obstacleLineList[k].y[1];
					//flag=false;

					if((thresholdDist(xR1,xO1) && thresholdDist(yR1,yO1)) || (thresholdDist(xR1,xO2) && thresholdDist(yR1,yO2)) || (thresholdDist(xR2,xO1) && thresholdDist(yR2,yO1)) || (thresholdDist(xR2,xO2) && thresholdDist(yR2,yO2))){
						//Check if these are two points on the same line

						if (k>0 && k<obstacleLineSize-1 && (((thresholdDist(xR1,xO1) && thresholdDist(yR1,yO1)) && (thresholdDist(xR2,xO2) && thresholdDist(yR2,yO2))) || ((thresholdDist(xR2,xO1) && thresholdDist(yR2,yO1)) && (thresholdDist(xR1,xO2) && thresholdDist(yR1,yO2))) )){

							if (	returnTwoVectorAngle(obstacleLineList[k-1].x[1], obstacleLineList[k-1].y[1], obstacleLineList[k-1].x[0], obstacleLineList[k-1].y[0], obstacleLineList[k].x[1], obstacleLineList[k].y[1])  >   0   ){
								//THIS HAPPENS IF ANGLE IS POSITIVE
								if (    returnTwoVectorAngle(obstacleLineList[k].x[1], obstacleLineList[k].y[1], obstacleLineList[k].x[0], obstacleLineList[k].y[0], obstacleLineList[k+1].x[1], obstacleLineList[k+1].y[1])  <   0   ){
									flag=true;
								}
							}

							if (	returnTwoVectorAngle(obstacleLineList[k-1].x[1], obstacleLineList[k-1].y[1], obstacleLineList[k-1].x[0], obstacleLineList[k-1].y[0], obstacleLineList[k].x[1], obstacleLineList[k].y[1])  <   0   ){
								//THIS HAPPENS IF ANGLE IS POSITIVE
								if (    returnTwoVectorAngle(obstacleLineList[k].x[1], obstacleLineList[k].y[1], obstacleLineList[k].x[0], obstacleLineList[k].y[0], obstacleLineList[k+1].x[1], obstacleLineList[k+1].y[1])  >   0   ){
									flag=true;
								}
							}

						}
					}else{

						if (segmentIntersection(xR1,xR2,yR1,yR2,xO1,xO2,yO1,yO2)){
							flag=true;
						}
					}
				}

						if (obstacleMapPoints.points[i].linkSize>1){
							if (concaveTest(obstacleMapPoints.points[i].angle[0],obstacleMapPoints.points[i].angle[1],returnTwoVectorAngle(xR1, yR1, xR1 + 1, yR1, xR2, yR2))){
								flag=true;
							}
						}

						if (obstacleMapPoints.points[j].linkSize>1){
							if (concaveTest(obstacleMapPoints.points[j].angle[0],obstacleMapPoints.points[j].angle[1],returnTwoVectorAngle(xR2, yR2, xR2 + 1, yR2, xR1, yR1))){
								flag=true;
							}
						}

			if(flag==false){
				for(int k=0;k<maxBots;k++){
					//if(turtleList[k].on==1){
						adjacencyMatrix[k][i+2][j+2]=1;
						adjacencyMatrix[k][j+2][i+2]=1;
					//}
				}
			}

			}
		}
	}

}



void calculateAdjacencyMatrixGoal(){

	int uniqueVertices=obstacleMapPoints.size;
	float xR1, xR2, yR1, yR2;  //Obstacle points trying to determine connection
	float xO1, xO2, yO1, yO2;  //Possible obstructions
	bool flag;

	for(int i=0;i<uniqueVertices;i++){
		for(int k=0;k<maxBots;k++){
			adjacencyMatrix[k][i+2][1]=0;
			adjacencyMatrix[k][1][i+2]=0;
			adjacencyMatrix[k][i+2][0]=0;
			adjacencyMatrix[k][0][i+2]=0;
		}

		xR1=muX;
		xR2=obstacleMapPoints.points[i].x;
		yR1=muY;
		yR2=obstacleMapPoints.points[i].y;
		flag=false;

		for (int k=0;k<obstacleLineSize;k++){

			xO1=obstacleLineList[k].x[0];
			yO1=obstacleLineList[k].y[0];
			xO2=obstacleLineList[k].x[1];
			yO2=obstacleLineList[k].y[1];

			if((thresholdDist(xR1,xO1) && thresholdDist(yR1,yO1)) || (thresholdDist(xR1,xO2) && thresholdDist(yR1,yO2)) || (thresholdDist(xR2,xO1) && thresholdDist(yR2,yO1)) || (thresholdDist(xR2,xO2) && thresholdDist(yR2,yO2))){

			}else{
				if (segmentIntersection(xR1,xR2,yR1,yR2,xO1,xO2,yO1,yO2)){
					flag=true;
				}

			}
		}

				if (obstacleMapPoints.points[i].linkSize>1){
					if (concaveTest(obstacleMapPoints.points[i].angle[0],obstacleMapPoints.points[i].angle[1],returnTwoVectorAngle(xR2, yR2, xR2 + 1, yR2, xR1, yR1))){
						flag=true;
					}
				}

		if(flag==false){
			for(int k=0;k<maxBots;k++){
				adjacencyMatrix[k][i+2][1]=1;
				adjacencyMatrix[k][1][i+2]=1;
			}
		}

	}


	xR1=0;
	xR2=0;
	yR1=0;
	yR2=0;


	for(int j=0;j<maxBots;j++) {

		xR1 = xValues[selectedIndices[j]];
		yR1 = yValues[selectedIndices[j]];
		adjacencyMatrix[j][1][0] = 0;
		adjacencyMatrix[j][0][1] = 0;
		adjacencyMatrix[j][1][0] = 0;
		adjacencyMatrix[j][0][1] = 0;

		for (int i = 0; i < uniqueVertices + 1; i++) {

			adjacencyMatrix[j][i + 2][0] = 0;
			adjacencyMatrix[j][0][i + 2] = 0;
			adjacencyMatrix[j][i + 2][0] = 0;
			adjacencyMatrix[j][0][i + 2] = 0;

			if (i == uniqueVertices) {
				xR2 = muX;
				yR2 = muY;
			} else {
				xR2 = obstacleMapPoints.points[i].x;
				yR2 = obstacleMapPoints.points[i].y;
			}

			flag = false;


			for (int k = 0; k < obstacleLineSize; k++) {

				xO1 = obstacleLineList[k].x[0];
				yO1 = obstacleLineList[k].y[0];
				xO2 = obstacleLineList[k].x[1];
				yO2 = obstacleLineList[k].y[1];


				if ((thresholdDist(xR1, xO1) && thresholdDist(yR1, yO1)) || (thresholdDist(xR1, xO2) && thresholdDist(yR1, yO2)) || (thresholdDist(xR2, xO1) && thresholdDist(yR2, yO1)) || (thresholdDist(xR2, xO2) && thresholdDist(yR2, yO2))) {

				} else {
					if (segmentIntersection(xR1, xR2, yR1, yR2, xO1, xO2, yO1, yO2)) {
						flag = true;
					}
				}
			}

					if (i != uniqueVertices) {
						if (obstacleMapPoints.points[i].linkSize > 1) {
							if (concaveTest(obstacleMapPoints.points[i].angle[0], obstacleMapPoints.points[i].angle[1], returnTwoVectorAngle(xR2, yR2, xR2 + 1, yR2, xR1, yR1))) {
								flag = true;
							}
						}
					}

			if (flag == false) {
				if (i == uniqueVertices) {
					adjacencyMatrix[j][1][0] = 1;
					adjacencyMatrix[j][0][1] = 1;
				} else {
					adjacencyMatrix[j][i + 2][0] = 1;
					adjacencyMatrix[j][0][i + 2] = 1;
				}
			}

		}
	}



/*
	for (int i=0;i<1;i++){

cout << "Adjacency Matrix: \n";
		for (int j=0;j<15;j++){
		cout << "mp" << obstacleMapPoints.points[j].x << " : " << obstacleMapPoints.points[j].y << "\n";}

		for (int j=0;j<15;j++){
			for (int k=0;k<15;k++){
			cout << " " << adjacencyMatrix[i][k][j];
			}
		cout << " ; \n";
		}
		cout << "\n\n";
	}
*/

}



void updatePreviousPath(){
//float dist(float x1,float x2,float y1,float y2){
//	for (int i=0;i<1;i++){


//		if (previousPathLine[i].size==0 || dist(turtleX,previousPathLine[i].points[previousPathLine[i].size-2].x,turtleY,previousPathLine[i].points[previousPathLine[i].size-2].y)>dist(turtleX,pathLine[i].points[pathLine[i].size-2].x,turtleY,pathLine[i].points[pathLine[i].size-2].y)){
//			previousPathLine[i]=pathLine[i];
//		}
//	}

//if (pathLine[0].size>2){
//previousPathLine[0]=pathLine[0];  //UNCOMMENT AFTER TESTING ~STABLE  <5
//if(pathLine[0].size>4){
//previousPathLine[0].points[3]=pathLine[0].points[pathLine[0].size-1];
//previousPathLine[0].points[2]=pathLine[0].points[pathLine[0].size-2];
//previousPathLine[0].points[1]=pathLine[0].points[pathLine[0].size-3];
//previousPathLine[0].points[0]=pathLine[0].points[pathLine[0].size-4];

//}else{
for (int i=0;i<5;i++){
previousPathLine[i]=pathLine[i];
}

//}

//}


}

void Dijkstra() {

	int c=0;
	float x1,x2;
	float y1,y2;
	int u;

	pointXY source[maxBots];
	pointXY goal ;
	goal.x=muX;
	goal.y=muY;
	goal.findUniqueID();
	pointXYList setQ;
	pointXYList setK;


	float compareDist=10000;
	float tempDist=0;

	int setSize = obstacleMapPoints.size + 2;
	float   distArray[setSize];
	int     prev[setSize];



	for (int i=0;i<100;i++){
		pathLine[i].size=0;
	}


	//?int path[]= new int[setSize];

//        Map<Integer, pointXY> Q = new HashMap<>();
//        Map<Integer, pointXY> K = new HashMap<>();
//        Set<Integer> setQ = new HashSet<>();

	for (int i=0;i<setSize;i++){
		distArray[i]=100000;
		prev[i]=-1;
		path[i]=0;
	}






        for (int j =0; j<maxBots;j++){

int checkSize=0;
        //CREATE SET OF VERTICES Q
		setQ.clearList();
		setK.clearList();
                distArray[1]=100000;
                prev[0]=-1; //-1 ~ UNDEFINED
                prev[1]=-1;
                u=-1;

		source[j].x=xValues[selectedIndices[j]];
		source[j].y=yValues[selectedIndices[j]];
		source[j].findUniqueID();

		for (int i = 2; i < setSize; i++) {
			setQ.points[i]=obstacleMapPoints.points[i-2];
			setQ.active[i]=true;
			setQ.size++;
			prev[i]=-1;
			distArray[i]=1000000;
		}

		setQ.points[0]=source[j];
		setQ.active[0]=true;
		setQ.points[1]=goal;
		setQ.active[1]=true;
		setQ.size++;
		setQ.size++;
		distArray[0]=0;
		setK=setQ;


                //MAIN

                while(setQ.size>0){
                    //Find min u
                    compareDist=10000;
                    u=1;

                    for (int i=0;i<setSize;i++){
                        if(distArray[i]<compareDist  &&  setQ.active[i]==true){
                            u=i;
                            compareDist=distArray[i];
                        }
                    }

                    setQ.active[u]=false;

                    if(u==1){
                        break;
                    }

		for (int v=0;v<setSize;v++){
			if (adjacencyMatrix[j][v][u]==1){
				tempDist = distArray[u]+dist(setQ.points[u].x,setQ.points[v].x,setQ.points[u].y,setQ.points[v].y);
				if(tempDist < distArray[v]){
					distArray[v]=tempDist;
					prev[v]=u;	
				}
			}
		}


                }


                //Output the Shortest path

                pathLine[j].clearList();

                int kk=0;


//cout << "begin insert \n";
		while (prev[u]!=-1){
//			pathLine[j].points[pathLine[j].size]=setQ.points[u];

//			pathLine[j].size++;

			path[kk]=u;
			kk++;
			u=prev[u];
		}

//			pathLine[j].points[pathLine[j].size]=setQ.points[u];
//			pathLine[j].size++;
			path[kk]=u;
			kk++;


pathLine[j].size=kk;


for (int z=0;z<kk;z++){
pathLine[j].points[z]=setQ.points[path[z]];
	//cout << path[z];
	//cout << "inserting: " << setQ.points[path[z]].x << "\n";
if (j==0){
cout <<path[z]<< ",  "; 
}
}



//cout << "Shortest Path: \n";
//	for (int i=0;i<pathLine[j].size;i++){
//		cout << " ~: " << path[i] <<"\n";
//	}

}

for (int kk=0;kk<pathLine[0].size;kk++){
	cout << "Index: " << kk << ", x: " << pathLine[0].points[kk].x << ", y: " << pathLine[0].points[kk].y << "\n"; 
}

updatePreviousPath();
}




	


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////END MAPPING PACKAGE

void obstacleCallback(const geometry_msgs::PoseStamped::ConstPtr& obsPtr){

/*
	//cout << "obstacle callback start \n";
	obstacleLineList[obstacleLineSize].x[0]=obsPtr->pose.position.x;
	obstacleLineList[obstacleLineSize].x[1]=obsPtr->pose.orientation.x;
	obstacleLineList[obstacleLineSize].y[0]=obsPtr->pose.position.y;
	obstacleLineList[obstacleLineSize].y[1]=obsPtr->pose.orientation.y;

	obstacleLineSize=obstacleLineSize+1;
	//cout << "obstacle callback end \n";
	toMapPoints();
	calculateAdjacencyMatrix();
*/

//OBSOLETE

}

void obstacleCallback2(const tf2_msgs::TFMessage::ConstPtr& posePtr ){





	if (posePtr -> transforms[0].transform.rotation.w==301){
	obstacleLineSize=0;

	for (int i=0;i< posePtr->transforms[0].transform.rotation.x;i++){

		obstacleLineList[i].x[0]=posePtr->transforms[i].transform.translation.x;
		obstacleLineList[i].x[1]=posePtr->transforms[i].transform.translation.z;
		obstacleLineList[i].y[0]=posePtr->transforms[i].transform.translation.y;
		obstacleLineList[i].y[1]=posePtr->transforms[i].transform.rotation.y;

	}


	obstacleLineSize=posePtr->transforms[0].transform.rotation.x;
	toMapPoints();
	calculateAdjacencyMatrix();

	}
}

void poseCallback(const tf2_msgs::TFMessage::ConstPtr& pose)
{
	countD=0;
	//centroidPositions = *pose;
	gotPose=true;
	const tf2_msgs::TFMessage& msg=*pose;

	for (int i=0;i<maxNum;i++)	
	{
		if ( (msg.transforms[i].transform.translation.x!=0 || msg.transforms[i].transform.translation.y!=0)){// && msg.transforms[i].transform.translation.x > minX & msg.transforms[i].transform.translation.x < maxX && msg.transforms[i].transform.translation.y < maxY && msg.transforms[i].transform.translation.y > minY
			xValues[i]=msg.transforms[i].transform.translation.x;	
			yValues[i]=msg.transforms[i].transform.translation.y;
			selectedIndices[countD]=i;	
			countD++;		
		}
	}
	toMapPoints();
	calculateAdjacencyMatrix();
	calculateAdjacencyMatrixGoal();
	Dijkstra();
}

void gCallback(const geometry_msgs::PoseArray::ConstPtr& gaussPtr)
{
		trackingFlag=false;
		if (gaussPtr->poses[0].position.z!=0){
		sigma=gaussPtr -> poses[0].position.z;
		muX=gaussPtr-> poses[0].position.x;
		muY=gaussPtr-> poses[0].position.y;
		if (gaussPtr->poses[0].orientation.w==1){
		trackingFlag=true;
		}
		}

	if (freeBoundaryFlag==true){
	minX=-1+muX;
	maxX=1+muX;

	minY=-1+muY;
	maxY=1+muY;
	}
	else{
	minX=StandardminX;
	maxX=StandardmaxX;

	minY=StandardminY;
	maxY=StandardmaxY;
	}

}

Matrix uniqueVertices(50,2);

Matrix::Matrix(int r, int c, float data) {
    rows = r;
    cols = c;
    elements= new float*[rows];
    for (int i=0; i<rows; i++)
    {
        elements[i] = new float[cols];
        for (int j=0; j < cols; j++)    {   elements[i][j]=data;    }
    }
}
Matrix::~Matrix(){
}
void Matrix::setMatrix(int r, int c, float data) {
    elements = new float*[r];
    for (int i=0; i<r; i++)
    {
        elements[i] = new float[c];
        for (int j=0; j<c; j++)
        {   elements[i][j]=data;  }
    }
    setRows(r);
    setCols(c);
}
float Matrix::getElement(int r, int c){
return elements[r][c];
}


void Matrix::printArray(string name) {
    cout << name << ": " << endl;
    for(int i=0;i<rows;i++) {
        for(int j=0;j<cols;j++) {   cout << elements[i][j] << "\t";   }
        cout << endl;
    }
    cout << endl;
}
int Matrix::setRows(int r) {
    rows = r;
    return rows;
}
int Matrix::setCols(int c) {
    cols = c;
    return cols;
}
void Matrix::setElement(int r, int c, float data){
    elements[r][c]=data;
}

float CoMGenerator::truncate(float num, int precision)        //truncate a floating point number
{   num = num*pow(10,precision);
    num = round(num);
    num = num/pow(10,precision);
    return num;
}

float CoMGenerator::distance(float siteX, float siteY, float vertX, float vertY)      //calculate the distance between a site and a vertice
{   float dist = sqrt(pow((vertY-siteY),2)+pow((vertX-siteX),2));
    dist = truncate(dist,3);        //using 4 as precision was identified some errors in calculations. So use 3 or less as precision
    return dist;
}


CoMGenerator::~CoMGenerator(){}

void CoMGenerator::getUniqVertices(std::vector<float> allVerticesVector)
{

    for (int i=0; i<allVerticesVector.size(); i++)  //as numbers are type float, need to truncate them in order to work
    {   allVerticesVector[i]=truncate(allVerticesVector[i], 3); }
    
    Matrix allVerticesMatrix(50,2); //initialize the matrix with 50 rows, but it will be replaced by the next line. So instead of 50, it can be any number
    allVerticesMatrix.rows = allVerticesVector.size()/2;
    
    //transform vector "allVerticesVector into a matrix
    int k=0;
    for (int i=0; i<allVerticesMatrix.rows; i++)
    {
        for (int j=0; j<2; j++) {    allVerticesMatrix.setElement(i,j,allVerticesVector[k]); k++;    }
    }
    //allVerticesMatrix.printArray("All Vertices");
    
    std::vector<int> repeatIndex;    //vector to store the indices(rows) of repeated vertices
    
    //find indices(rows) that repeats its elements == vertices that appear more than once
    for (int i=0; i<allVerticesMatrix.rows; i++){
        for (int j=i+1; j<allVerticesMatrix.rows; j++) {
            if (allVerticesMatrix.elements[i][0] == allVerticesMatrix.elements[j][0] && allVerticesMatrix.elements[i][1] == allVerticesMatrix.elements[j][1])
            {
                repeatIndex.push_back(i);       //if elements of a row repeats, add its index in the vector
                j=allVerticesMatrix.rows;       //break loop once it's found, otherwise if it repeats more than once, it'll store same index twice
            }
        }
    }
    
    //store non repeated vertices in the "uniqueVertices" matrix
    uniqueVertices.setRows(allVerticesMatrix.rows-repeatIndex.size());
    int a=0;
    for (int i=0; i<allVerticesMatrix.rows; i++) {
        int count=0;
        for (int j=0; j<repeatIndex.size(); j++) //make sure the index i is different from all indices storage in repeatIndex
        {
            if (i!=repeatIndex[j])  count++;
        }
        if (count==repeatIndex.size())   //if index 'i' isn't storage in repeatIndex: we can store the values of row (i) in uniqueVertices
        {
            uniqueVertices.setElement(a, 0, allVerticesMatrix.elements[i][0]);
            uniqueVertices.setElement(a, 1, allVerticesMatrix.elements[i][1]);
            a++;
        }
    }
    //uniqueVertices.printArray("Unique Vertices");
}

void CoMGenerator::getIndices(Matrix indicesMatrix, Matrix Sites, int nSites, int nVertices)
{
    const float E = 0.002;  //error value
    
    //find out which vertice is part of which cell
    for (int i=1; i<nSites; i++) {
        for (int j=1; j<=nVertices; j++) {             //go through each vertice to find what site is nearer to that vertice
            if (indicesMatrix.elements[i-1][j-1]<0) {  //check if element of matrix Indices is still with initial value -1, avoiding unecessary calculations
                int p=-1;
                for (int k=i; k<nSites; k++){          //compare the site (i) to every other site
                    float a = distance(Sites.elements[i-1][0], Sites.elements[i-1][1], uniqueVertices.elements[j-1][0], uniqueVertices.elements[j-1][1]);
                    float b = distance(Sites.elements[k][0], Sites.elements[k][1], uniqueVertices.elements[j-1][0], uniqueVertices.elements[j-1][1]);
                    if (a < b-E)    //if distance from site (i) to vertice (j) is smaller than from site (k) to vertice (j): vertice is not part of cell (k)
                    {   indicesMatrix.setElement(k, j-1, 0);  }
                    else if (a <= b+E && a >= b-E)  //if distances between two sites and a vertice are equal: vertice is part of both cells
                    {
                        indicesMatrix.setElement(i-1, j-1, j);
                        indicesMatrix.setElement(k, j-1, j);
                        p=k;    //store the value of indice (k), in case that the distance is equal but the cell is not the closest to the vertice (j)
                    }
                    else        //if distance from site (i) to vertice (j) is bigger than from site (k): vertice is not part of cell (i)
                    {
                        indicesMatrix.setElement(i-1, j-1, 0);
                        if (p>0) indicesMatrix.setElement(p, j-1, 0);   //if two sites have same distance to a vertice, but their not the closest to that vertice, it is used to correct that failure
                        k=nSites+1; //break loop, otherwise it will go back and compare the site (i) again with other sites, which is unnecessary
                    }
                }
            }
        }
    }
    for (int i=0; i<nSites; i++)    //some elements will still have the value -1. It means that those vertices are actually the plane's edges.
    {
        for (int j=0; j<nVertices; j++)
        {   if (indicesMatrix.elements[i][j]<0) indicesMatrix.setElement(i, j, j+1); }
    }                          //(they're only part of one single cell)
    

    //indicesMatrix.printArray("Indices");

}

void CoMGenerator::setIndicesOrder(Matrix angleMatrix, Matrix indicesMatrix, int nVertices, int row)
{
    double a;
    int b;
    for (int i=0; i<nVertices; i++){
        for (int j=(i+1); j<nVertices; j++){
            if (angleMatrix.elements[row][i] > angleMatrix.elements[row][j]) {
                a = angleMatrix.elements[row][i];                       //This part make swaps, ordering least to greatest
                b = indicesMatrix.elements[row][i];                     //for the angles, and aligns corrsponding indices
                angleMatrix.elements[row][i] = angleMatrix.elements[row][j];
                indicesMatrix.elements[row][i] = indicesMatrix.elements[row][j];
                angleMatrix.elements[row][j] = a;
                indicesMatrix.elements[row][j] = b;
            }
        }
    }
}

void CoMGenerator::sortIndices(Matrix sitesPosition, Matrix indicesMatrix, int nSites, int nVertices, Matrix angleMatrix)
{
    double xyVector[2];
    for (int i=0; i<nSites; i++){
        for (int j=0; j<nVertices; j++){
            if (indicesMatrix.elements[i][j] != 0) {
                for (int k=0; k<2; k++) //xyVector[0] = xVertice-xSite  and xyVector[1] = yVertice-ySite
                {
                    xyVector[k] = ( uniqueVertices.elements[ int(indicesMatrix.elements[i][j]-1) ][k] - sitesPosition.elements[i][k] );
                }
                angleMatrix.elements[i][j] = atan2 (xyVector[1],xyVector[0]) * (180.00 / PI);
            }
        }
        setIndicesOrder(angleMatrix, indicesMatrix, nVertices, i);
    }
    //indicesMatrix.printArray("Indices in Order");
    //angleMatrix.printArray("Angle Matrix");
}

void CoMGenerator::getLocalIndex(Matrix tempMatrix, int nVertices, int row, vector<int>& localIndex, int& noLocVertices)
{
    for (int j=0; j<nVertices; j++)     //store indices different from zero
    {
        if (tempMatrix.elements[row][j] != 0) localIndex.push_back (tempMatrix.elements[row][j]);
    }
    localIndex.push_back(localIndex.front());   //at the end, store the first element from the vector again
    noLocVertices = int(localIndex.size());
}

// Gets the center of mass for any triangle
void CoMGenerator::getTriangleCOM(double &Mass, double &c_x, double &c_y, double xpoints[3], double ypoints[3], int nGaussPoints)
{
cout << "COM RUNNING\n";
    //Gauss Info
    double rGauss[6] = {.4459,.4459,.1081,.0916,.0916,.8168,};
    double sGauss[6] = {.4459,.1081,.4459,.0916,.8168,.0916};
    double w[6] 	       = {.2234,.2234,.2234,.1099,.1099,.1099};
    nGaussPoints=6;
    
    //Local Coordinates
    double r[3] = {0, 1, 0};
    double s[3] = {0, 0, 1};
    
    //cout << "X: [" << xpoints[0] << ", " << xpoints[1] << ", " << xpoints[2] << "]" << endl;
    //cout << "Y: [" << ypoints[0] << ", " << ypoints[1] << ", " << ypoints[2] << "]" << endl;
    
    //Determinant in local cooridnates (r,s)
    double detM = ( 1.0*(r[1]*s[2]-s[1]*r[2]) + r[0]*(1.0*s[2]-1.0*s[1]) + s[0]*(1.0*r[2]-r[1]*1.0) );
    //Determinant in parent coordiantes (x,y)
    double detK = ( 1.0*(xpoints[1]*ypoints[2]-ypoints[1]*xpoints[2]) + xpoints[0]*(1.0*ypoints[1]-1.0*ypoints[2]) +
                   ypoints[0]*(1.0*xpoints[2]-xpoints[1]*1.0) );
    
    //Set Loop Sums to Zero
    double E_x = 0;
    double E_y = 0;
    Mass = 0;
    
    //Declare Variables
    double N1, N2, N3;
    double x, y;
    double DENSITY;
	float dist;
    
    for (int i=0; i<nGaussPoints; i++){
        //Shape functions in local coordinates (r,s)
        N1 = detM*(r[1]*s[2] - r[2]*s[1] + (s[1]-s[2])*rGauss[i] + (r[2]-r[1])*sGauss[i]);
        N2 = detM*(r[2]*s[0] - r[0]*s[2] + (s[2]-s[0])*rGauss[i] + (r[0]-r[2])*sGauss[i]);
        N3 = detM*(r[0]*s[1] - r[1]*s[0] + (s[0]-s[1])*rGauss[i] + (r[1]-r[0])*sGauss[i]);
        //Mapping from local (r,s) to parent (x,y) coordinates
        x = N1*xpoints[0] + N2*xpoints[1] + N3*xpoints[2];
        y = N1*ypoints[0] + N2*ypoints[1] + N3*ypoints[2];
        
	        


        //Density Funtion!!!!!
        //||----------------||
        //||----------------||sigma*(3-sqrt((x-muX)*(x-muX)+(y-muY)*(y-muY)))/1000000000+
//FIX FOR ALL TURTLEBOTS
//	dist=calculateDistance(x,geodesicPath[0].x[geodesicPath[0].size-1],y,geodesicPath[0].y[geodesicPath[0].size-1]);
updatePreviousPath();

if (previousPathLine[currentIndex].size>2){
cout << "check \n" <<previousPathLine[currentIndex].points[previousPathLine[currentIndex].size-2].x;
dist=calculateDistance2(x,muX,y,muY, turtleX, turtleY,previousPathLine[currentIndex].points[previousPathLine[currentIndex].size-2], previousPathLine[currentIndex].points[previousPathLine[currentIndex].size-3]);
//cout << "TRAVERSE: " <<  " : " << (path[pathLine[0].size-2]) <<  "  x: " << (pathLine[0].points[pathLine[0].size-1].x) << " y: "<<(pathLine[0].points[pathLine[0].size-1].y) <<"\n";
//cout << "PathLine Size: "<< pathLine[0].size <<", AT: " << pathLine[0].points[0].x << ", OBSTACLE: " << pathLine[0].points[1].x << ", GOAL: " << pathLine[0].points[2].x<<"\n";
}else{
dist=calculateDistance2(x,muX,y,muY, turtleX, turtleY,previousPathLine[currentIndex].points[previousPathLine[currentIndex].size-1], previousPathLine[currentIndex].points[previousPathLine[currentIndex].size-2]);
//cout << "TRAVERSE: " << (path[pathLine[0].size-2]) << " : " << (pathLine[0].size-2) << "\n";
//cout << "PathLine Size: "<< pathLine[0].size <<", AT: 0, OBSTACLE: " << (path[pathLine[0].size-2]) << ", GOAL: " << (path[pathLine[0].size-2])<<"\n";
}







	DENSITY =1/(sigma*sqrt(2*3.14))  *  exp(-dist/(2*pow(sigma,2))); 
	   //DENSITY = -1*(sqrt((x-muX)*(x-muX)+(y-muY)*(y-muY)));
	//DENSITY = 1/(sqrt((x-muX)*(x-muX)+(y-muY)*(y-muY)));
//cout << DENSITY << "\n\n";
		//DENSITY=1;
        //||----------------||
        //||----------------||
        
        //Apriori Expected Values and Mass
        E_x = E_x + w[i]*x*DENSITY;
        E_y = E_y + w[i]*y*DENSITY;
        Mass = Mass + w[i]*DENSITY;
    }
    //cout << "DETK: " << detK << endl;
    //Expected Values and Mass With Jacobian
    E_x = (detK/2.0)*E_x;
    E_y = (detK/2.0)*E_y;
    Mass = (detK/2)*Mass;
    
    //cout << "Ex, Ey: " << E_x << ", " << E_y << endl;
    //cout << "Mass: " << Mass << endl;
    
    //Center of Mass
    c_x = E_x/Mass;
    c_y = E_y/Mass;
    
    //cout << "\tCx, Cy: " << c_x << ", " << c_y << endl;
}

//Get the center of mass for any composite
void CoMGenerator::getCompositeCOM(double &c_x, double &c_y, double x[], double y[], double Mass[], int nLocalVertices)
{
    //Set to Zero
    double totalMass = 0;
    c_x = 0;
    c_y = 0;
    
    for(int i=0; i<nLocalVertices; i++){
        totalMass = totalMass + Mass[i];
        c_x = c_x + x[i]*Mass[i];
        c_y = c_y + y[i]*Mass[i];
    }
    
    //cout << "c_x, c_y: " << c_x << ", " << c_y << endl;
    
    c_x = c_x/totalMass;
    c_y = c_y/totalMass;
}

//Gets the center of mass for Cell
void CoMGenerator::getCenterOfMass(double &Mass, double &c_x, double &c_y, Matrix localVertices, double Site[2], int nLocalVertices, int nGaussPoints, Matrix centerOfMass, int row)
{
cout << "GET CENTER OF MASS\n";
    //Declare Variables
    int k = (nLocalVertices-1);
    double MassVector[k], c_xVector[k], c_yVector[k];
    double xpoints[3], ypoints[3];
    
    //Collect COM's -MODDED BY AARON
    xpoints[2] = Site[0];   
    ypoints[2] = Site[1];
	turtleX=Site[0];
	turtleY=Site[1];
	currentIndex=row;
	double cXp=0;
	double cYp=0;
	
	for (int i=0;i<nLocalVertices;i++){
		cXp=cXp+localVertices.elements[i][0];
		cYp=cYp+localVertices.elements[i][1];
	}
	xpoints[2]=cXp/nLocalVertices;
	ypoints[2]=cYp/nLocalVertices;


    for(int i=0; i<k; i++){
        //Form Triangle
        xpoints[0] = localVertices.elements[i][0];  ypoints[0] = localVertices.elements[i][1];
        xpoints[1] = localVertices.elements[i+1][0];    ypoints[1] = localVertices.elements[i+1][1];
        
        //Get Each Triangle's Center of Mass,poseCallbac
        getTriangleCOM(Mass, c_x, c_y, xpoints, ypoints, nGaussPoints);
        MassVector[i] = Mass;
        c_xVector[i] = c_x;
        c_yVector[i] = c_y;
    }
    getCompositeCOM(c_x, c_y, c_xVector, c_yVector, MassVector, nLocalVertices);
    centerOfMass.elements[row][0] = c_x;
    centerOfMass.elements[row][1] = c_y;
}













Matrix CoMGenerator::generateCenterOfMass(std::vector<float> allVertices, Matrix sitesPosition, int nSites)
{

//cout << "start \n";

				    getUniqVertices(allVertices);
					int newRowSize=0;
				//    uniqueVertices.setRows(15);
				//uniqueVertices.rows=15;

				    Matrix indicesMatrix(nSites, uniqueVertices.rows,-1); //initialize IndicesMatrix with value(-1). It'll store indices of vertices related to each site.

				    getIndices(indicesMatrix, sitesPosition, nSites, uniqueVertices.rows);

				    Matrix angleMatrix(nSites, uniqueVertices.rows);

				    sortIndices(sitesPosition, indicesMatrix, nSites, uniqueVertices.rows, angleMatrix);
//indicesMatrix.printArray("sorted");
					newRowSize=mergeObstacles(indicesMatrix, uniqueVertices);

//cout << newRowSize<< "newRowSize \n";
					 indicesMatrix.setCols(newRowSize);
Matrix tempMatrix(nSites, uniqueVertices.rows,-1); //initialize IndicesMatrix with value(-1). It'll store indices of vertices related to each site.
for (int j=0;j<countD;j++){
	//for (int i=0;i<voronoiCellBoundaries[j].size;i++){
	for (int i=0;i<uniqueVertices.rows;i++){
		flag=false;
		for (int k=0;k<uniqueVertices.rows &&flag==false;k++){
			if(voronoiCellBoundaries[j].x[i]==uniqueVertices.elements[k][0] && voronoiCellBoundaries[j].y[i]==uniqueVertices.elements[k][1] ){
				tempMatrix.setElement(j,i,k+1);
				flag=true;
			}
			if (flag==false){
				tempMatrix.setElement(j,i,0);
			}
		}

	}
}

//tempMatrix.printArray("afterSorted");
//uniqueVertices.printArray("vertices");
/*
Matrix tempMatrix(nSites, uniqueVertices.rows,-1); //initialize IndicesMatrix with value(-1). It'll store indices of vertices related to each site.




return tempMatrix;*/	

//cout << "checkpoint";
				    
				    Matrix centerOfMass(nSites,2);

	
				    
				    double Mass, c_x, c_y;
				    int nGaussPoints = 3;
				    
				    int nLocalVertices = 0;
cout << "nSites" << nSites<<"\n";
				    for (int siteNumber=0; siteNumber<nSites; siteNumber++){


					vector<int> localIndex;
		
					//Get Local Index: Supporting function. Returns: (localIndex, noLocIndices)
//					getLocalIndex(indicesMatrix, uniqueVertices.rows, siteNumber, localIndex, nLocalVertices);
					getLocalIndex(tempMatrix, uniqueVertices.rows, siteNumber, localIndex, nLocalVertices);
					Matrix localVertPos(nLocalVertices,2);

				/*
					cout << endl;
					cout << "DICTIONARY FOR SITE " << (siteNumber + 1) << ":" << endl;
					cout << "noLocVertices = "<< nLocalVertices << endl;
					cout << "localIndex    = ";

					for (int i=0; i<nLocalVertices; i++){
					    cout << localIndex[i] << ", ";
					}
					cout << endl;
				      */
					for (int i=0; i<nLocalVertices; i++){
					    localVertPos.elements[i][0]=uniqueVertices.elements[localIndex[i]-1][0];
					    localVertPos.elements[i][1]=uniqueVertices.elements[localIndex[i]-1][1];
					}
					//localVertPos.printArray("Local Vertices");
		
					//GET CENTER OF MASS: Returns (CenterOfMass)
					double Site[2]={sitesPosition.elements[siteNumber][0], sitesPosition.elements[siteNumber][1]};
		


					getCenterOfMass(Mass, c_x, c_y, localVertPos, Site, nLocalVertices, nGaussPoints, centerOfMass, siteNumber);
				    }
				    //cout << endl;
				    
				    //print CoM Matrix to work with Matlab
				/*
				    cout << "cmX=[";
				    for (int i=0; i<centerOfMass.rows; i++){
				    if (i==centerOfMass.rows-1) cout << centerOfMass.elements[i][0] << "];" << endl;
				    else cout << centerOfMass.elements[i][0] << ", "; }
				    cout << "cmY=[";
				    for (int i=0; i<centerOfMass.rows; i++){
				    if (i==centerOfMass.rows-1) cout << centerOfMass.elements[i][1] << "];" << endl;
				    else cout << centerOfMass.elements[i][1] << ", "; }
				    */
				    
				    //centerOfMass.printArray("Center of Mass");
				    

				    return centerOfMass;

}

// This function translates the list of transform messages that contain the centroids of the turtlebots
// into a list of PoseStamped messages required by move_base. This function uses a for loop that assumes
// the turtlebots are indexed in a consecutive and increasing order, with MICHELANGELO_INDEX being lowest
// and TITIAN_INDEX being highest.
void transform_to_pose(tf2_msgs::TFMessage centroidPositions, tf2_msgs::TFMessage cdots)
{
	for(int i = TITIAN_INDEX; i <= MASACCIO_INDEX; i++)
	{
		if (centroidPositions.transforms[i].transform.translation.x==centroidPositions.transforms[i].transform.translation.x && centroidPositions.transforms[i].transform.translation.y==centroidPositions.transforms[i].transform.translation.y){

					centroidPosesStamped[i].pose.position.x = centroidPositions.transforms[i].transform.translation.x;
			centroidPosesStamped[i].pose.position.y = -centroidPositions.transforms[i].transform.translation.y;
			centroidPosesStamped[i].pose.position.z = centroidPositions.transforms[i].transform.translation.z;
			centroidPosesStamped[i].pose.orientation = centroidPositions.transforms[i].transform.rotation;
			centroidPosesStamped[i].pose.orientation.w=-1;
			
			cDotTurtles[i].linear.x = cdots.transforms[i].transform.translation.x;
			cDotTurtles[i].linear.y = cdots.transforms[i].transform.translation.y;


		}


	}

	for(int i = PICASSO_INDEX; i <= GOYA_INDEX; i++)
	{
		centroidPosesStamped[i].pose.position.x = -1;
		centroidPosesStamped[i].pose.position.y = -1;
		centroidPosesStamped[i].pose.position.z = -1;
		centroidPosesStamped[i].pose.orientation = centroidPositions.transforms[i].transform.rotation;
		centroidPosesStamped[i].pose.orientation.w=-1;
		
		cDotTurtles[i].linear.x = cdots.transforms[i].transform.translation.x;
		cDotTurtles[i].linear.y = cdots.transforms[i].transform.translation.y;
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///

//THIS IS THE BOUNDARY EDITS

///

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////






vertexArray organizeList(vertexArray v){
        vertexArray t;
//cout <<"ORGANIZING!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!";
        float angle;
        int  z=-1;
        t.size=0;
        for (int i=0;i<v.size;i++){
            angle=360;
            for (int k=0;k<v.size;k++){
                if (v.angle[k]<angle && v.used[k]!=true){
                    z=k;
                    angle=v.angle[k];
                }

            }
            v.used[z]=true;
            t.x[t.size]=v.x[z];
            t.y[t.size]=v.y[z];

            t.angle[t.size]=v.angle[z];
            t.size=t.size+1;

        }

        return t;
    }


	float returnAngle(float turtleX, float turtleY, float X, float Y){
		return atan2( 1*(Y-turtleY) - 0*(X-turtleX) , 1*(X-turtleX)+0*(Y-turtleY));
	}



void generatebEquations(obstacleLine o, float x, float y, int k, int turtleIndex){

        float m=(o.y[1]-o.y[0])/(o.x[1]-o.x[0]);
        float tX=(o.y[0]-y-x/m-o.x[0]*m)/((-1/m)-m);
	float tY;


	//cout <<"starting gen b equations \n";
        if (tX<o.x[0]&&tX<o.x[1]){
            if (o.x[0]<o.x[1]){
                tX=o.x[0];
            }
            else{
                tX=o.x[1];
            }
        }
        else if (tX>o.x[0]&&tX>o.x[1]){
            if (o.x[0]>o.x[1]){
                tX=o.x[0];
            }
            else{
                tX=o.x[1];
            }
        }
        tY=m*(tX-o.x[0])+o.y[0];

        m=(y-tY)/(x-tX);
        y=tY+.05*(y-tY)/(sqrt((y-tY)*(y-tY)+(x-tX)*(x-tX)));
        x=tX+.05*(x-tX)/(sqrt((y-tY)*(y-tY)+(x-tX)*(x-tX)));


        bEquations[turtleIndex].x[k]=x;
        bEquations[turtleIndex].y[k]=y;
        bEquations[turtleIndex].m[k]=-1/m;
        bEquations[turtleIndex].active[k]=true;
//	cout << "m1: " <<  bEquations[turtleIndex].m[k] << "\n";
    }






int CoMGenerator::mergeObstacles(Matrix indicesMatrix, Matrix uniqueV){

//uniqueVertices.printArray("dkslaf");

        for (int j=0;j<countD;j++){
//cout << "turtle : " <<xValues[selectedIndices[j]]<<", "<< yValues[selectedIndices[j]]<<"\n";
		tempMax=0;
		for (int k=0;k<uniqueVertices.rows;k++){
			if (indicesMatrix.elements[j][k]!=0){
//NOT SURE IF ELEMENT 1 iS X OR Y

				voronoiCellBoundaries[j].x[tempMax]=uniqueVertices.elements[(int)indicesMatrix.elements[j][k]-1][0];
				voronoiCellBoundaries[j].y[tempMax]=uniqueVertices.elements[(int)indicesMatrix.elements[j][k]-1][1];
//cout << "Index Matrix Element" <<uniqueVertices.elements[(int)indicesMatrix.elements[j][k]][1]<<"\n";
				voronoiCellBoundaries[j].used[tempMax]=false;
				voronoiCellBoundaries[j].angle[tempMax]=returnAngle(xValues[selectedIndices[j]],yValues[selectedIndices[j]],voronoiCellBoundaries[j].x[tempMax],voronoiCellBoundaries[j].y[tempMax]);
//		cout << " voronoi XD: " << voronoiCellBoundaries[j].x[tempMax] << "\n";
//		cout << " voronoi YD: " << voronoiCellBoundaries[j].y[tempMax] << "\n";
				tempMax=tempMax+1;
				voronoiCellBoundaries[j].size=tempMax;
			}
		}
}	



        for (int j=0;j<countD;j++){

		

                for (int i=0;i<obstacleLineSize;i++){
			tempMax=voronoiCellBoundaries[j].size;
                    for (int k=0;k<tempMax;k++){
                        if (k==0){
                            tempVX=segmentLineintersection(bEquations[j].m[i], bEquations[j].x[i], bEquations[j].y[i], voronoiCellBoundaries[j].x[k], voronoiCellBoundaries[j].x[voronoiCellBoundaries[j].size - 1], voronoiCellBoundaries[j].y[k], voronoiCellBoundaries[j].y[voronoiCellBoundaries[j].size - 1]);



/*		cout << "VERTEX FOUND" << tempVX <<"\n";
		cout << "tempMax" << tempMax<<"\n";
		cout << "voronoi x: " << voronoiCellBoundaries[j].x[k];
		cout << " voronoi x2: " << voronoiCellBoundaries[j].x[tempMax-1] << "\n";
		cout << "voronoi y: " << voronoiCellBoundaries[j].y[k];
		cout << " voronoi y2: " << voronoiCellBoundaries[j].y[tempMax-1] << "\n";
*/
                            if (tempVX!=-100){
                                voronoiCellBoundaries[j].x[voronoiCellBoundaries[j].size]=tempVX;
                                voronoiCellBoundaries[j].y[voronoiCellBoundaries[j].size]=findYIntercept(bEquations[j].m[i], tempVX, bEquations[j].x[i], bEquations[j].y[i]);
                                voronoiCellBoundaries[j].used[voronoiCellBoundaries[j].size]=false;
                                voronoiCellBoundaries[j].angle[voronoiCellBoundaries[j].size]=returnAngle(xValues[selectedIndices[j]],yValues[selectedIndices[j]],voronoiCellBoundaries[j].x[voronoiCellBoundaries[j].size],voronoiCellBoundaries[j].y[voronoiCellBoundaries[j].size]);
                                voronoiCellBoundaries[j].size=voronoiCellBoundaries[j].size+1;
				uniqueVertices.setRows(uniqueVertices.rows+1);
				uniqueVertices.setElement(uniqueVertices.rows-1,0,tempVX);
				uniqueVertices.setElement(uniqueVertices.rows-1,1,voronoiCellBoundaries[j].y[voronoiCellBoundaries[j].size-1]); 

                            }
                        }




                        else {


                            tempVX=segmentLineintersection(bEquations[j].m[i], bEquations[j].x[i], bEquations[j].y[i], voronoiCellBoundaries[j].x[k], voronoiCellBoundaries[j].x[k - 1], voronoiCellBoundaries[j].y[k], voronoiCellBoundaries[j].y[k - 1]);
//		cout << "VERTEX FOUND" << tempVX <<"\n";

			if (tempVX==0){
					cout << "k:!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! " << k << "\n";
				}
                            if (tempVX!=-100){
                                voronoiCellBoundaries[j].x[voronoiCellBoundaries[j].size]=tempVX;
                                voronoiCellBoundaries[j].y[voronoiCellBoundaries[j].size]=findYIntercept(bEquations[j].m[i], tempVX, bEquations[j].x[i], bEquations[j].y[i]);
                                voronoiCellBoundaries[j].angle[voronoiCellBoundaries[j].size]=returnAngle(xValues[selectedIndices[j]],yValues[selectedIndices[j]], voronoiCellBoundaries[j].x[voronoiCellBoundaries[j].size], voronoiCellBoundaries[j].y[voronoiCellBoundaries[j].size]);
                                voronoiCellBoundaries[j].used[voronoiCellBoundaries[j].size]=false;
                                voronoiCellBoundaries[j].size=voronoiCellBoundaries[j].size+1;
				uniqueVertices.setRows(uniqueVertices.rows+1);
				uniqueVertices.setElement(uniqueVertices.rows-1,0,tempVX);
				uniqueVertices.setElement(uniqueVertices.rows-1,1,voronoiCellBoundaries[j].y[voronoiCellBoundaries[j].size-1]); 

                            }
                        }




                    voronoiCellBoundaries[j].used[k]=false;

                    }








                    if (findYIntercept(bEquations[j].m[i],xValues[selectedIndices[j]],bEquations[j].x[i],bEquations[j].y[i])<yValues[selectedIndices[j]]){ //DELETE VERTICES BELOW EQUATION
                        flag=false;

                        for (int k=0;k<voronoiCellBoundaries[j].size;k++){

                            if (findYIntercept(bEquations[j].m[i],voronoiCellBoundaries[j].x[k],bEquations[j].x[i],bEquations[j].y[i])>voronoiCellBoundaries[j].y[k]){ //THE VORONOI POINT IS BELOW EQUATION
                                for (int g=k+1;g<voronoiCellBoundaries[j].size;g++){
                                    voronoiCellBoundaries[j].x[g-1]=voronoiCellBoundaries[j].x[g];
                                    voronoiCellBoundaries[j].y[g-1]=voronoiCellBoundaries[j].y[g];
                                    voronoiCellBoundaries[j].angle[g-1]=voronoiCellBoundaries[j].angle[g];
                                }
                                voronoiCellBoundaries[j].size=voronoiCellBoundaries[j].size-1;
                                k=k-1;
                                flag=true;
                            }


                        }
//cout << "turtle : " <<xValues[selectedIndices[j]]<<", "<< yValues[selectedIndices[j]]<<"\n";

                    }
                    else{
                        for (int k=0;k<voronoiCellBoundaries[j].size;k++){
                            if (findYIntercept(bEquations[j].m[i],voronoiCellBoundaries[j].x[k],bEquations[j].x[i],bEquations[j].y[i])<voronoiCellBoundaries[j].y[k]){ //THE VORONOI POINT IS ABOVE EQUATION
                                for (int g=k+1;g<voronoiCellBoundaries[j].size;g++){
                                    voronoiCellBoundaries[j].x[g-1]=voronoiCellBoundaries[j].x[g];
                                    voronoiCellBoundaries[j].y[g-1]=voronoiCellBoundaries[j].y[g];
                                    voronoiCellBoundaries[j].angle[g-1]=voronoiCellBoundaries[j].angle[g];

                                }
                                voronoiCellBoundaries[j].size=voronoiCellBoundaries[j].size-1;
                                k=k-1;

                            }

                        }


                    }
                    tempArray=voronoiCellBoundaries[j];
                    voronoiCellBoundaries[j]=organizeList(tempArray);
                }
        }

	//indicesMatrix (j) then sort through columns and set in right ordering

return uniqueVertices.rows;
}







void geodesicDistance(){


/*
CURRENT VERSION: Works for 1 obstacle only




		If there is an intersection between turtlebot and the obstacle, then find the shortest path~
			
			Check distance from both indices of obstacleCallback 			not done
			Check if the indices of the obstacle are total boundaries			not done
			Make an array [turtlebot index] that specifies goal position, and intermediate position
			
bool segmentIntersection(float x1, float x2, float y1,float y2, float x3, float x4, float y3, float y4){
*/


float dist1;
float dist2;

	for (int j=0;j<countD;j++){
		geodesicPath[j].size=1;
		geodesicPath[j].x[0]=muX;
		geodesicPath[j].y[0]=muY;
		for (int i=0;i<obstacleLineSize;i++){
			if (segmentIntersection(obstacleLineList[i].x[0], obstacleLineList[i].x[1],obstacleLineList[i].y[0], obstacleLineList[i].y[1],xValues[selectedIndices[j]],muX,yValues[selectedIndices[j]],muY)==true){
				dist1=sqrt((obstacleLineList[i].x[0]-muX)*(obstacleLineList[i].x[0]-muX)+(obstacleLineList[i].y[0]-muY)*(obstacleLineList[i].y[0]-muY));
				dist1=dist1+sqrt((obstacleLineList[i].x[0]-xValues[selectedIndices[j]])*(obstacleLineList[i].x[0]-xValues[selectedIndices[j]])+(obstacleLineList[i].y[0]-yValues[selectedIndices[j]])*(obstacleLineList[i].y[0]-yValues[selectedIndices[j]]));

				dist2=sqrt((obstacleLineList[i].x[1]-muX)*(obstacleLineList[i].x[1]-muX)+(obstacleLineList[i].y[1]-muY)*(obstacleLineList[i].y[1]-muY));
				dist2=dist2+sqrt((obstacleLineList[i].x[1]-xValues[selectedIndices[j]])*(obstacleLineList[i].x[1]-xValues[selectedIndices[j]])+(obstacleLineList[i].y[1]-yValues[selectedIndices[j]])*(obstacleLineList[i].y[1]-yValues[selectedIndices[j]]));


				if (dist1<dist2){
					geodesicPath[j].x[1]=obstacleLineList[i].x[0];
					geodesicPath[j].y[1]=obstacleLineList[i].y[0];
					if (obstacleLineList[i].x[0]<obstacleLineList[i].x[1]){
						geodesicPath[j].x[1]=geodesicPath[j].x[1]+.5*cos(atan(bEquations[j].m[i]));
						geodesicPath[j].y[1]=geodesicPath[j].y[1]+.5*sin(atan(bEquations[j].m[i]));
						
					}
					else{
						geodesicPath[j].x[1]=geodesicPath[j].x[1]-.5*cos(atan(bEquations[j].m[i]));
						geodesicPath[j].y[1]=geodesicPath[j].y[1]-.5*sin(atan(bEquations[j].m[i]));
					}

				cout << "DISTANCE 1";
				}
				else{
					geodesicPath[j].x[1]=obstacleLineList[i].x[1];
					geodesicPath[j].y[1]=obstacleLineList[i].y[1];

					if (obstacleLineList[i].x[0]<obstacleLineList[i].x[1]){
						geodesicPath[j].x[1]=geodesicPath[j].x[1]+.5*cos(atan(bEquations[j].m[i]));
						geodesicPath[j].y[1]=geodesicPath[j].y[1]+.5*sin(atan(bEquations[j].m[i]));
					}
					else{
						geodesicPath[j].x[1]=geodesicPath[j].x[1]-.5*cos(atan(bEquations[j].m[i]));
						geodesicPath[j].y[1]=geodesicPath[j].y[1]-.5*sin(atan(bEquations[j].m[i]));
					}
				cout << "DISTANCE 2";

				}

				geodesicPath[j].size=geodesicPath[j].size+1;
			}
		}
	}
}







int main(int argc, char **argv)
{
    
    	//Store the position of the sites in a Matrix
	tf2_msgs::TFMessage centroidPositions;
	tf2_msgs::TFMessage lastCentroidPositions;
	tf2_msgs::TFMessage cDot;
    	centroidPositions.transforms.resize(maxNum);
	lastCentroidPositions.transforms.resize(maxNum);
	cDot.transforms.resize(maxNum);
	geometry_msgs::PoseStamped ps;
    	int nSites = Matrix_Size(xValues);
    	Matrix sitesPos(nSites,2);
    	for(int i=0; i<Matrix_Size(xValues);i++){
        	sitesPos.setElement(i, 0, xValues[i]);      //sitePos.elements[i][0] = xValues[i];
        	sitesPos.setElement(i, 1, yValues[i]);
    	}

	for (int i=0;i<5;i++){
		for (int j=0;j<maxNum;j++){
			lastX[i][j]=0;
			lastY[i][j]=0;
			
		}
	}

	for (int i=0;i<maxNum;i++){
		weirdReadings[i]=0;
	}
    
	ros::init(argc, argv, "Centroid_Generator"); 
	ros::start();
	ros::Rate loop_rate(T); //Set Ros frequency to 50/s (fast)
	ros::NodeHandle nh_;
	ros::Subscriber pos_sub_ ,g_sub_,boundary_sub_;
	ros::Subscriber obstacle_sub;
	ros::Subscriber obstacle_sub2;
	ros::Publisher centroid_pub_ ;
	ros::Publisher cDot_pub_;
	ros::Publisher goal_for_android;

	// Declare turtlebot publishers
	ros::Publisher michel_pub,michel_pub_Android;
	ros::Publisher dona_pub,dona_pub_Android;
	ros::Publisher bern_pub,bern_pub_Android;
	ros::Publisher leo_pub,leo_pub_Android;
	ros::Publisher boti_pub,boti_pub_Android;
	ros::Publisher gio_pub,gio_pub_Android;
	ros::Publisher bel_pub, bel_pub_Android;
	ros::Publisher ghiber_pub, ghiber_pub_Android;
	ros::Publisher masa_pub,masa_pub_Android;
	ros::Publisher titi_pub, titi_pub_Android;
	ros::Publisher picasso_pub, picasso_pub_Android;
	ros::Publisher dali_pub, dali_pub_Android;
	ros::Publisher goya_pub, goya_pub_Android;
	

	ros::Publisher michel_pub_cdot;
	ros::Publisher dona_pub_cdot;
	ros::Publisher bern_pub_cdot;
	ros::Publisher leo_pub_cdot;
	ros::Publisher boti_pub_cdot;
	ros::Publisher gio_pub_cdot;
	ros::Publisher bel_pub_cdot;
	ros::Publisher ghiber_pub_cdot;
	ros::Publisher masa_pub_cdot;
	ros::Publisher titi_pub_cdot;
ros::Publisher picasso_pub_cdot;
ros::Publisher dali_pub_cdot;
ros::Publisher goya_pub_cdot;

	pos_sub_= nh_.subscribe<tf2_msgs::TFMessage>("/poseEstimationC", 1000,poseCallback);
	g_sub_= nh_.subscribe<geometry_msgs::PoseArray>("/gauss", 1000,gCallback);
	boundary_sub_= nh_.subscribe<std_msgs::Bool>("/gauss/boundaryFlag", 1000,boundaryCallback);
	obstacle_sub=nh_.subscribe<geometry_msgs::PoseStamped>("linearObstacle",1000,obstacleCallback);
	obstacle_sub2= nh_.subscribe<tf2_msgs::TFMessage>("/tf", 25,obstacleCallback2);

	centroid_pub_ = nh_.advertise<tf2_msgs::TFMessage>("Centroids", 1000, true);
	cDot_pub_ = nh_.advertise<tf2_msgs::TFMessage>("cDot", 1000, true);
	goal_for_android = nh_.advertise<geometry_msgs::PoseStamped>("/poseEstimation", 1000, true);
	// Initialize turtlebot publishers
	michel_pub = nh_.advertise<geometry_msgs::PoseStamped>("/michelangelo/move_base_simple/goal", 1000, true);
	dona_pub = nh_.advertise<geometry_msgs::PoseStamped>("/donatello/move_base_simple/goal", 1000, true);
	bern_pub = nh_.advertise<geometry_msgs::PoseStamped>("/raphael/move_base_simple/goal", 1000, true);
	leo_pub = nh_.advertise<geometry_msgs::PoseStamped>("/leonardo/move_base_simple/goal", 1000, true);
	boti_pub = nh_.advertise<geometry_msgs::PoseStamped>("/boticelli/move_base_simple/goal", 1000, true);
	gio_pub = nh_.advertise<geometry_msgs::PoseStamped>("/giotto/move_base_simple/goal", 1000, true);
	bel_pub = nh_.advertise<geometry_msgs::PoseStamped>("/bellini/move_base_simple/goal", 1000, true);
	ghiber_pub = nh_.advertise<geometry_msgs::PoseStamped>("/ghiberti/move_base_simple/goal", 1000, true);
	masa_pub = nh_.advertise<geometry_msgs::PoseStamped>("/masaccio/move_base_simple/goal", 1000, true);
	titi_pub = nh_.advertise<geometry_msgs::PoseStamped>("/titian/move_base_simple/goal", 1000, true);
picasso_pub = nh_.advertise<geometry_msgs::PoseStamped>("/picasso/move_base_simple/goal", 1000, true);
	dali_pub = nh_.advertise<geometry_msgs::PoseStamped>("/dali/move_base_simple/goal", 1000, true);
	goya_pub = nh_.advertise<geometry_msgs::PoseStamped>("/goya/move_base_simple/goal", 1000, true);

	michel_pub_Android = nh_.advertise<geometry_msgs::PoseStamped>("/poseEstimation", 1000, true);
	dona_pub_Android = nh_.advertise<geometry_msgs::PoseStamped>("/poseEstimation", 1000, true);
	bern_pub_Android = nh_.advertise<geometry_msgs::PoseStamped>("/poseEstimation", 1000, true);
	leo_pub_Android = nh_.advertise<geometry_msgs::PoseStamped>("/poseEstimation", 1000, true);
	boti_pub_Android = nh_.advertise<geometry_msgs::PoseStamped>("/poseEstimation", 1000, true);
	gio_pub_Android = nh_.advertise<geometry_msgs::PoseStamped>("/poseEstimation", 1000, true);
	bel_pub_Android = nh_.advertise<geometry_msgs::PoseStamped>("/poseEstimation", 1000, true);
	ghiber_pub_Android = nh_.advertise<geometry_msgs::PoseStamped>("/poseEstimation", 1000, true);
	masa_pub_Android = nh_.advertise<geometry_msgs::PoseStamped>("/poseEstimation", 1000, true);
	titi_pub_Android = nh_.advertise<geometry_msgs::PoseStamped>("/poseEstimation", 1000, true);
picasso_pub_Android = nh_.advertise<geometry_msgs::PoseStamped>("/poseEstimation", 1000, true);
	dali_pub_Android = nh_.advertise<geometry_msgs::PoseStamped>("/poseEstimation", 1000, true);
	goya_pub_Android = nh_.advertise<geometry_msgs::PoseStamped>("/poseEstimation", 1000, true);


	michel_pub_cdot = nh_.advertise<geometry_msgs::Twist>("/michelangelo/cdot", 1000, true);
	dona_pub_cdot = nh_.advertise<geometry_msgs::Twist>("/donatello/cdot", 1000, true);
	bern_pub_cdot = nh_.advertise<geometry_msgs::Twist>("/raphael/cdot", 1000, true);
	leo_pub_cdot = nh_.advertise<geometry_msgs::Twist>("/leonardo/cdot", 1000, true);
	boti_pub_cdot = nh_.advertise<geometry_msgs::Twist>("/boticelli/cdot", 1000, true);
	gio_pub_cdot = nh_.advertise<geometry_msgs::Twist>("/giotto/cdot", 1000, true);
	bel_pub_cdot = nh_.advertise<geometry_msgs::Twist>("/bellini/cdot", 1000, true);
	ghiber_pub_cdot = nh_.advertise<geometry_msgs::Twist>("/ghiberti/cdot", 1000, true);
	masa_pub_cdot = nh_.advertise<geometry_msgs::Twist>("/masaccio/cdot", 1000, true);
	titi_pub_cdot = nh_.advertise<geometry_msgs::Twist>("/titian/cdot", 1000, true);
picasso_pub_cdot = nh_.advertise<geometry_msgs::Twist>("/ghiberti/cdot", 1000, true);
	dali_pub_cdot = nh_.advertise<geometry_msgs::Twist>("/masaccio/cdot", 1000, true);
	goya_pub_cdot = nh_.advertise<geometry_msgs::Twist>("/titian/cdot", 1000, true);


    centroidPosesStamped[5].header.frame_id="leonardo";
    centroidPosesStamped[2].header.frame_id="michelangelo";
    centroidPosesStamped[3].header.frame_id="donatello";
    centroidPosesStamped[9].header.frame_id="ghiberti";
    centroidPosesStamped[6].header.frame_id="boticelli";
    centroidPosesStamped[8].header.frame_id="bellini";
    centroidPosesStamped[7].header.frame_id="giotto";
    centroidPosesStamped[4].header.frame_id="raphael";
    centroidPosesStamped[1].header.frame_id="titian";
    centroidPosesStamped[10].header.frame_id="masaccio";
    centroidPosesStamped[41].header.frame_id="picasso";
    centroidPosesStamped[42].header.frame_id="dali";
    centroidPosesStamped[43].header.frame_id="goya";



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////






///////////////////////////////////////////////////////BEGIN MAIN SEQUENCE







///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    while (ros::ok())
    {
	ros::spinOnce();









	if (gotPose==true)
	{	
		float xValuesT[countD];
		float yValuesT[countD];  
			
			
		for (int i=0;i<maxNum;i++){
			for (int j=0; j<countD;j++){
				if(selectedIndices[j]==i){
					xValuesT[j]=xValues[i];
					yValuesT[j]=yValues[i];	
				}
			}
		}





		nSites = Matrix_Size(xValuesT);
		Matrix sitesPos(nSites,2);
		for(int i=0; i<Matrix_Size(xValuesT);i++){
			sitesPos.setElement(i, 0, xValuesT[i]);      //sitePos.elements[i][0] = xValues[i];
			sitesPos.setElement(i, 1, yValuesT[i]);
		}


for (int j=0;j<50;j++){

}


	for (int i=0;i<obstacleLineSize;i++){
		for (int j=0;j<countD;j++){
				generatebEquations(obstacleLineList[i],xValues[selectedIndices[j]], yValues[selectedIndices[j]],i,j);
		}
	}

		geodesicDistance();


        	CoMGenerator cg;
        	VoronoiDiagramGenerator vdg;
				
        	vdg.generateVoronoi(xValuesT,yValuesT,countD, minX,maxX,minY,maxY,0.001);
		
        
        	vdg.resetIterator();
        
        	float x1,y1,x2,y2;
        	int a=1;
        	while(vdg.getNext(x1,y1,x2,y2))
        	{
            		a++;
            		if (x1!=x2 || y1!=y2)  //if condition necessary due to some unknown problem (Fortune's Algorithm generating vertices that shouldn't exist)
            		{
                		cg.posVertVector.push_back(x1); cg.posVertVector.push_back(y1);
                		cg.posVertVector.push_back(x2); cg.posVertVector.push_back(y2);
            		}
        	}

        
        	//After store position of all vertices, store the position of the edges of the plane (rectangular plane)
        	cg.posVertVector.push_back(minX); cg.posVertVector.push_back(minY);
        	cg.posVertVector.push_back(maxX); cg.posVertVector.push_back(minY);
        	cg.posVertVector.push_back(maxX); cg.posVertVector.push_back(maxY);
        	cg.posVertVector.push_back(minX); cg.posVertVector.push_back(maxY);
        
        
        	//Return the position of the Centers of Mass
        	sitesPos = cg.generateCenterOfMass(cg.posVertVector, sitesPos, nSites);
		float tempX=0;
		float tempY=0;
		string cfi;

		for (int i=0; i<maxNum; i++) {
			cfi = "OFF";
			for (int j=0; j<countD;j++){
				if(selectedIndices[j]==i){
					cfi="ON";
					tempX=sitesPos.elements[j][0];
					tempY=sitesPos.elements[j][1];	
					break;					
				}
			}

			if (tempX!=0&&tempY!=0){

				if ((abs(tempX-filterX[i])>.01 || abs(tempY-filterY[i])>.01) && weirdReadings[i]<10){
				//cout << "SKIP!";
				weirdReadings[i]=weirdReadings[i]+1;

				}else{
					centroidPositions.transforms[i].transform.translation.x=tempX;
					centroidPositions.transforms[i].transform.translation.y=tempY;
					centroidPositions.transforms[i].transform.rotation.w=1;
					centroidPositions.transforms[i].child_frame_id=cfi;
					filterX[i]=tempX;
					filterY[i]=tempY;
					weirdReadings[i]=0;
				}

			}
			

			/*if (cfi.compare("ON")){
			ps.pose.position.x=tempX;
			ps.pose.position.y=tempY;
			ps.header.frame_id="goal";
			ps.pose.position.z=i;
			//goal_for_android.publish(ps);
			}*/
			


			if (trackingFlag==true){

			lastX[4][i]=lastX[3][i];
			lastX[3][i]=lastX[2][i];
			lastX[2][i]=lastX[1][i];
			lastX[1][i]=lastX[0][i];	
			lastX[0][i]=(centroidPositions.transforms[i].transform.translation.x-delayX[4][i])*30;
	
			delayX[4][i]=delayX[3][i];
			delayX[3][i]=delayX[2][i];
			delayX[2][i]=delayX[1][i];
			delayX[1][i]=delayX[0][i];
			delayX[0][i]=centroidPositions.transforms[i].transform.translation.x;

			lastY[4][i]=lastY[3][i];
			lastY[3][i]=lastY[2][i];
			lastY[2][i]=lastY[1][i];
			lastY[1][i]=lastY[0][i];
			lastY[0][i]=(centroidPositions.transforms[i].transform.translation.y-delayY[4][i])*30;

			delayY[4][i]=delayY[3][i];
			delayY[3][i]=delayY[2][i];
			delayY[2][i]=delayY[1][i];
			delayY[1][i]=delayY[0][i];
			delayY[0][i]=centroidPositions.transforms[i].transform.translation.y;

			cDot.transforms[i].transform.translation.x=(lastX[4][i]+lastX[3][i]+lastX[2][i]+lastX[1][i]+lastX[0][i])/5;
			cDot.transforms[i].transform.translation.y=(lastY[4][i]+lastY[3][i]+lastY[2][i]+lastY[1][i]+lastY[0][i])/5;
			cDot.transforms[i].transform.translation.z=(centroidPositions.transforms[i].transform.translation.z-lastCentroidPositions.transforms[i].transform.translation.z)*30;
			}
			else {
			cDot.transforms[i].transform.translation.x=0;
			cDot.transforms[i].transform.translation.y=0;
			cDot.transforms[i].transform.translation.z=0;
			}
		/*cout << "cDot: "<< cDot.transforms[i].transform << "\n";
		cout << "centroid: "<< centroidPositions.transforms[i].transform<<"\n";
		cout << "----------------------------------------------------------------------------------------------\n\n";*/
		}

		

		//cout << "min X: " << minX << "\n";
		//cout << "max X: " << maxX << "\n";
		//cout << "mean X: " << muX << "\n";

		//cout << "min Y: " << minY << "\n";
		//cout << "max Y: " << maxY << "\n";
		//cout << "mean Y: " << muY << "\n";
		//cout << "xdot: " << cDot.transforms[11].transform.translation.x << "\n\n";

		//cout << "mean X: " << muX<< "\n";
				//cout << "mean Y: " << muY<< "\n\n";
		lastCentroidPositions=centroidPositions;
		centroid_pub_.publish(centroidPositions);
		cDot_pub_.publish(cDot);
		
		
		transform_to_pose(centroidPositions,cDot);
		michel_pub.publish(centroidPosesStamped[MICHELANGELO_INDEX]);
		dona_pub.publish(centroidPosesStamped[DONATELLO_INDEX]);
		bern_pub.publish(centroidPosesStamped[RAPHAEL_INDEX]);
		leo_pub.publish(centroidPosesStamped[LEONARDO_INDEX]);
		boti_pub.publish(centroidPosesStamped[BOTICELLI_INDEX]);
		gio_pub.publish(centroidPosesStamped[GIOTTO_INDEX]);
		bel_pub.publish(centroidPosesStamped[BELLINI_INDEX]);
		ghiber_pub.publish(centroidPosesStamped[GHIBERTI_INDEX]);
		masa_pub.publish(centroidPosesStamped[MASACCIO_INDEX]);
		titi_pub.publish(centroidPosesStamped[TITIAN_INDEX]);
		picasso_pub.publish(centroidPosesStamped[PICASSO_INDEX]);
		dali_pub.publish(centroidPosesStamped[DALI_INDEX]);
		goya_pub.publish(centroidPosesStamped[GOYA_INDEX]);

		michel_pub_Android.publish(centroidPosesStamped[MICHELANGELO_INDEX]);
		dona_pub_Android.publish(centroidPosesStamped[DONATELLO_INDEX]);
		bern_pub_Android.publish(centroidPosesStamped[RAPHAEL_INDEX]);
		leo_pub_Android.publish(centroidPosesStamped[LEONARDO_INDEX]);
		boti_pub_Android.publish(centroidPosesStamped[BOTICELLI_INDEX]);
		gio_pub_Android.publish(centroidPosesStamped[GIOTTO_INDEX]);
		bel_pub_Android.publish(centroidPosesStamped[BELLINI_INDEX]);
		ghiber_pub_Android.publish(centroidPosesStamped[GHIBERTI_INDEX]);

//centroidPosesStamped[TITIAN_INDEX].pose.position.x=geodesicPath[0].x[1];
//centroidPosesStamped[TITIAN_INDEX].pose.position.y=geodesicPath[0].y[1];
		masa_pub_Android.publish(centroidPosesStamped[MASACCIO_INDEX]);
//masa_pub_Android.publish(androidTest);

		titi_pub_Android.publish(centroidPosesStamped[TITIAN_INDEX]);
/*		picasso_pub_Android.publish(centroidPosesStamped[PICASSO_INDEX]);
		dali_pub_Android.publish(centroidPosesStamped[DALI_INDEX]);
        	goya_pub_Android.publish(centroidPosesStamped[GOYA_INDEX]);
*/

		michel_pub_cdot.publish(cDotTurtles[MICHELANGELO_INDEX]);
		dona_pub_cdot.publish(cDotTurtles[DONATELLO_INDEX]);
		bern_pub_cdot.publish(cDotTurtles[RAPHAEL_INDEX]);
		leo_pub_cdot.publish(cDotTurtles[LEONARDO_INDEX]);
		boti_pub_cdot.publish(cDotTurtles[BOTICELLI_INDEX]);
		gio_pub_cdot.publish(cDotTurtles[GIOTTO_INDEX]);
		bel_pub_cdot.publish(cDotTurtles[BELLINI_INDEX]);
		ghiber_pub_cdot.publish(cDotTurtles[GHIBERTI_INDEX]);
		masa_pub_cdot.publish(cDotTurtles[MASACCIO_INDEX]);
		titi_pub_cdot.publish(cDotTurtles[TITIAN_INDEX]);
		picasso_pub_cdot.publish(cDotTurtles[PICASSO_INDEX]);
		dali_pub_cdot.publish(cDotTurtles[DALI_INDEX]);
		goya_pub_cdot.publish(cDotTurtles[GOYA_INDEX]);
		
		
		gotPose=false;
	}

cout << "PrevpathLine Size: " << previousPathLine[0].size<<"\n";
for (int kk=0;kk<previousPathLine[0].size;kk++){
	cout << "Index: " << kk << ", x: " << previousPathLine[0].points[kk].x << ", y: " << previousPathLine[0].points[kk].y << "\n"; 
}
cout << "\n\n";

    loop_rate.sleep();
    }
        
}


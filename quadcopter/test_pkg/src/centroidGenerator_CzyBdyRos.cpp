#include <iostream>
#include <stdio.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <geometry_msgs/PoseStamped.h>
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
#include <std_msgs/Float64.h>
#include <sstream>
#include <search.h>
#include <stdlib.h>
#include "VoronoiDiagramGenerator.h"
#include <tf2_msgs/TFMessage.h>
#include "CentroidGenerator_CzyBdy.h"

using namespace std;
double T=50;
bool gotPose=false;
const int maxNum=50;
int countD;
float xValues[maxNum];
float yValues[maxNum];   
int selectedIndices[maxNum]; 
//float minX = -.25, maxX = .25;    
//float minY = -.25, maxY = .25;
//geometry_msgs::PoseArray centroidPositions;

void poseCallback(const geometry_msgs::PoseArray::ConstPtr& pose)
{
	countD=0;
	//centroidPositions = *pose;
	gotPose=true;

	for (int i=0;i<maxNum;i++)
	{
		if (pose ->poses[i].position.x!=0 || pose ->poses[i].position.y!=0){
			xValues[i]=pose ->poses[i].position.x;	
			yValues[i]=pose ->poses[i].position.y;
			selectedIndices[countD]=i;	
			countD++;		
		}
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
    //cout << endl << "End program" << endl;
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


float CentroidGenerator::truncate(float num, int precision)        //truncate a floating point number
{   num = num*pow(10,precision);
    num = round(num);
    num = num/pow(10,precision);
    return num;
}

float CentroidGenerator::distance(float siteX, float siteY, float vertX, float vertY)      //calculate the distance between a site and a vertice
{   float dist = sqrt(pow((vertY-siteY),2)+pow((vertX-siteX),2));
    dist = truncate(dist,3);        //using 4 as precision was identified some errors in calculations. So use 3 or less as precision
    return dist;
}

CentroidGenerator::~CentroidGenerator(){}

void CentroidGenerator::getUniqVertices(std::vector<float> allVerticesVector)
{
    for (int i=0; i<allVerticesVector.size(); i++)  //as numbers are float, need to truncate them in order to work
    {   allVerticesVector[i]=truncate(allVerticesVector[i], 4); }
    
    Matrix allVerticesMatrix(50,2); //initialize the matrix with 50 rows, but it will be replaced by the next line. So instead of 50, it can be any number
    allVerticesMatrix.rows = allVerticesVector.size()/2;
    
    //transform vector "allVerticesVector into a matrix
    int k=0;
    for (int i=0;i<allVerticesMatrix.rows;i++)
    {
        for (int j=0;j<2;j++) {    allVerticesMatrix.setElement(i,j,allVerticesVector[k]); k++;    }
    }
    //allVerticesMatrix.printArray("All Vertices");
    
    std::vector<int> repeatIndex={};    //vector to store the indices(rows) of repeated vertices
    
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
        if (count==repeatIndex.size())   //if index i isn't storage in repeatIndex: we can storage the values of row (i) from Vertices to newVertices
        {
            uniqueVertices.setElement(a, 0, allVerticesMatrix.elements[i][0]);
            uniqueVertices.setElement(a, 1, allVerticesMatrix.elements[i][1]);
            a++;
        }
    }
    //uniqueVertices.printArray("Unique Vertices");
}

void CentroidGenerator::getIndices(Matrix indicesMatrix, Matrix Sites, int nSites, int nVertices)
{
    const float E = 0.002;
    //find out which vertice is part of which cell
    for (int i=1;i<nSites;i++) {
        for (int j=1;j<=nVertices;j++) {               //go through each vertice to find what site is nearer to that vertice
            if (indicesMatrix.elements[i-1][j-1]<0) {  //check if element of matrix Indices is still with initial value -1, avoiding unecessary calculations
                int p=-1;
                for (int k=i;k<nSites;k++){            //compare the site (i) to every other site
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

void CentroidGenerator::findVerticesOnCrazyBdy(float x1, float y1, float x0, float y0, int row, int nCzyBdyVert)
{
    float m1 = (y1-y0)/(x1-x0);
    float b1 = -x1*m1+y1;
    float a1, a2;
    int index;
    
    for (int i=uniqueVertices.rows-nCzyBdyVert; i<uniqueVertices.rows;i++)  //find which boundary vertice is nearer to the vertice at the boundary
    {
        if (i==uniqueVertices.rows-nCzyBdyVert) {
            a1 = distance(x1,y1,uniqueVertices.elements[i][0],uniqueVertices.elements[i][1]);
            index = i;
        }
        else {
            a2 = distance(x1,y1,uniqueVertices.elements[i][0],uniqueVertices.elements[i][1]);
            if (a1>a2) {
                a1=a2;
                index = i;
            }
        }
    }
    
    float x2 = uniqueVertices.elements[index][0], y2 = uniqueVertices.elements[index][1];
    
    float x3, y3, x4, y4;
    //cout << "aa: " << index << endl;
    if (index==uniqueVertices.rows-nCzyBdyVert) {
        x3 = uniqueVertices.elements[uniqueVertices.rows-1][0], y3 = uniqueVertices.elements[uniqueVertices.rows-1][1];
    }
    else x3 = uniqueVertices.elements[index-1][0], y3 = uniqueVertices.elements[index-1][1];
    if (index==uniqueVertices.rows-1) {
        x4 = uniqueVertices.elements[uniqueVertices.rows-nCzyBdyVert][0], y4 = uniqueVertices.elements[uniqueVertices.rows-nCzyBdyVert][1];
    }
    else x4 = uniqueVertices.elements[index+1][0], y4 = uniqueVertices.elements[index+1][1];
    //cout << "test: " << x4 << ", " << y4 << endl;
    
    float m4 = (y4-y2)/(x4-x2);
    float b4 = -x2*m4+y2;
    float m3 = (y3-y2)/(x3-x2);
    float b3 = -x2*m3+y2;
    
    float xp = (b4-b1)/(m1-m4), yp = m1*xp+b1;
    float xpp = (b3-b1)/(m1-m3), ypp = m1*xpp+b1;
    xp = truncate(xp, 4); yp = truncate(yp, 4);
    xpp = truncate(xpp, 4); ypp = truncate(ypp, 4);
    
    if (distance(xp, yp, x0, y0) > distance(xpp, ypp, x0, y0)) {
        uniqueVertices.elements[row][0]=xpp;
        uniqueVertices.elements[row][1]=ypp;
    }
    else {
        uniqueVertices.elements[row][0]=xp;
        uniqueVertices.elements[row][1]=yp;
    }
    
}

void CentroidGenerator::setVerticesOnBoundary(Matrix indicesMatrix, float minX, float maxX, float minY, float maxY, int nCzyBdyVert)
{
    for (int i=0; i<uniqueVertices.rows-nCzyBdyVert; i++)     //ignore the last nCzyBdyvert rows, because they're the edges of the CrazyBoundary
    {
        if (uniqueVertices.elements[i][0]==minX || uniqueVertices.elements[i][0]==maxX || uniqueVertices.elements[i][1]==minY || uniqueVertices.elements[i][1]==maxY)   //find if vertice is at the boundary
        {
            float x1 = uniqueVertices.elements[i][0], y1 = uniqueVertices.elements[i][1];
            int myvector[2] = {-1,-1};
            int j=0;
            for (int k=0; k<indicesMatrix.rows; k++)    //store the indices of the cells, which that vertice is part of
            {
                if (indicesMatrix.elements[k][i] == i+1)    {
                    myvector[j] = k;
                    j++;
                }
            }
            //cout << "abc: " << myvector[0] << ", " << myvector[1] << endl;
            for (int k=0; k<indicesMatrix.cols-nCzyBdyVert; k++)
            {
                if (indicesMatrix.elements[myvector[0]][k] == indicesMatrix.elements[myvector[1]][k] && indicesMatrix.elements[myvector[0]][k] != i+1 && indicesMatrix.elements[myvector[0]][k] > 0)    //find the "internal" vertice that is also part of the same two cells
                {
                    float x0 = uniqueVertices.elements[k][0], y0 = uniqueVertices.elements[k][1];
                    findVerticesOnCrazyBdy(x1, y1, x0, y0, i, nCzyBdyVert);
                    k=indicesMatrix.cols;   //break the loop
                }
            }
        }
    }
}

void CentroidGenerator::setIndicesOrder(Matrix angleMatrix, Matrix indicesMatrix, int nVertices, int row)
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

void CentroidGenerator::sortIndices(Matrix sitesPosition, Matrix indicesMatrix, int nSites, int nVertices, Matrix angleMatrix)
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

void CentroidGenerator::getLocalIndex(Matrix indicesMatrix, int nVertices, int row, vector<int>& localIndex, int& noLocVertices)
{
    for (int j=0; j<nVertices; j++)     //store indices different from zero
    {
        if (indicesMatrix.elements[row][j] != 0) localIndex.push_back (indicesMatrix.elements[row][j]);
    }
    localIndex.push_back(localIndex.front());   //at the end, store the first element from the vector again
    noLocVertices = int(localIndex.size());
}

void CentroidGenerator::getCentroid(int nLocalVertices, vector<int>& localIndex, Matrix centroidMatrix, int row)
{
    double xsum = 0, ysum = 0, area = 0;
    
    //Get local vertices
    Matrix localVerticesPos(nLocalVertices,2);
    for (int i=0; i<nLocalVertices; i++){
        for (int j=0; j<2; j++){
            localVerticesPos.elements[i][j] = (uniqueVertices.elements[ (localIndex[i]-1) ][j]);
        }
    }
    
    //Compute Area
    for (int i=0; i<(nLocalVertices-1); i++){
        area = area + localVerticesPos.elements[i][0]*localVerticesPos.elements[i+1][1] - localVerticesPos.elements[i+1][0]*localVerticesPos.elements[i][1];
    }
    area = area/2;
    
    //Compute Centroid
    for (int i=0;  i<(nLocalVertices-1); i++){
        xsum = xsum + (localVerticesPos.elements[i][0] + localVerticesPos.elements[i+1][0])*(localVerticesPos.elements[i][0]*localVerticesPos.elements[i+1][1] - localVerticesPos.elements[i+1][0]*localVerticesPos.elements[i][1]);
        ysum = ysum + (localVerticesPos.elements[i][1] + localVerticesPos.elements[i+1][1])*(localVerticesPos.elements[i][0]*localVerticesPos.elements[i+1][1] - localVerticesPos.elements[i+1][0]*localVerticesPos.elements[i][1]);
    }
    centroidMatrix.elements[row][0] = ( 1/(6*area) )*xsum;
    centroidMatrix.elements[row][1] = ( 1/(6*area) )*ysum;
}


Matrix CentroidGenerator::generateCentroid(std::vector<float> allVertices, Matrix sitesPosition, int nSites, float minX, float maxX, float minY, float maxY, int nCzyBdyVert)
{
    getUniqVertices(allVertices);
    
    Matrix indicesMatrix(nSites, uniqueVertices.rows,-1); //initialize IndicesMatrix with value(-1). It'll store indices of vertices related to each site.
    getIndices(indicesMatrix, sitesPosition, nSites, uniqueVertices.rows);
    
    setVerticesOnBoundary(indicesMatrix, minX, maxX, minY, maxY, nCzyBdyVert);
    
    //uniqueVertices.printArray("New Unique Vertices");
    
    Matrix angleMatrix(nSites, uniqueVertices.rows);
    sortIndices(sitesPosition, indicesMatrix, nSites, uniqueVertices.rows, angleMatrix);
    
    Matrix centroidMatrix(nSites, 2);
    
    int nLocalVertices = 0;
    for (int siteNumber=0; siteNumber<nSites; siteNumber++){
        vector<int> localIndex;
        
        //Get Local Index: Supporting function. Returns: (localIndex, noLocIndices)
        getLocalIndex(indicesMatrix, uniqueVertices.rows, siteNumber, localIndex, nLocalVertices);
        
        /*cout << endl;
        cout << "DICTIONARY FOR SITE " << (siteNumber + 1) << ":" << endl;
        cout << "noLocVertices = "<< nLocalVertices << endl;
        cout << "localIndex    = ";
        for (int i=0; i<nLocalVertices; i++){
            cout << localIndex[i] << ", ";
        }
        cout << endl;
        */
        
        //GET CENTROID: Returns (Centroid)
        getCentroid(nLocalVertices, localIndex, centroidMatrix, siteNumber);
        /*cout << "Centroid      = ";
        cout << "(" << centroidMatrix.elements[siteNumber][0] << ", " << centroidMatrix.elements[siteNumber][1] << ")";
        cout << endl;*/
    }
    //cout << endl;
    centroidMatrix.printArray("Centroid");
    
    
    //print Centroid Matrix to work with Matlab
    /*cout << "cX=[";
    for (int i=0; i<centroidMatrix.rows; i++){
        if (i==centroidMatrix.rows-1) cout << centroidMatrix.elements[i][0] << "];" << endl;
        else cout << centroidMatrix.elements[i][0] << ", "; }
    cout << "cY=[";
    for (int i=0; i<centroidMatrix.rows; i++){
        if (i==centroidMatrix.rows-1) cout << centroidMatrix.elements[i][1] << "];" << endl;
        else cout << centroidMatrix.elements[i][1] << ", "; }
    */
    
    return centroidMatrix;
}




//////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv) {
    //variables should be provided by "Graham Scan" Algorithm
    const int nCzyBdyVert = 8;
    float boundaryPos[nCzyBdyVert][2] = {{2, -15}, {35, -23}, {72, -23}, {112, -3}, {109, 98}, {100, 120}, {-5, 120}, {-43, 36}};
    
    //define the minimum and maximum X and Y values for the Fortune's Algorithm
    float minX, maxX, minY, maxY;
    for (int i=0; i<nCzyBdyVert; i++) {
        if (i==0) {
            minX = maxX = boundaryPos[i][0];
            minY = maxY = boundaryPos[i][1];
        }
        else {
            if (boundaryPos[i][0]<minX) minX = boundaryPos[i][0];
            else if(boundaryPos[i][0]>maxX) maxX = boundaryPos[i][0];
            if (boundaryPos[i][1]<minY) minY = boundaryPos[i][1];
            else if(boundaryPos[i][1]>maxY) maxY = boundaryPos[i][1];
        }
    }
 
    
    //Store the position of the sites in a Matrix
    int nSites = Matrix_Size(xValues);
    tf2_msgs::TFMessage centroidPositions;
    centroidPositions.transforms.resize(maxNum);
    countD=5;
    Matrix sitesPos(nSites,2);
    for(int i=0; i<Matrix_Size(xValues);i++){
        sitesPos.setElement(i, 0, xValues[i]);      //sitePos.elements[i][0] = xValues[i];
        sitesPos.setElement(i, 1, yValues[i]);
    }
 
        ros::init(argc, argv, "Centroid_Generator"); 
	ros::start();
	ros::Rate loop_rate(T); //Set Ros frequency to 50/s (fast)
	ros::NodeHandle nh_;
	ros::Subscriber pos_sub_ ;
	ros::Publisher centroid_pub_ ;

	void poseCallback(const geometry_msgs::PoseArray::ConstPtr& pose);

	pos_sub_= nh_.subscribe<geometry_msgs::PoseArray>("/toVoronoiDeployment", 1000,poseCallback);
	centroid_pub_ = nh_.advertise<tf2_msgs::TFMessage>("Centroids", 1000, true);
    
    while (ros::ok())
    {
          ros::spinOnce();
	  if (gotPose==true) {
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
		cout<<"\n\n";
		nSites = Matrix_Size(xValuesT);
		Matrix sitesPos(nSites,2);
		for(int i=0; i<Matrix_Size(xValuesT);i++){
		     sitesPos.setElement(i, 0, xValuesT[i]);      //sitePos.elements[i][0] = xValues[i];
		     sitesPos.setElement(i, 1, yValuesT[i]);
		     cout<<"Site: "<< i <<"\n";
		     cout<<"X: "<< xValuesT[i] <<"\n";
		     cout<<"Y: "<< yValuesT[i] <<"\n\n";
		}

		CentroidGenerator cg;
		VoronoiDiagramGenerator vdg;
		vdg.generateVoronoi(xValues, yValues, count, minX, maxX, minY, maxY, 3);
        
		vdg.resetIterator();
        
		float x1,y1,x2,y2;
		int a=1;
		printf("\n-------------------------------\n");
		while(vdg.getNext(x1,y1,x2,y2))
		{
		    //printf("GOT Line (%.4f,%.4f)->(%.4f,%.4f)\n", x1,y1,x2,y2);
		    //printf("v%dx = [%.4f,%.4f];\n", a, x1,x2);    //to work with MATLAB
		    //printf("v%dy = [%.4f,%.4f];\n", a, y1,y2);    //to work with MATLAB
		    a++;
		    if (x1!=x2 || y1!=y2)  //if condition necessary due to some unknown problem (Fortune's Algorithm generating vertices that shouldn't exist)
		    {
		        cg.posVertVector.push_back(x1); cg.posVertVector.push_back(y1);
			cg.posVertVector.push_back(x2); cg.posVertVector.push_back(y2);
		    }                       //even though it seems to not affect the centroids' position
		}
		cout << endl;
        
		//After store position of all vertices, store the position of the edges of the plane (polygon boundary)
		for (int i=0; i<nCzyBdyVert; i++) {
		      cg.posVertVector.push_back(boundaryPos[i][0]);
		      cg.posVertVector.push_back(boundaryPos[i][1]);
		}
        
		//Return the position of the centroids
		sitesPos = cg.generateCentroid(cg.posVertVector, sitesPos, nSites, minX, maxX, minY, maxY, nCzyBdyVert);
		cout<<"Checkpoint \n";
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
		      centroidPositions.transforms[i].transform.translation.x=tempX;
		      centroidPositions.transforms[i].transform.translation.y=tempY;
		      centroidPositions.transforms[i].transform.rotation.w=1;
		      centroidPositions.transforms[i].child_frame_id=cfi;
		}
			     	
		centroid_pub_.publish(centroidPositions);
		gotPose=false;
        
	  }
	  loop_rate.sleep();
    }
    
}


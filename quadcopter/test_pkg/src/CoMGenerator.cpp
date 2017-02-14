#include <iostream>
#include <stdio.h>
#include <search.h>
#include <stdlib.h>
#include <vector>
#include <math.h>
#include "CoMGenerator.h"
#include "VoronoiDiagramGenerator.h"
using namespace std;

Matrix uniqueVertices(50,2);    //it must be global, because it is used in every function

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
    /* use "delete" for dynamically allocated data
    for(int i =0; i<rows; i++)
    {
        delete[] elements[i];
    }
    delete[] elements;
    */
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

void CoMGenerator::getLocalIndex(Matrix indicesMatrix, int nVertices, int row, vector<int>& localIndex, int& noLocVertices)
{
    for (int j=0; j<nVertices; j++)     //store indices different from zero
    {
        if (indicesMatrix.elements[row][j] != 0) localIndex.push_back (indicesMatrix.elements[row][j]);
    }
    localIndex.push_back(localIndex.front());   //at the end, store the first element from the vector again
    noLocVertices = int(localIndex.size());
}

// Gets the center of mass for any triangle
void CoMGenerator::getTriangleCOM(double &Mass, double &c_x, double &c_y, double xpoints[3], double ypoints[3], int nGaussPoints)
{
    //Gauss Info
    double rGauss[3] = {1.0/6.0, 2.0/3.0, 1.0/6.0};
    double sGauss[3] = {1.0/6.0, 1.0/6.0, 2.0/3.0};
    double w = 1.0/3.0;
    
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
        //||----------------||
        DENSITY = pow(x,4) + pow(y,4); //1.0;
        //||----------------||
        //||----------------||
        
        //Apriori Expected Values and Mass
        E_x = E_x + w*x*DENSITY;
        E_y = E_y + w*y*DENSITY;
        Mass = Mass + w*DENSITY;
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
    //Declare Variables
    int k = (nLocalVertices-1);
    double MassVector[k], c_xVector[k], c_yVector[k];
    double xpoints[3], ypoints[3];
    
    //Collect COM's
    xpoints[2] = Site[0];   ypoints[2] = Site[1];
    for(int i=0; i<k; i++){
        //Form Triangle
        xpoints[0] = localVertices.elements[i][0];  ypoints[0] = localVertices.elements[i][1];
        xpoints[1] = localVertices.elements[i+1][0];    ypoints[1] = localVertices.elements[i+1][1];
        
        //Get Each Triangle's Center of Mass
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
    getUniqVertices(allVertices);
    
    Matrix indicesMatrix(nSites, uniqueVertices.rows,-1); //initialize IndicesMatrix with value(-1). It'll store indices of vertices related to each site.
    getIndices(indicesMatrix, sitesPosition, nSites, uniqueVertices.rows);
    
    Matrix angleMatrix(nSites, uniqueVertices.rows);
    sortIndices(sitesPosition, indicesMatrix, nSites, uniqueVertices.rows, angleMatrix);
    
    Matrix centerOfMass(nSites,2);
    
    double Mass, c_x, c_y;
    int nGaussPoints = 3;
    
    int nLocalVertices = 0;
    for (int siteNumber=0; siteNumber<nSites; siteNumber++){
        vector<int> localIndex;
        
        //Get Local Index: Supporting function. Returns: (localIndex, noLocIndices)
        getLocalIndex(indicesMatrix, uniqueVertices.rows, siteNumber, localIndex, nLocalVertices);
        
        Matrix localVertPos(nLocalVertices,2);
        
        /*cout << endl;
        cout << "DICTIONARY FOR SITE " << (siteNumber + 1) << ":" << endl;
        cout << "noLocVertices = "<< nLocalVertices << endl;
        cout << "localIndex    = ";
        for (int i=0; i<nLocalVertices; i++){
            cout << localIndex[i] << ", ";
        }
        cout << endl;*/
        
        for (int i=0; i<nLocalVertices; i++){
            localVertPos.elements[i][0]=uniqueVertices.elements[localIndex[i]-1][0];
            localVertPos.elements[i][1]=uniqueVertices.elements[localIndex[i]-1][1];
        }
        //localVertPos.printArray("Local Vertices");
        
        //GET CENTER OF MASS: Returns (CenterOfMass)
        double Site[2] = {sitesPosition.elements[siteNumber][0], sitesPosition.elements[siteNumber][1]};

        getCenterOfMass(Mass, c_x, c_y, localVertPos, Site, nLocalVertices, nGaussPoints, centerOfMass, siteNumber);
    }
    //cout << endl;
    
    //print CoM Matrix to work with Matlab
    /*cout << "cmX=[";
    for (int i=0; i<centerOfMass.rows; i++){
    if (i==centerOfMass.rows-1) cout << centerOfMass.elements[i][0] << "];" << endl;
    else cout << centerOfMass.elements[i][0] << ", "; }
    cout << "cmY=[";
    for (int i=0; i<centerOfMass.rows; i++){
    if (i==centerOfMass.rows-1) cout << centerOfMass.elements[i][1] << "];" << endl;
    else cout << centerOfMass.elements[i][1] << ", "; }
    */
    
    centerOfMass.printArray("Center of Mass");
    
    return centerOfMass;
}

int main(int argc, const char * argv[]) {
    //variables provided by publisher
    const int count = 8;     //number of sites(robots)
    float xValues[count] = {43, 37, 42, 58, 25, 69, 54, 65};    //X position of sites(robots)
    float yValues[count] = {44, 46, 16, 67, 62, 53, 27, 13};    //Y position of sites(robots)
    
    float minX = 0, maxX = 100;
    float minY = 0, maxY = 100;
    
    //Store the position of the sites in a Matrix
    int nSites = Matrix_Size(xValues);
    Matrix sitesPos(nSites,2);
    for(int i=0; i<Matrix_Size(xValues);i++){
        sitesPos.setElement(i, 0, xValues[i]);      //sitePos.elements[i][0] = xValues[i];
        sitesPos.setElement(i, 1, yValues[i]);
    }
    //sitesPos.printArray("Sites");
    
    int iteration=1;
    while (iteration<=100)
    {
        cout << endl << "Iteration " << iteration;
        CoMGenerator cg;
        VoronoiDiagramGenerator vdg;
        vdg.generateVoronoi(xValues,yValues,count, minX,maxX,minY,maxY,3);
        
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
            }
        }
        cout << endl;
        
        //After store position of all vertices, store the position of the edges of the plane (rectangular plane)
        cg.posVertVector.push_back(minX); cg.posVertVector.push_back(minY);
        cg.posVertVector.push_back(maxX); cg.posVertVector.push_back(minY);
        cg.posVertVector.push_back(maxX); cg.posVertVector.push_back(maxY);
        cg.posVertVector.push_back(minX); cg.posVertVector.push_back(maxY);
        
        
        //Return the position of the Centers of Mass
        sitesPos = cg.generateCenterOfMass(cg.posVertVector, sitesPos, nSites);
        //sitesPos.printArray("new");
        
        //Split the CoM Matrix in X and Y vectors in order to pass in to the Fortune's Algorithm
        for (int i=0; i<sitesPos.rows; i++) {
            xValues[i]=sitesPos.elements[i][0];
            yValues[i]=sitesPos.elements[i][1];
        }
        iteration++;
    }
    
    return 0;
}

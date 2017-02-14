#ifndef __CenterOfMass__CoMGenerator__
#define __CenterOfMass__CoMGenerator__

#include <iostream>
#include <stdio.h>
#include <vector>
#include <math.h>

#define Matrix_Size(x) (sizeof(x) / sizeof(x[0]))
#define PI 3.14159265

class Matrix {
public:
    Matrix(int r, int c, float data=0);
    ~Matrix();
    void setMatrix(int r, int c, float data=0);
    int setCols(int c);//to modify the columns
    int setRows(int r);//to modify the rows
    void printArray(std::string name="Matrix");
    void setElement(int r, int c, float data);
    float getElement(int r, int c);
    
    int rows, cols;
    float **elements;
};

class CoMGenerator
{
public:
    //CoMGenerator();
    ~CoMGenerator();
    std::vector<float> posVertVector;
    
    float truncate(float num, int precision);
    float distance(float siteX, float siteY, float vertX, float vertY);
    void setIndicesOrder(Matrix angleMatrix, Matrix indicesMatrix, int nVertices, int row);
    
    void getUniqVertices(std::vector<float> allVerticesVector);
    void getIndices(Matrix indicesMatrix, Matrix Sites, int nSites, int nVertices);
    void sortIndices(Matrix sitesPosition, Matrix indicesMatrix, int nSites, int nVertices, Matrix angleMatrix);
    void getLocalIndex(Matrix indicesMatrix, int nVertices, int row, std::vector<int>& localIndex, int& noLocVertices);
    
    void getTriangleCOM(double &Mass, double &c_x, double &c_y, double xpoints[3], double ypoints[3], int nGaussPoints);
    void getCompositeCOM(double &c_x, double &c_y, double x[], double y[], double Mass[], int nLocalVertices);
    void getCenterOfMass(double &Mass, double &c_x, double &c_y, Matrix localVertPos, double Site[2], int nLocalVertices, int nGaussPoints, Matrix centerOfMass, int row);
    int mergeObstacles(Matrix indicesMatrix, Matrix uniqueV);

    
    Matrix generateCenterOfMass(std::vector<float> allVertices, Matrix sitesPosition, int nSites);
};
    
#endif /* defined(__CenterOfMass__CoMGenerator__) */

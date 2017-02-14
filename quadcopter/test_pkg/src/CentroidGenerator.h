//
//  CentroidGenerator.h
//  Test2
//
//  Created by Bruno Maciel on 7/25/15.
//  Copyright (c) 2015 Bruno Maciel. All rights reserved.
//

#ifndef __Centroid_Generator__
#define __Centroid_Generator__

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
    
    int rows, cols;
    float **elements;
};


class CentroidGenerator
{
public:
    //CentroidGenerator();
    ~CentroidGenerator();
    std::vector<float> posVertVector;
    
    float truncate(float num, int precision);
    float distance(float siteX, float siteY, float vertX, float vertY);
    void setIndicesOrder(Matrix angleMatrix, Matrix indicesMatrix, int nVertices, int row);
    
    void getUniqVertices(std::vector<float> allVerticesVector);
    void getIndices(Matrix indicesMatrix, Matrix Sites, int nSites, int nVertices);
    void sortIndices(Matrix sitesPosition, Matrix indicesMatrix, int nSites, int nVertices, Matrix angleMatrix);
    void getLocalIndex(Matrix indicesMatrix, int nVertices, int row, std::vector<int>& localIndex, int& noLocVertices);
    void getCentroid(int nLocalVertices, std::vector<int>& localIndex, Matrix centroidMatrix, int row);
    Matrix generateCentroid(std::vector<float> allVertices, Matrix sitesPosition, int nSites);
};


#endif

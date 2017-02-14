//
//  DP.h
//  DensityPartitions
//
//  Created by Julio Martinez on 11/7/15.
//  Copyright (c) 2015 MURO Lab. All rights reserved.
//

#ifndef __DensityPartitions__DP__
#define __DensityPartitions__DP__

#include <stdio.h>


class DP{
private:
    void insertionSort(double A[], int lengthA);
    void quickSort(double A[], int lengthA);
    void sortDist(double mu_x, double mu_y, double site[][2],int siteIndex[]);
    void getImportance(double sigma[], int lengthSigma, double importance[]);
    void getMostIsolated(double mu[][2], double site[][2], int &index);
    void getAssignments(int partition[], double site[][2], int index);
    void truncateArray(double array[][2],int lower_limit, int upper_limit);
    
public:
    void getPartitions(double mu[][2], double sigma[], double site[][2], int noDensities, int noSites, int siteIndex[]);
    
};




#endif /* defined(__DensityPartitions__DP__) */

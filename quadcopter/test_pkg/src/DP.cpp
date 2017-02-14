//
//  DP.cpp
//  DensityPartitions
//
//  Created by Julio Martinez on 11/7/15.
//  Copyright (c) 2015 MURO Lab. All rights reserved.
//

#include "DP.h"

//Hold
void DP::insertionSort(double A[], int lengthA){
    
}

//Gerardo
void DP::quickSort(double A[], int lengthA){
    
}

//Gerardo
void DP::sortSite(double mu_x, double mu_y, double site[][2], int siteIndex[], double distance[]){
    //Make an array of distances
    // for i=0:(n-1)
    //      distance(i) = sqrt( (mu_x - site[i][0])^2 + (mu_y - site[i][1])^2)
    //
    // quickSort(distance) and arrange index array corresponding to distances
}

//Gerardo
void DP::getImportance(double sigma[], int lengthSigma, double importance[]){
    // importance[i] = sigma[i]/sum(sigma)
}

//Julio
void DP::getMostIsolated(double mu[][2], double site[][2], int &index){
    
}

//Julio
void DP::getAssignments(int partition[], double site[][2], int index){
    
}

//Julio
void DP::truncateArray(double array[][2],int lower_limit, int upper_limit){
    
}


void DP::getPartitions(double mu[][2], double sigma[], double site[][2], int noDensities, int noSites,int siteIndex[]){
    double importance[noDensities];
    int index;
    int partition[noSites];
    
    getImportance(sigma, noDensities, importance);
    
    for (int i = 0; i < noDensities; i++){
        getMostIsolated(mu, site, index);
        sortDist(mu[index][0], mu[index][1], site, siteIndex);
        getAssignments(partition, site,index);
        truncateArray(site, partition[index+1], (noSites-1));
        //truncateArray(mu,..);
        
    }
}

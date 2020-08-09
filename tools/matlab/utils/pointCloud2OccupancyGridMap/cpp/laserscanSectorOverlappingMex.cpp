#include<stdio.h>
#include"mex.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <string.h>
#include <math.h>

typedef Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> MatrixXi;

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]){
    //nlhs represent the number of parameters of the output
    //plhs is a array of the mxarray pointers, each pointing to the output
    //nrhs represents the number of parameters of the input
    //prhs is a array of the mxarray pointers, each pointing to the input

    /* check proper input and output */

    // prhs[0] params, struct of the params, with field sectorRes
    // prhs[1] child pointCloud, Nx2, | x | y |
    // prhs[2] parent pointCloud, Nx2, | x | y |

    if(nrhs < 3)
      mexErrMsgIdAndTxt( "laserscanSectorOverlappingMex:invalidNumInputs",
              "at least 3 input arguments required");
    else if(!mxIsStruct(prhs[0]))
        mexErrMsgIdAndTxt( "laserscanSectorOverlappingMex:inputNotStruct",
                "the first input should be a struct");

    ///////////////////////////////////////////////////////////////////////////////////////////
    // get the params
    mxArray * fPtr;
    // sector resolution with unit degree
    double sectorRes_;
    double overlappingThChild_;
    double overlappingThParent_;

    // get the parameter : sectorRes_
    {
      fPtr = mxGetField(prhs[0], 0, "sectorRes");
      if ((fPtr != NULL)){
          double *realPtr = mxGetPr(fPtr);
          sectorRes_ = realPtr[0];
          #ifdef DEBUG
              mexPrintf("%s%f\n", "sectorRes: ", sectorRes_);
          #endif
      }else{
          mexErrMsgIdAndTxt( "laserscanSectorOverlappingMex:invalidField",
              "field sectorRes not found in the params struct");
      }
    }

    // get the parameter : overlappingThChild_
    {
      fPtr = mxGetField(prhs[0], 0, "overlappingThChild");
      if ((fPtr != NULL)){
          double *realPtr = mxGetPr(fPtr);
          overlappingThChild_ = realPtr[0];
          #ifdef DEBUG
              mexPrintf("%s%f\n", "overlappingThChild: ", overlappingThChild_);
          #endif
      }else{
          mexErrMsgIdAndTxt( "laserscanSectorOverlappingMex:invalidField",
              "field overlappingThChild not found in the params struct");
      }
    }

    // get the parameter : overlappingThParent_
    {
      fPtr = mxGetField(prhs[0], 0, "overlappingThParent");
      if ((fPtr != NULL)){
          double *realPtr = mxGetPr(fPtr);
          overlappingThParent_ = realPtr[0];
          #ifdef DEBUG
              mexPrintf("%s%f\n", "overlappingThParent: ", overlappingThParent_);
          #endif
      }else{
          mexErrMsgIdAndTxt( "laserscanSectorOverlappingMex:invalidField",
              "field overlappingThParent not found in the params struct");
      }
    }

    // get the child pointCloud
    pcl::PointCloud<pcl::PointXYZ> child_cloud;
    const size_t *dimArrayOfChildPointCloud = mxGetDimensions(prhs[1]);
    double *ptrChildPointCloud = mxGetPr(prhs[1]);
    size_t numOfChildPoints = *dimArrayOfChildPointCloud;
    pcl::PointXYZ p_child;
    for(int i = 0; i < numOfChildPoints; i++){
      p_child.x = ptrChildPointCloud[(numOfChildPoints * 0) + i];
      p_child.y = ptrChildPointCloud[(numOfChildPoints * 1) + i];
      p_child.z = 0;
      child_cloud.push_back(p_child);
    }

    // get the parent pointCloud
    pcl::PointCloud<pcl::PointXYZ> parent_cloud;
    const size_t *dimArrayOfParentPointCloud = mxGetDimensions(prhs[2]);
    double *ptrParentPointCloud = mxGetPr(prhs[2]);
    size_t numOfParentPoints = *dimArrayOfParentPointCloud;
    pcl::PointXYZ p_parent;
    for(int i = 0; i < numOfParentPoints; i++){
      p_parent.x = ptrParentPointCloud[(numOfParentPoints * 0) + i];
      p_parent.y = ptrParentPointCloud[(numOfParentPoints * 1) + i];
      p_parent.z = 0;
      parent_cloud.push_back(p_parent);
    }

    // construct the sectorCounter
    int sectorNum = int(360.0 / sectorRes_);
    MatrixXi sectorCounter(sectorNum, 2);

    MatrixXi overlappingFlagChild(numOfChildPoints, 1);
    MatrixXi overlappingFlagParent(numOfParentPoints, 1);
    
    // init the sectorCounter to zeros
    for (int i = 0; i < sectorNum; i++) {
      for (int j = 0; j < 2; j++){
        sectorCounter(i, j) = 0;
      }
    }

    for (int i = 0; i < numOfChildPoints; i++) {
      overlappingFlagChild(i, 0) = 0;
    }

    for (int i = 0; i < numOfParentPoints; i++) {
      overlappingFlagParent(i, 0) = 0;
    }

    // ===========================================================================
    // mexPrintf("search the pointCloud to fill the sector counter...\n");

    // search the child_pointcloud
    for (size_t i = 0; i < child_cloud.size(); i++) {
      double current_x = child_cloud[i].x;
      double current_y = child_cloud[i].y;

      double h_angle = atan2(current_y, current_x);

      int h_index = int((h_angle + M_PI)/(2*M_PI)*sectorNum);
      if(h_index < 0){
        h_index = 0;
      }else if(h_index > sectorNum - 1){
        h_index = sectorNum - 1;
      }

      sectorCounter(h_index, 0)++;
    }

    // search the parent_pointcloud
    for (size_t i = 0; i < parent_cloud.size(); i++) {
      double current_x = parent_cloud[i].x;
      double current_y = parent_cloud[i].y;

      double h_angle = atan2(current_y, current_x);

      int h_index = int((h_angle + M_PI)/(2*M_PI)*sectorNum);
      if(h_index < 0){
        h_index = 0;
      }else if(h_index > sectorNum - 1){
        h_index = sectorNum - 1;
      }

      sectorCounter(h_index, 1)++;
    }

    for (size_t i = 0; i < child_cloud.size(); i++) {
      double current_x = child_cloud[i].x;
      double current_y = child_cloud[i].y;

      double h_angle = atan2(current_y, current_x);

      int h_index = int((h_angle + M_PI)/(2*M_PI)*sectorNum);
      if(h_index < 0){
        h_index = 0;
      }else if(h_index > sectorNum - 1){
        h_index = sectorNum - 1;
      }

      #ifdef DEBUG
          mexPrintf("%s: %d-th %d\n", "child_points h_index: ", i, h_index);
      #endif

      if(sectorCounter(h_index, 0) >= overlappingThChild_ && sectorCounter(h_index, 1) >= overlappingThParent_){
        overlappingFlagChild(i, 0) = 1;
      }else{
        overlappingFlagChild(i, 0) = 0;
      }
    }

    for (size_t i = 0; i < parent_cloud.size(); i++) {
      double current_x = parent_cloud[i].x;
      double current_y = parent_cloud[i].y;

      double h_angle = atan2(current_y, current_x);

      int h_index = int((h_angle + M_PI)/(2*M_PI)*sectorNum);
      if(h_index < 0){
        h_index = 0;
      }else if(h_index > sectorNum - 1){
        h_index = sectorNum - 1;
      }

      if(sectorCounter(h_index, 0) >= overlappingThChild_ && sectorCounter(h_index, 1) >= overlappingThParent_){
        overlappingFlagParent(i, 0) = 1;
      }else{
        overlappingFlagParent(i, 0) = 0;
      }
    }

    // define the output of the mex function
    size_t dimArrayOfSectorCounter[2] = { size_t(sectorNum), size_t(2)};
    plhs[0] = mxCreateNumericArray(2, dimArrayOfSectorCounter, mxDOUBLE_CLASS, mxREAL);
    double *sectorCounterPtr = (double *)mxGetData(plhs[0]);

    for (int i = 0; i < sectorNum; i++) {
      for (int j = 0; j < 2; j++){
        sectorCounterPtr[j * sectorNum + i] = sectorCounter(i, j);
      }
    }

    size_t dimArrayOfOverlappingFlagChild[2] = { size_t(numOfChildPoints), size_t(1)};
    plhs[1] = mxCreateNumericArray(2, dimArrayOfOverlappingFlagChild, mxDOUBLE_CLASS, mxREAL);
    double *overlappingFlagChildPtr = (double *)mxGetData(plhs[1]);

    for (int i = 0; i < numOfChildPoints; i++) {
      overlappingFlagChildPtr[i] = overlappingFlagChild(i, 0);
    }

    size_t dimArrayOfOverlappingFlagParent[2] = { size_t(numOfParentPoints), size_t(1)};
    plhs[2] = mxCreateNumericArray(2, dimArrayOfOverlappingFlagParent, mxDOUBLE_CLASS, mxREAL);
    double *overlappingFlagParentPtr = (double *)mxGetData(plhs[2]);

    for (int i = 0; i < numOfParentPoints; i++) {
      overlappingFlagParentPtr[i] = overlappingFlagParent(i, 0);
    }
}
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
    // prhs[1] child pointCloud, Nx3, | x | y | z |
    // prhs[2] parent pointCloud, Nx3, | x | y | z |

    if(nrhs < 3)
      mexErrMsgIdAndTxt( "points3dSectorOverlappingMex:invalidNumInputs",
              "at least 3 input arguments required");
    else if(!mxIsStruct(prhs[0]))
        mexErrMsgIdAndTxt( "points3dSectorOverlappingMex:inputNotStruct",
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
      p_child.z = ptrChildPointCloud[(numOfChildPoints * 2) + i];;
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
      p_parent.z = ptrParentPointCloud[(numOfParentPoints * 2) + i];;
      parent_cloud.push_back(p_parent);
    }

    // construct the sectorCounter
    int sectorNum = int(360.0 / sectorRes_);
    MatrixXi sectorCounter(sectorNum, 2);

    pcl::PointCloud<pcl::PointXYZ> overlapped_child_cloud;
    pcl::PointCloud<pcl::PointXYZ> overlapped_parent_cloud;
    
    // init the sectorCounter to zeros
    for (int i = 0; i < sectorNum; i++) {
      for (int j = 0; j < 2; j++){
        sectorCounter(i, j) = 0;
      }
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

      if(sectorCounter(h_index, 0) >= overlappingThChild_ && sectorCounter(h_index, 1) >= overlappingThParent_){
        overlapped_child_cloud.push_back(child_cloud[i]);
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
        overlapped_parent_cloud.push_back(parent_cloud[i]);
      }
    }

    // define the output of the mex function
    size_t overlapped_child_cloud_num = overlapped_child_cloud.size();
    size_t dimArrayOfOverlappingChildPoints[2] = { size_t(overlapped_child_cloud_num), size_t(3)};
    plhs[0] = mxCreateNumericArray(2, dimArrayOfOverlappingChildPoints, mxDOUBLE_CLASS, mxREAL);
    double *overlappingChildPointsPtr = (double *)mxGetData(plhs[0]);

    for (int i = 0; i < overlapped_child_cloud_num; i++) {
      overlappingChildPointsPtr[(overlapped_child_cloud_num * 0) + i] = overlapped_child_cloud[i].x;
      overlappingChildPointsPtr[(overlapped_child_cloud_num * 1) + i] = overlapped_child_cloud[i].y;
      overlappingChildPointsPtr[(overlapped_child_cloud_num * 2) + i] = overlapped_child_cloud[i].z;
    }

    size_t overlapped_parent_cloud_num = overlapped_parent_cloud.size();
    size_t dimArrayOfOverlappingParentPoints[2] = { size_t(overlapped_parent_cloud_num), size_t(3)};
    plhs[1] = mxCreateNumericArray(2, dimArrayOfOverlappingParentPoints, mxDOUBLE_CLASS, mxREAL);
    double *overlappingParentPointsPtr = (double *)mxGetData(plhs[1]);

    for (int i = 0; i < overlapped_parent_cloud_num; i++) {
      overlappingParentPointsPtr[(overlapped_parent_cloud_num * 0) + i] = overlapped_parent_cloud[i].x;
      overlappingParentPointsPtr[(overlapped_parent_cloud_num * 1) + i] = overlapped_parent_cloud[i].y;
      overlappingParentPointsPtr[(overlapped_parent_cloud_num * 2) + i] = overlapped_parent_cloud[i].z;
    }
}
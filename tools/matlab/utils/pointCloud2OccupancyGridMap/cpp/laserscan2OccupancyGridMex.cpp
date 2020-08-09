#include<stdio.h>
#include"mex.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> MatrixXi;

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]){
    //nlhs represent the number of parameters of the output
    //plhs is a array of the mxarray pointers, each pointing to the output
    //nrhs represents the number of parameters of the input
    //prhs is a array of the mxarray pointers, each pointing to the input

    /* check proper input and output */

    // prhs[0] laserscan2OccupancyGridMapParams
    // prhs[1] laserscan, Nx2, | x | y |

    if(nrhs < 2)
      mexErrMsgIdAndTxt( "laserscan2OccupancyGrid:invalidNumInputs",
              "at least 2 input arguments required");
    else if(!mxIsStruct(prhs[0]))
        mexErrMsgIdAndTxt( "laserscan2OccupancyGrid:inputNotStruct",
                "the first input should be a struct");

    ///////////////////////////////////////////////////////////////////////////////////////////
    // get the params
    mxArray * fPtr;
    // half of the side length of the range square along the x axis
    double xRange_;
    // half of the side length of the range square along the y axis
    double yRange_;
    // length of the side square
    double resGrid_;

    // get the parameter : xRange_
    {
      fPtr = mxGetField(prhs[0], 0, "xRange");
      if ((fPtr != NULL)){
          double *realPtr = mxGetPr(fPtr);
          xRange_ = realPtr[0];
          #ifdef DEBUG
              mexPrintf("%s%f\n", "xRange: ", xRange_);
          #endif
      }else{
          mexErrMsgIdAndTxt( "levelingPoints2OccupancyGrid:invalidField",
              "field xRange not found in the params struct");
      }
    }

    // get the parameter : yRange_
    {
      fPtr = mxGetField(prhs[0], 0, "yRange");
      if ((fPtr != NULL)){
          double *realPtr = mxGetPr(fPtr);
          yRange_ = realPtr[0];
          #ifdef DEBUG
              mexPrintf("%s%f\n", "yRange: ", yRange_);
          #endif
      }else{
          mexErrMsgIdAndTxt( "levelingPoints2OccupancyGrid:invalidField",
              "field yRange not found in the params struct");
      }
    }

    // get the parameter : resGrid_
    {
      fPtr = mxGetField(prhs[0], 0, "resGrid");
      if ((fPtr != NULL)){
          double *realPtr = mxGetPr(fPtr);
          resGrid_ = realPtr[0];
          #ifdef DEBUG
              mexPrintf("%s%f\n", "resGrid: ", resGrid_);
          #endif
      }else{
          mexErrMsgIdAndTxt( "levelingPoints2OccupancyGrid:invalidField",
              "field resGrid not found in the params struct");
      }
    }

    // get the laserscan
    pcl::PointCloud<pcl::PointXYZ> cloud;
    const size_t *dimArrayOfRawLaserscan = mxGetDimensions(prhs[1]);
    double *ptrRawLaserscan = mxGetPr(prhs[1]);
    size_t numOfPoints = *dimArrayOfRawLaserscan;

    // generate the occupancyGrid
    int occupancyGridXNum = 2 * int(xRange_/resGrid_);
    int occupancyGridYNum = 2 * int(yRange_/resGrid_);
    MatrixXi occupancyGrid_(occupancyGridYNum, occupancyGridXNum);
    
    // init the occupancyGrid_ to zeros
    for (int i = 0; i < occupancyGridYNum; i++) {
      for (int j = 0; j < occupancyGridXNum; j++){
        occupancyGrid_(i, j) = 0;
      }
    }

    // ===========================================================================
    // mexPrintf("search the pointCloud to fill the occupancy counter...\n");

    for (size_t i = 0; i < numOfPoints; i++) {
      double current_x = ptrRawLaserscan[(numOfPoints * 0) + i];
      double current_y = ptrRawLaserscan[(numOfPoints * 1) + i];

      int x_index = int(current_x/resGrid_) + occupancyGridXNum/2;
      int y_index = int(current_y/resGrid_) + occupancyGridYNum/2;

      if(x_index > occupancyGridXNum - 1 || x_index < 0 || y_index > occupancyGridYNum - 1 || y_index < 0){
        continue;
      }else{
        occupancyGrid_(y_index, x_index) = 1;
      }
    }

    // define the output of the mex function
    size_t dimArrayOfOccupancyGrid[2] = { size_t(occupancyGridYNum), size_t(occupancyGridXNum)};
    plhs[0] = mxCreateNumericArray(2, dimArrayOfOccupancyGrid, mxDOUBLE_CLASS, mxREAL);
    double *occupancyGridPtr = (double *)mxGetData(plhs[0]);

    for (int i = 0; i < occupancyGridYNum; i++) {
      for (int j = 0; j < occupancyGridXNum; j++){
        occupancyGridPtr[j * occupancyGridYNum + i] = occupancyGrid_(i, j);
      }
    }
}
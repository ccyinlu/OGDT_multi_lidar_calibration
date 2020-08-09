#include<stdio.h>
#include"mex.h"

#include<ceres/ceres.h>
#include<ceres/cubic_interpolation.h>
using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry>

double map_res = 0;
double map_origin[2] = {0};
int map_width = 0;
int map_height = 0;

struct MAP_DT_COST
{
  MAP_DT_COST(ceres::BiCubicInterpolator<ceres::Grid2D<double,1>> map_dt_interpolator, Eigen::Matrix<double, 1, 2> laserScan_xy):_map_dt_interpolator(map_dt_interpolator),_laserScan_xy(laserScan_xy){}
  template <typename T>
  bool operator()(const T *cur2map_pose, T *residual)const
  {
    // convert the laserpoint to the map uv coordinates
    // Quaternion [w, x, y, z]
    double cur_x = _laserScan_xy(0, 0);
    double cur_y = _laserScan_xy(0, 1);

    T pose_x = cur2map_pose[0];
    T pose_y = cur2map_pose[1];
    T pose_yaw = cur2map_pose[2];

    T cur_x_transformed = cos(pose_yaw) * cur_x - sin(pose_yaw) * cur_y + pose_x;
    T cur_y_transformed = sin(pose_yaw) * cur_x + cos(pose_yaw) * cur_y + pose_y;

    T cur_u = (cur_x_transformed - map_origin[0])/map_res;
    T cur_v = double(map_height) - (cur_y_transformed - map_origin[1])/map_res;

    // if ( cur_u-2<0 || ( cur_u+2 ) > map_width || ( cur_v-2 ) <0 || ( cur_v+2 ) > map_height )
    // {
    //     residual[0] = T(10000.0);
    // }
    // else
    // {
        _map_dt_interpolator.Evaluate(cur_u, cur_v, &residual[0]);
    // }
    
    return true;
  }
  const ceres::BiCubicInterpolator<ceres::Grid2D<double,1>> _map_dt_interpolator;
  const Eigen::Matrix<double, 1, 2> _laserScan_xy;
};

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]){
    // nlhs represent the number of parameters of the output
    // plhs is a array of the mxarray pointers, each pointing to the output
    // nrhs represents the number of parameters of the input
    // prhs is a array of the mxarray pointers, each pointing to the input

    // prhs[0], MxN double, map_dt
    // prhs[1], Px2 double, laserScan_xy
    // prhs[2], 1x3 double, init tranform [x y yaw]
    // prhs[3], 1x1 double, map_res [meters]
    // prhs[4], 1x2 double, map_origin [meters] [x y]
    // prhs[5], 1x1 double, cauchy_c
    // prhs[6], 1x1 double, verbose

    if(nrhs < 6){
        mexErrMsgIdAndTxt( "estimateExtrinsic2dMapDtMatchingCeresMex:invalidNumInputs", "at least 6 input arguments required");
        return;
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // get the map_dt [0]
    const size_t *dimArrayOfMapDt = mxGetDimensions(prhs[0]);
    size_t sizeRowsMapDt = *(dimArrayOfMapDt + 0);
    size_t sizeColsMapDt = *(dimArrayOfMapDt + 1);

    map_width = sizeColsMapDt;
    map_height = sizeRowsMapDt;

    double *ptrMapDt = (double *)(mxGetPr(prhs[0]));

    ceres::Grid2D<double, 1> map_dt_array(ptrMapDt, 0, sizeRowsMapDt, 0, sizeColsMapDt);
    ceres::BiCubicInterpolator<ceres::Grid2D<double,1>> map_dt_interpolator(map_dt_array);

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // get the laserScan_xy [1]
    const size_t *dimArrayOfLaserScanXy = mxGetDimensions(prhs[1]);
    size_t sizeRowsLaserScanXy = *(dimArrayOfLaserScanXy + 0);
    size_t sizeColsLaserScanXy = *(dimArrayOfLaserScanXy + 1);
    if(sizeColsLaserScanXy != 2){
        mexErrMsgIdAndTxt( "estimateExtrinsic2dMapDtMatchingCeresMex:invalidInputs", "the 2st param should be Nx2");
        return;
    }

    double *ptrLaserScanXy = (double *)(mxGetPr(prhs[1]));

    #ifdef DEBUG
        mexPrintf("observation points number: %d\n", sizeRowsLaserScanXy);
    #endif

    // load the laserScan points to vector
    std::vector<Eigen::Matrix<double, 1, 2>>laserScan_xy;
    for(int i = 0; i < sizeRowsLaserScanXy; i++){
        Eigen::Matrix<double, 1, 2> current_laserScan_xy;
        for(int j = 0; j < 2; j++){
            current_laserScan_xy(0, j) = ptrLaserScanXy[j * sizeRowsLaserScanXy + i];
        }
        laserScan_xy.push_back(current_laserScan_xy);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // get the init transform pose [x y yaw] [2]
    double cur2map_pose[3] = {0};
    const size_t *dimArrayOfCur2MapPose = mxGetDimensions(prhs[2]);
    size_t sizeRowsCur2MapPose = *(dimArrayOfCur2MapPose + 0);
    size_t sizeColsCur2MapPose = *(dimArrayOfCur2MapPose + 1);
    if(sizeColsCur2MapPose != 2 && sizeRowsCur2MapPose != 1){
        mexErrMsgIdAndTxt( "estimateExtrinsic2dMapDtMatchingCeresMex:invalidInputs", "the 3st param should be 1x3");
        return;
    }
    double *ptrCur2MapPose = (double *)(mxGetPr(prhs[2]));
    for(int i = 0; i < 3; i++){
        cur2map_pose[i] = ptrCur2MapPose[i];
    }

    #ifdef DEBUG
        mexPrintf("init cur2map_pose: [%.4f,%.4f,%.4f]\n", \
                    cur2map_pose[0], \
                    cur2map_pose[1], \
                    cur2map_pose[2]);
    #endif

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // get the map_res [3]
    const size_t *dimArrayOfMapRes = mxGetDimensions(prhs[3]);
    size_t sizeRowsMapRes = *(dimArrayOfMapRes + 0);
    size_t sizeColsMapRes = *(dimArrayOfMapRes + 1);
    if(sizeRowsMapRes != 1 && sizeColsMapRes != 1){
        mexErrMsgIdAndTxt( "estimateExtrinsic2dMapDtMatchingCeresMex:invalidInputs", "the 4st param should be 1x1");
        return;
    }
    double *ptrMapRes = (double *)(mxGetPr(prhs[3]));
    map_res = ptrMapRes[0];

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // get the map_origin [4]
    const size_t *dimArrayOfMapOrigin = mxGetDimensions(prhs[4]);
    size_t sizeRowsMapOrigin = *(dimArrayOfMapOrigin + 0);
    size_t sizeColsMapOrigin = *(dimArrayOfMapOrigin + 1);
    if(sizeRowsMapOrigin != 1 && sizeColsMapOrigin != 2){
        mexErrMsgIdAndTxt( "estimateExtrinsic2dMapDtMatchingCeresMex:invalidInputs", "the 5st param should be 1x2");
        return;
    }
    double *ptrMapOrigin = (double *)(mxGetPr(prhs[4]));
    for(int i = 0; i < 2; i++){
        map_origin[i] = ptrMapOrigin[i];
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // get the cauchy_c [5]
    double cauchy_c = 0;
    const size_t *dimArrayOfCauchyC = mxGetDimensions(prhs[5]);
    size_t sizeRowsCauchyC = *(dimArrayOfCauchyC + 0);
    size_t sizeColsCauchyC = *(dimArrayOfCauchyC + 1);
    if(sizeRowsCauchyC != 1 && sizeColsCauchyC != 1){
        mexErrMsgIdAndTxt( "estimateExtrinsic2dMapDtMatchingCeresMex:invalidInputs", "the 6st param should be 1x1");
        return;
    }
    double *ptrCauchyC = (double *)(mxGetPr(prhs[5]));
    cauchy_c = ptrCauchyC[0];

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // get the verbose [6]
    double verbose = 0;
    const size_t *dimArrayOfVerbose = mxGetDimensions(prhs[6]);
    size_t sizeRowsVerbose = *(dimArrayOfVerbose + 0);
    size_t sizeColsVerbose = *(dimArrayOfVerbose + 1);
    if(sizeRowsVerbose != 1 && sizeColsVerbose != 1){
        mexErrMsgIdAndTxt( "estimateExtrinsic2dMapDtMatchingCeresMex:invalidInputs", "the 7st param should be 1x1");
        return;
    }
    double *ptrVerbose = (double *)(mxGetPr(prhs[6]));
    verbose = ptrVerbose[0];

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ceres::Problem problem;
    for(int i=0; i<sizeRowsLaserScanXy; i++)
    {
        problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<MAP_DT_COST, 1, 3>(
            new MAP_DT_COST(map_dt_interpolator, laserScan_xy[i])
        ),
        new ceres::CauchyLoss(cauchy_c),
        // nullptr,
        cur2map_pose
        );
    }

    #ifdef DEBUG
        mexPrintf("AddResidualBlock done!\n");
    #endif

    //配置求解器并求解，输出结果
    ceres::Solver::Options options;
    options.linear_solver_type=ceres::DENSE_QR;
    options.minimizer_progress_to_stdout=bool(verbose);
    ceres::Solver::Summary summary;
    ceres::Solve(options,&problem,&summary);

    #ifdef DEBUG
        mexPrintf("estimated cur2map_pose: [%.4f,%.4f,%.4f]\n", \
                    cur2map_pose[0], \
                    cur2map_pose[1], \
                    cur2map_pose[2]);
    #endif

    // the output cur2map_pose will be 1x3
    size_t dimArrayOfParams[2] = { 1, 3 };
    plhs[0] = mxCreateNumericArray(2, dimArrayOfParams, mxDOUBLE_CLASS, mxREAL);
    double *out_params = (double *)mxGetData(plhs[0]);

    for(int i = 0; i < 3; i++){
        out_params[i] = cur2map_pose[i];
    }
}
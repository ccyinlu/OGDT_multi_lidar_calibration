#include<stdio.h>
#include"mex.h"

#include<ceres/ceres.h>
using namespace std;

struct CURVE_FITTING_COST
{
  CURVE_FITTING_COST(double x,double y):_x(x),_y(y){}
  template <typename T>
  bool operator()(const T* const abc,T* residual)const
  {
    residual[0]=_y-ceres::exp(abc[0]*_x*_x+abc[1]*_x+abc[2]);
    return true;
  }
  const double _x,_y;
};

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]){
    // nlhs represent the number of parameters of the output
    // plhs is a array of the mxarray pointers, each pointing to the output
    // nrhs represents the number of parameters of the input
    // prhs is a array of the mxarray pointers, each pointing to the input

    // prhs[0], Nx1 double, x_data
    // prhs[1], Nx1 double, y_data

    if(nrhs < 2){
        mexErrMsgIdAndTxt( "curveFittingCeresExample:invalidNumInputs", "at least 2 input arguments required");
        return;
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // get the x_data
    const size_t *dimArrayOfXData = mxGetDimensions(prhs[0]);
    size_t sizeRowsXData = *(dimArrayOfXData + 0);
    size_t sizeColsXData = *(dimArrayOfXData + 1);
    if(sizeColsXData != 1){
        mexErrMsgIdAndTxt( "curveFittingCeresExample:invalidInputs", "the 1st param should be Nx1");
        return;
    }

    double *ptrXData = (double *)(mxGetPr(prhs[0]));
    vector<double> x_data;
    for(int i = 0; i < sizeRowsXData; i++){
        x_data.push_back(*(ptrXData + i));
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // get the y_data
    const size_t *dimArrayOfYData = mxGetDimensions(prhs[0]);
    size_t sizeRowsYData = *(dimArrayOfYData + 0);
    size_t sizeColsYData = *(dimArrayOfYData + 1);
    if(sizeColsYData != 1 || sizeRowsYData != sizeRowsXData){
        mexErrMsgIdAndTxt( "curveFittingCeresExample:invalidInputs", "the 2st param should be Nx1");
        return;
    }

    double *ptrYData = (double *)(mxGetPr(prhs[1]));
    vector<double> y_data;
    for(int i = 0; i < sizeRowsYData; i++){
        y_data.push_back(*(ptrYData + i));
    }

    #ifdef DEBUG
        mexPrintf("observation number: %d\n", sizeRowsXData);
    #endif


    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //反复使用AddResidualBlock方法（逐个散点，反复N次）
    //将每个点的残差累计求和构建最小二乘优化式
    //不使用核函数，待优化参数是abc
    double abc[3]={0,0,0};
    ceres::Problem problem;
    for(int i=0;i<sizeRowsXData;i++)
    {
        problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<CURVE_FITTING_COST,1,3>(
            new CURVE_FITTING_COST(x_data[i],y_data[i])
        ),
        nullptr,
        abc
        );
    }

    #ifdef DEBUG
        mexPrintf("AddResidualBlock done!\n");
    #endif

    //配置求解器并求解，输出结果
    ceres::Solver::Options options;
    options.linear_solver_type=ceres::DENSE_QR;
    options.minimizer_progress_to_stdout=true;
    ceres::Solver::Summary summary;
    ceres::Solve(options,&problem,&summary);
    #ifdef DEBUG
        mexPrintf("a = %f, b = %f, c = %f\n", abc[0], abc[1], abc[2]);
    #endif

    // the output xi will be 3x1
    size_t dimArrayOfParams[2] = { 3, 1 };
    plhs[0] = mxCreateNumericArray(2, dimArrayOfParams, mxDOUBLE_CLASS, mxREAL);
    double *out_params = (double *)mxGetData(plhs[0]);

    for(int i = 0; i < 3; i++){
        out_params[i] = abc[i];
    }
}
#include "mex.h"
#include "UGVNavigation.h"

// Usage: First in Matlab type: mex obstacleRecognition.cpp UGVNavigation.cpp
// then you can simply call obstacleRecognition(distances, thetas) in Matlab.

void mexFunction( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[])
{
    double multiplier;              /* input scalar */
    double *distances;
    double *thetas;               /* 1xN input matrix */
    size_t readings;                   /* size of matrix */
    double *outMatrix;              /* output matrix */

    /* check for proper number of arguments */
    if(nrhs!=2) {
        mexErrMsgIdAndTxt("MyToolbox:arrayProduct:nrhs","Two inputs required.");
    }
    
    /* make sure the second input argument is type double */
    if( !mxIsDouble(prhs[1]) || 
         mxIsComplex(prhs[1])) {
        mexErrMsgIdAndTxt("MyToolbox:arrayProduct:notDouble","Input matrix must be type double.");
    }
    
    /* check that number of rows in second input argument is 1 */
    if(mxGetM(prhs[1])!=1) {
        mexErrMsgIdAndTxt("MyToolbox:arrayProduct:notRowVector","Input must be a row vector.");
    }
    
    /* get dimensions of the input matrix */
    readings = mxGetN(prhs[1]);

    if(mxGetN(prhs[0])!=readings){
        mexErrMsgIdAndTxt("GapFinding:obstacleRecognition:inputNotSameSize","The two input vector should have the same size.");
    }
    /* create a pointer to the real data in the input matrix  */
    distances = mxGetPr(prhs[0]);
    thetas = mxGetPr(prhs[1]);


    /* call the computational routine */
    std::vector<double> res = obstacleRecognition(distances, thetas, readings);

    /* create the output matrix */
    plhs[0] = mxCreateDoubleMatrix(1,(mwSize)res.size(),mxREAL);

    /* get a pointer to the real data in the output matrix */
    outMatrix = mxGetPr(plhs[0]);
    outMatrix[0] = 2;

    std::copy(res.begin(), res.end(), outMatrix);

    /* get a pointer to the real data in the output matrix */
    std::cout<<outMatrix[0]<<outMatrix[1]<<outMatrix[2]<<outMatrix[3];
    /* create the output matrix */
    std::cout<<res.size();

}
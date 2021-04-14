#include "mex.h"

#include "code/CollisionDetector.h"

void mexFunction( int nlhs, mxArray *plhs[], 
		  int nrhs, const mxArray*prhs[])
     
{ 
  /* Check for proper number of arguments */    
  if (nrhs != 4) { 
    mexErrMsgIdAndTxt( "MATLAB:planner:invalidNumInputs",
              "Four input arguments required."); 
  } else if (nlhs != 2) {
    mexErrMsgIdAndTxt( "MATLAB:planner:maxlhs",
              "Two output argument required."); 
  } 
      
  double x0 = *mxGetPr(prhs[0]);
  double y0 = *mxGetPr(prhs[1]);
  double x1 = *mxGetPr(prhs[2]);
  double y1 = *mxGetPr(prhs[3]);

  std::vector<std::pair<int,int>> indices = CollisionDetector::raster_line(x0,y0,x1,y1);

  int num_indices = indices.size();

  // Create output plan
  plhs[0] = mxCreateNumericMatrix( (mwSize)num_indices, (mwSize)2, mxDOUBLE_CLASS, mxREAL); 
  double* plan_out = mxGetPr(plhs[0]);

  for(int i = 0; i < num_indices; i++)
  {
    plan_out[i] = indices.at(i).first;
    plan_out[i+num_indices] = indices.at(i).second;
  }

  // Create output plan size
  plhs[1] = mxCreateNumericMatrix( (mwSize)1, (mwSize)1, mxINT16_CLASS, mxREAL); 
  int* planlength_out = (int*) mxGetPr(plhs[1]);
  *planlength_out = num_indices; 

  return;
}
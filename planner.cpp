#include <math.h>
#include <vector>
#include "mex.h"

#include "code/MultiGoalRRTConnect.h"
#include "code/planner_utils.h"

/* Input Arguments */
#define	START_IN	prhs[0]
#define	GOAL_IN     prhs[1]
#define	VEHICLE_DIMS_IN     prhs[2]
#define	MAP_IN     prhs[3]
#define	CELL_SIZE_IN     prhs[4]

/* Output Arguments */
#define	PLAN_OUT	plhs[0]
#define	PLANLENGTH_OUT	plhs[1]

void mexFunction( int nlhs, mxArray *plhs[], 
		  int nrhs, const mxArray*prhs[])
     
{ 
  /* Check for proper number of arguments */    
  if (nrhs != 5) { 
    mexErrMsgIdAndTxt( "MATLAB:planner:invalidNumInputs",
              "Five input arguments required."); 
  } else if (nlhs != 2) {
    mexErrMsgIdAndTxt( "MATLAB:planner:maxlhs",
              "Two output argument required."); 
  } 
      
  /* get the start and goal states*/     
  int state_size = (int) (MAX(mxGetM(START_IN), mxGetN(START_IN)));
  if(state_size <= 2){
    mexErrMsgIdAndTxt( "MATLAB:planner:invalidstatesize",
              "it should be at least 3");         
  }
  double* start_array = mxGetPr(START_IN);
  State start_state (start_array, start_array + state_size);

  if (state_size != MAX(mxGetM(GOAL_IN), mxGetN(GOAL_IN))){
            mexErrMsgIdAndTxt( "MATLAB:planner:invalidstatesize",
              "statesize of goalstate is different from startstate");         
  }
  double* goal_array = mxGetPr(GOAL_IN);
  State goal_state (goal_array, goal_array + state_size);

  // get the vehicle dimensions
  int vehicle_dims_size = (int) (MAX(mxGetM(VEHICLE_DIMS_IN), mxGetN(VEHICLE_DIMS_IN)));
  if (vehicle_dims_size != 2){
            mexErrMsgIdAndTxt( "MATLAB:planner:invalid_vehicle_dims_size",
              "vehicle dims should be of size 2 (length,width)");         
  }
  double* vehicle_dims = mxGetPr(VEHICLE_DIMS_IN);
  double vehicle_length = vehicle_dims[0];
  double vehicle_width = vehicle_dims[1];

  // get the occupancy grid
  int x_size = (int) mxGetM(MAP_IN);
  int y_size = (int) mxGetN(MAP_IN);
  bool* occupancy_grid = mxGetLogicals(MAP_IN);

  // get the cell size
  double cell_size = *mxGetPr(CELL_SIZE_IN);
  printf("Cell size: %f\n", cell_size);

  // call the planner
  MultiGoalRRTConnect planner(vehicle_length, vehicle_width, x_size, y_size, occupancy_grid, cell_size);
  std::vector<State> plan = planner.plan(start_state, goal_state);

  int planlength = plan.size();
  printf("planner returned plan of length %d\n", planlength); 

  // Create output plan
  PLAN_OUT = mxCreateNumericMatrix( (mwSize)planlength, (mwSize)state_size, mxDOUBLE_CLASS, mxREAL); 
  double* plan_out = mxGetPr(PLAN_OUT);

  for(int i = 0; i < planlength; i++)
  {
    for (int j = 0; j < state_size; j++)
    {
      plan_out[j*planlength + i] = plan.at(i).at(j);
    }
  }

  // Create output plan size
  PLANLENGTH_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)1, mxINT8_CLASS, mxREAL); 
  int* planlength_out = (int*) mxGetPr(PLANLENGTH_OUT);
  *planlength_out = planlength;

  return;
}
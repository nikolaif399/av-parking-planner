#include <math.h>
#include <vector>
#include "mex.h"

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

/* Input Arguments */
#define	START_IN	prhs[0]
#define	GOAL_IN     prhs[1]

/* Output Arguments */
#define	PLAN_OUT	plhs[0]
#define	PLANLENGTH_OUT	plhs[1]

typedef std::vector<double> State;

inline double GetRand(double min, double max) {
  return min + (max - min) * (double) rand()/RAND_MAX;
}

State get_intermediate_state(State q1, State q2, double r, int n) {
  State q_interm(n);

  for (int i = 0; i < n; ++i) {
    q_interm[i] = q1[i] + r*(q2[i] - q1[i]);
  }
  return q_interm;
}

static void planner(
      State start_state,
      State goal_state,
      std::vector<State> &plan)
{
  int state_size = start_state.size();

	// Arbitrary plan to test mex bindings
  int plan_length = 100;
  plan.resize(plan_length);

  // Just interpolate from start to goal
  for (int i = 0; i < plan.size(); ++i) {
    plan.at(i) = get_intermediate_state(start_state,goal_state,(double)i/(plan_length-1),state_size);
  }
}

void mexFunction( int nlhs, mxArray *plhs[], 
		  int nrhs, const mxArray*prhs[])
     
{ 
  /* Check for proper number of arguments */    
  if (nrhs != 2) { 
    mexErrMsgIdAndTxt( "MATLAB:planner:invalidNumInputs",
              "Two input arguments required."); 
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
  
  //call the planner
  std::vector<State> plan;
  planner(start_state, goal_state, plan);   
  int planlength = plan.size();
  printf("planner returned plan of length=%d\n", planlength); 

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
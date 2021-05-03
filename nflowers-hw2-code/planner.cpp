/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
#include <math.h>
#include "mex.h"

#include "src/helpers.h"
#include "src/tree.h"
#include "src/planner_base.h"
#include "src/rrt.h"
#include "src/rrt_connect.h"
#include "src/rrt_star.h"
#include "src/prm.h"

/* Input Arguments */
#define	MAP_IN      prhs[0]
#define	ARMSTART_IN	prhs[1]
#define	ARMGOAL_IN     prhs[2]
#define	PLANNER_ID_IN     prhs[3]

/* Planner Ids */
#define RRT         0
#define RRTCONNECT  1
#define RRTSTAR     2
#define PRM         3

/* Output Arguments */
#define	PLAN_OUT	plhs[0]
#define	PLANLENGTH_OUT	plhs[1]
#define	VERTICES_OUT	plhs[2]

static void planner(
      double*	map,
      int x_size,
      int y_size,
      double* armstart_anglesV_rad,
      double* armgoal_anglesV_rad,
      int numofDOFs,
      int planner_id,
      double*** plan,
      int* planlength,
      int* numVertices)
{
	//no plan by default
	*plan = NULL;
	*planlength = 0;

  // First verify that start and goal configurations are valid (randomly generated)
  if (!IsValidArmConfiguration(armstart_anglesV_rad, numofDOFs, map, x_size, y_size) ||
      !IsValidArmConfiguration(armgoal_anglesV_rad, numofDOFs, map, x_size, y_size)) {
    printf("Invalid config passed in.\n");
    return;
  }

  State q_init(armstart_anglesV_rad, armstart_anglesV_rad + numofDOFs);
  State q_goal(armgoal_anglesV_rad, armgoal_anglesV_rad + numofDOFs);

  printf("Planning from ");
  PrintState(q_init);
  printf(" to ");
  PrintState(q_goal);
  printf("\n");

  srand (1);

  std::shared_ptr<PlannerBase> planner_obj;
  switch(planner_id) {
  case RRT:
    printf("Using RRT.\n");
    planner_obj = std::make_shared<PlannerRRT>(numofDOFs,map,x_size,y_size,20000,0.3,0.1);
    break;
  case RRTCONNECT:
    printf("Using RRT-Connect.\n");
    planner_obj = std::make_shared<PlannerRRTConnect>(numofDOFs,map,x_size,y_size,2000,0.3);
    break;
  case RRTSTAR:
    printf("Using RRT-Star.\n");
    planner_obj = std::make_shared<PlannerRRTStar>(numofDOFs,map,x_size,y_size,5000,0.3,0.1);
    break;
  case PRM:
    printf("Using PRM.\n");
    planner_obj = std::make_shared<PlannerPRM>(numofDOFs,map,x_size,y_size, 500, 1);
    break;
  }

  planner_obj->plan(q_init,q_goal, plan, planlength);
  *numVertices = planner_obj->getVertices();

  return;
}

//prhs contains input parameters (3): 
//1st is matrix with all the obstacles
//2nd is a row vector of start angles for the arm 
//3nd is a row vector of goal angles for the arm 
//plhs should contain output parameters (2): 
//1st is a 2D matrix plan when each plan[i][j] is the value of jth angle at the ith step of the plan
//(there are D DoF of the arm (that is, D angles). So, j can take values from 0 to D-1
//2nd is planlength (int)
void mexFunction( int nlhs, mxArray *plhs[], 
		  int nrhs, const mxArray*prhs[])
     
{ 
    /* Check for proper number of arguments */    
    if (nrhs != 4) { 
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidNumInputs",
                "Four input arguments required."); 
    } else if (nlhs != 3) {
	    mexErrMsgIdAndTxt( "MATLAB:planner:maxlhs",
                "Three output argument required."); 
    } 
        
    /* get the dimensions of the map and the map matrix itself*/     
    int x_size = (int) mxGetM(MAP_IN);
    int y_size = (int) mxGetN(MAP_IN);
    double* map = mxGetPr(MAP_IN);
    
    /* get the start and goal angles*/     
    int numofDOFs = (int) (MAX(mxGetM(ARMSTART_IN), mxGetN(ARMSTART_IN)));
    if(numofDOFs <= 1){
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidnumofdofs",
                "it should be at least 2");         
    }
    double* armstart_anglesV_rad = mxGetPr(ARMSTART_IN);
    if (numofDOFs != MAX(mxGetM(ARMGOAL_IN), mxGetN(ARMGOAL_IN))){
        	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidnumofdofs",
                "numofDOFs in startangles is different from goalangles");         
    }
    double* armgoal_anglesV_rad = mxGetPr(ARMGOAL_IN);
 
    //get the planner id
    int planner_id = (int)*mxGetPr(PLANNER_ID_IN);
    if(planner_id < 0 || planner_id > 3){
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidplanner_id",
                "planner id should be between 0 and 3 inclusive");         
    }
    
    //call the planner
    double** plan = NULL;
    int planlength = 0;
    int numVertices;
    planner(map,x_size,y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, planner_id, &plan, &planlength,&numVertices); 
    
    printf("planner returned plan of length=%d\n", planlength); 

    if(planlength > 0)
    {
        PLAN_OUT = mxCreateNumericMatrix( (mwSize)planlength, (mwSize)numofDOFs, mxDOUBLE_CLASS, mxREAL); 
        double* plan_out = mxGetPr(PLAN_OUT);        
        //copy the values
        int i,j;
        for(i = 0; i < planlength; i++)
        {
            for (j = 0; j < numofDOFs; j++)
            {
                plan_out[j*planlength + i] = plan[i][j];
            }
        }
    }
    else
    {
        PLAN_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)numofDOFs, mxDOUBLE_CLASS, mxREAL); 
        double* plan_out = mxGetPr(PLAN_OUT);
        //copy the values
        int j;
        for(j = 0; j < numofDOFs; j++)
        {
                plan_out[j] = armstart_anglesV_rad[j];
        }     
    }

    PLANLENGTH_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)1, mxINT8_CLASS, mxREAL); 
    int* planlength_out = (int*) mxGetPr(PLANLENGTH_OUT);
    *planlength_out = planlength;

    VERTICES_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)1, mxINT16_CLASS, mxREAL);
    int* vertices_out = (int*) mxGetPr(VERTICES_OUT);
    *vertices_out = numVertices;

    return;
}






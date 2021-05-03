#ifndef RRT_STAR_PLANNER_H
#define RRT_STAR_PLANNER_H

#include "rrt.h"

class PlannerRRTStar : public PlannerRRT{
public:
  PlannerRRTStar(int dof, double *map, int x_size, int y_size, int k, double eps, double goal_prob);

protected:
  void extend(State q_sample, bool connect2goal, int& ret_flag, int& new_index) override;

  void rewire(int nearest_index, int new_index);

  bool reached_goal = false;
};


#endif
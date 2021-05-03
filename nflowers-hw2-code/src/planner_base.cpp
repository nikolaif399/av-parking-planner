#include "planner_base.h"
#include "helpers.h"
#include <iostream>

PlannerBase::PlannerBase(int dof, double *map, int x_size, int y_size) {
  dof_ = dof;
  x_size_ = x_size;
  y_size_ = y_size;

  map_ = new double[x_size_ * y_size_];
  memcpy(map_, map, x_size_ * y_size_ * sizeof(double));
}

PlannerBase::~PlannerBase() {
  delete[] map_;
}

std::vector<double> PlannerBase::random_valid_sample() {
  std::vector<double> rand_sample(dof_);

  while(true) {
    for (int i = 0; i < dof_; ++i) {
      // Wrap from -2pi to 2pi (allows for one wrap)
      rand_sample[i] = GetRand(0,2*PI);//((double) rand() / (RAND_MAX)) * 4 * PI - 2*PI;
    }

    if (IsValidArmConfiguration(rand_sample.data(),dof_,map_,x_size_,y_size_)) {
      return rand_sample;
    }
  }
}

std::pair<State,bool> PlannerBase::getNewState(State q1, State q2, double eps) {
  double dist = StateDistance(q1,q2);
  if (dist < eps) return std::make_pair(q2,true);
  
  return std::make_pair(this->getIntermediateState(q1,q2,eps/dist),false);
}

State PlannerBase::getIntermediateState(State q1, State q2, double r) {
  State q_interm(dof_);

  for (int i = 0; i < dof_; ++i) {
    q_interm[i] = q1[i] + r*(q2[i] - q1[i]);
  }
  return q_interm;
}

// True means valid, false means collision
bool PlannerBase::checkCollision(State q1, State q2, int N = 100) {
  double dist = StateDistance(q1,q2); // normalized difference

  int numPoints = dist * N;

  for (int i = 0; i <= numPoints; ++i) {
    double r = static_cast<double>(i)/static_cast<double>(numPoints);
    State q = this->getIntermediateState(q1,q2,r);
    if (!IsValidArmConfiguration(q.data(), dof_, map_, x_size_, y_size_)) return false;
  }

  return true;
}

std::vector<State> PlannerBase::interpolatePath(std::vector<State> plan_states) {
  int N = 25;

  std::vector<State> new_path;
  int num_points;

  for (int i = 0; i < plan_states.size() - 1; ++i) {
    double dist = StateDistance(plan_states[i], plan_states[i+1]);
    num_points = MAX(dist * N,2);
    for (int j = 0 ; j < num_points; ++j) {
      double r = double(j)/double(num_points);
      State interp_state = this->getIntermediateState(plan_states[i], plan_states[i+1], r);
      new_path.push_back(interp_state);
    }
  }

  return new_path;
}

std::vector<State> PlannerBase::shortcutPath(std::vector<State> plan_states) {
  bool converged = true;

  int i = 0;
  std::vector<State> new_path;
  new_path.push_back(plan_states.front());

  while(i < 20) {
    State active = new_path.back();

    for (int comp_ind = plan_states.size() - 1; comp_ind >= 0; comp_ind--) {
      State comp = plan_states.at(comp_ind);
      if (checkCollision(active,comp,500)) {
        // Shorter collision free path found!
        new_path.push_back(comp);
        if (comp_ind == plan_states.size() - 1) {
          return new_path;
        }
        break;
      }
    }
    i++;
  }
  
  return new_path;
}

void PlannerBase::transferRefinedPlan(double*** plan, int* plan_length, std::vector<State> plan_states) {
  // Allocate plan matrix
  *plan_length = plan_states.size();
  *plan = (double**) malloc(*plan_length*sizeof(double*));

  // Copy elements over to plan
  for (int i = 0; i < plan_states.size(); ++i) {
    (*plan)[i] = (double*) malloc(dof_*sizeof(double));
    for (int j = 0; j < dof_; ++j) {
      (*plan)[i][j] = plan_states[i][j];
    }
  }
}
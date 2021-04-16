#include "MultiGoalRRTConnect.h"
#include <iostream>

MultiGoalRRTConnect::MultiGoalRRTConnect(double vehicle_length, double vehicle_width, int x_size, int y_size, bool* occupancy_grid, double cell_size) {
  vehicle_length_ = vehicle_length;
  vehicle_width_ = vehicle_width;
  x_size_ = x_size;
  y_size_ = y_size;
  cell_size_ = cell_size;

  state_lo_ = {0,0,-M_PI};
  state_lo_ = {x_size*cell_size,y_size*cell_size,M_PI};

  collision_detector_ = std::make_shared<CollisionDetector>(vehicle_length, vehicle_width, x_size, y_size, occupancy_grid, cell_size);
}

// Simple plan just to test mex bindings
std::vector<State> MultiGoalRRTConnect::planSimple(State start_state, State goal_state) {

	// Arbitrary plan to test mex bindings
  int plan_length = 200;
  std::vector<State> plan(plan_length);

  // Just interpolate from start to goal
  for (int i = 0; i < plan.size(); ++i) {
    plan.at(i) = GetIntermediateState(start_state,goal_state,(double)i/(plan_length-1));
    bool collision = collision_detector_->checkCollision(plan.at(i));
    //printf("Collision at index %d : %d\n", i, collision);
    plan.at(i).back() = collision ? 1 : 0;
  }

  return plan;
}

// Implement main plan here
std::vector<State> MultiGoalRRTConnect::plan(State start_state, State goal_state) {

  int plan_length = 200;
  std::vector<State> plan(plan_length);

  return plan;
}

std::pair<State,bool> MultiGoalRRTConnect::get_new_state(State q1, State q2, double eps) {
  double dist = StateDistance(q1,q2);
  if (dist < eps) return std::make_pair(q2,true);
  
  return std::make_pair(GetIntermediateState(q1,q2,eps/dist),false);
}

State MultiGoalRRTConnect::random_valid_sample() {
  int num_in_state = state_lo_.size();
  State rand_sample(num_in_state);

  while(true) {
    for (int i = 0; i < num_in_state; ++i) {
      rand_sample[i] = GetRand(state_lo_.at(i),state_hi_.at(i));
    }

    if (!collision_detector_->checkCollision(rand_sample)) {
      return rand_sample;
    }
  }
}

std::vector<State> MultiGoalRRTConnect::interpolatePath(std::vector<State> plan_states) {
  int N = 25;

  std::vector<State> new_path;
  int num_points;

  for (int i = 0; i < plan_states.size() - 1; ++i) {
    double dist = StateDistance(plan_states[i], plan_states[i+1]);
    num_points = MAX(dist * N,2);
    for (int j = 0 ; j < num_points; ++j) {
      double r = double(j)/double(num_points);
      State interp_state = GetIntermediateState(plan_states[i], plan_states[i+1], r);
      new_path.push_back(interp_state);
    }
  }

  return new_path;
}

std::vector<State> MultiGoalRRTConnect::shortcutPath(std::vector<State> plan_states) {
  bool converged = true;

  int i = 0;
  std::vector<State> new_path;
  new_path.push_back(plan_states.front());

  while(i < 20) {
    State active = new_path.back();

    for (int comp_ind = plan_states.size() - 1; comp_ind >= 0; comp_ind--) {
      State comp = plan_states.at(comp_ind);
      if (collision_detector_->checkCollisionLine(active,comp,500)) {
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
#include "MultiGoalRRTConnect.h"
#include <iostream>
using namespace std;


MultiGoalRRTConnect::MultiGoalRRTConnect(double vehicle_length, double vehicle_width, int x_size, int y_size, bool* occupancy_grid, double cell_size, int k, double eps) {
  vehicle_length_ = vehicle_length;
  vehicle_width_ = vehicle_width;
  x_size_ = x_size;
  y_size_ = y_size;
  cell_size_ = cell_size;

  k_ = k;
  eps_ = eps;

  start_tree_ = std::make_shared<Tree>(state_lo_.size());

  state_lo_ = {0,0,-1.5*M_PI};
  state_hi_ = {x_size*cell_size,y_size*cell_size,1.5*M_PI};
  
  collision_detector_ = std::make_shared<CollisionDetector>(vehicle_length, vehicle_width, x_size, y_size, occupancy_grid, cell_size);
  state_connector_ = std::make_shared<ReedsSheppStateSpace>(6); // Turning radius
}

std::vector<State> MultiGoalRRTConnect::getPath(int start_ind, int start_ind_mid, int goal_ind, int goal_ind_mid, std::shared_ptr<Tree> goal_tree) {
  // Compute plan from start to midpoint through start tree
  std::vector<State> plan_states_start = this->start_tree_->getPath(start_ind, start_ind_mid);
  
  // Compute plan from goal to midpoint through goal tree
  std::vector<State> plan_states_goal = goal_tree->getPath(goal_ind, goal_ind_mid);
  
  // Flip path through goal tree
  std::reverse(plan_states_goal.begin(), plan_states_goal.end());

  // Concatenate start -> midpoint path with midpoint -> goal path
  std::vector<State> plan_states;
  plan_states.insert(plan_states.begin(), plan_states_start.begin(), plan_states_start.end());
  plan_states.insert(plan_states.end(), plan_states_goal.begin(), plan_states_goal.end());
  //printf("Raw plan size: %zu\n", plan_states.size());

  //return plan_states;
  //Remove shortcuttable states
  plan_states = this->shortcutPath(plan_states);
  //printf("Plan size after shortcutting: %zu\n", plan_states.size());

  // Interpolate along path to get smooth trajectory
  plan_states = this->interpolatePath(plan_states);
  //printf("Plan size after interpolating: %zu\n", plan_states.size());

  return plan_states;
}

// Implement main planner here
std::vector<State> MultiGoalRRTConnect::plan(State start_state, std::vector<State> goal_states) {

  if (collision_detector_->checkCollision(start_state)) {
    throw std::invalid_argument(" START STATE NOT VALID!");
  }

  // Add start state to start tree
  int start_ind = start_tree_->addVertex(start_state);

  // Create one goal tree for each of the goal states
  std::vector<int> goal_inds;
  for (int i = 0; i < goal_states.size(); ++i) {
    if (collision_detector_->checkCollision(goal_states.at(i))) {
      std::string msg = " GOAL STATE " + std::to_string(i) + " NOT VALID!";
      throw std::invalid_argument(msg);
    }
    
    goal_trees_.push_back(std::make_shared<Tree>(state_lo_.size()));
    goal_inds.push_back(goal_trees_.back()->addVertex(goal_states.at(i)));
  }

  std::vector<std::vector<State>> possible_paths;

  int goal_tree_cur_index = 0;
  
  cur_tree_ = start_tree_;
  other_tree_ = goal_trees_.at(goal_tree_cur_index);
  cur_equal_start_ = true;

  // RRT-Connect iterations
  for (int i = 0; i < k_; ++i) { 

    // Draw a random valid sample from the state space
    State q_sample = this->random_valid_sample();

    // Try to extend the current tree towards to the new sample
    int ret_flag,new_index_extend;
    // cout<<"before extending"<<endl;
    this->extend(q_sample, cur_tree_, ret_flag, new_index_extend);

    // cout<<"after extending"<<endl;
    // cout<<"ret flag is"<<ret_flag<<endl;
    // If not trapped, try to connect to other tree
    if (ret_flag != TRAPPED) {
      State q_new = cur_tree_->getState(new_index_extend);
      int new_index_connect;

      this->connect(q_new, ret_flag, new_index_connect);

      // If connect reached, we have a path from start to goal!
      // Compute the index of the midpoint state in both trees
      if (ret_flag == REACHED) {
        //printf("Reached goal!!\n");
        //printf("Goal tree index: %d\n", goal_tree_cur_index);
        int start_ind_mid,goal_ind_mid;
        if (cur_equal_start_) {
          start_ind_mid = new_index_extend;
          goal_ind_mid = new_index_connect;
        }
        else {
          start_ind_mid = new_index_connect;
          goal_ind_mid = new_index_extend;
        }
        possible_paths.push_back(this->getPath(start_ind, start_ind_mid, goal_inds.at(goal_tree_cur_index), goal_ind_mid, goal_trees_.at(goal_tree_cur_index)));
        goal_inds.erase(goal_inds.begin() + goal_tree_cur_index);
        goal_trees_.erase(goal_trees_.begin() + goal_tree_cur_index);
        if (goal_trees_.empty()) {
            std::vector<State> best_plan = possible_paths.front();
            double best_plan_quality = getPlanCost(best_plan);
            for (int i = 0; i < possible_paths.size(); ++i) {
                double plan_quality = getPlanCost(possible_paths.at(i));
                if (plan_quality < best_plan_quality) {
                    best_plan = possible_paths.at(i);
                    best_plan_quality = plan_quality;
                }
            }
            return best_plan;
        }
      }
    }

    // Exchange trees
    std::shared_ptr<Tree> temp_cur = cur_tree_;
    if (cur_equal_start_) {
      // Move to next index in goal tree vector
      goal_tree_cur_index++;
      if (goal_tree_cur_index >= goal_trees_.size()) {
        goal_tree_cur_index = 0;
      }
      cur_tree_ = goal_trees_.at(goal_tree_cur_index);
      other_tree_ = start_tree_;
    }
    else {
      if (goal_tree_cur_index >= goal_trees_.size()) {
        goal_tree_cur_index = 0;
      }
      cur_tree_ = start_tree_;
      other_tree_ = goal_trees_.at(goal_tree_cur_index);
    }

    cur_equal_start_ = !cur_equal_start_;
  }
}

void MultiGoalRRTConnect::extend(State q_sample, std::shared_ptr<Tree> tree_extending_from, int& ret_flag, int& new_index) {
  // Get nearest neighbor to qrand
  int nearest_index = tree_extending_from->getNearestVertex(q_sample);

  // Couldn't find new point to extend towards goal
  if (nearest_index == -1) {
    ret_flag = TRAPPED;
    //cout<<"Trapped"<<endl;
    return;
  }
  
  // Extend tree towards nearest state in other tree
  State q_nearest = tree_extending_from->getState(nearest_index);
  std::pair<State,bool> new_state = this->get_new_state(q_nearest,q_sample,eps_);
  State q_new = new_state.first;
  // cout<<"checking for collisions"<<endl;
  // Check for collision using twenty intermediate states (can be increased if cutting through obstacles)
  if(!collision_detector_->checkCollisionLine(q_nearest, q_new, 20)) {
    ret_flag = new_state.second ? REACHED : ADVANCED;
    new_index = tree_extending_from->addVertex(q_new);
    tree_extending_from->addEdge(nearest_index, new_index);
    return;
  }
  
  ret_flag = TRAPPED;
  new_index = -1;
  return;
}

void MultiGoalRRTConnect::connect(State q_new, int& ret_flag, int& new_index) {
  while(true) {
    this->extend(q_new, other_tree_, ret_flag, new_index);
    if (ret_flag != ADVANCED) break;
  }
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

  while(i < 100) {
    State active = new_path.back();

    for (int comp_ind = plan_states.size() - 1; comp_ind >= 0; comp_ind--) {
      State comp = plan_states.at(comp_ind);
      if (!collision_detector_->checkCollisionLine(active,comp,500)) {
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

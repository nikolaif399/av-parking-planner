#include "prm.h"

PlannerPRM::PlannerPRM(int dof, double *map, int x_size, int y_size, int N, double neighborhood) :
  PlannerBase(dof, map, x_size, y_size){
  N_ = N;
  neighborhood_ = neighborhood;

  graph_ = std::make_shared<Graph>(dof);
  
}

int PlannerPRM::getVertices() {
  return graph_->getSize();
}

void PlannerPRM::plan(State q_init, State q_goal, double*** plan, int* plan_length) {
  std::vector<State> nodes(N_);
  
  int start_index = graph_->addState(q_init);
  int goal_index = graph_->addState(q_goal);

  for (int i = 0; i < N_; ++i) {
    State q_sample = this->random_valid_sample();
    int sample_index = graph_->addState(q_sample);

    std::vector<int> neighbors = graph_->getKNearestNeigbors(q_sample,20);

    for (int neighbor : neighbors) {
      // Verify no collision
      if (!this->checkCollision(q_sample,graph_->getState(neighbor),20)) continue;

      //printf("Sample index component size before edge: %d\n", graph_->getComponentSize(sample_index));
      graph_->addEdge(sample_index, neighbor);
      //printf("Sample index component size after: %d\n", graph_->getComponentSize(sample_index));
    }

    if (graph_->connected(start_index, goal_index)) {
      printf("Start and goal connected!\n");
      std::vector<State> plan_states = graph_->search(start_index,goal_index);
      printf("Path found with size: %zu\n", plan_states.size());

      plan_states = this->shortcutPath(plan_states);
      printf("Plan size after shortcutting: %zu\n", plan_states.size());

      plan_states = this->interpolatePath(plan_states);
      printf("Plan size after interpolating: %zu\n", plan_states.size());

      graph_->saveGraph(x_size_);

      this->transferRefinedPlan(plan, plan_length, plan_states);
      return;
    }
  }
  graph_->saveGraph(x_size_);
}
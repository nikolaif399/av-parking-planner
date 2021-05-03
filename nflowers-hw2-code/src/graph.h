#ifndef PLANNER_GRAPH_H
#define PLANNER_GRAPH_H

#include "helpers.h"
#include <map>
#include <vector>
#include <fstream>
#include <unordered_set>
#include <algorithm>
#include <queue>

class Graph{
public:
  Graph(int dof);

  ~Graph() = default;

  int addState(State q);

  void addEdge(int ind1, int ind2);

  void saveGraph(int x_size);

  State getState(int index);

  std::vector<int> getKNearestNeigbors(State q, int k);

  std::vector<int> getNeighbors(State q, double neighborhood);

  bool connected(int ind1, int ind2);

  std::vector<State> search(int start_index, int goal_index);

  std::vector<State> getPath(int start_index, int goal_index, std::map<int,int> parents);

  int getComponentSize(int index);

  int getSize() {return cur_index_;};

private:
  std::map<int,std::vector<std::pair<int,double>>> conn_;
  std::map<int,State> data_;
  std::map<int,std::unordered_set<int>> component_;

  int cur_index_ = 0;
  int dof_;
};


#endif
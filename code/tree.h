#ifndef PLANNER_TREE_H
#define PLANNER_TREE_H

#include <vector>
#include <algorithm>
#include <map>
#include <limits>
#include <iostream>
#include <fstream>
#include <bits/stdc++.h>

#include "planner_utils.h"

class Tree {
public:
  Tree(int num_dofs);

  int addVertex(State q);

  void addEdge(int ind1, int ind2);

  int getNearestVertex(State q);

  int getNearestVertexExcluding(State q, std::unordered_set<int> exclude);

  std::vector<int> getNeighborVertices(State q, double epsilon);

  State getState(int index); 

  // void saveTree(int x_size, std::string filename);

  void debugPrint();

  std::vector<State> getPath(int start_index, int goal_index);

  int getSize() {return num_vertices_;}

  int num_vertices_ = 0;

  int num_edges_ = 0;

  int num_dofs_;

private:

  std::map<int,int> parents_; // Value is node immediately prior to the key

  std::map<int,State> states_;

  int next_vertex_index_ = 0;

};

#endif
#ifndef PLANNER_UTILS_H
#define PLANNER_UTILS_H

#include <math.h>
#include <vector>
#include "mex.h"
#include <iostream>
#include "reeds_shepp.h"
using namespace std;

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) (Y*XSIZE + X)

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

typedef std::vector<double> State;

// Print out a state
inline void PrintState(State s) {
  printf("State: [");
  for (int i = 0; i < s.size()-1; ++i) {
    printf("%.2f, ", s.at(i));
  }
  printf("%.2f]\n", s.back());
}

inline double getPlanCost(std::vector<State> plan) {
    double d = 0;
    double weights[3] = {1,1,20};

    for (int j = 0; j < plan.size()-1; ++j) {
        for (int i  = 0; i < 3; ++i) {
            d += weights[i]*abs(plan.at(j).at(i) - plan.at(j+1).at(i));
        }
    }
    return d;
}

// Get a random double between min and max
inline double GetRand(double min, double max) {
  return min + (max - min) * (double) rand()/RAND_MAX;
}

/* Returns distance between states, used in nearest neighbors search */
inline double StateDistance(State q1, State q2) {

  // replacing manhattan distance with the length of the Reeds-Shepp path
  double Q1[3];
  double Q2[3];

  for (int i  = 0; i < 3; i++) {
    Q1[i] = q1[i];
    Q2[i] = q2[i];
  }

  ReedsSheppStateSpace rs_ = ReedsSheppStateSpace(6);
  ReedsSheppStateSpace::ReedsSheppPath rspath_ = rs_.reedsShepp(Q1,Q2);
  double d = rspath_.length();
  return d;
}

/* Get intermediate state between q1 and q2 (r is the ratio along that vector
  r = 0 will return q1
  r = 0.5 will return halfway between q1 and q2
  r = 1 with return q2, etc
  */
inline State GetIntermediateState(State q1, State q2, double ratio) {
  
  // State q_interm(q1.size());
  // for (int i = 0; i < q1.size(); ++i) {
  //   q_interm[i] = q1[i] + ratio*(q2[i] - q1[i]);
  // }
  // return q_interm;


  // computing the point that is at a distance of ratio*length_of_reedshepppath along the reedshepp path
  double Q1[3];
  double Q2[3];
  State q_interm(q1.size());

  for (int i  = 0; i < 3; i++) {
    Q1[i] = q1[i];
    Q2[i] = q2[i];
  }

  ReedsSheppStateSpace rs_ = ReedsSheppStateSpace(6);
  ReedsSheppStateSpace::ReedsSheppPath rspath_ = rs_.reedsShepp(Q1,Q2);
  double interpolated_state_[3];
  rs_.interpolate(Q1,rspath_, (ratio)*rspath_.length(), interpolated_state_);

  for (int i = 0; i < 3; i++) {
    q_interm[i] = interpolated_state_[i];
  }
  return q_interm;
}

#endif
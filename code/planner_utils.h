#ifndef PLANNER_UTILS_H
#define PLANNER_UTILS_H

#include <math.h>
#include <vector>
#include "mex.h"

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

// Get a random double between min and max
inline double GetRand(double min, double max) {
  return min + (max - min) * (double) rand()/RAND_MAX;
}

/* Returns distance between states, used in nearest neighbors search */
inline double StateDistance(State q1, State q2) {
  double d = 0;

  // Just manhattan distance for now
  for (int i  = 0; i < 2; ++i) {
    d += abs(q1[i] - q2[i]);
  }

  return d;
}

/* Get intermediate state between q1 and q2 (r is the ratio along that vector
  r = 0 will return q1
  r = 0.5 will return halfway between q1 and q2
  r = 1 with return q2, etc
  */
inline State GetIntermediateState(State q1, State q2, double r) {
  State q_interm(q1.size());

  for (int i = 0; i < q1.size(); ++i) {
    q_interm[i] = q1[i] + r*(q2[i] - q1[i]);
  }
  return q_interm;
}

#endif
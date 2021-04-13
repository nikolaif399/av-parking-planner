#ifndef PLANNER_UTILS_H
#define PLANNER_UTILS_H

#include <math.h>
#include <vector>

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) (Y*XSIZE + X)

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif


typedef std::vector<double> State;

inline double GetRand(double min, double max) {
  return min + (max - min) * (double) rand()/RAND_MAX;
}

// Get state 
inline State get_intermediate_state(State q1, State q2, double r) {
  State q_interm(q1.size());

  for (int i = 0; i < q1.size(); ++i) {
    q_interm[i] = q1[i] + r*(q2[i] - q1[i]);
  }
  return q_interm;
}

#endif
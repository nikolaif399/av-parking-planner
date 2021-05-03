function[armplan] = armplanner(envmap, armstart, armgoal, planner_id)
%call the planner in C
[armplan, armplanlength,numVertices] = planner(envmap, armstart, armgoal, planner_id);


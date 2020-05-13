#include "controller_2link_planar/planner_cartesian.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, NAME_OF_THIS_NODE);
  
  planner_cartesian planner_cartesian_node;
   
  planner_cartesian_node.Prepare();
  
  planner_cartesian_node.RunPeriodically(planner_cartesian_node.RunPeriod);
   
  planner_cartesian_node.Shutdown();
  
  return (0);
}


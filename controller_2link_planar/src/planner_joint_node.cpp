#include "controller_2link_planar/planner_joint.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, NAME_OF_THIS_NODE);
  
  planner_joint planner_joint_node;
   
  planner_joint_node.Prepare();
  
  planner_joint_node.RunPeriodically(planner_joint_node.RunPeriod);
   
  planner_joint_node.Shutdown();
  
  return (0);
}


#include "controller_2link_planar/controller_joint.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, NAME_OF_THIS_NODE);
  
  controller_joint controller_joint_node;
   
  controller_joint_node.Prepare();
  
  controller_joint_node.RunPeriodically(controller_joint_node.RunPeriod);
   
  controller_joint_node.Shutdown();
  
  return (0);
}


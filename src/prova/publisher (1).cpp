
#include "ros/ros.h"

#define RUN_PERIOD_DEFAULT 1.0

 
#define NAME_OF_THIS_NODE "Publisher"
    



#include "std_msgs/String.h"
#include "std_msgs/Float32.h"

    

class Publisher
{
  private: 
    ros::NodeHandle Handle;
    
    ros::Publisher Publisher;
    
    void PeriodicTask(int count);
    
    public:
    double RunPeriod;
    
    
    void Prepare(void)
    
    void RunPeriodically(float Period);
    
    void Shutdown(void);
    
};

//-----------------------------------------------------------------
//-----------------------------------------------------------------

void Publisher::Prepare()
{
  RunPeriod = RUN_PERIOD_DEFAULT;

  std::string FullParamName;
  
  
   Publisher = Handle.advertise<msg_pkg::Float32.h>("r", 1000);
  /***********************************************************
   * Uncomment if your node publishes to the topic called
   * name_of_the_topic. Substitute length_of_the_queue with an
   * integer defining how many messages the output queue should
   * be able to contain.
   * msg_type must be substituted with the name of the type of 
   * message that your node will publish on the topic; msg_pkg 
   * must be substituted with the name of the ROS package which
   * contains the .msg file defining such message type.
   * A package which defines many common ROS message types is
   * std_msgs: a list of the message types it provides is here:
   * http://www.ros.org/doc/api/std_msgs/html/index-msg.html
   *
   * One such instruction is required for each topic that the node
   * publishes to.
   */
   
  // Client = Handle.serviceClient<name_of_server_package::name_of_the_srv_file>("name_of_the_service");
  /***********************************************************
   * Uncomment if your node needs to act as a client of a service
   * called name_of_the_service, provided by a ROS server defined
   * by file name_of_the_srv_file.srv which is part of the ROS
   * package called name_of_the_server_package.
   *
   * Add one similar statement for each additional ROS server
   * that your node needs to access as a client.
   */
   
  // Service = Handle.advertiseService("name_of_the_service", &ROSnode::ProvideService, this);
  //ROS_INFO("ROS service %s available (provided by node %s).", "name_of_the_service", ros::this_node::getName().c_str());
  /***********************************************************
   * Uncomment the first statement if your node acts as a ROS
   * server, providing a service called name_of_the_service
   * to clients. The service is implemented by method
   * ProvideService.
   * Also uncomment the second statement if you want to highlight
   * the availability of the service, for instance for debugging
   * purposes.
   *
   * Add similar statements for each additional ROS service that
   * your node provides.
   */
   
  // TimeoutTimer = Handle.createTimer(ros::Duration(duration_of_the_timeout), &ROSnode::TimeoutCallback, this, put_here_true_or_false);
  /***********************************************************
   * Uncomment if your node requires a timeout. Substitute 
   * duration_of_the_timeout with the required value, in seconds,
   * expressed as a float (e.g., 14.0).
   * put_here_true_or_false should be substituted with true or
   * false. In the first case, after the timeout expires method
   * TimeoutCallback will be called only once ("once-only"
   * timeout). In the second case, once the timeout expires,
   * TimeoutCallback is called periodically, with period equal to
   * duration_of_the_timeout.
   * This statement also starts the timeout.
   *
   * You need one such instruction for each TimeoutTimer
   * attribute that you defined above.
   */
  
  /***********************************************************
   * RETRIEVING PARAMETER VALUES FROM THE ROS PARAMETER SERVER
   *
   * Uncomment the following piece of code if you want to
   * retrieve a parameter named my_param from the ROS parameter
   * server, and store its value in variable ParamVar (which
   * you need to have declared as a member variable of class
   * ROSNode; you should choose whether to make ParamVar a public
   * or private variable depending on who needs to access it).
   *
     // FullParamName = ros::this_node::getName()+"/my_param";
     //uncomment this if my_param is a private parameter of the
     //node, i.e. if its full name is 
     // /name_of_the_namespace/NAME_OF_THIS_NODE/my_param
     
     // FullParamName = Handle.getNamespace()+"my_param";
     //uncomment this if, instead, my_param is a global parameter
     //of the namespace that the node belongs to, i.e. if its
     //full name is /name_of_the_namespace/my_param
 
     if (true == Handle.getParam(FullParamName, ParamVar))
     {
       ROS_INFO("Node %s: retrieved parameter %s.",
       ros::this_node::getName().c_str(), FullParamName.c_str());
     }
     else
     {
       ROS_ERROR("Node %s: unable to retrieve parameter %s.",
       ros::this_node::getName().c_str(), FullParamName.c_str());
     }
   *
   * You need one piece of code like this for each parameter
   * value that your node needs to retrieve from the ROS parameter
   * server.
   */
   
   ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}




void Publisher::RunPeriodically(float Period)
{
  ros::Rate LoopRate(1.0/Period);
  
  ROS_INFO("Node %s running periodically (T=%.2fs, f=%.2fHz).", ros::this_node::getName().c_str(), Period, 1.0/Period);
  
  int count=0;
  while (ros::ok())
  {
    PeriodicTask(count);
    count++;
    ros::spinOnce();
    /* From ROS documentation:
     * "ros::spinOnce() will call all the callbacks waiting to be
     * called at that point in time. ." */
    
    LoopRate.sleep();
  }
}


void ROSnode::Shutdown(void)
{
  ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
  
  /* 
   * ROS does not require you to do anything to shut down cleanly
   * your node. However, if your node needs to do do something
   * before shutting down, put the relevant code here.
   */
}



void ROSnode::PeriodicTask(void)
{
  std_msgs::String msg;
  std_msgs::Float msg;
  std::stringstream ss	
 */


/*************************************************************
 * Uncomment the following block if you defined the corresponding
 * method of ROSnode.
 *
void ROSnode::UseService(void)
{
  Server.request.name_of_input = expression of appropriate type;
  
  where name_of_input must be substituted with the correct name of
  the input field, as defined by the .srv file of the server.
  
  if (Client.call(Server))
  {
    variable of appropriate type = Server.response.name_of_output;
    
    where name_of_output must be substituted with the correct name
    of the output field, as defined by the .srv file of the server.
  }
  else
  {
    Put here the code to manage the "server not responding" 
    condition. 
    Please note that if the server node is active but is not
    running at the time of the request by the client because it
    runs periodically and is currently sleeping, this is NOT a
    "server not responding" condition. The request of the client
    is queued by ROS, and the client will get the response at
    the next run of the server.
  }
}
 *
 */


/*************************************************************
 * Uncomment the following block if you defined the corresponding
 * method of ROSnode.
 *
bool ROSnode::ProvideService(name_of_the_server_package::name_of_the_srv_file::Request  &Req,  name_of_the_server_package::name_of_the_srv_file::Response &Res)
{
  suppose that the .srv file defines two input fields called in1
  and in2, and two output fields called out1 and out2. Then you
  have to write something like:
    
  Res.out1 = expression using Req.in1 and Req.in2;
  Res.out2 = expression using Req.in1 and Req.in2;
}
 *
 */


/*************************************************************
 * Uncomment the following blocks if you defined the corresponding
 * methods of ROSnode.
 *
void ROSnode::StopTimeout(void)
{
  TimeoutTimer.stop();
  ...add here similar statements to stop other timers, if needed
}

void ROSnode::StartTimeout(void)
{
  TimeoutTimer.setPeriod(ros::Duration(duration_of_the_timeout));
  ...substitute duration_of_the_timeout with the required value, in
  seconds, expressed as a float (e.g., 14.0)  
  TimeoutTimer.start();
  
  ...add here similar statements to start other timers, if needed
}
 *
 */


//-----------------------------------------------------------------
//-----------------------------------------------------------------


int main(int argc, char **argv)
{
  ros::init(argc, argv, NAME_OF_THIS_NODE);
  /* NOTE: the call to ros::init should be the FIRST statement of
   * the 'main' block. You get compilation errors if you use any
   * part of the ROS system before that statement. */

  /* NOTE. If this node is launched with rosrun, a new node
   * gets added to the running ROS system, the name of which is
   * the string assigned to NAME_OF_THIS_NODE. If the node is
   * launched with roslaunch, it is possible that this choice of
   * name has been overridden, and that the newly added node takes
   * different name than NAME_OF_THIS_NODE. This happens when the
   * <node> statement in the launchfile used to launch the node
   * specifies a name for the node (which is not mandatory). */
  
  ROSnode MyNode;
   
  MyNode.Prepare();
  
  // MyNode.RunContinuously();
  // MyNode.RunPeriodically(MyNode.RunPeriod);
  /*
   * Uncomment ONE AND ONLY ONE of the above statements.
   */ 
   
  MyNode.Shutdown();
  
  return (0);
}


/*************************************************************
 * NOTE. If you want this file to be compiled and the
 * corresponding executable to be generated, remember to add a
 * suitable rosbuild_add_executable line to the CMakeLists.txt
 * file of your package. [Warning: the contents of this note is
 * valid for versions of ROS that use the (older) rosbuild
 * build system. They may be obsolete if your version of ROS is,
 * instead, based on the newer catkin build system.]
 *************************************************************/

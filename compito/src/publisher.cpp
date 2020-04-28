#include "ros/ros.h"

#define RUN_PERIOD_DEFAULT 1
#define X 2
#define Y 3
#define NAME_OF_THIS_NODE "publisher"

#include "std_msgs/Float32.h"
/*************************************************************
 * Uncomment if your node publishes or receives ROS messages.
 * name_of_the_msg_type must be substituted with the name of
 * the type of ROS message that your node uses; 
 * name_of_the_msg_package must be substituted with the name of
 * the ROS package where the .msg file that defines the internal
 * structure of such type of message is located.
 * A package which defines many common ROS message types is
 * std_msgs: a list of the message types it provides is here:
 * http://www.ros.org/doc/api/std_msgs/html/index-msg.html
 *
 * If your node uses more than one type of message, one #include
 * directive for each one of those types must be used.
 *
 * Remember that the internal structure of the messages used by
 * your node must be defined by a dedicated .msg file. Such file
 * can be part of a standard ROS package or be part of your own
 * package. In the second case, the CMakeLists.txt file of your 
 * package should include the rosbuild_gensrv() line, to enable
 * generation of the relevant .h file during compilation.
 */
 
//-----------------------------------------------------------------
//-----------------------------------------------------------------

class Publisher
{
  private: 
    ros::NodeHandle Handle;
    
    ros::Publisher Publisher;

    void PeriodicTask(int count);    
    
    float operazione (int t);
    
  public:
    double RunPeriod;
    float a;
    float b;
    
    void Prepare(void);
    /* prepares the node for running: all the preparatory tasks
     * that the node has to perform (both due to the interaction
     * with the ROS system and to your particular application)
     * should be performed by this method */
    
    void RunPeriodically(float Period);
    /* used to run the node if it has to run periodically; Period
     * is the distance in time between runs, in seconds.
     * WARNING: this method and RunContinuously should never be
     * both called for the same node. */
    
    void Shutdown(void);
    /* performs all the activities that the node is required to 
     * perform immediately before it shuts down
     * NOTE: all the elements of the node that are provided by ROS
     * do not require you to do anything to shut down a node; this 
     * method is provided as a container for code that your own
     * node may be required to run in order to shut down its
     * activities in a clean way. */
};

//-----------------------------------------------------------------
//-----------------------------------------------------------------

void Publisher::Prepare(void)
{
  RunPeriod = RUN_PERIOD_DEFAULT;
  
  Publisher = Handle.advertise<std_msgs::Float32>("R", 1000);

   a = (float) X * Y;
   b = (float) X / Y;
   
   ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}

void Publisher::RunPeriodically(float Period)
{
  ros::Rate LoopRate(1.0/Period);
  
  ROS_INFO("Node %s running periodically (T=%.2fs, f=%.2fHz).", ros::this_node::getName().c_str(), Period, 1.0/Period);
  int count = 0;

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

void Publisher::Shutdown(void)
{
  ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
}

void Publisher::PeriodicTask(int count)
{
  std_msgs::Float32 msg;//crea un'instanza di un  messaggio (in questo caso di tipo std_msg::String
  msg.data = operazione(count);

  Publisher.publish(msg);//pubblica il messaggio nel topic 
  ROS_INFO("I've published: '%f'", msg.data);
}

float Publisher::operazione (int t){
  float r = a*t*t + b*t;
  return r;
}

//-----------------------------------------------------------------
//-----------------------------------------------------------------


int main(int argc, char **argv)
{
  ros::init(argc, argv, NAME_OF_THIS_NODE);
  
  Publisher publisher;

  publisher.Prepare();
  
  publisher.RunPeriodically(publisher.RunPeriod);
   
  publisher.Shutdown();
  
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

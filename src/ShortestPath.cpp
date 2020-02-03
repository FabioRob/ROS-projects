// Ex. 5 : Shortest Path      

/*
      
                 TWO
               /\| \
               / |  \                            
              /  |   \                                           
             /   \/   \					  |0  1  1  0|
            / __FOUR_  \				A=|0  0  1  1|
           / /       /\ \				  |1  0  0  1|
          / /         \  \  		  |1  0  0  0|
         / /           \  \        
        / /             \  \
       / \/              \ \/ 
      ONE <------------> THREE
      
      
Each robot communicates with the next two(1-->2,3 ; 2-->3,4; 3-->4,1) except for the last which communicates only with the next one(4-->1)
Implementation of Bellman-Ford algorithm to build spanning tree from source node one
*/

#include "ros/ros.h"
#include <vector>
#include <array>
#include <iostream>

#include "ros_ex/Short.h" // custom message for Bellman-Ford shortest path algorithm

/*
  methods(except for header) of message Short
  
  std::string name; // robot's name
  std::string gen;  // robot's parent
  double d;	    // distance from source

*/

struct Edge 
{
  int tail;
  int head;
  double weight;  
};
				      // class SHORT
class SHORT
{
public:
  ros::NodeHandle n;
  
  static std::string robot_name;
  int node_number;
  
  const int number_of_nodes = 4;
  std::vector<Edge> Edges; // set of communication's grapgh edges, in the example (cost set to 1 for all edges) : { {1,2,1},{1,3,1},{2,3,1},{2,4,1},{3,4,1},{3,1,1},{4,1,1} }
  std::array<std::array<double,4>,4> A; // adjacency matrix of the communication graph 

  std::vector<ros::Publisher> status_pub; // publishers to other robots' Short messages
  std::vector<ros::Subscriber> status_sub; // subscribers from other robots' Short messages
  std::vector<int> inStar; // array of nodes from which this node received
  std::array<double,4> inCosts; // costs of in edges ( non existing edges will be set to inf cost)
  
  static std::array<ros_ex::Short,4> robots_state; // states received from other robot(components respective to not communicating robots will be empty)
  
  // in-edge and out_edge costs ( theese will be all set to 1.0 for simplicity)
  std::vector<double> costs_in;
  std::vector<double> costs_out;
  
  ros_ex::Short self_status; // robot's status: contains name,gen and d
  
// member functions

  static int getNodeNumber(std::string node_name);
  std::string getNodeName(int num);
  void set_adj_matrix(std::vector<Edge> E); // build adjacency matrix from edges set Edges:  A={ {0,1,1,0} {0,0,1,1} {1,0,0,1} {1,0,0,0} }
  void setCommunications(); // create publishers and subscribers adjacency from matrix A
  
  void initAlgorithm();
  
  void publishMsg(); // publish status message to all communicating robots
  
  static void statusCallback  (const ros_ex::Short::ConstPtr& msg);
  
  void step ();
  
  void Algorithm();
  
};

					    // main
int main(int argc, char **argv)
{
  ros::init(argc,argv,"shortest_path");
  
  ros::NodeHandle node("~");
  
  SHORT S;
  S.n=node;
  
  if (node.getParam("robot_name", S.robot_name) )
    ROS_INFO("Setting robot name: %s", S.robot_name.c_str());
  
  S.Edges = { {1,2,1}, {1,3,1}, {2,3,1}, {2,4,1}, {3,4,1}, {3,1,1}, {4,1,1} } ; // edit to change costs if desired
  S.set_adj_matrix(S.Edges);  
  S.setCommunications();
  
  S.initAlgorithm();
  
  sleep(1.0);
  
  S.Algorithm();
  
 return 0; 
 
}


						// class SHORT

// define SHORT's static member variables
std::string SHORT::robot_name;
std::array<ros_ex::Short,4> SHORT::robots_state;

// define SHORT's member functions
int SHORT::getNodeNumber(std::string node_name)
{
  if (node_name=="turtle1") return 1;
  if (node_name=="turtle2") return 2;
  if (node_name=="turtle3") return 3;
  if (node_name=="turtle4") return 4;

}

std::string SHORT::getNodeName(int num)
{
  if (num==1) return "turtle1";
  if (num==2) return "turtle2";
  if (num==3) return "turtle3";
  if (num==4) return "turtle4";
  
}

void SHORT::set_adj_matrix(std::vector<Edge> E)
{
  for ( int raw=0; raw<4 ; ++raw)
  {
   for ( int column=0; column<4; ++column)
   {
      for (unsigned int ei=0; ei<E.size() ; ++ei)
      {
	     if ( (E[ei].tail -1 ==raw && E[ei].head - 1==column)  )   // NB indexs of raws and columns are shifted by one in respect to
	     {							                                           // to node number
	       A[raw][column] = E[ei].weight;
	       break;
	     }
	     else
	       A[raw][column] = 0;	
      }
   }
  }
  
  if (robot_name=="turtle1")
  { 
    ROS_INFO(" Adjacency matrix of the communication graph is:");
    for ( int raw=0; raw<4 ; ++raw)  // print adjacency matrix
    {
      std::cout<<"| ";
      for ( int column=0; column<4; ++column)
      {
	std::cout<< A[raw][column] << " " ;
      }
      std::cout << "|\n";
    }    
  }
  
}

void SHORT::setCommunications()
{
  int node_num = getNodeNumber(robot_name);
  for (int i=0 ; i<4 ; i++) 
  {
    if ( A[node_num - 1][i] != 0 ) // create publishers
    {
      status_pub.push_back( n.advertise<ros_ex::Short>( "/edge_" + robot_name + "_" + getNodeName(i+1) , 100) ); // queue size = 100
      //ROS_INFO(" creating publisher: /edge_%s_%s",  robot_name.c_str(), getNodeName(i+1).c_str() );
    }
  }
  
  inCosts = {30000, 30000, 30000, 30000};
  for (int i=0; i<4 ; i++)
  {
     if ( A[i][node_num-1] != 0 ) // create subscribers
     {
      status_sub.push_back( n.subscribe( "/edge_" + getNodeName(i+1) + "_" + robot_name , 100, statusCallback) ); // queue size = 100
      //ROS_INFO(" creating subscriber: /edge_%s_%s",   getNodeName(i+1).c_str() , robot_name.c_str() ); 
      inStar.push_back(i+1);
      inCosts[i]=(A[i][node_num-1]); // set in edges costs
     }  
  }
 
}

void SHORT::initAlgorithm()
{
 
  for (int i=0; i<4; i++)
    robots_state[i].d=30000; 

 self_status.name = robot_name;
 self_status.gen = " -- ";
 if (robot_name=="turtle1")
 {
  ROS_INFO(" %s : I'm the source node" , robot_name.c_str() );
  self_status.d = 0.0; 
  self_status.gen = "turtle1";
 }
 else
 {
   self_status.d = 30000.0 ; // 30000 stats for "infinite" cost
 }
}

void SHORT::publishMsg()
{
  self_status.header.stamp = ros::Time::now();
  
  for (unsigned int i=0; i<status_pub.size(); i++)
  {
    status_pub[i].publish(self_status); 
  
  /*  if (self_status.d != 30000)
      ROS_INFO(" %s publish d = %f  to %s ", robot_name.c_str() , self_status.d , status_pub[i].getTopic().c_str()  );
    else 
      ROS_INFO(" %s publish d = Inf  to %s ", robot_name.c_str() , status_pub[i].getTopic().c_str()  );
   */
  }
}


void SHORT::statusCallback  (const ros_ex::Short::ConstPtr& msg)
{
  int node = getNodeNumber(msg->name);
  
  robots_state.at(node-1)=*msg;
  if (msg->d != 30000)
    ROS_INFO(" %s received d= %f from %s ", robot_name.c_str(), msg->d , msg->name.c_str());
  else
    ROS_INFO(" %s received d= Inf from %s ", robot_name.c_str(), msg->name.c_str()); 
 
}


void SHORT::step()
{
  
  int min = 100000;
  int min_index;
  for (int i=0; i<4 ; ++i)
  {   
      if ( robots_state[i].d + inCosts[i] < min ) 
      {
	min=robots_state[i].d + inCosts[i];
	min_index = i;
      }
  }
  
  if (min < self_status.d)
  {
   self_status.d=min;
   self_status.gen = getNodeName(min_index+1); 
  }
  
}


void SHORT::Algorithm()
{
  int N_cycles=0;
  std::string step_number;
  while ( ros::ok() && n.ok() &&  (N_cycles < number_of_nodes-1) ) // three steps
  {
    ros::Time start = ros::Time::now();
    ros::Rate rate = 1;

    while (ros::ok() && n.ok() && (ros::Time::now()-start).toSec()<1.0)
    {
      ros::spinOnce();
      publishMsg();
      rate.sleep();
    }

    step();
    
    switch (N_cycles)
    {
      case 0:
      {
	step_number="first";
	break;
      }
      case 1:
      {
	step_number="second";
	break;
      }
      case 2:
	step_number="third";
    }
    if (self_status.d != 30000)
      ROS_WARN(" After %s step, %s has parent %s and distance %f" , step_number.c_str(), robot_name.c_str(), self_status.gen.c_str(), self_status.d );
    else
      ROS_WARN(" After %s step, %s has parent %s and distance Inf" , step_number.c_str(), robot_name.c_str(), self_status.gen.c_str());
    
    sleep(1.0);
  
    ++N_cycles;
  }
  
}

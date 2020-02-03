// Ex. 3 : Consensus          

/* HP: Number of robots=3 ; Communication graph is complete(3 robots are all communicating with each other):

				2
                               / \
                              /   \
                             /     \
                            1-------3                                                              
                                
*/                                                                
                                
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <iostream>
#include <cmath>
#include <array>

#include "ros_ex/Status_cons.h"  // custom message for consensus algorithm
/*
 methods(except for header) of message Status_cons:
 
 std::string robot_id; // robot's name("turtle1" ... "turtle3");
 double x; // x position
 double y; // y position
 int step_number; // current step of the consensus' algorithm
 bool next_step; // true when all communicating robots have completed current step
 bool consensus; // true if consensus has been reached(status becomes constant)

*/

const double pi = 3.141592654;
					  // class ComunicationGraph
class CommunicationGraph
{
public:
  static std::string robot_name; 
 
  std::array<std::array<int,3>,3> A; // adjacency matrix of the communication graph

  std::vector<ros::Publisher> status_pub; // publishers to other robots' Status_cons messages
  std::vector<ros::Subscriber> status_sub; // subscribers from other robots' Status_cons messages
  
// member function
  
  void set_adj_matrix(); // build adjacency matrix: 3 robots are all communicating with each other--> A={ {0,1,1} {1,0,1} {1,1,0} }
  
};

					// class CONSENSUS
class CONSENSUS : public CommunicationGraph
{
public:
// member variables  
  ros::NodeHandle n;
  
  static turtlesim::Pose cur_pos; // store current position of the robot
  
  ros::Subscriber pos_sub; // subscriber to /turtle#/pose
  
  ros_ex::Status_cons self_status; // status of the robot at time k (robot_id, x , y , step_number, next_step, consensus)
  ros_ex::Status_cons passed_status; // status of the robot at time k-1
  
  static std::array<ros_ex::Status_cons,3> robots_status;  // contains status of all robots
                                              // in order from turtle1 to turtle3
  // following variables are used to synchronize the robots communications
  static int num_of_msgs; // it counts how many messages have been received on current step of consensus' algorithm 
  static std::array<bool, 3> states_received; // each element is set to true if the respective state has been received on current step
  static std::vector<bool> other_ready; // set to true only when other robots have completed current step
  
  bool team_ready; // set to true when consensus has been achieved
  
  double int_step; // integration step of consensus' algorithm: it must be less than 1/dmax=1/2=0.5 (each turle communicates with other two so dmax=2)
  int iter; // iteration number of consensus' algorithm
  
// member functions

  static void position_callback(const turtlesim::Pose::ConstPtr& pos_msg); // store position in variable cur_pose
  
  void init();
  
  static int get_node_number(std::string name); // "turtle1" corresponds to 1, "turtle2" corresponds to 2 and so on
  std::string get_node_name(int num); // 1 corresponds to "turtle1" ,  2 corresponds to "turtle2" and so on
  
  static void statusCallback(const ros_ex::Status_cons::ConstPtr& msg); // process states of the other robots
  
  void setCommunications(); // build publishers and subscribers from adjacency matrix A (create a publisher and a subscriber for each robot communicating with)
  
  void publish_status(ros::Publisher pub); // publish self_status to communicating robots
  
  void step(); // step of the discrete consensus' algorithm
  
  void ConsensusAlgorithm(); // 
  
}; 

				// class CONTROLLER
class CONTROLLER : public  CONSENSUS
{
public:
  //member variables
  ros::Publisher twist_pub; // publisher to /turtle#/cmd_vel  
  
  static geometry_msgs::Twist cmd; // command velocity 
  
  float Kp_lin; // proportional gain on linear position error
  float Kp_ang; // proportional gain on angular position error
  
  float target_x ; // consensus' x value
  float target_y ; // consensus' y value 
  
  //member functions
  float getAngErr(double x , double y);
  float getLinErr(double x , double y);
  
  void move(double x , double y);  
}; 

					  // main
int main(int argc, char** argv)
{
  
 ros::init(argc,argv , "consensus" );
 ros::NodeHandle node("~"); 

 CONSENSUS C;
 C.n = node;
 
 if (node.getParam("robot_name" , C.robot_name) )
   ROS_INFO(" setting robot's name : %s", C.robot_name.c_str() ); 
 
 C.pos_sub = node.subscribe("/" + C.robot_name + "/pose" , 100 , C.position_callback);
  
 C.init();
 
 C.set_adj_matrix(); 
 
 C.setCommunications();
 
 C.int_step = 0.3;
 
 C.ConsensusAlgorithm();
 
 CONTROLLER MOVE;
 
 MOVE.cur_pos = C.cur_pos; 
 MOVE.twist_pub = node.advertise<geometry_msgs::Twist>("/"+C.robot_name + "/cmd_vel" , 10);
 MOVE.pos_sub = C.pos_sub; 
 
 MOVE.Kp_lin = 1.0;
 MOVE.Kp_ang= 6.0;
 
 MOVE.move(C.self_status.x , C.self_status.y); // move to final position given by consensus' state : the centroid
 
 
 return 0;
 
}

						// class ComunicationGraph
					  
// initialize CommunicationGraph's static member variables
std::string CommunicationGraph::robot_name;

// define CommunicationGraph's member functions

void CommunicationGraph::set_adj_matrix()
{
 for ( int raw=0; raw<3 ; ++raw)
 {
   for ( int column=0; column<3; ++column)
   {
      if (raw!=column) A[raw][column]=1;    // complete graph
      else A[raw][column]=0;
   }
 }
 
/* for ( int raw=0; raw<3 ; ++raw)   // uncomment to print adjacenxy matrix A
 {
   for ( int column=0; column<3; ++column)
   {
      std::cout<< A[raw][column] << " " ;
   }
   std::cout << "\n";
 }
*/ 
 
}

					      // class CONSENSUS

// initialize CONSENSUS' static member variables
turtlesim::Pose CONSENSUS::cur_pos;
std::array<ros_ex::Status_cons,3> CONSENSUS::robots_status;
int CONSENSUS::num_of_msgs;
std::array<bool, 3> CONSENSUS::states_received;
std::vector<bool> CONSENSUS::other_ready;
 


// define CONSENSUS' member functions
void CONSENSUS::position_callback(const turtlesim::Pose::ConstPtr& pos_msg)
{
  cur_pos = *pos_msg;
}
void CONSENSUS::init()
{
 ros::Rate init_rate = 100;
 self_status.robot_id = robot_name;
 cur_pos.x = -1.0;
 cur_pos.y = -1.0;
 self_status.step_number=0;
 self_status.next_step=false;
 self_status.consensus=false;
 ros::spinOnce();
 if (cur_pos.x<0.0 || cur_pos.y<0.0)
 {
  ROS_INFO("Waiting for turtlesim");
  while (ros::ok() && n.ok() && (cur_pos.x<0.0 || cur_pos.y<0.0 ) ) // to wait for turtlesim 
  {
    ros::spinOnce();
    init_rate.sleep();   
  }
 }
 
 ROS_INFO("Initial %s state is : x[0]=%f  y[0]=%f ", self_status.robot_id.c_str(), cur_pos.x , cur_pos.y) ;
 
  self_status.x=cur_pos.x; 
  self_status.y=cur_pos.y; 
  
}

int CONSENSUS::get_node_number(std::string name)
{
  if (name=="turtle1") return 1;
  if (name=="turtle2") return 2;
  if (name=="turtle3") return 3;
}

std::string CONSENSUS::get_node_name(int num)
{
  if (num==1) return "turtle1";
  if (num==2) return "turtle2";
  if (num==3) return "turtle3";
  
}

void CONSENSUS::statusCallback(const ros_ex::Status_cons::ConstPtr& msg)
{  
  int node_num=get_node_number(msg->robot_id);
  
  if ( ! (states_received.at(node_num-1)) )
  {
    //ROS_INFO(" %s received %s' status: x=%f  y=%f ", robot_name.c_str(), msg->robot_id.c_str(), msg->x, msg->y );
  
    ros_ex::Status_cons status_rec;
    status_rec.robot_id = msg->robot_id;
    status_rec.x=msg->x;
    status_rec.y=msg->y;
    status_rec.step_number=msg->step_number;
    status_rec.next_step=msg->next_step;
    status_rec.consensus=msg->consensus;
  
    robots_status.at(node_num-1)=status_rec;
  
    states_received.at(node_num-1) = true;
  }
  else
  {
    robots_status.at(node_num-1).step_number=msg->step_number;
    robots_status.at(node_num-1).next_step=msg->next_step; 
    robots_status.at(node_num-1).consensus=msg->consensus; 
  }
 
}

void CONSENSUS::setCommunications()
{
  int node_num = get_node_number(robot_name);
  for (int i=0 ; i<3 ; i++) 
  {
    if ( A[node_num - 1][i] == 1 ) // create a publisher and a subscriber for each robot communicating with
    {
      status_pub.push_back( n.advertise<ros_ex::Status_cons>( "/edge_" + robot_name + "_" + get_node_name(i+1) , 100) ); // queue size = 100
      //ROS_INFO(" creating publisher: /edge_%s_%s",  robot_name.c_str(), get_node_name(i+1).c_str() );
      status_sub.push_back( n.subscribe( "/edge_" + get_node_name(i+1) + "_" + robot_name , 100, statusCallback) ); // queue size = 100
      //ROS_INFO(" creating subscriber: /edge_%s_%s",   get_node_name(i+1).c_str() , robot_name.c_str() );
    }
  }
  
}

void CONSENSUS::publish_status(ros::Publisher pub)
{
  self_status.header.stamp = ros::Time::now();
  
  pub.publish(self_status);
}

void CONSENSUS::step()
{
  bool ready=false; // ready becomes true only when all communicating robots are ready to go to next step
  self_status.next_step=false; // next_step becomes true only when messages from all communicating robots have been received
  int counter_next_step; // counts robots that are ready to go to next step of the agorithm
  states_received = { {false,false,false} };
  passed_status=self_status; // x(k-1)
  
  ros::Rate communication_rate=10; // rate of processing data loop: 10Hz
  
  while(ros::ok() && n.ok() && !ready  )  // process data
  {
   
    for (int i=0; i<status_pub.size() ; ++i) // publish self_status to all communicating robots
    {
      publish_status(status_pub[i]);
    }
    
    ros::spinOnce();
    
    num_of_msgs = 0;
    
    for (int i=0; i<3; i++)
    {
     if (states_received.at(i) == true)  num_of_msgs++; 
    }
    
    //ROS_INFO(" %s has received %i states ", robot_name.c_str(), num_of_msgs );
    
    self_status.next_step = (num_of_msgs==status_sub.size()) ; // next_step becomes true only when messages from all communicating robots have been received
    
    counter_next_step=0;
    
    for (int i=0; i<3; i++)
    {
     if (robots_status.at(i).next_step == true)  counter_next_step++; 
    }
    ready= (counter_next_step==status_sub.size() ); // ready becomes true only when all communicating robots are ready to go to next step
    
    communication_rate.sleep();
  }
  /*
   for (int i=0; i<3; i++)
  {
   ROS_INFO(" %s : status[%i]-> x=%f y=%f ", C.robot_name.c_str(), i,  C.robots_status[i].x,  C.robots_status[i].y);
    
  } */
  
  // now that all data has been processed correctly, update self_status x(k+1)
  
  double x=self_status.x;
  double y=self_status.y;
  
  for (int i=0; i<3; i++)
  {
   if (robots_status[i].x==0.0 && robots_status[i].y==0.0) continue; // states of not communicating robots remains empty so they are not considered (in this case each robot doesn't communicate only with itself)
   else
   {
     x=x-int_step * (self_status.x - robots_status[i].x );  // 
     y=y-int_step * (self_status.y - robots_status[i].y );  // 
   }
  }
  
  self_status.x=x;  // x(k)
  self_status.y=y;

  self_status.step_number += 1;
  
  //ROS_INFO( "%s : x[%i]=%f y[%i]=%f ", robot_name.c_str(), self_status.x , self_status.y );
  double cons_res = 0.01;
  if ( std::abs(self_status.x-passed_status.x) < cons_res && std::abs(self_status.y-passed_status.y) < cons_res ) // when status becomes constant, consensus has been achieved
    self_status.consensus=true;
}

void CONSENSUS::ConsensusAlgorithm()
{
  iter=0;
  ros::Rate wait_rate = 10; // this loop rate is for the "wait" loop 
  int count = 0; // counts how many communicating robots have completed the current step of the algorithm
  bool wait = true; //false only when all communicating robots have completed current step, so when count is equal to number of communicating robots
  team_ready = false; // true only when all robots received consensus=true from respectives communicating robots and its self_status.consenus his also true
  int ready_counter = 0; // counts ho many communicating robots have achieved consensus(including itself)
  
  while (ros::ok() && n.ok() && !team_ready )
  {
      step(); // computes a step
      ++iter;
      
      count=0;
      wait=true;
      
      while (wait && ros::ok() && n.ok())  // "wait" loop
      {
	     ros::spinOnce();
	     for (int i=0; i<status_pub.size() ; ++i)  // publish self_status to all communicating robots
	     {
	       publish_status(status_pub[i]);
	     }
	
	     count=0;
	     ready_counter=0;
	     if (self_status.consensus) ready_counter++;
	     for (int i=0; i<3; i++)
	     {
	      if (robots_status.at(i).step_number==iter  ) ++count;
	       if (robots_status.at(i).consensus ) ++ready_counter;
	     }
	 
	     wait = ! (count==status_sub.size()) ;
	     team_ready = (ready_counter==status_sub.size()+1 );
	
	     wait_rate.sleep();
      }
 
      ROS_INFO( "%s : x[%i]=%f y[%i]=%f ", robot_name.c_str(), iter , self_status.x , iter, self_status.y );
     
       sleep(1.0); 
  } 
  
  ROS_INFO (" %s : consensus achieved! " , robot_name.c_str() );
}

					      // class CONTROLLER 

// initialize CONTROLLER's static member variables
geometry_msgs::Twist CONTROLLER::cmd;

// define CONTROLLER's member functions 
float CONTROLLER::getAngErr(double x , double y)
{
  float des_ang = std::atan2(y - cur_pos.y ,x - cur_pos.x); 
  float Eang= des_ang - cur_pos.theta;
  return Eang;
}

float CONTROLLER::getLinErr(double x , double y)
{
  float Elin = std::hypot(x - cur_pos.x , y - cur_pos.y) * std::cos(getAngErr(x,y));
  return Elin;
}

void CONTROLLER::move(double x , double y)
{
  ros::Rate controller_rate(10);
  double Elin =30.0 ;
  double Eang = 10.0 ;
  
  if (std::abs(getLinErr(x,y))<0.01)  
  {
    ROS_INFO(" %s : I'm already there!" , robot_name.c_str() );
  }
  else
  {
    /*if ( cur_pos.x-x > 0.0  &&  std::abs(cur_pos.y - y) < 1.0) 
    {
      ROS_WARN(" %s : Difficult path for me -> changing trajetory... ", robot_name.c_str() );
      float final_x=x;
      float final_y= y;
    
      float through_x;
      float through_y;
      through_x= ( x + cur_pos.x ) / 2.0;
      if (y<9.0)
	through_y= y + 2.0;
      else
	through_y= y - 2.0;
    
      //ROS_INFO("Intermediate point's coordinates : [%f]  y:[%f] " , through_x , through_y);
    
      x=through_x;
      y=through_y;  
      
      while( ros::ok() && (std::abs(Elin) >0.01 || std::abs(Eang)>0.0001) && n.ok()  ) 
      {
	ros::spinOnce();
	Elin=getLinErr(x,y);
	Eang=getAngErr(x,y);
	cmd.linear.x= Kp_lin *Elin;
	cmd.angular.z=Kp_ang * Eang;
	twist_pub.publish(cmd); 
	controller_rate.sleep();
      }
    
      x=final_x;
      y=final_y;
   } */
  
    Elin =30.0 ;
    Eang = 10.0 ;
    
    while( ros::ok() && (std::abs(Elin) >0.01 || std::abs(Eang)>0.0001)  && n.ok() )
    {
      ros::spinOnce();
      Elin=getLinErr(x,y);
      Eang=getAngErr(x,y);
      cmd.linear.x= Kp_lin *Elin;
      cmd.angular.z=Kp_ang * Eang;
      twist_pub.publish(cmd); 
      controller_rate.sleep();
    }
  }
  
  ROS_INFO( " %s in position " , robot_name.c_str() );
  
}

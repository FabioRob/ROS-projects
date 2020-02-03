// Esercizio 6(anno accademico 2017/2018) : Formation             Fabio D'Onofrio 556505

/*   Formation choosen in this example is set by formation graph (a possible desired configuration in parenthesis):
  
                        1
             (2,4)TWO---------ONE(3,4)
                  /         / | |
                 /         /  | |
        sqrt(2) /         /   | | 1
               / ________/   _| |
              / / sqrt(5)   |   FOUR(3,3)
             / /            |    /
      (1,3) THREE   sqrt(5) |   /
                \           |  /  sqrt(2)
         sqrt(2) \          | /
                  \_____    |/
                         FIVE (2,2)        
               
*/

#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include <array>
#include <vector>
#include <iostream>
#include <iomanip>
#include <cmath>

#include "ros_ex/Status_cons.h" // custom message for consensus algorithm for formation

/*
 methods(except for header) of message Status_cons

 std::string robot_id; // robot's name("turtle1" ... "turtle5");
 double x; // x position
 double y; // y position
 int step_number; // current step of the consensus' algorithm for formation
 bool next_step; // true when all communicating robots have completed current step
 bool consensus; // true if consensus has been reached(status becomes constant)
 bool move; // true when all robots are in positions and are ready to move in formation
*/

const double pi = 3.141592654;

struct Edge // not oriented edge in the formation graph
{
 int n1; // first vertex
 int n2; // second vertex
 double d;
};

struct Des_Pos_i // desired position of i-th robot in final desired configuration
{
 double des_x;
 double des_y; 
};

			       // class FORMATION
class FORMATION
{
public:
  static std::string robot_name; 
  
  std::array<Edge , 7 > Ef ;  // specifying 7 distances in the formation, the graph will be minimally rigid (because 2n-3=10-3=7)
  //std::array<double , 7> d; // distances that define the formation
  std::array<std::array<double,5>,5> A; // adjacency matrix
  
  std::array<Des_Pos_i, 5> des_conf  ; // a desired configuration that verifies the formation 
  
  std::vector<ros::Publisher> status_pub; // contain publishers to robots specified by the edges set Ef
  std::vector<ros::Subscriber> status_sub; // contain subscribers from robots specified by the edges set Ef
  
// member function
  
  bool verifyConfiguration(std::array<Des_Pos_i, 5> conf);
  void set_adj_matrix(std::array<Edge,7> E); // build adjacency matrix from Ef 
};

			      // class CONSENSUS

class CONSENSUS : public FORMATION
{
public:
// member variables  
  ros::NodeHandle n;
  
  static turtlesim::Pose cur_pos; // store current position of the robot
  
  ros::Subscriber pos_sub; // subscriber to /turtle#/pose
  
  ros_ex::Status_cons self_status; // status of the robot at time k (robot_id, x , y , step, next_step, consensus ,move)
  ros_ex::Status_cons passed_status; // status of the robot at time k-1
  static std::array<ros_ex::Status_cons,5> robots_status;  // contains status of all five robots (the ones that aren't communicating with this node's robot will be empty)
                                                           // in order from turtle1 to turtle5
  // following variables are used to synchronize the robots
  static int num_of_msgs; // it counts how many messages have been received on current step of consensus' algorithm for formation
  static std::array<bool, 5> states_received; // each element is set to true if the respective state has been received on current step
  static std::vector<bool> other_ready; // set to true only when other robots have completed current steè
  
  double int_step; // integration step of consensus' algorithm: it must be less than 1/dmax=1/4=0.25 (turtle1 communicates with all other 4 turtles so dmax=4)
  int iter; // iteration number of consensus' algorithm
  
  bool start_formation; // true if initial positions of the robots already realize the formation (only turtle1 can state this because it communicates with all other 4 turtles)
  static bool team_in_formation; // ture when formation has been reached
// member functions

  static void position_callback(const turtlesim::Pose::ConstPtr& pos_msg); // store position in variable cur_pose
  
  void init();
  
  static int get_node_number(std::string name); // "turtle1" corresponds to 1, "turtle2" corresponds to 2 and so on
  std::string get_node_name(int num); // 1 corresponds to "turtle1" ,  2 corresponds to "turtle2" and so on
  
  static void statusCallback(const ros_ex::Status_cons::ConstPtr& msg); // process states of the other robots
  
  void setCommunications(); // build publishers and subscribers from adjacency matrix A (create a publisher and a subscriber for each robot communicating with)
  
  void publish_status(ros::Publisher pub); // publish self_status to comunicating robots
  
  void step(); // step of the discrete consensus' algorithm for formation
  
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
  
  void waitForTeam();
}; 


						      // main
int main(int argc, char **argv)
{
 ros::init(argc,argv , "formation" );
 ros::NodeHandle node("~"); 

 CONTROLLER C;
 C.n = node;
 
 if (node.getParam("robot_name" , C.robot_name) )
   ROS_INFO(" setting robot's name : %s", C.robot_name.c_str() ); 
  
 // specify desired formation through formation's graph (set of edges Ef with relative distances d)
 C.Ef = {{ {1,2,1.0}, {1,3,std::sqrt(5.0) }, {1,4,1.0}, {1,5,std::sqrt(5.0) }, {2,3,std::sqrt(2.0) }, {3,5,std::sqrt(2.0) }, {4,5,std::sqrt(2.0) } }}; 
 // distances are  1.0 , std::sqrt(5.0) , 1.0 , std::sqrt(5.0), std::sqrt(2.0) ,  std::sqrt(2.0) , std::sqrt(2.0) 
 
 /*for (int i=0; i<C.Ef.size(); ++i)
   std::cout << "( " << C.Ef[i].n1 << " , " << C.Ef[i].n2 << " )\n"; */
  
 C.des_conf = {{ {3,4} , {2,4}, {1,3} , {3,3} , {2,2} }};  // desired configuration that realize the formation
 
 // check if desired configuration realizes the formation
 bool valid_configuration = C.verifyConfiguration(C.des_conf);
 
 if (!valid_configuration)
 {
   ROS_WARN( "Desired configuration doesn't realize the formation ");
   ROS_WARN("Change configuration or formation through variables C.des_conf and C.Ef ");
   return 1;
 }
 
 C.pos_sub = node.subscribe("/" + C.robot_name + "/pose" , 100 , C.position_callback);
  
 C.init();
 
 C.set_adj_matrix(C.Ef); 
 
 C.setCommunications();
 
 C.int_step = 0.2;
 
 C.ConsensusAlgorithm();
 
 C.twist_pub = node.advertise<geometry_msgs::Twist>("/" + C.robot_name + "/cmd_vel" , 100);
 
 C.Kp_lin = 1.0;
 C.Kp_ang= 6.0;
 
 C.move(C.self_status.x , C.self_status.y); // move to final position in the translational invariant formation
 
 sleep(2.0);
 
 C.waitForTeam();
 
 return 0;
}


						      // class FORMATION
// initialize FORMATION's static member variables
std::string FORMATION::robot_name;
					    
// define FORMATION's member functions

bool FORMATION::verifyConfiguration(std::array<Des_Pos_i, 5> conf)
{
   double err;
   for (int i=0; i<7 ; i++)
   {
     
     err = std::abs ( std::hypot( conf[ Ef[i].n1 -1 ].des_x -  conf[ Ef[i].n2 -1 ].des_x  , conf[ Ef[i].n1 -1 ].des_y -  conf[ Ef[i].n2 -1].des_y ) - Ef[i].d );
     
    if (err > 0.1 )     return false ;
    
   }
   
   return true;
} 

void FORMATION::set_adj_matrix(std::array<Edge,7> E)
{
 for ( int raw=0; raw<5 ; ++raw)
 {
   for ( int column=0; column<5; ++column)
   {
      for ( int ei=0; ei<E.size() ; ++ei)
      {											//NB not oriented graph
	     if ( (E[ei].n1 -1 ==raw && E[ei].n2 - 1==column) || (E[ei].n1 - 1==column && E[ei].n2 - 1 ==raw) )   // NB indexs of raws and columns are shifted by one in respect to node number
	     {												     
	       A[raw][column] = E[ei].d;
	        break;
	     }
	     else
	        A[raw][column] = 0;	
      }
   }
 }
 
 if(robot_name=="turtle1")      // print formation graph's adjacency matrix
 {
  ROS_INFO(" Formation graph's adjacency matrix is: ");
  std::setprecision(2);
  for ( int raw=0; raw<5 ; ++raw)
  {
    for ( int column=0; column<5; ++column)
    {
      std::cout<< A[raw][column] << " " ;
    }
    std::cout << "\n";
  }
 }
 
}

					      // class CONSENSUS
					      
// initialize CONSENSUS' static member variables
turtlesim::Pose CONSENSUS::cur_pos;
std::array<ros_ex::Status_cons,5> CONSENSUS::robots_status;
int CONSENSUS::num_of_msgs;
std::array<bool, 5> CONSENSUS::states_received;
std::vector<bool> CONSENSUS::other_ready;
bool CONSENSUS::team_in_formation;

// define CONSENSUS' member functions
void CONSENSUS::position_callback(const turtlesim::Pose::ConstPtr& pos_msg)
{
  cur_pos.x=pos_msg->x;
  cur_pos.y=pos_msg->y;
  cur_pos.theta=pos_msg->theta;
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
 ROS_INFO("Waiting for turtlesim");
 while (ros::ok() && n.ok() && (cur_pos.x<0.0 || cur_pos.y<0.0 ) ) // to wait for turtlesim 
 {
   ros::spinOnce();
   init_rate.sleep();   
 };
 
 ROS_INFO("Initial %s state is : x[0]=%f  y[0]=%f ", self_status.robot_id.c_str(), cur_pos.x , cur_pos.y) ;
 
  self_status.x=cur_pos.x; 
  self_status.y=cur_pos.y; 
  
}

int CONSENSUS::get_node_number(std::string name)
{
  if (name=="turtle1") return 1;
  if (name=="turtle2") return 2;
  if (name=="turtle3") return 3;
  if (name=="turtle4") return 4;
  if (name=="turtle5") return 5;
}

std::string CONSENSUS::get_node_name(int num)
{
  if (num==1) return "turtle1";
  if (num==2) return "turtle2";
  if (num==3) return "turtle3";
  if (num==4) return "turtle4";
  if (num==5) return "turtle5";
  
}

void CONSENSUS::statusCallback(const ros_ex::Status_cons::ConstPtr& msg)
{
  
  int node_num=get_node_number(msg->robot_id);
  
  if ( ! (states_received.at(node_num-1)) )
  {
    //ROS_INFO(" %s received %s' status: x=%f  y=%f ", robot_name.c_str(), msg->robot_id.c_str(), msg->x, msg->y );
  
    robots_status.at(node_num-1)=*msg;
  
    states_received.at(node_num-1) = true;
  }
  else
  {
    robots_status.at(node_num-1).step_number=msg->step_number;
    robots_status.at(node_num-1).next_step=msg->next_step; 
    robots_status.at(node_num-1).consensus=msg->consensus; 
    robots_status.at(node_num-1).move=msg->move; 
  }
 
}

void CONSENSUS::setCommunications()
{
  int node_num = get_node_number(robot_name);
  for (int i=0 ; i<5 ; i++) 
  {
    if ( A[node_num - 1][i] != 0 ) // create a publisher and a subscriber for each robot communicating with
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
  states_received = { {false,false,false,false,false} };
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
    
    for (int i=0; i<5; i++)
    {
     if (states_received.at(i) == true)  num_of_msgs++; 
    }
    
    //ROS_INFO(" %s has received %i states ", robot_name.c_str(), num_of_msgs );
    
    self_status.next_step = (num_of_msgs==status_sub.size()) ; // next_step becomes true only when messages from all communicating robots have been received
    
    counter_next_step=0;
    
    for (int i=0; i<5; i++)
    {
     if (robots_status.at(i).next_step == true)  counter_next_step++; 
    }
    ready= (counter_next_step==status_sub.size() ); // ready becomes true only when all communicating robots are ready to go to next step
    
    communication_rate.sleep();
  }
  /*
   for (int i=0; i<5; i++)
  {
   ROS_INFO(" %s : status[%i]-> x=%f y=%f ", C.robot_name.c_str(), i,  C.robots_status[i].x,  C.robots_status[i].y);
    
  } */

  if (robot_name=="turtle1" && iter==0 )  // robot turtle1 communicates with all other 4 robots, so at the first step(iter=0), robots_status contains initial positions of all robots
  {
     std::array<Des_Pos_i , 5> initial_configuration = { self_status.x,self_status.y }; 
     
     //ROS_INFO(" %s position : x=%f y=%f ", robot_name.c_str(), initial_configuration[0].des_x, initial_configuration[0].des_y );
     for (int i=1; i<5 ; i++)
     {
	     initial_configuration[i] = { robots_status[i].x , robots_status[i].y }; 
	     //ROS_INFO(" %s position : x=%f y=%f ", get_node_name(i+1).c_str(), initial_configuration[i].des_x, initial_configuration[i].des_y );
     }
      
     if ( verifyConfiguration(initial_configuration) )  // if initial positions of all robots already verify the formation, don't compute algorithm
     {
      ROS_INFO( " %s : all robots are already in formation " , robot_name.c_str() );
      start_formation=true;
      self_status.consensus = true;
     }    
  }
  
  // now that all data has been processed correctly, update self_status x(k+1)
  
  double x=self_status.x;
  double y=self_status.y;
  
  for (int i=0; i<5; i++)
  {
   if (robots_status[i].x==0.0 && robots_status[i].y==0.0) continue; // states of not communicating robots remains empty so they are not considered
   else
   {
     x=x-int_step * (self_status.x - robots_status[i].x - ( des_conf[get_node_number(robot_name)-1].des_x -  des_conf[i].des_x )  );  // 
     y=y-int_step * (self_status.y - robots_status[i].y - ( des_conf[get_node_number(robot_name)-1].des_y -  des_conf[i].des_y )  );  // 
   }
  }
  
  self_status.x=x;  // x(k)
  self_status.y=y;

  self_status.step_number += 1;
  
  //ROS_INFO( "%s : x[%i]=%f y[%i]=%f ", robot_name.c_str(), self_status.x , self_status.y );
  double cons_res = 0.15;
  if ( std::abs(self_status.x-passed_status.x) < cons_res && std::abs(self_status.y-passed_status.y) < cons_res ) // when status becomes constant, consensus has been achieved
    self_status.consensus=true;
}

void CONSENSUS::ConsensusAlgorithm()
{
  iter=0;
  ros::Rate wait_rate = 10; // this loop rate is for the "wait" loop 
  start_formation=false; // true when robots' initial positions already verify formation
  int count = 0; // counts how many communicating robots have completed the current step of the algorithm
  bool wait = true; //false only when all communicating robots have completed current step, so when count is equal to number of communicating robots
  team_in_formation = false; // true only when all robots received consensus=true from respectives communicating robots and its self_status.consensus his also true
  int ready_counter = 0; // counts how many communicating robots have achieved consensus(including itself)
  
  while (ros::ok() && n.ok() && !team_in_formation )
  {
      step(); // computes a step
      
      if (start_formation) ROS_INFO(" robot are already in formation! " );
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
	     for (int i=0; i<5; i++)
	     {
	       if (robots_status.at(i).step_number==iter  ) ++count;
	       if (robots_status.at(i).consensus ) ++ready_counter;
	     }
	     wait = ! (count==status_sub.size()) ;
	     team_in_formation = (ready_counter==status_sub.size()+1 );
	     wait_rate.sleep();
      }
 
      ROS_INFO( "%s : x[%i]=%f y[%i]=%f ", robot_name.c_str(), iter , self_status.x , iter, self_status.y );
     
      sleep(1.0); 
  } 
   
  
  ROS_INFO (" %s : consensus achieved! " , robot_name.c_str() );
}	

							  // class CONTROLLER
// initilalize CONTROLLER's static member variables 
geometry_msgs::Twist CONTROLLER::cmd;

// defineCONTROLLER's member functions
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
  self_status.move=false; // used to wait for other robots to be in position
  if (std::abs(getLinErr(x,y))<0.01)  
  {
    ROS_INFO(" %s : I'm already in desired configuration!" , robot_name.c_str() );
  }
  else
  {
  /*  if ( cur_pos.x-x > 0.0  &&  std::abs(cur_pos.y - y) < 1.0) 
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
   } 
  
    Elin =30.0 ;
    Eang = 10.0 ;
   */
  
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
  
  double des_orientation = pi/4;
  //rotate robot till its orientation is equal to des_orientation
  ROS_INFO(" rotation... ");
  while (ros::ok() && std::abs(cur_pos.theta-des_orientation)>0.5  && n.ok() )
  {
   ros::spinOnce();
   if (cur_pos.theta<0.0) cur_pos.theta += 2*pi;
   cmd.linear.x=0.0;
   cmd.angular.z= 0.5;
   //cmd.angular.z= 0.2 * (cur_pos.theta-des_orientation) ;
   twist_pub.publish(cmd); 
   controller_rate.sleep();   
  } 
  
  cmd.linear.x=0.0;
  cmd.linear.z=0.0;
  twist_pub.publish(cmd); 
  self_status.move=true;
  ROS_INFO( " %s in position " , robot_name.c_str() );
  
}


void CONTROLLER::waitForTeam()
{
  ros::Rate rate = 10;
  bool team_in_position=false;
  int count=0;
  
  if (robot_name=="turtle1")
  {
    self_status.move = false; // leader turtle1 wait that all other robots are in position
    while (ros::ok() && n.ok() && !team_in_position)
    {
      ros::spinOnce();
    
      
      for (int i=0; i<status_pub.size() ; ++i) // publish self_status to all communicating robots
      {
        publish_status(status_pub[i]);
      }
      
      count=0;
      for (auto rob : robots_status )
      {
	     if (rob.move ) count++;
      }
      
      team_in_position= (count == status_sub.size() ) ;
      rate.sleep();
    
    }
    self_status.move =true; // leader now knows that all robots are in positions and can tell them to move
    ros::Time start =ros::Time::now();
    while(ros::ok() && n.ok() && (ros::Time::now()-start).toSec()<1.0 )  // if leader arrives last
    {
      for (int i=0; i<status_pub.size() ; ++i) // publish self_status to all communicating robots
      {
        publish_status(status_pub[i]);
      }
    }
    cmd.linear.x=0.0;
    cmd.angular.z=0.0;
    ROS_INFO(" %s : follow me now", robot_name.c_str() );   
    
  }
  else
  {
    robots_status[0].move =false;
    while (ros::ok() && n.ok() && !robots_status[0].move  )
    {
      ros::spinOnce();   
      
      for (int i=0; i<status_pub.size() ; ++i) // publish self_status to all communicating robots
      {
       publish_status(status_pub[i]);
      }
    
      rate.sleep();
    }
    cmd.linear.x=0.0;
    cmd.angular.z=0.0;
    ROS_INFO( " %s ready to follow the leader ", robot_name.c_str() );
  }
   
  
}

/*

								  // class MOVE_FORMATION
class MOVE_FORMATION : public CONTROLLER
{
public:

  ros::Subscriber twist_sub;
  std::vector<ros::Publisher> leader_twist_pubs;
  
  static void twistCallback(const geometry_msgs::Twist::ConstPtr& leader_cmd); 
  void follow();
};

void MOVE_FORMATION::twistCallback(const geometry_msgs::Twist::ConstPtr& leader_cmd)
{
  
    cmd.linear.x=leader_cmd->linear.x;
    cmd.angular.z=leader_cmd->angular.z; 
}

void MOVE_FORMATION::follow()
{ 
  leader_twist_pubs.resize(status_sub.size() );
  for ( unsigned int i=0 ; i<leader_twist_pubs.size() ; i++)
  {
    leader_twist_pubs.at(i) = n.advertise<geometry_msgs::Twist>("/" + robots_status[i+1].robot_id + "/cmd_vel" , 10 ); // latched connection
  }
  sleep(1.0);
  twist_sub=n.subscribe("/turtle1/cmd_vel", 100, twistCallback);
  ros::Rate rate = 10;
  while( ros::ok() && n.ok() ) 
  {
     ros::spinOnce();
     if (twist_sub.getNumPublishers() == 2)
     {
      for (auto p : leader_twist_pubs )
	p.publish(cmd); 
     }
     else
     {
       cmd.linear.x=0.0;
       cmd.angular.z=0.0;
       for (auto p : leader_twist_pubs )
	p.publish(cmd); 
     }
     rate.sleep();
  } 
  
}
*/




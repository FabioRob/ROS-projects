// Esercizio 4(anno accademico 2017/2018) : Exploration                Fabio D'Onofrio 556505

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <cmath>
#include <array>

#include "ros_ex/robot_status.h" // custom message for status
/*
  methods(except for header) of message robot_status

  std::string robot_id; // robot's name
  bool ready; // true when all other robots' positions has been received
  turtlesim::Pose position; // self position

*/
					// class Exploration
class Exploration
{
public:
  ros::NodeHandle n;
  
  static std::string robot_name;
  
  static bool team_ready; // true only when all robots have received other robots' positions
  
  static int robot_number; // number of robots: 3
  static int pos_received; // number of position received
  static int counter; // number of robots that have received all other robots' positions
  
  static std::vector<ros_ex::robot_status> robots_state; // contain robots' states, from turtle1 to turtle3
  static std::array<bool,3> states_received; // each component is true only if respective robot's position has been received
  
  static turtlesim::Pose cur_pos;
  
  ros::Subscriber pos_sub;
  ros::Subscriber status_sub;
  ros::Publisher status_pub;
  
  static void positionCallback (const turtlesim::Pose::ConstPtr& pos_msg);
  static void statusCallback (const ros_ex::robot_status::ConstPtr& status_msg);
  
  void init();
  
  void publishStatus();
  static int getRobotNumber(std::string name);
  
  void waitForTeam();
  
};

					      // class Controller

class Controller : public Exploration
{
public:
  turtlesim::Pose start_pos;
  geometry_msgs::Twist cmd;
  float Kp_lin;
  float Kp_ang;
  
  ros::Publisher twist_pub;
  
  float target_x ;
  float target_y ;
  
  float getAngErr(float x , float y);
  float getLinErr(float x , float y);
  
  void move(float x , float y);
  
  void explore();
  
};


int main(int argc, char **argv)
{
 
  ros::init(argc,argv,"expl");
  
  ros::NodeHandle node("~");
  
  ros::Rate loop_rate(10);
   
  Exploration E;
  E.n=node;
  
  if (node.getParam("robot_name", E.robot_name ) )
  {
    ROS_INFO("Setting robot name : %s", E.robot_name.c_str() );
  }
  
  E.pos_sub = node.subscribe("/" + E.robot_name + "/pose" , 100 , E.positionCallback);
  
  E.status_sub = node.subscribe("/status_topic" , 10 , E.statusCallback);
  
  E.status_pub = node.advertise<ros_ex::robot_status>("/status_topic" , 10);
  
  E.init();
  
  E.publishStatus();
  
  E.waitForTeam();
  
  sleep(1.0);  
  
  Controller M;
  M.robot_name=E.robot_name;
  M.cur_pos=E.cur_pos;
  M.robots_state=E.robots_state;
  
  M.Kp_lin=1.0;
  M.Kp_ang=6.0;
  
  M.twist_pub = node.advertise<geometry_msgs::Twist>("/" + M.robot_name + "/cmd_vel" , 10);
  
  M.explore();

  return 0;
}

					  // class Exploration
// initialize Exploration's static member variables
std::string Exploration::robot_name ;
bool Exploration:: team_ready ;
int Exploration::robot_number;
int Exploration::pos_received;
int Exploration::counter;
turtlesim::Pose Exploration::cur_pos;  
std::vector<ros_ex::robot_status> Exploration::robots_state;
std::array<bool,3> Exploration::states_received; 

// define Exploration's member functions
void Exploration::publishStatus()
{
  ros_ex::robot_status status_msg;
  status_msg.header.stamp = ros::Time::now();
  status_msg.robot_id = robot_name;
  status_msg.position.x=cur_pos.x;
  status_msg.position.y=cur_pos.y;

  //ROS_INFO(" %s received position of  %i  robots", robot_name.c_str(), pos_received);
  
  if (pos_received == robot_number) // this robot received positions of other two robots
  {
     status_msg.ready=true;   
  }
  else status_msg.ready=false;
  
  //sleep(1.0);
    
  status_pub.publish(status_msg);
}

int Exploration::getRobotNumber(std::string name)
{
  if (name=="turtle1") return 0;
  if (name=="turtle2") return 1;
  if (name=="turtle3") return 2;   
}

void Exploration::positionCallback (const turtlesim::Pose::ConstPtr& pos_msg)
{
  cur_pos = *pos_msg;
}

void Exploration::init()
{
  ros::Rate init_rate = 100;
  robot_number = 3;
  pos_received=0;
  counter=0;
  robots_state.resize(robot_number);
  int rob_numb = getRobotNumber(robot_name); // turtle1->0 ; turtle2->1 ; turtle3->2
  states_received[rob_numb] = true; // a robot knows itself position 
   
  cur_pos.x = -1.0;
  cur_pos.y = -1.0;
  
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
 
 robots_state[rob_numb].robot_id=robot_name;
 robots_state[rob_numb].ready=false;
 robots_state[rob_numb].position=cur_pos;
}

void Exploration::statusCallback (const ros_ex::robot_status::ConstPtr& status_msg)
{
  if (team_ready) return; 
  
  std::string name = status_msg->robot_id;
  int rob_numb = getRobotNumber(name);
  
  if ( !states_received[rob_numb] )
  {
    ROS_INFO(" %s received position of %s",  robot_name.c_str(), status_msg->robot_id.c_str() );
   
    robots_state[rob_numb] = *status_msg;
    
    states_received[rob_numb]=true;
  }
  
  robots_state[rob_numb].ready = status_msg->ready;
  
  pos_received=0;
  for (auto state : states_received)
  {
   if (state) pos_received++; 
  }
  
  counter=0;
  for (auto state : robots_state)
  {
   if (state.ready) counter++; 
  }
  
  if(counter == robot_number)
  {   
    ROS_INFO(" %s: I'm ready, move now " , robot_name.c_str() );
    team_ready=true;    
  }    
  
}


void Exploration::waitForTeam()
{
  ros::Rate rate(1.0);
  
  while(ros::ok() && n.ok() && !team_ready )
  {
    publishStatus();
    ros::spinOnce();
    rate.sleep();
  }
  
}

						// class Controller
// define Controller's member functions
float Controller::getAngErr(float x , float y)
{
  float des_ang = std::atan2(y - cur_pos.y , x - cur_pos.x); 
  float Eang= des_ang - cur_pos.theta;
  return Eang;
}

float Controller::getLinErr(float x , float y)
{
  float Elin = std::hypot(x - cur_pos.x , y - cur_pos.y) * std::cos(getAngErr(x,y));
  return Elin;
}

void Controller::move(float x , float y)
{
  ros::Rate rate(10);
  double Elin =30.0 ;
  double Eang = 10.0 ;
  /*
  if ( cur_pos.x-x > 0.0  &&  std::abs(cur_pos.y - y) < 1.0)
  {
    ROS_INFO(" %s : Difficult path-> changing trajetory" , robot_name.c_str() );
    
    float through_x;
    float through_y;
    through_x= (x + cur_pos.x ) / 2.0;
    if (y<9.0)
      through_y= y + 2.0;
    else
      through_y= y - 2.0;
    
    //ROS_INFO("Intermediate point's coordinates : [%f]  y:[%f] " , through_x , through_y);
    
    while( ros::ok && (std::abs(Elin) >0.01 || std::abs(Eang)>0.0001) )
    {
      ros::spinOnce();
      Elin=getLinErr(through_x,through_y);
      Eang=getAngErr(through_x,through_y);
      cmd.linear.x= Kp_lin * Elin;
      cmd.angular.z=Kp_ang * Eang;
      twist_pub.publish(cmd); 
      rate.sleep();
    }

  }
  */
  Elin =30.0 ;
  Eang = 10.0 ;
    
  while( ros::ok && (std::abs(Elin) >0.01 || std::abs(Eang)>0.0001) )
  {
    ros::spinOnce();
    Elin=getLinErr(x,y);
    Eang=getAngErr(x,y);
    cmd.linear.x= Kp_lin * Elin;
    cmd.angular.z=Kp_ang * Eang;
    twist_pub.publish(cmd); 
    rate.sleep();
  }
  
}

void Controller::explore()  
{
  // following code is for make robots visit each other positions circularly(in clockwise order)
  int rob_ind = getRobotNumber(robot_name); // initial position in robots_state vector(0 for turtle1, 1 for turtle2, 2 for turtle3)
  int new_ind=-1; // will be final position in sorted robots_state vector
  
  for (int i=0; i<3; i++)
  {
   for(int cur_i=i+1; cur_i<3 ; cur_i++)
   {
    if (robots_state.at(i).position.x > robots_state.at(cur_i).position.x)
    {
      std::swap(robots_state.at(i),robots_state.at(cur_i) );
      if (i==rob_ind)
      {
	     new_ind=cur_i;
	     rob_ind=cur_i;
      }
      else if (cur_i==rob_ind)
      {
	     new_ind=i;
	     rob_ind=i;
      }  
    }    
   }
  }
  
  if (new_ind==-1) new_ind=rob_ind; // position in the array didn't change in the sorting
  
  //ROS_INFO(" robots_state[0].x=%f robots_state[1].x=%f robots_state[2].x=%f ", robots_state[0].position.x , robots_state[1].position.x, robots_state[2].position.x);
  
  if( std::abs(getLinErr(robots_state[(new_ind+1) % 3].position.x,robots_state[(new_ind+1) % 3].position.y))<0.1 &&
    std::abs(getLinErr(robots_state[(new_ind+2) % 3].position.x,robots_state[(new_ind+2) % 3].position.y))<0.1 ) // same initial positions
  {
    ROS_WARN(" No positions to explore " );
    return ;
  }
  
  int i=0;
  
  while (i<3)
  {
   ++i;
   if ( std::abs(getLinErr(robots_state.at( (new_ind+i) % 3 ).position.x, robots_state.at( (new_ind+i) % 3 ).position.y) ) < 0.1 )
      ROS_INFO(" %s : I'm already in %s's initial position!" , robot_name.c_str() , robots_state[(new_ind+i) % 3].robot_id.c_str() );
   else
    move( robots_state.at( (new_ind+i) % 3 ).position.x, robots_state.at( (new_ind+i) % 3 ).position.y );  
   
  }
  
}
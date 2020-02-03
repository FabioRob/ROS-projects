// Ex. 2 : Point to point control      

#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include <cmath>

#include "ros_ex/position_error.h"   // custom message with position error in respect to final target

/*
  methods of position_error(except for header):
  double Elin; // linear position error
  double Eang; // angular position error
*/

const float pi = 3.141592654;
				    // class P2P
class P2P
{
public:
  ros::Subscriber m_pose_sub; // subscriber to /turtle1/pose topic
  ros::Publisher  m_twist_pub; // publisher to /turtle1/cmd_vel topic
  ros::Publisher m_error_pub; // publisher to pos_err topic
  
  static turtlesim::Pose m_current_pose;  
  static bool m_turtle_not_spawned; // indicates if turtlesim_node has not been launched yet(true) or not(false)
  geometry_msgs::Twist m_cmd_vel;
  ros_ex::position_error m_err; // position error(linear and angular) published on topic pos_err
  
  float m_x; // desired x position : default=8.0
  float m_y; // desired y position : defauòt=8.0
  
  float m_Elin; // position error on the turtle's local x axis
  float m_Eang; // angular error: angle between local x axis and segment from turtle to final target
  
  float m_kp_v; // proportional gain for linear velocity
  
  float m_kp_th ; // proportional gain for angular velocity
  
  static void poseCallback ( const turtlesim::Pose::ConstPtr& pose_msg);  
  float getAngularError();
  float getLinearError();
  void move();
  
};

							// main
int main(int argc, char **argv)
{
  ros::init(argc,argv,"PointToPoint");
  
  ros::NodeHandle node;
  
  P2P controller;
  
  controller.m_x=8.0; // default x target position
  controller.m_y=8.0; // default y target position
  
  controller.m_kp_v = 1.0; // proportional gain for linear velocity
  controller.m_kp_th = 6.0; // proportional gain for angular velocity
  
  ros::Rate loop_rate = 10;
  
  bool launched = false; // indicates if this node is being launched via launch file P2P.launch
  
  // to pass final target position through launch file P2P.launch (default is x=8.0 y=8.0)
  if (node.getParam("x" , controller.m_x)) launched=true;
  if (node.getParam("y" , controller.m_y)) launched=true;
  if (launched && controller.m_x==8.0 && controller.m_y==8.0) ROS_INFO("Default target position");
  
  // to pass final target position through command line arguments running this single node (default is x=8.0 y=8.0)
  if (argc==2)
  {
    controller.m_x = atof(argv[1]);
  }
  if (argc==3)
  {
    controller.m_x = atof(argv[1]); 
    controller.m_y = atof(argv[2]); 
  }
  else if (!launched && controller.m_x==8.0 && controller.m_y==8.0) ROS_INFO("Default target position");
  
  ROS_INFO("Desired x position is: %f", controller.m_x);
  ROS_INFO("Desired y position is: %f", controller.m_y);
  
  controller.m_pose_sub = node.subscribe("/turtle1/pose", 100, controller.poseCallback);
  controller.m_twist_pub = node.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",10);
  controller.m_error_pub = node.advertise<ros_ex::position_error>("/pos_err" , 10);
  
  sleep(1.0); // to wait for turtlesim_node when launched with launch file P2P.launch
  
  ros::spinOnce();
  if (controller.m_turtle_not_spawned) ROS_INFO("Waiting for turtlesim...");
  
  while (ros::ok() && node.ok() && controller.m_turtle_not_spawned) // to wait for turtlesim_node when this node is launched alone
  {
     ros::spinOnce();

     loop_rate.sleep();
  }
  
  if (std::abs(controller.getLinearError())<=0.1)  
  {
    ROS_INFO("I'm already there!");
    return 0;
  }
  else ROS_INFO("Let's move");
 
  ROS_INFO("deltaX is [%f] ", controller.m_current_pose.x-controller.m_x);
  ROS_INFO("deltaY is [%f] ", controller.m_current_pose.y-controller.m_y);

  /*if (controller.m_current_pose.x-controller.m_x > 0.0 && std::abs(controller.m_current_pose.y-controller.m_y )<1.0 ) // difficult path, so add an intermediate point to get there
  {
    ROS_INFO("Difficult path-->changing trajectory");
    
    float final_x=controller.m_x;
    float final_y=controller.m_y;
    
    float through_x;
    float through_y;
    through_x= ( controller.m_x + controller.m_current_pose.x ) / 2.0;
    if (controller.m_y<9.0)
      through_y= controller.m_y + 2.0;
    else
      through_y= controller.m_y - 2.0;
    
    ROS_INFO("Intermediate point's coordinates : [%f]  y:[%f] " , through_x , through_y);
    
    controller.m_x=through_x;
    controller.m_y=through_y;
    
    while(ros::ok() && node.ok() && (std::abs(controller.m_Elin)>0.1 || std::abs(controller.m_Eang)>0.01) )  
    {                                                                                                       
      ros::spinOnce();
    
      controller.move();
     
      loop_rate.sleep();
    } 
    
    controller.m_x=final_x;
    controller.m_y=final_y;
  } 
  */
  
  while(ros::ok && node.ok() && (std::abs(controller.m_Elin)>0.01 || std::abs(controller.m_Eang)>0.01) )  
  {                                                                                                       
    ros::spinOnce();
    
    controller.move();
     
    loop_rate.sleep();
  }
  
  if (launched) // if this node has been launched through launch file P2P.launch , delete params "x" and "y"
  {
    ros::param::del("x");
    ros::param::del("y");  
  }
  
  ROS_INFO("Final target has been achieved");
  
  return 0;
}


						  // class P2P
// initialize P2P's static member variables
turtlesim::Pose P2P::m_current_pose ;
bool P2P::m_turtle_not_spawned = true;

//define P2P's member functions
float P2P::getAngularError()
{
  
  float des_ang = std::atan2( m_y-m_current_pose.y , m_x-m_current_pose.x);
  
  m_Eang = des_ang - m_current_pose.theta;
  
  return m_Eang;
  
}

float P2P::getLinearError()
{
 float Ex= m_x-m_current_pose.x;
 float Ey= m_y-m_current_pose.y;
  
 float distance = std::sqrt ( std::pow(Ex,2) + std::pow(Ey,2) );
 
 m_Elin = distance * std::cos(getAngularError());
 
 return m_Elin;
 
}

void P2P::move ()
{ 
    m_Elin=getLinearError();
    m_Eang=getAngularError();
    
    m_err.Elin = m_Elin;
    m_err.Eang = m_Eang;
    
    //ROS_WARN("Linear error: [%f] , Angular error: [%f]", m_Elin, m_Eang);
    
    // Proportional controller 
    float linear_vel = m_kp_v * m_Elin;
    float angular_vel = m_kp_th * m_Eang;
 
    m_cmd_vel.linear.x=linear_vel;
    if (std::abs(m_Eang)>0.01 || std::abs(m_Eang-2*pi)>0.01 )
    m_cmd_vel.angular.z=angular_vel;
    else
     m_cmd_vel.angular.z=0.0; 
    
    m_twist_pub.publish(m_cmd_vel);
    m_err.header.stamp=ros::Time::now();
    m_error_pub.publish(m_err);
  
}

void P2P::poseCallback (const turtlesim::Pose::ConstPtr& pose_msg)
{
    m_current_pose = *pose_msg;
    m_turtle_not_spawned = false;
    //ROS_INFO("Current position is x: [%f], y:[%f]", m_current_pose.x, m_current_pose.y);
}

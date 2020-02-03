// Ex. 1 :  Workspace limits avoidance

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <cmath>

const float pi = 3.141592654;
				  // class AvoidLimits
class AvoidLimits
{
public:
  ros::NodeHandle n;
  
  static turtlesim::Pose m_current_position; // turtle's current position obtained through pose_sub's callback
  static geometry_msgs::Twist m_command_velocity; // velocity command subscribed from /cmd_vel topic
  static bool m_turtle_not_spawned; // indicates if turtlesim_node has not been launched yet(true) or not(false)
  static bool m_no_command_vel; // indicates if velocity commands are provided or not
  
  ros::Subscriber m_pose_sub; // subscriber to /pose topic
  ros::Publisher m_twist_pub; // publisher to /cmd_vel topic
  geometry_msgs::Twist m_avoid_command; // velocity command to avoid border
  ros::Subscriber m_twist_sub; // subscriber to /cmd_vel topic
  bool m_forward; // indicates if velocity input in robot's x direction is positive(true) or not(false)
  static void poseCallback (const turtlesim::Pose::ConstPtr& pose_msg);
  static void twistCallback (const geometry_msgs::Twist::ConstPtr& twist_msg);
  void avoidBorder(); 
};

				// main

int main(int argc, char** argv)
{
  ros::init(argc,argv,"limit_avoidance");
  ros::NodeHandle node;
  
  double freq = 200; //Hz
  
  ros::Rate loop_rate(freq);
  
  float linear = 0.0; // default linear velocity command
  float angular = 0.0; // default angular velocity command
  bool launched=false; // indicates if this node is being launched via launch file avoidance.launch 
  bool extern_input=false; // no external input by default
  geometry_msgs::Twist ext_cmd;
  
  AvoidLimits avoidance;
  avoidance.n=node;
  // to pass a linear x velocity and angular z velocity commands through launch avoidance.launch
  if (node.getParam("linear",linear) && linear!=0.0 ) // to provide a linear velocity command
  {
    extern_input = true;
    launched=true;
  }
  if (node.getParam("angular",angular) && angular!=0.0 ) // to provide an angular velocity command
  {
    extern_input = true;
    launched=true;
  }
  
  // to pass a linear x veocity and angular z velocity commands from command line when this node is launched alone 
  if (argc==2)
  {
    linear=atof(argv[1]);
    extern_input = true;
  }
  else if(argc==3)
  {
    linear=atof(argv[1]);
    angular=atof(argv[2]);
    extern_input = true;
  }
  
  ROS_INFO("Linear velocity command: [%f]", linear);
  ROS_INFO("Angular velocity command: [%f]", angular);
  ext_cmd.linear.x=linear;
  ext_cmd.angular.z=angular;
  
  avoidance.m_pose_sub = node.subscribe("/turtle1/pose", 100, avoidance.poseCallback);
  avoidance.m_twist_pub = node.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 100);
  avoidance.m_twist_sub = node.subscribe("/turtle1/cmd_vel", 10, avoidance.twistCallback);
 
  sleep(1.0); // to wait for turtlesim_node when launched with launch file avoidance.launch
  ros::spinOnce();
  while (ros::ok() && node.ok() && avoidance.m_turtle_not_spawned) // to wait for turtlesim_node when this node is launched alone
  {
     ros::spinOnce();
     ROS_INFO("Waiting for turtlesim...");
     loop_rate.sleep();
  }
 
  while (ros::ok() && node.ok() && !extern_input && avoidance.m_no_command_vel) // to wait for velocity commands when this node is launched alone 
  {
    ros::spinOnce();
    ROS_INFO("Waiting for velocity commands...");
    loop_rate.sleep();
  }
  
  float x_margin = 1.0; // when robot is at this distance from border, it starts the avoidance maneuver
  float y_margin = x_margin;
  float border_position = 11.0;
 
  while (ros::ok() && node.ok())
  {
    ros::spinOnce();
    
    if (extern_input && (linear!=0 || angular!=0 ) ) avoidance.m_twist_pub.publish(ext_cmd);
    
    // conditions are, in order: left border, right border , low border and  high border   
    if ( avoidance.m_current_position.x<x_margin || avoidance.m_current_position.x > border_position-x_margin ||
      avoidance.m_current_position.y<y_margin || avoidance.m_current_position.y > border_position-y_margin )
    {
      
      avoidance.avoidBorder();    
    }
    
    loop_rate.sleep();
  }
  
  if (launched) // if this node has been launched through launch file avoidance.launch , delete params "linear" and "angular"
  {
    ros::param::del("linear");
    ros::param::del("angular");  
  }
  
  return 0; 
}

						       // class AvoidLimits
						       
// initialize AvoidLimits' static member variables
turtlesim::Pose AvoidLimits::m_current_position; 
geometry_msgs::Twist AvoidLimits::m_command_velocity;
bool AvoidLimits::m_turtle_not_spawned = true;
bool AvoidLimits::m_no_command_vel = true;

// define AvoidLimits' member functions
void AvoidLimits::poseCallback (const turtlesim::Pose::ConstPtr& pose_msg)
{
  m_turtle_not_spawned = false;
  m_current_position = *pose_msg;
  //ROS_INFO("Current position is x:[%f] y:[%f]  theta:[%f]", m_current_position.x, m_current_position.y, m_current_position.theta);
}

void AvoidLimits::twistCallback(const geometry_msgs::Twist::ConstPtr& twist_msg)
{
 m_no_command_vel=false;
 m_command_velocity.linear.x = twist_msg->linear.x;
 m_command_velocity.linear.y = twist_msg->linear.y;
 m_command_velocity.angular.z = twist_msg->angular.z;
 //ROS_INFO("Command velocity received is: linear_x:[%f] , linear_y:[%f] , angular_z:[%f]", m_command_velocity.linear.x , m_command_velocity.linear.y , m_command_velocity.angular.z);
}

void AvoidLimits::avoidBorder()
{
  ROS_WARN("Border avoidance: CHANGING DIRECTION");
  // rotate of about 115 degrees
  float rot_ang = 2.0;
  float start_angle = m_current_position.theta;
  float start_x= m_current_position.x;
  float start_y = m_current_position.y;
  bool rescale_angle=false;
  float start_angle_r = start_angle;
  
  if (start_angle >= 2*pi - rot_ang) 
  {
    start_angle_r -= 2*pi;
    rescale_angle=true;
  }
  
  float start_forward_velocity = m_command_velocity.linear.x;
  m_forward = start_forward_velocity > 0.0;
  //ROS_INFO("Forward velocity input is : [%f]", start_forward_velocity);
  
  ros::Rate rate(100);
  
  m_current_position.theta= start_angle_r;
  while (ros::ok() && n.ok() && std::abs(m_current_position.theta-start_angle_r) < rot_ang )
  {
    ros::spinOnce(); 
    if (rescale_angle && m_current_position.theta >= 2*pi - rot_ang) 
      m_current_position.theta -= 2.0*pi;
     //ROS_INFO("Current theta is: [%f]", m_current_position.theta); 
    m_avoid_command.linear.x= 0.2* start_forward_velocity;
    m_avoid_command.linear.y=0.0; 
  
    if (m_forward)
    {
      if ( (start_x<1.0 && start_angle<pi) || (start_x>10.0 && start_angle>1.5*pi) || (start_y>10.0 && start_angle<0.5*pi) || (start_y<1.0 && start_angle<1.5*pi) )
	// clockwise rotation (negative)
	m_avoid_command.angular.z= -2.0 * start_forward_velocity ;
      else 
	m_avoid_command.angular.z= + 2.0 * start_forward_velocity; // counter-clockwise rotation(positive)
    }
    else // m_avoid_command.angular.z= -2.0 * start_forward_velocity ;
    {
      if ( (start_x<1.0 && start_angle<pi) || (start_x>10.0 && start_angle>1.5*pi) || (start_y>10.0 && start_angle<0.5*pi) || (start_y<1.0 && start_angle<1.5*pi) )
	// counter-clockwise rotation(positive) 
	m_avoid_command.angular.z= -2.0 * start_forward_velocity ;
      else 
	m_avoid_command.angular.z= + 2.0 * start_forward_velocity; // clockwise rotation(negative)
    }
    //ROS_WARN("Command velocity to avoid border is: linear_x:[%f] , angular_z:[%f]", m_avoid_command.linear.x , m_avoid_command.angular.z);
    m_twist_pub.publish(m_avoid_command);
    //ROS_INFO("Current position is x:[%f] y:[%f]  theta:[%f]", m_current_position.x, m_current_position.y, m_current_position.theta);
  
    if (m_current_position.x >= 11.0 || m_current_position.y >= 11.0 ) 
      ROS_WARN("Collision");
    rate.sleep();
  }
  
  // move forward with velocity input given before the rotation, for about 0.2 second
 ros::Time start = ros::Time::now();
 double time = 0.2;
 while(ros::ok() && n.ok() && (ros::Time::now() - start < ros::Duration((time))) )
 {
  ros::spinOnce(); 
  m_avoid_command.linear.x= start_forward_velocity ;
  m_avoid_command.linear.y=0.0;
  m_avoid_command.angular.z= 0.0;
  m_twist_pub.publish(m_avoid_command);
  
  if (m_current_position.x >= 11.0 || m_current_position.y >= 11.0 ) 
    ROS_WARN("Collision");
  rate.sleep();
 }  
}

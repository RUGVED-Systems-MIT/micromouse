#include "ros/ros.h"
#include "micromouse/position.h"
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <math.h>

class controller
{

  public:
    controller(ros::NodeHandle &nh);
    void odomCB(const nav_msgs::Odometry::ConstPtr& msg);
    bool get_position(micromouse::position::Request  &req, micromouse::position::Response &res);
    double kp,kd, x, y, theta;
    double prev_error,prev_theta_err;
    float yaw1;

  private:

    ros::Publisher pub;
    ros::Subscriber sub;
    ros::ServiceServer server;
    ros::NodeHandle nh;

};

controller::controller(ros::NodeHandle &nh)
{
  this->server = nh.advertiseService("position", &controller::get_position,this);
  this->kp=1.0;
  this->kd=0.0;
}

void controller::odomCB(const nav_msgs::Odometry::ConstPtr& msg)
{
  double roll,pitch,yaw;
  geometry_msgs::Twist speed;
  tf::Quaternion quat;
	tf::quaternionMsgToTF(msg->pose.pose.orientation,quat);
	tf::Matrix3x3(quat).getRPY(roll,pitch,yaw);
  
  double distance_err= sqrt(pow(this->x-msg->pose.pose.position.x,2)+pow(this->y-msg->pose.pose.position.y,2));
  this->theta=M_PI/2+atan2(this->y-msg->pose.pose.position.y,this->x-msg->pose.pose.position.x);
  this->yaw1=yaw;
  double theta_err=this->theta-yaw;
  printf("theta_err: %f",theta_err);
  std::cout<<std::endl;
    if(distance_err>0.1)
    { 
      speed.angular.y=0;
      speed.linear.x=1;
      if(abs(theta_err)>0.05)
        speed.angular.z=1.5*theta_err;
      else
        speed.angular.z = 0;
    }
    else
    {
      speed.linear.x=0;
    }
    pub.publish(speed);
  
}

bool controller::get_position(micromouse::position::Request  &req,
        micromouse::position::Response &res)
{
  if(req.x <= 16 && req.y <= 16) 
  {
    this->x=req.x;
    this->y=req.y;
    res.goal_received={1,this->yaw1};
    this->pub = this->nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
    this->sub = this->nh.subscribe<nav_msgs::Odometry>("/odom", 100, &controller::odomCB, this);
  }
  else
    res.goal_received={0,this->yaw1};
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "position_server");
  ros::NodeHandle nh;
  controller c(nh);
  ros::spin();
  return 0;
}
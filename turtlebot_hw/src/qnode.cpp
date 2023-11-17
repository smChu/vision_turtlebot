/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/turtlebot_hw/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace turtlebot_hw
{
/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv) : init_argc(argc), init_argv(argv)
{
}

QNode::~QNode()
{
  if (ros::isStarted())
  {
    ros::shutdown();  // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
  wait();
}

bool QNode::init()
{
  ros::init(init_argc, init_argv, "turtlebot_hw");
  if (!ros::master::check())
  {
    return false;
  }
  ros::start();  // explicitly needed since our nodehandle is going out of scope.
  ros::NodeHandle n;
  sub = n.subscribe("/camera/image", 10, &QNode::myCallBack, this);
  pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 50);
  // Add your ros communications here.
  start();
  return true;
}

void QNode::run()
{
  ros::Rate loop_rate(33);
  while (ros::ok())
  {
    pub.publish(turtle);
    ros::spinOnce();
    loop_rate.sleep();
  }
  std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
  Q_EMIT rosShutdown();  // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::myCallBack(const sensor_msgs::Image::ConstPtr& imageMsg)
{
  if (!isReceived && cam_img == NULL)
  {
    cam_img = new cv::Mat(cv_bridge::toCvCopy(imageMsg, sensor_msgs::image_encodings::RGB8)->image);
    if (cam_img != NULL)
    {
      isReceived = true;
      Q_EMIT msgSignal();
    }
  }
  else
    return;
}

void QNode::go()
{
  _x = 0.2;
  _z = 0;
  turtle.linear.x = _x;
  turtle.angular.z = _z;
}


void QNode::left()
{
   _x = 0.0;
   _z = 0.4;
  turtle.linear.x = _x;
  turtle.angular.z = _z;

}
// void QNode::right()
// {
//    _x = 0.0;
//    _z = -0.5;
//   turtle.linear.x = _x;
//   turtle.angular.z = _z;

// }


void QNode::stop()
{
  turtle.linear.x = 0;
  turtle.angular.z = 0;

}


}  // namespace turtlebot_hw

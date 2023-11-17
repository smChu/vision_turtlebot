/**
 * @file /include/turtlebot_hw/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef turtlebot_hw_QNODE_HPP_
#define turtlebot_hw_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Twist.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/
using namespace cv;
namespace turtlebot_hw
{
  /*****************************************************************************
  ** Class
  *****************************************************************************/

  class QNode : public QThread
  {
    Q_OBJECT
  public:
    QNode(int argc, char** argv);
    virtual ~QNode();
    bool init();
    void run();
    double _x = 0;
    double _z = 0;
    geometry_msgs::Twist turtle;
    
    bool isReceived = false;
    cv::Mat* cam_img = nullptr;
    cv::Mat resize_img;
    void go();
    void left();
    void right();
    void stop();


  Q_SIGNALS:
    void rosShutdown();
    void msgSignal();

  private:
    int init_argc;
    char** init_argv;
    ros::Subscriber sub;
    void myCallBack(const sensor_msgs::Image::ConstPtr& imageMsg);
    ros::Publisher pub;
  };

}  // namespace s

#endif /* turtlebot_hw_QNODE_HPP_ */

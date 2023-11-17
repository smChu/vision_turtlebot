/**
 * @file /include/turtlebot_hw/main_window.hpp
 *
 * @brief Qt based gui for %(package)s.
 *
 * @date November 2010
 **/
#ifndef turtlebot_hw_MAIN_WINDOW_H
#define turtlebot_hw_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <vector>

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace turtlebot_hw
{
  /*****************************************************************************
  ** Interface [MainWindow]
  *****************************************************************************/
  /**
   * @brief Qt central, all operations relating to the view part here.
   */
  class MainWindow : public QMainWindow
  {
    Q_OBJECT

  public:
    MainWindow(int argc, char** argv, QWidget* parent = 0);
    ~MainWindow();

  public Q_SLOTS:
    void toUi();
    void on_horizontalSlider_valueChanged(int value);
  
  
  private:
    Ui::MainWindowDesign ui;
    QNode qnode;
    QImage img;
    QImage binaryImg;
    QImage blurImg;
    QImage roiImg;
    QImage redImg, yellowImg, GBImg, whiteImg;

    cv::Mat img_binary;
    cv::Mat img_blur;
    cv::Mat img_roi;
    cv::Mat img_red, img_yellow, img_GB, img_white;

    cv::Mat Roi(Mat img, Point * points);

    std::vector<Vec4i> red_line, yellow_line, gb_line, black_line;
    int white_line_point[4] = {0, };
    int yellow_line_point[4] = {0, };

    



    int sliderValue[7]= {0, };
  };

}  // namespace s

#endif  // turtlebot_hw_MAIN_WINDOW_H

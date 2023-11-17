/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/turtlebot_hw/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace turtlebot_hw
{
using namespace Qt;
using namespace std;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget* parent) : QMainWindow(parent), qnode(argc, argv)
{
  ui.setupUi(this);  // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.

  setWindowIcon(QIcon(":/images/icon.png"));

  qnode.init();

  connect(ui.horizontalSlider, SIGNAL(valueChanged(int)), this, SLOT(on_horizontalSlider_valueChanged(int)));
  connect(ui.horizontalSlider_2, SIGNAL(valueChanged(int)), this, SLOT(on_horizontalSlider_valueChanged(int)));
  connect(ui.horizontalSlider_3, SIGNAL(valueChanged(int)), this, SLOT(on_horizontalSlider_valueChanged(int)));
  connect(ui.horizontalSlider_4, SIGNAL(valueChanged(int)), this, SLOT(on_horizontalSlider_valueChanged(int)));
  connect(ui.horizontalSlider_5, SIGNAL(valueChanged(int)), this, SLOT(on_horizontalSlider_valueChanged(int)));
  connect(ui.horizontalSlider_6, SIGNAL(valueChanged(int)), this, SLOT(on_horizontalSlider_valueChanged(int)));

  QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
  QObject::connect(&qnode, SIGNAL(msgSignal()), this, SLOT(toUi()));
}

MainWindow::~MainWindow()
{
}
void MainWindow::toUi()
{
  // HSV 값 조절
  // cv::Scalar lowerBound(sliderValue[0], sliderValue[2], sliderValue[4]);
  // cv::Scalar upperBound(sliderValue[1], sliderValue[3], sliderValue[5]);

  // 빨간 선 hsv 값
  cv::Scalar lowerBound_r(169, 0, 0);
  cv::Scalar upperBound_r(255, 0, 0);

  // 흰 선 hsv 값
  cv::Scalar lowerBound_w(244, 195, 197);
  cv::Scalar upperBound_w(255, 255, 255);
  
  // 노란 선 hsv 값
  cv::Scalar lowerBound_y(0, 2, 0);
  cv::Scalar upperBound_y(255, 255, 0);
  
  // 파란 초록 선 hsv 값
  cv::Scalar lowerBound_gb(0, 93, 0);
  cv::Scalar upperBound_gb(0, 255, 255);

  // 관심영역 좌표
  cv::Point points[4];
  points[0] = cv::Point(0, 0);
  points[1] = cv::Point(320, 0);
  points[2] = cv::Point(320, 240);
  points[3] = cv::Point(0, 240); 

  img_blur = qnode.cam_img->clone();
  // 가우시안 블러 처리
  GaussianBlur(img_blur, img_blur, Size(5, 5), 0);  // Size는 커널 크기로 블러 효과의 크기

  // 이진화
  //cv::inRange(img_binary, lowerBound, upperBound, img_blur);
  cv::inRange(img_blur, lowerBound_r, upperBound_r, img_red);
  cv::inRange(img_blur, lowerBound_y, upperBound_y, img_yellow);
  cv::inRange(img_blur, lowerBound_gb, upperBound_gb, img_GB);
  cv::inRange(img_blur, lowerBound_w, upperBound_w, img_white);

  // 관심영역 설정
  img_red = Roi(img_red, points);
  img_yellow = Roi(img_yellow, points);
  img_GB = Roi(img_GB, points);
  img_white = Roi(img_white, points);


  HoughLinesP(img_red, red_line, 1, (CV_PI / 180), 50, 20, 30);
  for (size_t i = 0; i < red_line.size(); i++)
    {
      Vec4i l = red_line[i];
      cv::line(img_blur, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 255, 255), 2, 8);
    }
  HoughLinesP(img_yellow, yellow_line, 1, (CV_PI / 180), 50, 20, 30);
  for (size_t i = 0; i < yellow_line.size(); i++)
    {
      Vec4i l = yellow_line[i];
      cv::line(img_blur, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(255, 255, 0), 2, 8);
      yellow_line_point[0] = l[0];
      yellow_line_point[1] = l[1];
      yellow_line_point[2] = l[2];
      yellow_line_point[3] = l[3];
    }
  HoughLinesP(img_GB, gb_line, 1, (CV_PI / 180), 50, 20, 30);
  for (size_t i = 0; i < gb_line.size(); i++)
    {
      Vec4i l = gb_line[i];
      cv::line(img_blur, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(60, 255, 255), 2, 8);
    }
  HoughLinesP(img_white, black_line, 1, (CV_PI / 180), 50, 20, 30);
  for (size_t i = 0; i < black_line.size(); i++)
    {
      Vec4i l = black_line[i];
      cv::line(img_blur, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(255, 0, 255), 2, 8);
      white_line_point[0] = l[0];
      white_line_point[1] = l[1];
      white_line_point[2] = l[2];
      white_line_point[3] = l[3];
    }
    
    // cout << yellow_line_point[0]<<"//" <<yellow_line_point[1]<< "//" <<yellow_line_point[2] <<"//" <<yellow_line_point[3] <<endl;
    // cout << white_line_point[0]<<"//" <<white_line_point[1]<< "//" <<white_line_point[2] <<"//" <<white_line_point[3] <<endl;
    
    int x = (white_line_point[0] + yellow_line_point[2]) / 2;
    cout << x << endl;

    if(x>=130 && x<= 160)
    {
      qnode.go();
    }
    else if(black_line.empty())
    {
      qnode.left();
    }
    // else if(yellow_line.empty())
    // {
    //   qnode.right();
    // }


  // QImage
  img = QImage(qnode.cam_img->data, qnode.cam_img->cols, qnode.cam_img->rows, qnode.cam_img->step, QImage::Format_RGB888);
  blurImg = QImage(img_blur.data, img_blur.cols, img_blur.rows, img_blur.step, QImage::Format_RGB888);
  binaryImg = QImage(img_binary.data, img_binary.cols, img_binary.rows, img_binary.step, QImage::Format_Grayscale8);
  roiImg = QImage(img_roi.data, img_roi.cols, img_roi.rows, img_roi.step, QImage::Format_Grayscale8);
  redImg = QImage(img_red.data, img_red.cols, img_red.rows, img_red.step, QImage::Format_Grayscale8);
  yellowImg = QImage(img_yellow.data, img_yellow.cols, img_yellow.rows, img_yellow.step, QImage::Format_Grayscale8);
  GBImg = QImage(img_GB.data, img_GB.cols, img_GB.rows, img_GB.step, QImage::Format_Grayscale8);
  whiteImg = QImage(img_white.data, img_white.cols, img_white.rows, img_white.step, QImage::Format_Grayscale8);
  // ui로 띄우기
  ui.label->setPixmap(QPixmap::fromImage(blurImg));
  ui.label_2->setPixmap(QPixmap::fromImage(redImg));
  ui.label_9->setPixmap(QPixmap::fromImage(yellowImg));
  ui.label_10->setPixmap(QPixmap::fromImage(GBImg));
  ui.label_11->setPixmap(QPixmap::fromImage(whiteImg));
  // 초기화
  qnode.isReceived = false;
  qnode.cam_img = nullptr;
}

void MainWindow::on_horizontalSlider_valueChanged(int value)
{
  QSlider *changedSlider = qobject_cast<QSlider *>(sender());
    if (changedSlider == ui.horizontalSlider)
    {
      ui.horizontalSlider->setValue(value);
      sliderValue[0] = ui.horizontalSlider->value();
      ui.label_3->setText(QString::number(sliderValue[0]));
    }
    else if (changedSlider == ui.horizontalSlider_2)
    {
      ui.horizontalSlider_2->setValue(value);
      sliderValue[1] = ui.horizontalSlider_2->value();
      ui.label_4->setText(QString::number(sliderValue[1]));
    }
    else if (changedSlider == ui.horizontalSlider_3)
    {
      ui.horizontalSlider_3->setValue(value);
      sliderValue[2] = ui.horizontalSlider_3->value();
      ui.label_5->setText(QString::number(sliderValue[2]));
    }
    else if (changedSlider == ui.horizontalSlider_4)
    {
      ui.horizontalSlider_4->setValue(value);
      sliderValue[3] = ui.horizontalSlider_4->value();
      ui.label_6->setText(QString::number(sliderValue[3]));
    }
    else if (changedSlider == ui.horizontalSlider_5)
    {
      ui.horizontalSlider_5->setValue(value);
      sliderValue[4] = ui.horizontalSlider_5->value();
      ui.label_7->setText(QString::number(sliderValue[4]));
    }
    else if (changedSlider == ui.horizontalSlider_6)
    {
      ui.horizontalSlider_6->setValue(value);
      sliderValue[5] = ui.horizontalSlider_6->value();
      ui.label_8->setText(QString::number(sliderValue[5]));
    }
}

cv::Mat MainWindow::Roi(Mat img, Point *points)
  {
    cv::Mat img_roi = Mat::zeros(img.rows, img.cols, CV_8UC1);
    const Point *ppt[1] = {points}; // position of points
    int npt[] = {4};                // the number of points

    cv::fillPoly(img_roi, ppt, npt, 1, Scalar(255, 255, 255), LINE_8); // draw trapeziod

    cv::Mat img_roied;
    bitwise_and(img, img_roi, img_roied); // img && img_roi ==> img_roied
    return img_roied;
  }
/*****************************************************************************
** Functions
*****************************************************************************/

}  // namespace turtlebot_hw

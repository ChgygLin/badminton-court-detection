//
// Created by Chlebus, Grzegorz on 28.08.17.
// Copyright (c) Chlebus, Grzegorz. All rights reserved.
//

#include "TennisCourtModel.h"
#include "GlobalParameters.h"
#include "DebugHelpers.h"
#include "geometry.h"
#include "TimeMeasurement.h"

using namespace cv;

/**                           vline1  vline2    vline3    vline4    vline5
        * 横向坐标             0,0.04  0.46,0.5  3.03,3.07  5.6,5.64  6.06,6.1           
        *                      0.02    0.48      3.05        5.62        6.08
        * 纵向坐标  
        * --------------------------------------------------------------------------
hline1  * 0,0.04       0.02                                                            
        * --------------------------------------------------------------------------
hline2  * 0.76,0.8     0.78                        
        * --------------------------------------------------------------------------
hline3  * 4.68,4.72    4.7                         
        *
        *                             center: (3.05, 6.7)
        * 
        * --------------------------------------------------------------------------
hline4  * 8.68,8.72    8.7                         
        * --------------------------------------------------------------------------
hline5  * 12.6,12.64   12.62                       
        * --------------------------------------------------------------------------
hline6  * 13.36,13.4   13.38
*/

TennisCourtModel::TennisCourtModel()
{
  Point2f hVector(1, 0);    // 单位向量
  const Line hline1 = Line(Point2f(0.02, 0.02), hVector);
  const Line hline2 = Line(Point2f(0.02, 0.78), hVector);
  const Line hline3 = Line(Point2f(0.02, 4.7), hVector);
  const Line hline4 = Line(Point2f(0.02, 8.7), hVector);
  const Line hline5 = Line(Point2f(0.02, 12.62), hVector);
  const Line hline6 = Line(Point2f(0.02, 13.38), hVector);

  hLines = {
    hline1, \
    hline2, \
    hline3, \
    hline4, \
    hline5, \
    hline6
  };

  Point2f vVector(0, 1);    // 单位向量
  const Line vline1 = Line(Point2f(0.02, 0.02), vVector);
  const Line vline2 = Line(Point2f(0.48, 0.02), vVector);
  const Line vline3 = Line(Point2f(3.05, 0.02), vVector);
  const Line vline4 = Line(Point2f(5.62, 0.02), vVector);
  const Line vline5 = Line(Point2f(6.08, 0.02), vVector);

  vLines = {
    vline1, \
    vline2, \
    vline3, \
    vline4, \
    vline5
  };


  // hLinePairs = getPossibleLinePairs(hLines);
  // vLinePairs = getPossibleLinePairs(vLines);

  hLinePairs.push_back(std::make_pair(hline3, hline1));
  hLinePairs.push_back(std::make_pair(hline3, hline2));

  hLinePairs.push_back(std::make_pair(hline4, hline3));
  hLinePairs.push_back(std::make_pair(hline4, hline2));
  hLinePairs.push_back(std::make_pair(hline4, hline1));

  vLinePairs.push_back(std::make_pair(vline1, vline4));
  vLinePairs.push_back(std::make_pair(vline1, vline5));
  vLinePairs.push_back(std::make_pair(vline2, vline4));
  vLinePairs.push_back(std::make_pair(vline2, vline5));


  courtPoints.assign( {Point2f(0.02, 0.02),   Point2f(0.48, 0.02),   Point2f(3.05, 0.02),   Point2f(5.62, 0.02),   Point2f(6.08, 0.02),
                        Point2f(0.02, 0.78),   Point2f(0.48, 0.78),   Point2f(3.05, 0.78),   Point2f(5.62, 0.78),   Point2f(6.08, 0.78),
                        Point2f(0.02, 4.7),    Point2f(0.48, 4.7),    Point2f(3.05, 4.7),    Point2f(5.62, 4.7),    Point2f(6.08, 4.7),
                        Point2f(0.02, 8.7),    Point2f(0.48, 8.7),    Point2f(3.05, 8.7),    Point2f(5.62, 8.7),    Point2f(6.08, 8.7),
                        Point2f(0.02, 12.62),  Point2f(0.48, 12.62),  Point2f(3.05, 12.62),  Point2f(5.62, 12.62),  Point2f(6.08, 12.62),
                        Point2f(0.02, 13.38),  Point2f(0.48, 13.38),  Point2f(3.05, 13.38),  Point2f(5.62, 13.38),  Point2f(6.08, 13.38) });
                        //Point2f(0, 6.7, 1.55), Point2f(6.1, 6.7, 1.55))

  // std::cout << "courtPoints.size() " << courtPoints.size() << std::endl;
  assert(courtPoints.size() == 30);
}

TennisCourtModel::TennisCourtModel(const TennisCourtModel& o)
  : transformationMatrix(o.transformationMatrix)
{
  courtPoints = o.courtPoints;
  hLinePairs = o.hLinePairs;
  vLinePairs = o.vLinePairs;
  hLines = o.hLines;
  vLines = o.vLines;
}

TennisCourtModel& TennisCourtModel::operator=(const TennisCourtModel& o)
{
  transformationMatrix = o.transformationMatrix;
  return *this;
}

float TennisCourtModel::fit(const LinePair& hLinePair, const LinePair& vLinePair,
  const cv::Mat& binaryImage, const cv::Mat& rgbImage)
{
  float bestScore = GlobalParameters().initialFitScore;
  std::vector<Point2f> points = getIntersectionPoints(hLinePair, vLinePair);
  //TODO Check whether the intersection points make sense

  int ox = rgbImage.cols;
  int oy = rgbImage.rows;
  Mat M = (Mat_<double>(3, 3) << 1, 0, -ox, 0, 1, -oy, 0, 0, 1);

  // printf("model hsize: %d\n", hLinePairs.size());
  // printf("model vsize: %d\n", vLinePairs.size());

  for (auto& modelHLinePair: hLinePairs)
  {
    for (auto& modelVLinePair: vLinePairs)
    {
      std::vector<Point2f> modelPoints = getIntersectionPoints(modelHLinePair, modelVLinePair);   // 两横两竖, 4个交点, 若交点在图像外怎么处理?
      Mat matrix = getPerspectiveTransform(modelPoints, points);    // 4对点计算透视变换矩阵
      std::vector<Point2f> transformedModelPoints(30);    // 整个球场所有的交点数, 透视变换的点在图像外怎么处理?
      perspectiveTransform(courtPoints, transformedModelPoints, matrix);

      #if 0
        Mat Hbar = M*matrix;
        double f_square = -(Hbar.at<double>(0, 0)*Hbar.at<double>(0, 1) + Hbar.at<double>(1, 0)*Hbar.at<double>(1,1)) / (Hbar.at<double>(2, 0)*Hbar.at<double>(2, 1));

        if (f_square < 0)
        {
          // printf("f_square < 0\n");
          continue;
        }

        double beta_square = (Hbar.at<double>(0, 1)*Hbar.at<double>(0, 1) + Hbar.at<double>(1, 1)*Hbar.at<double>(1, 1) + f_square*Hbar.at<double>(2, 1)*Hbar.at<double>(2, 1)) / \
                          (Hbar.at<double>(0, 0)*Hbar.at<double>(0, 0) + Hbar.at<double>(1, 0)*Hbar.at<double>(1, 0) + f_square*Hbar.at<double>(2, 0)*Hbar.at<double>(2, 0));
  
        // printf("beta_square : %f\n", beta_square);
        if ( (beta_square >= 4) || (beta_square <= 0.25) )
          continue;
      #endif

      float score = evaluateModel(transformedModelPoints, binaryImage);
      if (score > bestScore)
      {
        bestScore = score;
        transformationMatrix = matrix;
      }
    //  Mat image = rgbImage.clone();
    //  drawModel(transformedModelPoints, image);
    //  displayImage("TennisCourtModel", image, 1);
    }
  }
  return bestScore;
}


std::vector<cv::Point2f> TennisCourtModel::getIntersectionPoints(const LinePair& hLinePair,
  const LinePair& vLinePair)
{
  std::vector<Point2f> v;
  Point2f point;

  if (hLinePair.first.computeIntersectionPoint(vLinePair.first, point))
  {
    v.push_back(point);
  }
  if (hLinePair.first.computeIntersectionPoint(vLinePair.second, point))
  {
    v.push_back(point);
  }
  if (hLinePair.second.computeIntersectionPoint(vLinePair.first, point))
  {
    v.push_back(point);
  }
  if (hLinePair.second.computeIntersectionPoint(vLinePair.second, point))
  {
    v.push_back(point);
  }

  assert(v.size() == 4);

  return v;
}

std::vector<LinePair> TennisCourtModel::getPossibleLinePairs(std::vector<Line>& lines)
{
  std::vector<LinePair> linePairs;
  for (size_t first = 0; first < lines.size(); ++first)
//  for (size_t first = 0; first < 1; ++first)
  {
    for (size_t second = first + 1; second < lines.size(); ++second)
    {
      linePairs.push_back(std::make_pair(lines[first], lines[second]));
    }
  }
  return linePairs;
}

void TennisCourtModel::show()
{
  int w,h, cw, ch;

  int ratio = 100;
  int dx,dy;

  w = 1000;
  h = 1600;
  cw = w/2;
  ch = h/2;

  Mat image(h, w, CV_8UC3, Scalar(255, 255, 255));
  Point2f center_p(3.05, 6.7);

  dx = cw - ratio * center_p.x;
  dy = ch - ratio * center_p.y;

  std::vector<Point2f> court = {
                                  Point2f(0.02, 13.38), Point2f(0.48, 13.38), Point2f(3.05, 13.38), Point2f(5.62, 13.38), Point2f(6.08, 13.38), \
                                  Point2f(0.02, 12.62), Point2f(0.48, 12.62), Point2f(3.05, 12.62), Point2f(5.62, 12.62), Point2f(6.08, 12.62), \
                                  Point2f(0.02, 8.7),  Point2f(0.48, 8.7), Point2f(3.05, 8.7), Point2f(5.62, 8.7), Point2f(6.08, 8.7), \
                                  Point2f(0.02, 4.7),  Point2f(0.48, 4.7), Point2f(3.05, 4.7), Point2f(5.62, 4.7), Point2f(6.08, 4.7), \
                                  Point2f(0.02, 0.78), Point2f(0.48, 0.78), Point2f(3.05, 0.78), Point2f(5.62, 0.78), Point2f(6.08, 0.78), \
                                  Point2f(0.02, 0.02), Point2f(0.48, 0.02), Point2f(3.05, 0.02), Point2f(5.62, 0.02), Point2f(6.08, 0.02)
                                };

                                 // 网线  (0.02, 6.7)   (6.08, 6.7)   柱子30度角倾斜: x坐标 - ( 1.55*sin(30) = 0.775 ),   y坐标 - ( 1.55*cos(30) = 1.342 )
                                 // (-0.773, 5.358)   (5.305, 5.358)

  std::vector<Point2f> transformedModelPoints(30);
  char text[10];
  for (int i=0; i<30; i++)
  {
    transformedModelPoints[i].x = ratio * court[i].x + dx;
    transformedModelPoints[i].y = ratio * court[i].y + dy;

    sprintf(text, "%d", i+1);
    circle(image, Point(transformedModelPoints[i].x, transformedModelPoints[i].y), 8, Scalar(0, 0, 255), -1);
    putText(image, text, Point(transformedModelPoints[i].x-20, transformedModelPoints[i].y-10), FONT_HERSHEY_COMPLEX, 1.0, Scalar(0, 0, 255), 2);
  }

  drawModel(transformedModelPoints, image, (0, 0, 0));

  // 网带
  std::vector<Point2f> net = { Point2f(0.02, 6.7), Point2f(6.08, 6.7), Point2f(-0.773, 5.358), Point2f(5.305, 5.358) };
  for (int i=0; i<4; i++)
  {
    net[i].x = ratio * net[i].x + dx;
    net[i].y = ratio * net[i].y + dy;
  }

  drawLine(net[0], net[2], image, (0, 255, 255)); // 左柱
  drawLine(net[1], net[3], image, (0, 255, 255)); // 右柱
  drawLine(net[2], net[3], image, (0, 255, 255)); // 网

  // 左
  sprintf(text, "%d", 31);
  circle(image, Point(net[2].x, net[2].y), 8, Scalar(0, 0, 255), -1);
  putText(image, text, Point(net[2].x-20, net[2].y-10), FONT_HERSHEY_COMPLEX, 1.0, Scalar(0, 0, 255), 2);

  // 右
  sprintf(text, "%d", 32);
  circle(image, Point(net[3].x, net[3].y), 8, Scalar(0, 0, 255), -1);
  putText(image, text, Point(net[3].x-20, net[3].y-10), FONT_HERSHEY_COMPLEX, 1.0, Scalar(0, 0, 255), 2);


  imwrite("../court.png", image);
  displayImage("TennisCourtModel", image);
}


void TennisCourtModel::drawModel(cv::Mat& image, Scalar color)
{
  std::vector<Point2f> transformedModelPoints(30);
  perspectiveTransform(courtPoints, transformedModelPoints, transformationMatrix);
  drawModel(transformedModelPoints, image, color);
}

void TennisCourtModel::drawModel(std::vector<Point2f>& courtPoints, Mat& image, Scalar color)
{
  // 6 lines
  drawLine(courtPoints[0], courtPoints[4], image, color);
  drawLine(courtPoints[5], courtPoints[9], image, color);
  drawLine(courtPoints[10], courtPoints[14], image, color);
  drawLine(courtPoints[15], courtPoints[19], image, color);
  drawLine(courtPoints[20], courtPoints[24], image, color);
  drawLine(courtPoints[25], courtPoints[29], image, color);


  // 6 lines
  drawLine(courtPoints[0], courtPoints[25], image, color);
  drawLine(courtPoints[1], courtPoints[26], image, color);
  drawLine(courtPoints[2], courtPoints[12], image, color);
  drawLine(courtPoints[17], courtPoints[27], image, color);
  drawLine(courtPoints[3], courtPoints[28], image, color);
  drawLine(courtPoints[4], courtPoints[29], image, color);
}


float TennisCourtModel::evaluateModel(const std::vector<cv::Point2f>& courtPoints, const cv::Mat& binaryImage)
{
  float score = 0;

  // TODO: heuritic to see whether the model makes sense
  float d1 = distance(courtPoints[0], courtPoints[4]);
  float d2 = distance(courtPoints[4], courtPoints[29]);
  float d3 = distance(courtPoints[29], courtPoints[25]);
  float d4 = distance(courtPoints[25], courtPoints[0]);
  float t = 30;
  if (d1 < t || d2 < t || d3 < t || d4 < t)
  {
    return GlobalParameters().initialFitScore;
  }

  score += computeScoreForLineSegment(courtPoints[0], courtPoints[4], binaryImage);
  score += computeScoreForLineSegment(courtPoints[5], courtPoints[9], binaryImage);
  score += computeScoreForLineSegment(courtPoints[10], courtPoints[14], binaryImage);
  score += computeScoreForLineSegment(courtPoints[15], courtPoints[19], binaryImage);
  score += computeScoreForLineSegment(courtPoints[20], courtPoints[24], binaryImage);
  score += computeScoreForLineSegment(courtPoints[25], courtPoints[29], binaryImage);
  score += computeScoreForLineSegment(courtPoints[0], courtPoints[25], binaryImage);
  score += computeScoreForLineSegment(courtPoints[1], courtPoints[26], binaryImage);
  score += computeScoreForLineSegment(courtPoints[3], courtPoints[28], binaryImage);
  score += computeScoreForLineSegment(courtPoints[4], courtPoints[29], binaryImage);

//  std::cout << "Score = " << score << std::endl;

  return score;
}

float TennisCourtModel::computeScoreForLineSegment(const cv::Point2f& start, const cv::Point2f& end,
  const cv::Mat& binaryImage)
{
  float score = 0;
  float fgScore = 1;      // 前景像素的得分
  float bgScore = -0.5;   // 背景像素的得分
  int length = round(distance(start, end));   // 计算线段长度并四舍五入为整数

  Point2f vec = normalize(end-start);     // 计算线段的单位向量

  for (int i = 0; i < length; ++i)    // 迭代线段上的每个点
  {
    Point2f p = start + i*vec;      // 计算线段上的点的坐标
    int x = round(p.x);
    int y = round(p.y);
    if (isInsideTheImage(x, y, binaryImage))
    {
      uchar imageValue = binaryImage.at<uchar>(y,x);
      if (imageValue == GlobalParameters().fgValue)   // 根据像素值分配得分,fgValue为白色255
      {
        score += fgScore;
      }
      else
      {
        score += bgScore;
      }
    }
  }
  return score;
}


bool TennisCourtModel::isInsideTheImage(float x, float y, const cv::Mat& image)
{
  return (x >= 0 && x < image.cols) && (y >= 0 && y < image.rows);
}

void TennisCourtModel::writeToFile(const std::string& filename)
{
  std::vector<Point2f> transformedModelPoints(16);
  perspectiveTransform(courtPoints, transformedModelPoints, transformationMatrix);

  std::ofstream outFile(filename);
  if (!outFile.is_open())
  {
    throw std::runtime_error("Unable to open file: " + filename);
  }
  for (auto& point: transformedModelPoints)
  {
    outFile << point.x << ";" << point.y << std::endl;
  }
}

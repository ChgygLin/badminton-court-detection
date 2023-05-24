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

/**
 * 横向坐标             0,0.04  0.46,0.5  3.03,3.07  5.6,5.64  6.06,6.1           
 *                      0.02    0.48      3.05        5.62        6.08
 * 纵向坐标  
 * --------------------------------------------------------------------------
 * 0,0.04       0.02
 * --------------------------------------------------------------------------
 * 0.76,0.8     0.78
 * --------------------------------------------------------------------------
 * 4.68,4.72    4.7
 * --------------------------------------------------------------------------
 * 8.68,8.72    8.7     
 * --------------------------------------------------------------------------
 * 12.6,12.64   12.62  
 * --------------------------------------------------------------------------                                                            
 * 13.36,13.4   13.38   
*/

TennisCourtModel::TennisCourtModel()
{
  Point2f hVector(1, 0);    // 单位向量
  const Line up_long_service_line_for_singles = Line(Point2f(0.02, 0.02), hVector);
  const Line up_long_service_line_for_doubles = Line(Point2f(0.02, 0.78), hVector);
  const Line up_short_service_line = Line(Point2f(0.02, 4.7), hVector);
  const Line down_short_service_line = Line(Point2f(0.02, 8.7), hVector);
  const Line down_long_service_line_for_doubles = Line(Point2f(0.02, 12.62), hVector);
  const Line down_long_service_line_for_singles = Line(Point2f(0.02, 13.38), hVector);

  hLines = {
    up_long_service_line_for_singles, \
    up_long_service_line_for_doubles, \
    up_short_service_line, \
    down_short_service_line, \
    down_long_service_line_for_doubles, \
    down_long_service_line_for_singles
  };

  Point2f vVector(0, 1);    // 单位向量
  const Line left_side_line_for_doubles = Line(Point2f(0.02, 0.02), vVector);
  const Line left_side_line_for_singles = Line(Point2f(0.48, 0.02), vVector);
  const Line centre_line = Line(Point2f(3.05, 0.02), vVector);
  const Line right_side_line_for_singles = Line(Point2f(5.62, 0.02), vVector);
  const Line right_side_line_for_doubles = Line(Point2f(6.08, 0.02), vVector);

  vLines = {
    left_side_line_for_doubles, \
    left_side_line_for_singles, \
    centre_line, \
    right_side_line_for_singles, \
    right_side_line_for_doubles
  };


  hLinePairs = getPossibleLinePairs(hLines);
  vLinePairs = getPossibleLinePairs(vLines);


  Point2f point;
  if (up_long_service_line_for_singles.computeIntersectionPoint(left_side_line_for_doubles, point))
  {
    courtPoints.push_back(point); // P1
  }
  if (up_long_service_line_for_singles.computeIntersectionPoint(left_side_line_for_singles, point))
  {
    courtPoints.push_back(point); // P2
  }
  if (up_long_service_line_for_singles.computeIntersectionPoint(centre_line, point))
  {
    courtPoints.push_back(point); // P3
  }
  if (up_long_service_line_for_singles.computeIntersectionPoint(right_side_line_for_singles, point))
  {
    courtPoints.push_back(point);  // P4
  }
  if (up_long_service_line_for_singles.computeIntersectionPoint(right_side_line_for_doubles, point))
  {
    courtPoints.push_back(point);  // P5
  }


  if (up_long_service_line_for_doubles.computeIntersectionPoint(left_side_line_for_doubles, point))
  {
    courtPoints.push_back(point); // P1
  }
  if (up_long_service_line_for_doubles.computeIntersectionPoint(left_side_line_for_singles, point))
  {
    courtPoints.push_back(point); // P2
  }
  if (up_long_service_line_for_doubles.computeIntersectionPoint(centre_line, point))
  {
    courtPoints.push_back(point); // P3
  }
  if (up_long_service_line_for_doubles.computeIntersectionPoint(right_side_line_for_singles, point))
  {
    courtPoints.push_back(point);  // P4
  }
  if (up_long_service_line_for_doubles.computeIntersectionPoint(right_side_line_for_doubles, point))
  {
    courtPoints.push_back(point);  // P5
  }


  if (up_short_service_line.computeIntersectionPoint(left_side_line_for_doubles, point))
  {
    courtPoints.push_back(point); // P1
  }
  if (up_short_service_line.computeIntersectionPoint(left_side_line_for_singles, point))
  {
    courtPoints.push_back(point); // P2
  }
  if (up_short_service_line.computeIntersectionPoint(centre_line, point))
  {
    courtPoints.push_back(point); // P3
  }
  if (up_short_service_line.computeIntersectionPoint(right_side_line_for_singles, point))
  {
    courtPoints.push_back(point);  // P4
  }
  if (up_short_service_line.computeIntersectionPoint(right_side_line_for_doubles, point))
  {
    courtPoints.push_back(point);  // P5
  }


  if (down_short_service_line.computeIntersectionPoint(left_side_line_for_doubles, point))
  {
    courtPoints.push_back(point); // P1
  }
  if (down_short_service_line.computeIntersectionPoint(left_side_line_for_singles, point))
  {
    courtPoints.push_back(point); // P2
  }
  if (down_short_service_line.computeIntersectionPoint(centre_line, point))
  {
    courtPoints.push_back(point); // P3
  }
  if (down_short_service_line.computeIntersectionPoint(right_side_line_for_singles, point))
  {
    courtPoints.push_back(point);  // P4
  }
  if (down_short_service_line.computeIntersectionPoint(right_side_line_for_doubles, point))
  {
    courtPoints.push_back(point);  // P5
  }


  if (down_long_service_line_for_doubles.computeIntersectionPoint(left_side_line_for_doubles, point))
  {
    courtPoints.push_back(point); // P1
  }
  if (down_long_service_line_for_doubles.computeIntersectionPoint(left_side_line_for_singles, point))
  {
    courtPoints.push_back(point); // P2
  }
  if (down_long_service_line_for_doubles.computeIntersectionPoint(centre_line, point))
  {
    courtPoints.push_back(point); // P3
  }
  if (down_long_service_line_for_doubles.computeIntersectionPoint(right_side_line_for_singles, point))
  {
    courtPoints.push_back(point);  // P4
  }
  if (down_long_service_line_for_doubles.computeIntersectionPoint(right_side_line_for_doubles, point))
  {
    courtPoints.push_back(point);  // P5
  }


  if (down_long_service_line_for_singles.computeIntersectionPoint(left_side_line_for_doubles, point))
  {
    courtPoints.push_back(point); // P1
  }
  if (down_long_service_line_for_singles.computeIntersectionPoint(left_side_line_for_singles, point))
  {
    courtPoints.push_back(point); // P2
  }
  if (down_long_service_line_for_singles.computeIntersectionPoint(centre_line, point))
  {
    courtPoints.push_back(point); // P3
  }
  if (down_long_service_line_for_singles.computeIntersectionPoint(right_side_line_for_singles, point))
  {
    courtPoints.push_back(point);  // P4
  }
  if (down_long_service_line_for_singles.computeIntersectionPoint(right_side_line_for_doubles, point))
  {
    courtPoints.push_back(point);  // P5
  }


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

  for (auto& modelHLinePair: hLinePairs)
  {
    for (auto& modelVLinePair: vLinePairs)
    {
      std::vector<Point2f> modelPoints = getIntersectionPoints(modelHLinePair, modelVLinePair);   // 两横两竖, 4个交点, 若交点在图像外怎么处理?
      Mat matrix = getPerspectiveTransform(modelPoints, points);    // 4对点计算透视变换矩阵
      std::vector<Point2f> transformedModelPoints(30);    // 整个球场所有的交点数, 透视变换的点在图像外怎么处理?
      perspectiveTransform(courtPoints, transformedModelPoints, matrix);
      float score = evaluateModel(transformedModelPoints, binaryImage);
      if (score > bestScore)
      {
        bestScore = score;
        transformationMatrix = matrix;
      }
//      Mat image = rgbImage.clone();
//      drawModel(transformedModelPoints, image);
//      displayImage("TennisCourtModel", image, 0);
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


void TennisCourtModel::drawModel(cv::Mat& image, Scalar color)
{
  std::vector<Point2f> transformedModelPoints(16);
  perspectiveTransform(courtPoints, transformedModelPoints, transformationMatrix);
  drawModel(transformedModelPoints, image, color);
}

void TennisCourtModel::drawModel(std::vector<Point2f>& courtPoints, Mat& image, Scalar color)
{
  drawLine(courtPoints[0], courtPoints[1], image, color);
  drawLine(courtPoints[1], courtPoints[2], image, color);
  drawLine(courtPoints[2], courtPoints[3], image, color);
  drawLine(courtPoints[3], courtPoints[0], image, color);

  drawLine(courtPoints[4], courtPoints[5], image, color);
  drawLine(courtPoints[6], courtPoints[7], image, color);

  drawLine(courtPoints[8], courtPoints[9], image, color);
  drawLine(courtPoints[10], courtPoints[11], image, color);

  drawLine(courtPoints[12], courtPoints[13], image, color);
  drawLine(courtPoints[14], courtPoints[15], image, color);
}


float TennisCourtModel::evaluateModel(const std::vector<cv::Point2f>& courtPoints, const cv::Mat& binaryImage)
{
  float score = 0;

  // TODO: heuritic to see whether the model makes sense
  float d1 = distance(courtPoints[0], courtPoints[1]);
  float d2 = distance(courtPoints[1], courtPoints[2]);
  float d3 = distance(courtPoints[2], courtPoints[3]);
  float d4 = distance(courtPoints[3], courtPoints[0]);
  float t = 30;
  if (d1 < t || d2 < t || d3 < t || d4 < t)
  {
    return GlobalParameters().initialFitScore;
  }

  score += computeScoreForLineSegment(courtPoints[0], courtPoints[1], binaryImage);
  score += computeScoreForLineSegment(courtPoints[1], courtPoints[2], binaryImage);
  score += computeScoreForLineSegment(courtPoints[2], courtPoints[3], binaryImage);
  score += computeScoreForLineSegment(courtPoints[3], courtPoints[0], binaryImage);
  score += computeScoreForLineSegment(courtPoints[4], courtPoints[5], binaryImage);
  score += computeScoreForLineSegment(courtPoints[6], courtPoints[7], binaryImage);
  score += computeScoreForLineSegment(courtPoints[8], courtPoints[9], binaryImage);
  score += computeScoreForLineSegment(courtPoints[10], courtPoints[11], binaryImage);
  score += computeScoreForLineSegment(courtPoints[12], courtPoints[13], binaryImage);
//  score += computeScoreForLineSegment(courtPoints[14], courtPoints[14], binaryImage);

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

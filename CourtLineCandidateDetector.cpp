//
// Created by Chlebus, Grzegorz on 27.08.17.
// Copyright (c) Chlebus, Grzegorz. All rights reserved.
//

#include "CourtLineCandidateDetector.h"
#include "GlobalParameters.h"
#include "TimeMeasurement.h"
#include "DebugHelpers.h"

using namespace cv;

bool CourtLineCandidateDetector::debug = false;
const std::string CourtLineCandidateDetector::windowName = "CourtLineCandidateDetector";

CourtLineCandidateDetector::Parameters::Parameters()
{
  houghThreshold = 120;
  distanceThreshold = 8;
  refinementIterations = 5;
}

CourtLineCandidateDetector::CourtLineCandidateDetector()
  : CourtLineCandidateDetector(Parameters())
{

}


CourtLineCandidateDetector::CourtLineCandidateDetector(CourtLineCandidateDetector::Parameters p)
  : parameters(p)
{

}


std::vector<Line> CourtLineCandidateDetector::run(const cv::Mat& binaryImage, const cv::Mat& rgbImage)
{
  TimeMeasurement::start("CourtLineCandidateDetector::run");

  std::vector<Line> lines;
  TimeMeasurement::start("\textractLines");
  lines = extractLines(binaryImage, rgbImage);
  TimeMeasurement::start("\textractLines");

  for (int i = 0; i < parameters.refinementIterations; ++i)
  {
    TimeMeasurement::start("\tgetRefinedParameters");
    refineLineParameters(lines, binaryImage, rgbImage);
    TimeMeasurement::stop("\tgetRefinedParameters");

    TimeMeasurement::start("\tremoveDuplicateLines");
    removeDuplicateLines(lines, rgbImage);
    TimeMeasurement::stop("\tremoveDuplicateLines");
  }

  TimeMeasurement::stop("CourtLineCandidateDetector::run");
  return lines;
}

std::vector<Line> CourtLineCandidateDetector::extractLines(const cv::Mat& binaryImage,
  const cv::Mat& rgbImage)
{
  std::vector<cv::Point2f> tmpLines;

  HoughLines(binaryImage, tmpLines, 1, CV_PI / 180, parameters.houghThreshold*1.2);   // 按照1度的分辨率对二值化图像进行霍夫变换，提取线条候选区域 

  std::vector<Line> lines;
  for (size_t i = 0; i < tmpLines.size(); ++i)
  {
    lines.push_back(Line::fromRhoTheta(tmpLines[i].x, tmpLines[i].y));    // 将极坐标形式的线条参数转换为笛卡尔坐标系下的线条参数
  }

  if (debug)
  {
    std::cout << "CourtLineCandidateDetector::extractLines line count = " << lines.size() << std::endl;
    Mat image = rgbImage.clone();
    drawLines(lines, image);
    displayImage(windowName, image);
  }

  return lines;
}


void CourtLineCandidateDetector::refineLineParameters(std::vector<Line>& lines,
  const Mat& binaryImage, const Mat& rgbImage)
{
  for (auto& line: lines)
  {
    line = getRefinedParameters(line, binaryImage, rgbImage);   // 对当前直线进行精细化调整，并将结果存储在当前直线中
  }
  if (debug)
  {
    Mat image = rgbImage.clone();
    drawLines(lines, image);
    displayImage(windowName, image);
  }
}

bool lineEqual(const Line& a, const Line& b)
{
  return a.isDuplicate(b);
}


bool CourtLineCandidateDetector::operator()(const Line& a, const Line& b)
{
  Mat tmpImage = image.clone();
  drawLine(a, tmpImage);
  drawLine(b, tmpImage);
  displayImage(windowName, tmpImage, 1);
  return a.isDuplicate(b);
}

void CourtLineCandidateDetector::removeDuplicateLines(std::vector<Line>& lines, const cv::Mat& rgbImage)
{
  image = rgbImage.clone();
  auto it = std::unique(lines.begin(), lines.end(), lineEqual);   // 将相邻的重复元素移动到容器的末尾, 并返回去重后的向量的末尾迭代器
  lines.erase(it, lines.end());   // 将重复元素后面的所有元素从容器中删除
  if (debug)
  {
    std::cout << "CourtLineCandidateDetector::removeDuplicateLines line count =  " << lines.size() << std::endl;
    Mat image = rgbImage.clone();
    drawLines(lines, image);
    displayImage(windowName, image);
  }
}

Line CourtLineCandidateDetector::getRefinedParameters(Line line, const Mat& binaryImage,
  const cv::Mat& rgbImage)
{
  Mat A = getClosePointsMatrix(line, binaryImage, rgbImage);    // 获取与当前直线相邻的所有边缘点, 并将它们存储在一个矩阵A中。
  Mat X = Mat::zeros(1, 4, CV_32F);

  // 对矩阵A中的边缘点进行拟合, 得到直线参数, 
  // 其中前两个元素为直线在笛卡尔坐标系下的方向向量，后两个元素为直线上的一点
  fitLine(A, X, DIST_L2, 0, 0.01, 0.01);

  Point2f v(X.at<float>(0,0), X.at<float>(0,1));
  Point2f p(X.at<float>(0,2), X.at<float>(0,3));

  return Line(p, v);
}

Mat CourtLineCandidateDetector::getClosePointsMatrix(Line line, const Mat& binaryImage,
  const cv::Mat& rgbImage)
{
  Mat M = Mat::zeros(0, 2, CV_32F);   // 为什么不使用vector<Point>

  Mat image = rgbImage.clone(); // debug

  for (unsigned int x = 0; x < binaryImage.cols; ++x)
  {
    for (unsigned int y = 0; y < binaryImage.rows; ++y)
    {
      if (binaryImage.at<uchar>(y, x) == GlobalParameters().fgValue)
      {
        float distance = line.getDistance(Point2f(x, y));   // 点到直线的距离小于阈值的即为邻近点
        if (distance < parameters.distanceThreshold)
        {
//          drawPoint(Point2f(x, y), image, Scalar(255,0,0));
          Mat point = Mat::zeros(1, 2, CV_32F);
          point.at<float>(0, 0) = x;
          point.at<float>(0, 1) = y;
          M.push_back(point);
        }
      }
    }
  }

  if (false)
  {
    drawLine(line, image);
    displayImage(windowName, image);
  }

  return M;
}

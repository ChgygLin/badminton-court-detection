//
// Created by Chlebus, Grzegorz on 25.08.17.
// Copyright (c) Chlebus, Grzegorz. All rights reserved.
//

#include "Line.h"
#include "geometry.h"

using namespace cv;

/*
  从极坐标形式的线条参数（即极径和极角）转换到笛卡尔坐标系下的线条参数。
  线条在极坐标形式下的参数 rho 和 theta。
*/
Line Line::fromRhoTheta(float rho, float theta)
{
  double a = cos(theta), b = sin(theta);  // 根据极角theta计算出直线在笛卡尔坐标系下的斜率a,b
  double x0 = a * rho, y0 = b * rho;  // 根据极径rho和斜率a、b,计算出直线在笛卡尔坐标系下的截距

  // 根据截距x0,y0,以及一个固定长度的线段（这里是2000), 计算出直线在笛卡尔坐标系下的两个端点p1和p2.
  Point2f p1((x0 + 2000 * (-b)), (y0 + 2000 * (a)));
  Point2f p2((x0 - 2000 * (-b)), (y0 - 2000 * (a)));

  return Line::fromTwoPoints(p1, p2);
}


Line Line::fromTwoPoints(cv::Point2f p1, cv::Point2f p2)
{
  Point2f vec = p2 - p1;
  return Line(p1, vec);
}

Line::Line(cv::Point2f point, cv::Point2f vector)
  : u(point)
  , v(normalize(vector))
{

}

cv::Point2f Line::getPoint() const
{
  return u;
}


cv::Point2f Line::getVector() const
{
  return v;
}

float Line::getDistance(const cv::Point2f& point) const
{
  Point2f pointOnLine = getPointOnLineClosestTo(point);
  return distance(point, pointOnLine);
}

cv::Point2f Line::getPointOnLineClosestTo(const cv::Point2f point) const
{
  Point2f n;
  float c;
  toImplicit(n, c);
  float q = c - n.dot(point);
  return point - q*n;
}

bool Line::isDuplicate(const Line& otherLine) const
{
  Point2f n1, n2;
  float c1, c2;
  toImplicit(n1, c1);
  otherLine.toImplicit(n2, c2);
  float dot = fabs(n1.dot(n2));
  double dotThreshold = cos(1*CV_PI/180);
  if ((dot > dotThreshold) && fabs(fabs(c1)-fabs(c2)) < 10)
  {
    return true;
  }
  else
  {
    return false;
  }
}

void Line::toImplicit(cv::Point2f& n, float& c) const
{
  n = perpendicular(v);   // 直线的法向量. 如果(a, b)是直线的方向向量, 则(-b, a)是直线的法向量
  c = n.dot(u);   // 直线到原点的距离
}


bool Line::isVertical() const
{
  Point2f n;
  float c;
  toImplicit(n, c);

  // atan2(n.y, n.x) : 法向量的极角theta
  // 判断theta是否在65度与-65度之间
  return (fabs(atan2(n.y, n.x)) < 65 * CV_PI / 180) || (fabs(atan2(-n.y, -n.x)) < 65 * CV_PI / 180);
}

bool Line::computeIntersectionPoint(const Line& line, cv::Point2f& intersectionPoint) const
{
  Point2f x = line.getPoint() - u;
  Point2f d1 = v;
  Point2f d2 = line.getVector();
  float cross = d1.x * d2.y - d1.y * d2.x;
  if (abs(cross) < /*EPS*/1e-8)
    return false;
  double t1 = (x.x * d2.y - x.y * d2.x) / cross;
  intersectionPoint = u + d1 * t1;
  return true;
}

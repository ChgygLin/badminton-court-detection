//
// Created by Chlebus, Grzegorz on 26.08.17.
// Copyright (c) Chlebus, Grzegorz. All rights reserved.
//

#include "CourtLinePixelDetector.h"
#include "GlobalParameters.h"
#include "DebugHelpers.h"
#include "TimeMeasurement.h"

using namespace cv;

bool CourtLinePixelDetector::debug = false;
const std::string CourtLinePixelDetector::windowName = "CourtLinePixelDetector";

CourtLinePixelDetector::Parameters::Parameters()
{
  threshold = 80;
  diffThreshold = 20;
  t = 4;
  gradientKernelSize = 3;
  kernelSize = 41;
}

CourtLinePixelDetector::CourtLinePixelDetector()
  : CourtLinePixelDetector(Parameters())
{

}

CourtLinePixelDetector::CourtLinePixelDetector(CourtLinePixelDetector::Parameters p)
  : parameters(p)
{

}

Mat CourtLinePixelDetector::run(const Mat& frame)
{
  TimeMeasurement::start("CourtLinePixelDetector::run");

  TimeMeasurement::start("\tgetLuminanceChannel");
  Mat luminanceImage = getLuminanceChannel(frame);
  TimeMeasurement::stop("\tgetLuminanceChannel");

  TimeMeasurement::start("\tdetectLinePixels");
  Mat image = detectLinePixels(luminanceImage);
  TimeMeasurement::stop("\tdetectLinePixels");

  TimeMeasurement::start("\tfilterLinePixels");
  image = filterLinePixels(image, luminanceImage);
  TimeMeasurement::stop("\tfilterLinePixels");

  TimeMeasurement::stop("CourtLinePixelDetector::run");
  return image;
}

/*  
    提取亮度通道是因为在彩色图像中，每个像素都由 RGB 三个通道的值组成，而这些通道的值可能受到光照和颜色等因素的影响，
导致像素之间的差异很大。而亮度通道只与像素的亮度有关，不受颜色的影响，因此可以更准确地提取出线条的轮廓。
另外，二值化的图像可能会丢失一些细节信息，而提取亮度通道可以保留更多的信息，有利于后续的线条检测和分析。
*/
cv::Mat CourtLinePixelDetector::getLuminanceChannel(const cv::Mat& frame)
{
  Mat imgYCbCr;
  cvtColor(frame, imgYCbCr, COLOR_RGB2YCrCb);
  Mat luminanceChannel(frame.rows, frame.cols, CV_8UC1);
  const int from_to[2] = {0, 0};
  mixChannels(&frame, 1, &luminanceChannel, 1, from_to, 1);   // 提取亮度通道

  if (debug)
  {
    displayImage(windowName, frame);
    displayImage(windowName, imgYCbCr);
    displayImage(windowName, luminanceChannel);
  }

  return luminanceChannel;
}

// 此函数在给定的图像中检测线条像素。
// 函数返回一个二值化图像，其中线条像素的值为 255，非线条像素的值为 0。

Mat CourtLinePixelDetector::detectLinePixels(const cv::Mat& image)
{
  Mat pixelImage(image.rows, image.cols, CV_8UC1, Scalar(GlobalParameters().bgValue));  // 创建一个与输入图像大小相同的单通道图像，并初始化为背景值0

  for (unsigned int x = 0; x < image.cols; ++x)
  {
    for (unsigned int y = 0; y < image.rows; ++y)
    {
      pixelImage.at<uchar>(y,x) = isLinePixel(image, x, y); // 检测当前像素是否为线条像素，并将结果存储到输出图像中
    }
  }

  if (debug)
  {
    displayImage(windowName, pixelImage);
  }

  return pixelImage;
}


uchar CourtLinePixelDetector::isLinePixel(const Mat& image, unsigned int x, unsigned int y)
{
  if (x < parameters.t || image.cols - x <= parameters.t) // 如果像素在图像边缘，则认为其不是线条像素
  {
    return GlobalParameters().bgValue;
  }
  if (y < parameters.t || image.rows - y <= parameters.t)
  {
    return GlobalParameters().bgValue;
  }

  uchar value = image.at<uchar>(y,x);
  if (value < parameters.threshold) // 如果灰度值小于阈值，则认为其不是线条像素
  {
    return GlobalParameters().bgValue;
  }

  uchar topValue = image.at<uchar>(y-parameters.t, x);    // 获取当前像素上方的像素灰度值
  uchar lowerValue = image.at<uchar>(y+parameters.t, x);  // 获取当前像素下方的像素灰度值
  uchar leftValue = image.at<uchar>(y,x-parameters.t);    // 获取当前像素左侧的像素灰度值
  uchar rightValue = image.at<uchar>(y,x+parameters.t);   // 获取当前像素右侧的像素灰度值

  // 如果当前像素的灰度值与左右两侧像素的灰度值之差都大于阈值，则认为其是线条像素
  if ((value - leftValue > parameters.diffThreshold) && (value - rightValue > parameters.diffThreshold))
  {
    return GlobalParameters().fgValue;
  }

  // 如果当前像素的灰度值与上下两侧像素的灰度值之差都大于阈值，则认为其是线条像素
  if ((value - topValue > parameters.diffThreshold) && (value - lowerValue > parameters.diffThreshold))
  {
    return GlobalParameters().fgValue;
  }

  return GlobalParameters().bgValue;
}

// 此函数对输入的二值化图像进行滤波，去除非线条像素。
// 函数有两个参数：
// - `binaryImage`：输入的二值化图像，其中线条像素的值为 255，非线条像素的值为 0。
// - `luminanceImage`：输入的亮度通道图像。
// 函数返回一个二值化图像，其中线条像素的值为 255，非线条像素的值为 0。
Mat CourtLinePixelDetector::filterLinePixels(const Mat& binaryImage, const Mat& luminanceImage)
{
  Mat dx2, dxy, dy2;
  computeStructureTensorElements(luminanceImage, dx2, dxy, dy2);    // 计算结构张量元素
  Mat outputImage(binaryImage.rows, binaryImage.cols, CV_8UC1, Scalar(GlobalParameters().bgValue));
  for (unsigned int x = 0; x < binaryImage.cols; ++x)
  {
    for (unsigned int y = 0; y < binaryImage.rows; ++y)
    {
      uchar value = binaryImage.at<uchar>(y,x);
      if (value == GlobalParameters().fgValue)
      {
        Mat t(2, 2, CV_32F);
        t.at<float>(0, 0) = dx2.at<float>(y,x);   // 获取当前像素的结构张量元素
        t.at<float>(0, 1) = dxy.at<float>(y,x);
        t.at<float>(1, 0) = dxy.at<float>(y,x);
        t.at<float>(1, 1) = dy2.at<float>(y,x);
        Mat l;
        eigen(t, l);    // 计算结构张量的特征值
        if (l.at<float>(0,0) > 4* l.at<float>(0,1))   // 如果第一个特征值大于第二个特征值的四倍，则认为当前像素是线条像素
        {
          outputImage.at<uchar>(y,x) = GlobalParameters().fgValue;  // 将该像素标记为线条像素
        }
      }
    }
  }

  if (debug)
  {
    displayImage(windowName, outputImage);
  }

  return outputImage;
}


void CourtLinePixelDetector::computeStructureTensorElements(const cv::Mat& image, cv::Mat& dx2, cv::Mat& dxy, cv::Mat& dy2)
{
  Mat floatImage, dx, dy;
  image.convertTo(floatImage, CV_32F);
  GaussianBlur(floatImage, floatImage, Size(5,5), 0);   // 对图像进行高斯滤波
  Sobel(floatImage, dx, CV_32F, 1, 0, parameters.gradientKernelSize);   // 计算 x 方向的 Sobel 梯度
  Sobel(floatImage, dy, CV_32F, 0, 1, parameters.gradientKernelSize);   // 计算 y 方向的 Sobel 梯度
  multiply(dx, dx, dx2);  // 计算 dx2
  multiply(dx, dy, dxy);  // 计算 dxy
  multiply(dy, dy, dy2);  // 计算 dy2
  Mat kernel = Mat::ones(parameters.kernelSize, parameters.kernelSize, CV_32F); // 创建一个全为 1 的卷积核
  filter2D(dx2, dx2, -1, kernel); // 对 dx2 进行卷积
  filter2D(dxy, dxy, -1, kernel); // 对 dxy 进行卷积
  filter2D(dy2, dy2, -1, kernel); // 对 dy2 进行卷积
}

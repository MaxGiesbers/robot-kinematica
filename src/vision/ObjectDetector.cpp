#include "vision/ObjectDetector.h"
#include <cmath>

namespace
{
  const double SCREEN_HEIGHT = 480;
  const double SCREEN_WIDTH = 640;
  const double SCREEN_WIDTH_CM = 18;
  const double SCREEN_HEIGHT_CM = 12.144;
}

ObjectDetector::ObjectDetector(cv::Mat frame, cv::Mat filteredFrame) : m_frame(frame), m_filtered_frame(filteredFrame)
{
}

ObjectDetector::~ObjectDetector()
{
}

double ObjectDetector:: convertPixelToCmXPosition(const double pixel_value)
{
  double scale = SCREEN_WIDTH_CM / SCREEN_WIDTH;
  return scale * pixel_value;
}


void ObjectDetector::setText(std::shared_ptr<ColorObject>& color_object)
{
  putText(m_frame, "x-pos: " + std::to_string((int)color_object->getCenterXPos()),
          cv::Point((int)color_object->getCenterXPos(), ((int)color_object->getCenterYPos() - 70)), 1, 1,
          cv::Scalar(255, 255, 255), 1, 1);
  putText(m_frame, "y-pos: " + std::to_string((int)color_object->getCenterYPos()),
          cv::Point((int)color_object->getCenterXPos(), ((int)color_object->getCenterYPos() - 90)), 1, 1,
          cv::Scalar(255, 255, 255), 1, 1);

  putText(m_frame, "Oppervlakte: " + std::to_string((int)color_object->getArea()),
          cv::Point((int)color_object->getCenterXPos(), ((int)color_object->getCenterYPos() - 120)), 1, 1,
          cv::Scalar(255, 255, 255), 1, 1);
  putText(m_frame, color_object->getFigure() + " " + color_object->getColor(),
          cv::Point((int)color_object->getCenterXPos(), ((int)color_object->getCenterYPos() - 140)), 1, 1,
          cv::Scalar(255, 255, 255), 1, 1);
}

void ObjectDetector::setCenterPoint(const std::shared_ptr<ColorObject>& color_object, std::vector<cv::Point>& contour)
{
  auto moments = cv::moments(contour);
  color_object->setCenterXPos(int(moments.m10 / moments.m00));
  color_object->setCenterYPos(int(moments.m01 / moments.m00));
  std::cout << color_object->getCenterXPos() << std::endl;
  std::cout << convertPixelToCmXPosition(color_object->getCenterXPos()) << std::endl;
}

bool ObjectDetector::checkSquareAndRectangle(std::shared_ptr<ColorObject>& color_object, std::vector<cv::Point>& approx)
{
  const double sideUpper = std::fabs(approx[3].x - approx[0].x);
  const double sideDown = std::fabs(approx[2].x - approx[1].x);
  const double sideLeft = std::fabs(approx[0].y - approx[1].y);
  const double sideRight = std::fabs(approx[3].y - approx[2].y);

  // camera father away is a smaller value.
  const short deviationSquare = 10;
  const short deviationRectangle = 20;

  if (sideUpper > deviationSquare && sideDown > deviationSquare && sideLeft > deviationSquare &&
      sideRight > deviationSquare)
  {
    if (std::fabs(sideUpper - sideDown) <= deviationSquare && std::fabs(sideLeft - sideRight) <= deviationSquare)
    {
      if (std::fabs(sideUpper - sideLeft) <= deviationSquare && color_object->getInputFigure().compare("vierkant") == 0)
      {
        color_object->setFigure("vierkant");
        return true;
      }
      else if (std::fabs(sideUpper - sideLeft) > deviationRectangle && color_object->getInputFigure().compare("rechthoe"
                                                                                                              "k") == 0)
      {
        color_object->setFigure("rechthoek");
        return true;
      }
    }
  }
  return false;
}

bool ObjectDetector::checkCircle(std::shared_ptr<ColorObject>& color_object,
                                 std::vector<std::vector<cv::Point>>& contours, int element)
{
  const double area = cv::contourArea(contours.at(element));
  cv::Rect r = cv::boundingRect(contours.at(element));
  const int radius = r.width / 2;
  const double deviationCircle = 0.2;

  if (std::abs(1 - ((double)r.width / r.height)) <= deviationCircle &&
      std::abs(1 - (area / (CV_PI * std::pow(radius, 2)))) <= deviationCircle &&
      color_object->getInputFigure().compare("cirkel") == 0)
  {
    return true;
  }
  return false;
}

bool ObjectDetector::semiCircle(std::shared_ptr<ColorObject>& color_object,
                                std::vector<std::vector<cv::Point>>& contours, int element)
{
  double area = cv::contourArea(contours.at(element));
  cv::Rect r = cv::boundingRect(contours.at(element));
  const int radius = r.width / 2;
  const int radius2 = r.height / 2;
  const double deviationSemiCircleUpAndDown = 0.4;
  const double deviationSemiCircleRightAndLeft = 0.5;

  if (std::abs(1 - ((double)r.width / (r.height * 2))) <= deviationSemiCircleUpAndDown &&
      std::abs(1 - 2 * (area / (CV_PI * std::pow(radius, 2)))) <= deviationSemiCircleUpAndDown &&
      color_object->getInputFigure().compare("halve cirkel") == 0)
  {
    return true;
  }
  else if (std::abs(1 - ((double)r.height / (r.width * 2))) <= deviationSemiCircleRightAndLeft &&
           std::abs(1 - 2 * (area / (CV_PI * std::pow(radius2, 2)))) <= deviationSemiCircleRightAndLeft &&
           color_object->getInputFigure().compare("halve cirkel") == 0)
  {
    return true;
  }
  return false;
}

bool ObjectDetector::findShape(std::shared_ptr<ColorObject>& color_object)
{
  bool found_shape = true;
  // Canny
  cv::Mat bw;
  cv::Canny(color_object->getColorMask(), bw, 0, 50, 5);

  // Find contours
  std::vector<std::vector<cv::Point>> contours;

  // contains the element number of the found contour.
  std::vector<int> contourElements;

  cv::findContours(bw.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

  std::vector<cv::Point> approx;
  cv::Mat dst = color_object->getColorMask().clone();

  for (unsigned int i = 0; i < contours.size(); i++)
  {
    cv::approxPolyDP(cv::Mat(contours[i]), approx, cv::arcLength(cv::Mat(contours[i]), true) * 0.02, true);
    if (std::fabs(cv::contourArea(contours[i])) < 100 || !cv::isContourConvex(approx))
    {
      continue;
    }

    if (approx.size() == 3 && color_object->getInputFigure().compare("driehoek") == 0)
    {
      color_object->setFigure("driehoek");
      contourElements.push_back(i);
    }
    else if (approx.size() == 4 && checkSquareAndRectangle(color_object, approx))
    {
      contourElements.push_back(i);
    }
    else if (approx.size() != 4 && approx.size() != 3)
    {
      if (checkCircle(color_object, contours, (int)i))
      {
        contourElements.push_back(i);
        color_object->setFigure("cirkel");
      }
      else if (semiCircle(color_object, contours, i))
      {
        contourElements.push_back(i);
        color_object->setFigure("halve cirkel");
      }
    }
  }

  // print found shapes
  for (int i : contourElements)
  {
    double area = cv::contourArea(contours.at(i));
    color_object->setArea(area);
    DrawImageContours(contours, color_object, i);
    setCenterPoint(color_object, contours.at(i));
    setText(color_object);
    color_object->printColorObject();
  }

  if (color_object->getInputFigure() != color_object->getFigure())
  {
    std::cout << color_object->getInputFigure() << " " << color_object->getColor() << " is niet gevonden. "
              << std::endl;
    found_shape = false;
  }
  return found_shape;
}

void ObjectDetector::filterColor(std::shared_ptr<ColorObject>& color_object)
{
  cv::Mat frameHSV;
  // Changes contrast of color;
  cv::Mat colorMask = BrightnessAndContrastAuto(m_filtered_frame, 5);
  cvtColor(colorMask, frameHSV, cv::COLOR_BGR2HSV);
  inRange(frameHSV,
          cv::Scalar(color_object->getColorScale().iLowH, color_object->getColorScale().iLowS,
                     color_object->getColorScale().iLowV),
          cv::Scalar(color_object->getColorScale().iHighH, color_object->getColorScale().iHighS,
                     color_object->getColorScale().iHighV),
          colorMask);

  color_object->setColorMask(colorMask);
  imshow("colormask", colorMask);
}

cv::Mat ObjectDetector::BrightnessAndContrastAuto(const cv::Mat& frame, double clip_hist_percent = 0)
{
  CV_Assert(clip_hist_percent >= 0);
  CV_Assert((frame.type() == CV_8UC1) || (frame.type() == CV_8UC3) || (frame.type() == CV_8UC4));

  cv::Mat dst;

  int histSize = 256;
  float alpha, beta;
  double minGray = 0, maxGray = 0;

  // to calculate grayscale histogram
  cv::Mat gray;
  if (frame.type() == CV_8UC1)
    gray = frame;
  else if (frame.type() == CV_8UC3)
    cv::cvtColor(frame, gray, CV_BGR2GRAY);
  else if (frame.type() == CV_8UC4)
    cv::cvtColor(frame, gray, CV_BGRA2GRAY);
  if (clip_hist_percent == 0)
  {
    // keep full available range
    cv::minMaxLoc(gray, &minGray, &maxGray);
  }
  else
  {
    cv::Mat hist;  // the grayscale histogram

    float range[] = { 0, 256 };
    const float* histRange = { range };
    bool uniform = true;
    bool accumulate = false;
    cv::calcHist(&gray, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange, uniform, accumulate);

    // calculate cumulative distribution from the histogram
    std::vector<float> accumulator(histSize);
    accumulator[0] = hist.at<float>(0);
    for (int i = 1; i < histSize; i++)
    {
      accumulator[i] = accumulator[i - 1] + hist.at<float>(i);
    }

    // locate points that cuts at required value
    float max = accumulator.back();
    clip_hist_percent *= (max / 100.0);  // make percent as absolute
    clip_hist_percent /= 2.0;            // left and right wings
    // locate left cut
    minGray = 0;
    while (accumulator[(int)minGray] < clip_hist_percent)
      minGray++;

    // locate right cut
    maxGray = histSize - 1;
    while (accumulator[(int)maxGray] >= (max - clip_hist_percent))
      maxGray--;
  }

  // current range
  float inputRange = (float)maxGray - (float)minGray;

  alpha = (float)(histSize - 1) / inputRange;  // alpha expands current range to histsize range
  beta = (float)-minGray * alpha;              // beta shifts current range so that minGray will go to 0

  // Apply brightness and contrast normalization
  // convertTo operates with saurate_cast
  frame.convertTo(dst, -1, alpha, beta);

  // restore alpha channel from source
  if (dst.type() == CV_8UC4)
  {
    int from_to[] = { 3, 3 };
    cv::mixChannels(&frame, 4, &dst, 1, from_to, 1);
  }
  return dst;
}

void ObjectDetector::DrawImageContours(const std::vector<std::vector<cv::Point>>& contour,
                                       const std::shared_ptr<ColorObject>& colorObject, const int contour_number)
{
  cv::Mat mask = colorObject->getColorMask();
  cv::Mat mask_rgb;
  cv::cvtColor(mask, mask_rgb, CV_GRAY2BGR);
  cv::drawContours(m_frame, contour, contour_number, cv::Scalar(0, 165, 255), 10);
  cv::drawContours(mask_rgb, contour, contour_number, cv::Scalar(0, 165, 255), 10);
}
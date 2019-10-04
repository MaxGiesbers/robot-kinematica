#include "vision/ObjectDetector.hpp"

ObjectDetector::ObjectDetector(cv::Mat aFrame, cv::Mat aFilteredFrame) : frame(aFrame), filteredFrame(aFilteredFrame)
{
}

ObjectDetector::~ObjectDetector()
{
}

void ObjectDetector::setText(std::shared_ptr<ColorObject>& colorObject)
{
  putText(frame, "x-pos: " + std::to_string((int)colorObject->getCenter_x_pos()),
          cv::Point((int)colorObject->getCenter_x_pos(), ((int)colorObject->getCenter_y_pos() - 70)), 1, 1,
          cv::Scalar(255, 255, 255), 1, 1);
  putText(frame, "y-pos: " + std::to_string((int)colorObject->getCenter_y_pos()),
          cv::Point((int)colorObject->getCenter_x_pos(), ((int)colorObject->getCenter_y_pos() - 90)), 1, 1,
          cv::Scalar(255, 255, 255), 1, 1);

  putText(frame, "Oppervlakte: " + std::to_string((int)colorObject->getArea()),
          cv::Point((int)colorObject->getCenter_x_pos(), ((int)colorObject->getCenter_y_pos() - 120)), 1, 1,
          cv::Scalar(255, 255, 255), 1, 1);
  putText(frame, colorObject->getFigure() + " " + colorObject->getColor(),
          cv::Point((int)colorObject->getCenter_x_pos(), ((int)colorObject->getCenter_y_pos() - 140)), 1, 1,
          cv::Scalar(255, 255, 255), 1, 1);
}

void ObjectDetector::setCenterPoint(const std::shared_ptr<ColorObject>& colorObject, std::vector<cv::Point>& contour)
{
  auto moments = cv::moments(contour);
  colorObject->setCenter_x_pos(int(moments.m10 / moments.m00));
  colorObject->setCenter_y_Pos(int(moments.m01 / moments.m00));
}

bool ObjectDetector::checkSquareAndRectangle(std::shared_ptr<ColorObject>& colorObject, std::vector<cv::Point>& approx)
{
  const double sideUpper = std::fabs(approx[3].x - approx[0].x);
  const double sideDown = std::fabs(approx[2].x - approx[1].x);
  const double sideLeft = std::fabs(approx[0].y - approx[1].y);
  const double sideRight = std::fabs(approx[3].y - approx[2].y);

  // camera father away is a smaller value.
  const short deviationSquare = 5;
  const short deviationRectangle = 20;

  if (sideUpper > deviationSquare && sideDown > deviationSquare && sideLeft > deviationSquare &&
      sideRight > deviationSquare)
  {
    if (std::fabs(sideUpper - sideDown) <= deviationSquare && std::fabs(sideLeft - sideRight) <= deviationSquare)
    {
      if (std::fabs(sideUpper - sideLeft) <= deviationSquare && colorObject->getInputFigure().compare("vierkant") == 0)
      {
        colorObject->setFigure("vierkant");
        return true;
      }
      else if (std::fabs(sideUpper - sideLeft) > deviationRectangle && colorObject->getInputFigure().compare("rechthoe"
                                                                                                             "k") == 0)
      {
        colorObject->setFigure("rechthoek");
        return true;
      }
    }
  }
  return false;
}

bool ObjectDetector::checkCircle(std::shared_ptr<ColorObject>& colorObject,
                                 std::vector<std::vector<cv::Point>>& contours, int element)
{
  const double area = cv::contourArea(contours.at(element));
  cv::Rect r = cv::boundingRect(contours.at(element));
  const int radius = r.width / 2;
  const double deviationCircle = 0.2;

  if (std::abs(1 - ((double)r.width / r.height)) <= deviationCircle &&
      std::abs(1 - (area / (CV_PI * std::pow(radius, 2)))) <= deviationCircle &&
      colorObject->getInputFigure().compare("cirkel") == 0)
  {
    return true;
  }
  return false;
}

bool ObjectDetector::semiCircle(std::shared_ptr<ColorObject>& colorObject,
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
      colorObject->getInputFigure().compare("halve cirkel") == 0)
  {
    return true;
  }
  else if (std::abs(1 - ((double)r.height / (r.width * 2))) <= deviationSemiCircleRightAndLeft &&
           std::abs(1 - 2 * (area / (CV_PI * std::pow(radius2, 2)))) <= deviationSemiCircleRightAndLeft &&
           colorObject->getInputFigure().compare("halve cirkel") == 0)
  {
    return true;
  }
  return false;
}

void ObjectDetector::findShape(std::shared_ptr<ColorObject>& colorObject)
{
  // Canny
  cv::Mat bw;
  cv::Canny(colorObject->getColorMask(), bw, 0, 50, 5);

  // Find contours
  std::vector<std::vector<cv::Point>> contours;

  // contains the element number of the found contour.
  std::vector<int> contourElements;

  cv::findContours(bw.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

  std::vector<cv::Point> approx;
  cv::Mat dst = colorObject->getColorMask().clone();

  for (unsigned int i = 0; i < contours.size(); i++)
  {
    cv::approxPolyDP(cv::Mat(contours[i]), approx, cv::arcLength(cv::Mat(contours[i]), true) * 0.02, true);
    if (std::fabs(cv::contourArea(contours[i])) < 100 || !cv::isContourConvex(approx))
    {
      continue;
    }

    if (approx.size() == 3 && colorObject->getInputFigure().compare("driehoek") == 0)
    {
      colorObject->setFigure("driehoek");
      contourElements.push_back(i);
    }
    else if (approx.size() == 4 && checkSquareAndRectangle(colorObject, approx))
    {
      contourElements.push_back(i);
    }
    else if (approx.size() != 4 && approx.size() != 3)
    {
      if (checkCircle(colorObject, contours, (int)i))
      {
        contourElements.push_back(i);
        colorObject->setFigure("cirkel");
      }
      else if (semiCircle(colorObject, contours, i))
      {
        contourElements.push_back(i);
        ;
        colorObject->setFigure("halve cirkel");
      }
    }
  }

  // print found shapes
  for (int i : contourElements)
  {
    double area = cv::contourArea(contours.at(i));
    colorObject->setArea(area);
    DrawImageContours(contours, colorObject, i);
    setCenterPoint(colorObject, contours.at(i));
    setText(colorObject);
    colorObject->printColorObject();
  }

  if (colorObject->getInputFigure() != colorObject->getFigure())
  {
    std::cout << colorObject->getInputFigure() << " " << colorObject->getColor() << " is niet gevonden. " << std::endl;
  }
}

void ObjectDetector::filterColor(std::shared_ptr<ColorObject>& colorObject)
{
  cv::Mat frameHSV;
  // Changes contrast of color;
  cv::Mat colorMask = BrightnessAndContrastAuto(filteredFrame, 5);
  cvtColor(colorMask, frameHSV, cv::COLOR_BGR2HSV);
  inRange(frameHSV,
          cv::Scalar(colorObject->m_colorScale.iLowH, colorObject->m_colorScale.iLowS, colorObject->m_colorScale.iLowV),
          cv::Scalar(colorObject->m_colorScale.iHighH, colorObject->m_colorScale.iHighS, colorObject->m_colorScale.iHighV),
          colorMask);

  colorObject->setColorMask(colorMask);
  imshow("colormask", colorMask);
}

cv::Mat ObjectDetector::BrightnessAndContrastAuto(const cv::Mat& frame, double clipHistPercent = 0)
{
  CV_Assert(clipHistPercent >= 0);
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
  if (clipHistPercent == 0)
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
    clipHistPercent *= (max / 100.0);  // make percent as absolute
    clipHistPercent /= 2.0;            // left and right wings
    // locate left cut
    minGray = 0;
    while (accumulator[(int)minGray] < clipHistPercent)
      minGray++;

    // locate right cut
    maxGray = histSize - 1;
    while (accumulator[(int)maxGray] >= (max - clipHistPercent))
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
                                       const std::shared_ptr<ColorObject>& colorObject, const int contourNumber)
{
  cv::Mat mask = colorObject->getColorMask();
  cv::Mat mask_rgb;
  cv::cvtColor(mask, mask_rgb, CV_GRAY2BGR);
  cv::drawContours(frame, contour, contourNumber, cv::Scalar(0, 165, 255), 10);
  cv::drawContours(mask_rgb, contour, contourNumber, cv::Scalar(0, 165, 255), 10);
}

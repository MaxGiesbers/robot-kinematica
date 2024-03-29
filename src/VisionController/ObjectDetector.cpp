#include "VisionController/ObjectDetector.h"
#include <cmath>

ObjectDetector::ObjectDetector()
{
}

ObjectDetector::~ObjectDetector()
{
}

void ObjectDetector::filterFrame(cv::Mat& filtered_frame)
{
  cv::GaussianBlur(filtered_frame, filtered_frame, cv::Size(3, 3), 0, 0);
  cv::erode(filtered_frame, filtered_frame, cv::Mat(), cv::Point(-1, -1), 2, 1, 1);
  cv::dilate(filtered_frame, filtered_frame, cv::Mat(), cv::Point(-1, -1), 2, 1, 1);
}
void ObjectDetector::setText(std::shared_ptr<ColorObject>& color_object, cv::Mat& drawing_frame)
{
  putText(drawing_frame, "x-pos: " + std::to_string((int)color_object->getCenterXPos()),
          cv::Point((int)color_object->getCenterXPos(), ((int)color_object->getCenterYPos() - 70)), 1, 1,
          cv::Scalar(255, 255, 255), 1, 1);
  putText(drawing_frame, "y-pos: " + std::to_string((int)color_object->getCenterYPos()),
          cv::Point((int)color_object->getCenterXPos(), ((int)color_object->getCenterYPos() - 90)), 1, 1,
          cv::Scalar(255, 255, 255), 1, 1);

  putText(drawing_frame, "Oppervlakte: " + std::to_string((int)color_object->getArea()),
          cv::Point((int)color_object->getCenterXPos(), ((int)color_object->getCenterYPos() - 120)), 1, 1,
          cv::Scalar(255, 255, 255), 1, 1);
  putText(drawing_frame, color_object->getFigure() + " " + color_object->getColor(),
          cv::Point((int)color_object->getCenterXPos(), ((int)color_object->getCenterYPos() - 140)), 1, 1,
          cv::Scalar(255, 255, 255), 1, 1);
}

void ObjectDetector::setCenterPoint(const std::shared_ptr<ColorObject>& color_object, std::vector<cv::Point>& contour)
{
  auto moments = cv::moments(contour);
  int center_x_pos = moments.m10 / moments.m00;
  int center_y_pos = moments.m01 / moments.m00;

  color_object->setCenterXPos(center_x_pos);
  color_object->setCenterYPos(center_y_pos);
}

bool ObjectDetector::checkSquareAndRectangle(std::shared_ptr<ColorObject>& color_object, std::vector<cv::Point>& approx)
{
  const double SIDE_UPPER = std::hypot(approx[3].x - approx[0].x, approx[3].y - approx[0].y);
  const double SIDE_DOWN = std::hypot(approx[2].x - approx[1].x, approx[2].y - approx[1].y);
  const double SIDE_LEFT = std::hypot(approx[0].x - approx[1].x, approx[1].y - approx[0].y);
  const double SIDE_RIGHT = std::hypot(approx[3].x - approx[2].x, approx[2].y - approx[3].y);
  bool found_object = false;

  // camera father away is a smaller value.
  const int8_t DEVIATION_SQUARE = 10;
  const int8_t DEVIATION_RECTANGLE = 30;

  if (SIDE_UPPER > DEVIATION_SQUARE && SIDE_DOWN > DEVIATION_SQUARE && SIDE_LEFT > DEVIATION_SQUARE &&
      SIDE_RIGHT > DEVIATION_SQUARE)
  {
    if (std::fabs(SIDE_UPPER - SIDE_DOWN) <= DEVIATION_SQUARE && std::fabs(SIDE_LEFT - SIDE_RIGHT) <= DEVIATION_SQUARE)
    {
      if (std::fabs(SIDE_UPPER - SIDE_LEFT) <= DEVIATION_SQUARE &&
          color_object->getInputFigure().compare("vierkant") == 0)
      {
        color_object->setFigure("vierkant");
        color_object->m_approx = approx;
        found_object = true;
      }
      else if (std::fabs(SIDE_UPPER - SIDE_LEFT) > DEVIATION_RECTANGLE &&
               color_object->getInputFigure().compare("rechthoek") == 0)
      {
        color_object->setFigure("rechthoek");
        color_object->m_approx = approx;
        found_object = true;
      }
    }
  }

  return found_object;
}

bool ObjectDetector::checkCircle(std::shared_ptr<ColorObject>& color_object,
                                 std::vector<std::vector<cv::Point>>& contours, int element)
{
  const double AREA = cv::contourArea(contours.at(element));
  const cv::Rect RECTANGLE = cv::boundingRect(contours.at(element));
  const int RADIUS = RECTANGLE.width / 2;
  const double DEVIATION_CIRCLE = 0.1;
  bool found_circle = false;

  if (std::abs(1 - ((double)RECTANGLE.width / RECTANGLE.height)) <= DEVIATION_CIRCLE &&
      std::abs(1 - (AREA / (CV_PI * std::pow(RADIUS, 2)))) <= DEVIATION_CIRCLE &&
      color_object->getInputFigure().compare("cirkel") == 0 && RADIUS >= 15)
  {
    found_circle = true;
  }
  return found_circle;
}

bool ObjectDetector::semiCircle(std::shared_ptr<ColorObject>& color_object,
                                std::vector<std::vector<cv::Point>>& contours, int element)
{
  const double AREA = cv::contourArea(contours.at(element));
  const cv::Rect RECTANGLE = cv::boundingRect(contours.at(element));
  const int RADIUS_WIDTH = RECTANGLE.width / 2;
  const int RADIUS_HEIGHT = RECTANGLE.height / 2;
  const double DEVIATION_SEMI_CIRCLE_UP_AND_DOWN = 0.4;
  const double DEVIATION_SEMI_CIRCLE_RIGHT_AND_LEFT = 0.5;
  bool found_semi_circle = false;

  if (std::abs(1 - ((double)RECTANGLE.width / (RECTANGLE.height * 2))) <= DEVIATION_SEMI_CIRCLE_UP_AND_DOWN &&
      std::abs(1 - 2 * (AREA / (CV_PI * std::pow(RADIUS_WIDTH, 2)))) <= DEVIATION_SEMI_CIRCLE_UP_AND_DOWN &&
      color_object->getInputFigure().compare("halve cirkel") == 0)
  {
    found_semi_circle = true;
  }
  else if (std::abs(1 - ((double)RECTANGLE.height / (RECTANGLE.width * 2))) <= DEVIATION_SEMI_CIRCLE_RIGHT_AND_LEFT &&
           std::abs(1 - 2 * (AREA / (CV_PI * std::pow(RADIUS_HEIGHT, 2)))) <= DEVIATION_SEMI_CIRCLE_RIGHT_AND_LEFT &&
           color_object->getInputFigure().compare("halve cirkel") == 0)
  {
    found_semi_circle = true;
  }
  return found_semi_circle;
}

void ObjectDetector::findShape(std::shared_ptr<ColorObject>& color_object, cv::Mat& drawing_frame)
{
  // Canny
  cv::Mat bw;
  cv::Canny(color_object->getColorMask(), bw, 0, 50, 5);

  // Find contours
  std::vector<std::vector<cv::Point>> contours;

  // contains the element number of the found contour.
  std::vector<int> contour_elements;

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
      contour_elements.push_back(i);
    }
    else if (approx.size() == 4 && checkSquareAndRectangle(color_object, approx))
    {
      contour_elements.push_back(i);
    }
    else if (approx.size() != 4 && approx.size() != 3)
    {
      if (checkCircle(color_object, contours, (int)i))
      {
        contour_elements.push_back(i);
        color_object->setFigure("cirkel");
      }
      else if (semiCircle(color_object, contours, i))
      {
        contour_elements.push_back(i);
        color_object->setFigure("halve cirkel");
      }
    }
  }

  // print found shapes
  for (int i : contour_elements)
  {
    double area = cv::contourArea(contours.at(i));
    color_object->setArea(area);
    DrawImageContours(contours, color_object, i, drawing_frame);
    setCenterPoint(color_object, contours.at(i));
    setText(color_object, drawing_frame);
    color_object->printColorObject();
  }

  if (color_object->getInputFigure() != color_object->getFigure())
  {
    ROS_INFO_STREAM(color_object->getInputFigure() << " " << color_object->getColor() << " is niet gevonden. ");
    color_object->setObjectDetected(false);
  }
  else
  {
    color_object->setObjectDetected(true);
  }
}

void ObjectDetector::filterColor(const std::shared_ptr<ColorObject>& color_object, cv::Mat& filtered_frame,
                                 cv::Mat& color_mask)
{
  cv::Mat frame_hsv;
  // Changes contrast of color;
  color_mask = BrightnessAndContrastAuto(filtered_frame, 5);
  cvtColor(color_mask, frame_hsv, cv::COLOR_BGR2HSV);
  inRange(frame_hsv,
          cv::Scalar(color_object->getColorScale().iLowH, color_object->getColorScale().iLowS,
                     color_object->getColorScale().iLowV),
          cv::Scalar(color_object->getColorScale().iHighH, color_object->getColorScale().iHighS,
                     color_object->getColorScale().iHighV),
          color_mask);

  color_object->setColorMask(color_mask);
}

cv::Mat ObjectDetector::BrightnessAndContrastAuto(const cv::Mat& frame, double clip_hist_percent = 0)
{
  CV_Assert(clip_hist_percent >= 0);
  CV_Assert((frame.type() == CV_8UC1) || (frame.type() == CV_8UC3) || (frame.type() == CV_8UC4));

  cv::Mat dst;

  int histSize = 256;
  float alpha, beta = 0;
  double min_gray = 0, max_gray = 0;

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
    cv::minMaxLoc(gray, &min_gray, &max_gray);
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
    min_gray = 0;
    while (accumulator[(int)min_gray] < clip_hist_percent)
      min_gray++;

    // locate right cut
    max_gray = histSize - 1;
    while (accumulator[(int)max_gray] >= (max - clip_hist_percent))
      max_gray--;
  }

  // current range
  float inputRange = (float)max_gray - (float)min_gray;

  alpha = (float)(histSize - 1) / inputRange;  // alpha expands current range to histsize range
  beta = (float)-min_gray * alpha;             // beta shifts current range so that min_gray will go to 0

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
                                       const std::shared_ptr<ColorObject>& colorObject, const int contour_number,
                                       cv::Mat& drawing_frame)
{
  cv::Mat mask = colorObject->getColorMask();
  cv::Mat mask_rgb;
  cv::cvtColor(mask, mask_rgb, CV_GRAY2BGR);
  cv::drawContours(drawing_frame, contour, contour_number, cv::Scalar(0, 165, 255), 10);
  cv::drawContours(mask_rgb, contour, contour_number, cv::Scalar(0, 165, 255), 10);
}

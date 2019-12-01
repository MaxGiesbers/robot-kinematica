#include "VisionController/Calibration.h"
#include <array>

namespace
{
const int WINDOW_SIZE = 600;
const int SPACEBAR_KEY_ASCII = 32;
const int ENTER_KEY_ASCII = 10;
}  // namespace

Calibration::Calibration()
  : m_iLowH(0), m_iHighH(0), m_iLowS(0), m_iHighS(0), m_iLowV(0), m_iHighV(0), m_color_scale_iterator(0)
{
  setDefaultColorScales();
}

Calibration::~Calibration()
{
}

void Calibration::setCalibratedColorValues()
{
  m_color_scales.at(m_color_scale_iterator).iLowH = m_iLowH;
  m_color_scales.at(m_color_scale_iterator).iHighH = m_iHighH;
  m_color_scales.at(m_color_scale_iterator).iLowS = m_iLowS;
  m_color_scales.at(m_color_scale_iterator).iHighS = m_iHighS;
  m_color_scales.at(m_color_scale_iterator).iLowV = m_iLowV;
  m_color_scales.at(m_color_scale_iterator).iHighV = m_iHighV;

  std::cout << "New color values: " << m_iLowH << ", " << m_iHighH << ", " << m_iLowS << ", " << m_iHighS << ", "
            << m_iLowV << ", " << m_iHighV << " for color " << m_color_scales.at(m_color_scale_iterator).color
            << std::endl;
}

void Calibration::setColorValues()
{
  ColorScale color_scale = m_color_scales.at(m_color_scale_iterator);

  m_iLowH = color_scale.iLowH;
  m_iHighH = color_scale.iHighH;
  m_iLowS = color_scale.iLowS;
  m_iHighS = color_scale.iHighS;
  m_iLowV = color_scale.iLowV;
  m_iHighV = color_scale.iHighV;

  cv::setTrackbarPos("iLowH", "trackBarWindow", m_iLowH);
  cv::setTrackbarPos("iHighH", "trackBarWindow", m_iHighH);
  cv::setTrackbarPos("iLowS", "trackBarWindow", m_iLowS);
  cv::setTrackbarPos("iHighS", "trackBarWindow", m_iHighS);
  cv::setTrackbarPos("iLowV", "trackBarWindow", m_iLowV);
  cv::setTrackbarPos("iHighV", "trackBarWindow", m_iHighV);
}

cv::Mat Calibration::BrightnessAndContrastAuto(const cv::Mat& frame, double clip_hist_percent = 0)
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

ColorScale Calibration::getColorScale(std::string color) const
{
  ColorScale colorScale;
  if (color.compare("groen") == 0)
  {
    colorScale = m_color_scales.at(0);
  }
  else if (color.compare("rood") == 0)
  {
    colorScale = m_color_scales.at(1);
  }
  else if (color.compare("blauw") == 0)
  {
    colorScale = m_color_scales.at(2);
  }
  else if (color.compare("geel") == 0)

  {
    colorScale = m_color_scales.at(3);
  }
  else if (color.compare("zwart") == 0)
  {
    colorScale = m_color_scales.at(4);
  }

  else if (color.compare("wit") == 0)
  {
    colorScale = m_color_scales.at(5);
  }
  return colorScale;
}

void Calibration::updateSetRange()
{
  cv::Mat imgHSV;
  cv::Mat mask1 = BrightnessAndContrastAuto(m_treshold, 5);
  cv::cvtColor(mask1, imgHSV, cv::COLOR_BGR2HSV);  // Convert the captured frame from BGR to HSV
  cv::inRange(imgHSV, cv::Scalar(m_iLowH, m_iLowS, m_iLowV), cv::Scalar(m_iHighH, m_iHighS, m_iHighV), mask1);
  cv::namedWindow("filterWindow3", 0);
  cv::resizeWindow("filterWindow3", WINDOW_SIZE, WINDOW_SIZE);
  cv::imshow("filterWindow3", mask1);
}

void Calibration::setDefaultColorScales()
{
  m_color_scales.push_back(ColorScale{ 53, 95, 41, 193, 40, 255, "groen" });
  m_color_scales.push_back(ColorScale{ 139, 179, 62, 255, 156, 255, "rood" });
  m_color_scales.push_back(ColorScale{ 76, 176, 139, 255, 69, 255, "blauw" });
  m_color_scales.push_back(ColorScale{ 14, 40, 41, 255, 229, 255, "geel" });
  m_color_scales.push_back(ColorScale{ 0, 179, 9, 100, 0, 51, "zwart" });
  m_color_scales.push_back(ColorScale{ 0, 174, 0, 255, 152, 255, "wit" });
}

void Calibration::startCalibration(const cv::VideoCapture& cap)
{
  cv::namedWindow("trackBarWindow", WINDOW_SIZE);
  cv::createTrackbar("iLowH", "trackBarWindow", &m_iLowH, 180, calibrate, this);
  cv::createTrackbar("iHighH", "trackBarWindow", &m_iHighH, 180, calibrate, this);
  cv::createTrackbar("iLowS", "trackBarWindow", &m_iLowS, 255, calibrate, this);
  cv::createTrackbar("iHighS", "trackBarWindow", &m_iHighS, 255, calibrate, this);
  cv::createTrackbar("iLowV", "trackBarWindow", &m_iLowV, 255, calibrate, this);
  cv::createTrackbar("iHighV", "trackBarWindow", &m_iHighV, 255, calibrate, this);

  m_cap = cap;
  ColorScale colorScale = m_color_scales.at(m_color_scale_iterator);
  ROS_INFO_STREAM("press space bar to capture and calibrate on color: " << colorScale.color);

  while (m_color_scale_iterator < m_color_scales.size())
  {
    m_cap >> m_capture_window;
    imshow("captureWindow", m_capture_window);
    cv::moveWindow("captureWindow", 1500, 20);
    cv::imshow("trackBarWindow", 0);
    cv::moveWindow("trackBarWindow", 800, 20);

    cv::waitKey(30);

    m_cap >> m_treshold;
    setColorValues();

    if (cv::waitKey() == ENTER_KEY_ASCII)
    {
      setCalibratedColorValues();

      ++m_color_scale_iterator;
      if (m_color_scale_iterator < m_color_scales.size())
      {
        colorScale = m_color_scales.at(m_color_scale_iterator);
      }
    }
  }
  cv::destroyAllWindows();
}

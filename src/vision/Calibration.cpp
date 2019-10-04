#include "vision/Calibration.hpp"
#include <array>

namespace
{
const int WINDOW_SIZE = 600;
const int CAMERA_ID = 1;
const int SPACEBAR_KEY_ASCII = 32;
const int ENTER_KEY_ASCII = 10;
}  // namespace

Calibration::Calibration() : m_iLowH(0), m_iLowS(0), m_iLowV(0), m_iHighH(0), m_iHighS(0), m_iHighV(0)
{
}

Calibration::~Calibration()
{
}

cv::Mat Calibration::BrightnessAndContrastAuto(const cv::Mat& frame, double clipHistPercent = 0)
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

ColorScale Calibration::getColorScale(std::string color) const
{
  ColorScale colorScale;
  if (color.compare("groen") == 0)
  {
    colorScale = m_colorScales.at(0);
  }
  else if (color.compare("rood") == 0)
  {
    colorScale = m_colorScales.at(1);
  }
  else if (color.compare("blauw") == 0)
  {
    colorScale = m_colorScales.at(2);
  }
  else if (color.compare("geel") == 0)

  {
    colorScale = m_colorScales.at(3);
  }
  else if (color.compare("zwart") == 0)
  {
    colorScale = m_colorScales.at(4);
  }

  else if (color.compare("wit") == 0)
  {
    colorScale = m_colorScales.at(5);
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

void Calibration::startCalibration()
{
  bool capturedImage = false;
  std::array<std::string, 6> colors{ "groen", "rood", "blauw", "geel", "zwart", "wit" };
  int iterator = 0;
  m_cap.open(CAMERA_ID);

  std::cout << "press space bar to capture and calibrate on color: " << colors[iterator] << std::endl;

  while (m_colorScales.size() != 5)
  {
    m_cap >> m_captureWindow;
    imshow("captureWindow", m_captureWindow);
    cv::waitKey(30);

    if (cv::waitKey() == SPACEBAR_KEY_ASCII)
    {
      m_cap >> m_treshold;
      capturedImage = true;
    }
    else if (cv::waitKey() == ENTER_KEY_ASCII)
    {
      ++iterator;
      ColorScale colorScale{ m_iLowH, m_iHighH, m_iLowS, m_iHighS, m_iLowV, m_iHighV };
      m_colorScales.push_back(colorScale);
      std::cout << "The values: " << m_iLowH << ", " << m_iHighH << ", " << m_iLowS << ", " << m_iHighS << ", "
                << m_iLowV << ", " << m_iHighV << " for color " << colors[iterator] << std::endl;
      std::cout << "Calibrate on color: " << colors[iterator] << std::endl;
    }

    if (capturedImage)
    {
      cv::namedWindow("trackBarWindow", WINDOW_SIZE);
      cv::createTrackbar("iLowH", "trackBarWindow", &m_iLowH, 180, calibrate, this);
      cv::createTrackbar("iHighH", "trackBarWindow", &m_iHighH, 180, calibrate, this);
      cv::createTrackbar("iLowS", "trackBarWindow", &m_iLowS, 255, calibrate, this);
      cv::createTrackbar("iHighS", "trackBarWindow", &m_iHighS, 255, calibrate, this);
      cv::createTrackbar("iLowV", "trackBarWindow", &m_iLowV, 255, calibrate, this);
      cv::createTrackbar("iHighV", "trackBarWindow", &m_iHighV, 255, calibrate, this);
      cv::imshow("trackBarWindow", 0);
    }
  }
  cv::destroyAllWindows();
  m_cap.release();
  cv::waitKey(0);
}

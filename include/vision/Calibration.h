#include <ros/ros.h>
#include "vision/ColorScale.h"
#include "opencv2/core.hpp"
#include <opencv2/opencv.hpp>

class Calibration
{
public:
  Calibration();
  ~Calibration();
  void startCalibration(const cv::VideoCapture& cap);
  ColorScale getColorScale(std::string color) const;

private:
  cv::Mat BrightnessAndContrastAuto(const cv::Mat& frame, double clip_hist_percent);
  void updateSetRange();
  void setDefaultColorScales();
  void setColorValues();
  void setCalibratedColorValues();

  static void calibrate(int v, void* ptr)
  {
    v += 1;  // To prevent wExtra complains

    Calibration* calibration = static_cast<Calibration*>(ptr);
    calibration->updateSetRange();
  }

  int m_iLowH, m_iHighH, m_iLowS, m_iHighS, m_iLowV, m_iHighV;
  std::string sliderWindow = "CALIBRATE";

  std::size_t m_iterator;
  cv::Mat m_treshold;
  cv::Mat m_src;
  std::vector<ColorScale> m_color_scales;
  cv::VideoCapture m_cap;
  cv::Mat m_capture_window;
};
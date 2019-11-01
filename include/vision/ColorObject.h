#include "ColorScale.h"
#include <ros/ros.h>
#include <vector>
#include "opencv2/core.hpp"
#include <opencv2/opencv.hpp>

class ColorObject
{
public:
  explicit ColorObject(const std::string& input_figure, const std::string& color);
  ~ColorObject();
  void setColorMask(const cv::Mat& color_mask);
  void printColorObject();
  void setArea(const double aArea);

  void setCenterXPos(const double center_x_pos);
  void setCenterYPos(const double center_y_pos);

  void setXDimension(const double x_dimension);
  void setYDimension(const double y_dimension);

  void setObjectDetected(bool object_detected);

  void setColorScale(const ColorScale& color_scale);
  void setFigure(const std::string& figure);
  std::string getFigure() const;

  double getCenterXPos() const;
  double getCenterYPos() const;

  double getXDimension() const;
  double getYDimension() const;

  double getArea() const;
  bool getObjectDetected() const;

  const std::string getShapeName() const;
  const std::string getInputFigure() const;
  const std::string getColor() const;
  const cv::Mat getColorMask() const;
  const ColorScale& getColorScale() const;

private:
  ColorScale m_color_scale;

  std::string m_input_figure;
  std::string m_color;
  cv::Mat m_color_mask;
  std::string m_figure;
  double m_center_x_pos;
  double m_center_y_pos;
  double m_area;

  double m_x_dimension;
  double m_y_dimension;
  double m_z_dimension;

  bool m_object_detected;
};
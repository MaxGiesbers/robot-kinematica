#include "vision/ColorObject.h"

ColorObject::~ColorObject()
{
}

ColorObject::ColorObject(const std::string& input_figure, const std::string& color)
  : m_color_scale()
  , m_input_figure(input_figure)
  , m_color(color)
  , m_center_x_pos(0)
  , m_center_y_pos(0)
  , m_area(0)
  , m_object_detected(false)
{
}

const std::string ColorObject::getColor() const
{
  return m_color;
}
bool ColorObject::getObjectDetected() const
{
  return m_object_detected;
}

const ColorScale& ColorObject::getColorScale() const
{
  return m_color_scale;
}

const std::string ColorObject::getInputFigure() const
{
  return m_input_figure;
}

void ColorObject::setColorMask(const cv::Mat& color_mask)
{
  m_color_mask = color_mask;
}

const cv::Mat ColorObject::getColorMask() const
{
  return m_color_mask;
}

std::string ColorObject::getFigure() const
{
  return m_figure;
}

void ColorObject::printColorObject()
{
  ROS_INFO_STREAM(m_figure << " " << m_color << " "
                           << "oppervlakte: " << m_area << " middelpunt x: " << m_center_x_pos
                           << " middelpunt y: " << m_center_y_pos << " is gevonden.");
}

void ColorObject::setFigure(const std::string& figure)
{
  m_figure = figure;
}

void ColorObject::setColorScale(const ColorScale& color_scale)
{
  m_color_scale = color_scale;
}

double ColorObject::getCenterXPos() const
{
  return m_center_x_pos;
}

double ColorObject::getCenterYPos() const
{
  return m_center_y_pos;
}

double ColorObject::getArea() const
{
  return m_area;
}

void ColorObject::setObjectDetected(bool object_detected)
{
  m_object_detected = object_detected;
}
void ColorObject::setArea(const double area)
{
  m_area = area;
}

void ColorObject::setCenterXPos(const double center_x_pos)
{
  m_center_x_pos = center_x_pos;
}

void ColorObject::setCenterYPos(const double center_y_pos)
{
  m_center_y_pos = center_y_pos;
}
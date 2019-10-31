#include "vision/ColorObject.h"

ColorObject::~ColorObject()
{
}

ColorObject::ColorObject(const std::string& input_figure, const std::string& color)
  : m_color_scale(), m_input_figure(input_figure), m_color(color), m_center_x_pos(0), 
      m_center_y_pos(0), m_area(0), m_x_dimension(0), m_y_dimension(0), m_z_dimension(0),
        m_x_origin(0), m_y_origin(0), m_z_origin(0), m_x_destination(0),
         m_y_destination(0), m_z_destination(0), m_object_detected(false)
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
  std::cout << m_figure << " " << m_color << " "
            << "oppervlakte: " << m_area << " middelpunt x: " << m_center_x_pos << " middelpunt y: " << m_center_y_pos
            << " is gevonden." << std::endl;
}

void ColorObject::setFigure(const std::string& figure)
{
  m_figure = figure;
}

void ColorObject::setColorScale(ColorScale color_scale)
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

double ColorObject::getXDimension() const
{
  return m_x_dimension;
}

double ColorObject::getYDimension() const
{
  return m_y_dimension;
}

double ColorObject::getZDimension() const
{
  return m_z_dimension;
}

double ColorObject::getXOrigin() const
{
  return m_x_origin;
}

double ColorObject::getYOrigin() const
{
  return m_y_origin;
}

double ColorObject::getZOrigin() const
{
 return m_z_origin;
}

double ColorObject::getXDestination() const
{
    return m_x_destination;
}
double ColorObject::getYDestination() const
{
    return m_y_destination;
}
double ColorObject::getZDestination() const
{
    return m_z_destination;
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


void ColorObject::setXDimension(const double x_dimension)
{
  m_x_dimension = x_dimension;
}

void ColorObject::setYDimension(const double y_dimension)
{
  m_y_dimension = y_dimension;
}

void ColorObject::setXOrigin(const double x_origin)
{
  m_x_origin = x_origin;

}

void ColorObject::setXDestination(const double x_destination)
{
    m_x_destination = x_destination;
}

void ColorObject::setYDestination(const double y_destination)
{   
    m_y_destination = y_destination;
}
void ColorObject::setZDestination(const double z_destination)
{
    m_z_destination = z_destination;
}

void ColorObject::setYOrigin(const double y_origin)
{
  m_y_origin = y_origin;
}

void ColorObject::setZOrigin(const double z_origin)
{
  m_z_origin = z_origin;
}

void ColorObject::setCenterXPos(const double center_x_pos)
{
  m_center_x_pos = center_x_pos;
}

void ColorObject::setCenterYPos(const double center_y_pos)
{
  m_center_y_pos = center_y_pos;
}
#include "vision/ColorObject.hpp"

ColorObject::~ColorObject()
{
}

ColorObject::ColorObject(const std::string& aInputFigure, const std::string& aColor)
  : m_colorScale(), inputFigure(aInputFigure), color(aColor), center_x_pos(0), center_y_pos(0), area(0)
{
}

const std::string ColorObject::getColor() const
{
  return color;
}

const std::string ColorObject::getInputFigure() const
{
  return inputFigure;
}

void ColorObject::setColorMask(const cv::Mat& aColorMask)
{
  colorMask = aColorMask;
}

const cv::Mat ColorObject::getColorMask() const
{
  return colorMask;
}

std::string ColorObject::getFigure() const
{
  return figure;
}

void ColorObject::printColorObject()
{
  std::cout << figure << " " << color << " "
            << "oppervlakte: " << area << " middelpunt x: " << center_x_pos << " middelpunt y: " << center_y_pos
            << " is gevonden." << std::endl;
}

void ColorObject::setFigure(const std::string& aFigure)
{
  figure = aFigure;
}

void ColorObject::setColorScale(ColorScale colorScale)
{
    m_colorScale = colorScale; 
    std::cout << m_colorScale.iHighH << std::endl;

}

double ColorObject::getCenter_x_pos() const
{
  return center_x_pos;
}

double ColorObject::getCenter_y_pos() const
{
  return center_y_pos;
}

double ColorObject::getArea() const
{
  return area;
}

void ColorObject::setArea(const double aArea)
{
  area = aArea;
}

void ColorObject::setCenter_x_pos(const double aCenter_x_pos)
{
  center_x_pos = aCenter_x_pos;
}

void ColorObject::setCenter_y_Pos(const double aCenter_y_pos)
{
  center_y_pos = aCenter_y_pos;
}
#ifndef COLOROBJECT_H
#define COLOROBJECT_H

#include "ColorScale.h"
#include <ros/ros.h>
#include <vector>
#include "opencv2/core.hpp"
#include <opencv2/opencv.hpp>

class ColorObject
{
public:
  /**
   * @brief Construct a new Color Object object
   *
   * @param input_figure contains the name of the figure
   * @param color contains the name of the color
   */
  explicit ColorObject(const std::string& input_figure, const std::string& color);

  /**
   * @brief Destroy the Color Object object
   *
   */
  ~ColorObject();

  /**
   * @brief Set the Color Mask object
   *
   * @param color_mask which contains the color mask
   */
  void setColorMask(const cv::Mat& color_mask);

  /**
   * @brief Function to print the found object, includes x, y and center
   *
   */
  void printColorObject();

  /**
   * @brief Set the Area
   *
   * @param aArea contains the area in pixels
   */
  void setArea(const double aArea);

  /**
   * @brief Set the Center X Pos
   *
   * @param center_x_pos contains the x center location in pixels
   */
  void setCenterXPos(const double center_x_pos);

  /**
   * @brief Set the Center Y Pos
   *
   * @param center_y_pos contains the y center location center in pixels
   */
  void setCenterYPos(const double center_y_pos);

  /**
   * @brief Boolean to check whether an object is detected
   *
   * @param object_detected
   */
  void setObjectDetected(bool object_detected);

  /**
   * @brief Set the Color Scale
   *
   * @param color_scale an struct with HSV values
   */
  void setColorScale(const ColorScale& color_scale);

  /**
   * @brief Set the shape of the object
   *
   * @param figure name as string variable
   */
  void setFigure(const std::string& figure);

  /**
   * @brief Get the shape
   *
   * @return std::string figure
   */
  std::string getFigure() const;

  /**
   * @brief Get the Center X Pos
   *
   * @return double
   */
  double getCenterXPos() const;

  /**
   * @brief Get the Center Y Pos
   *
   * @return double
   */
  double getCenterYPos() const;

  /**
   * @brief Get the Area
   *
   * @return double
   */
  double getArea() const;

  /**
   * @brief Function to check if the requested object was found
   *
   * @return true
   * @return false
   */
  bool getObjectDetected() const;

  /**
   * @brief Get the Shape Name
   *
   * @return const std::string
   */
  const std::string getShapeName() const;

  /**
   * @brief Get the Input Shape
   *
   * @return const std::string
   */
  const std::string getInputFigure() const;

  /**
   * @brief Get the Color
   *
   * @return const std::string
   */
  const std::string getColor() const;

  /**
   * @brief Get the Color Mask
   *
   * @return const cv::Mat
   */
  const cv::Mat getColorMask() const;

  /**
   * @brief Get the Color Scale
   *
   * @return const ColorScale&
   */
  const ColorScale& getColorScale() const;

  /**
   * @brief Vector that contains contour data about the detected shape
   *
   */
  std::vector<cv::Point> m_approx;

private:
  /**
   * @brief struct that contains HSV value for the color
   *
   */
  ColorScale m_color_scale;

  /**
   * @brief The shape that was requested
   *
   */
  std::string m_input_figure;

  /**
   * @brief The color that was requested
   *
   */
  std::string m_color;

  /**
   * @brief the colorMask in HSV
   *
   */
  cv::Mat m_color_mask;

  /**
   * @brief The shape that was detected
   *
   */
  std::string m_figure;

  /**
   * @brief The x coordinate in pixles of the center position
   *
   */
  double m_center_x_pos;

  /**
   * @brief The y coordinate in pixels of the center position
   *
   */
  double m_center_y_pos;

  /**
   * @brief The area in pixels of the detected object
   *
   */
  double m_area;

  /**
   * @brief Boolean to check whether the requested object was detected
   *
   */
  bool m_object_detected;
};

#endif  // COLOROBJECT_H
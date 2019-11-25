#ifndef VISIONCONTROLLER_H
#define VISIONCONTROLLER_H

#include <ros/ros.h>
#include "ObjectDetector.h"
#include <thread>
#include "vision/Calibration.h"
#include "robot_kinematica/found_object.h"

class VisionController
{
public:
  /**
   * @brief Construct a new Vision Controller object
   * 
   */
  VisionController();

  /**
   * @brief Destroy the Vision Controller object
   * 
   */
  ~VisionController();

  
  /**
   * @brief 
   * 
   * @param message 
   */
  void splitString(std::string message);
  
  /**
   * @brief 
   * 
   */
  void splitAndStoreLinesBasedOnRegex();
  
  /**
   * @brief Function to check the values of  
   * 
   * @param figure 
   * @param color 
   */
  void checkStringValues(const std::string& figure, const std::string& color);
  
  /**
   * @brief 
   * 
   */
  void applicationLoop();
  
  /**
   * @brief Set the Filtered Frame
   * 
   * @param filtered_frame 
   */
  void setFilteredFrame(const cv::Mat& filtered_frame);
  
  /**
   * @brief Read the user input from the command line
   * 
   */
  void readCommandLineInput();
  
  /**
   * @brief Function for finding a specified shape with a specified color
   * 
   * @param input_color The requested color
   * @param input_figure The requested shape
   */
  void findColorAndShape(const std::string& input_color, const std::string& input_figure);
  
  /**
   * @brief 
   * 
   */
  void sendObjectCoordinates();
  
  /**
   * @brief 
   * 
   */
  void cloneFrames();
  
  /**
   * @brief 
   * 
   * @param color_object 
   */
  void findObjectLoop(std::shared_ptr<ColorObject>& color_object);
  
  /**
   * @brief Set the Approx Values
   * 
   * @param found_object_message 
   */
  void setApproxValues(robot_kinematica::found_object& found_object_message);

  
  /**
   * @brief Get the Object Corrected X Position (corrects for camera distortion)
   * 
   * @param object_center_x The center of the object as detected by the camera
   * @return double 
   */
  double getObjectCorrectedXPosition(double object_center_x);

  
  /**
   * @brief 
   * 
   * @return std::thread 
   */
  std::thread readInputThread();
  
  /**
   * @brief 
   * 
   * @return std::thread 
   */
  std::thread videoCamThread();
  
  /**
   * @brief 
   * 
   */
  void readVideoCam();
  
  /**
   * @brief Function for converting pixels to centimeters (x-coordinate)
   * 
   * @param pixel_value 
   * @return double 
   */
  double convertPixelToCmXPosition(const double pixel_value);
  
  /**
   * @brief Function for converting pixels to centimeters (y-coordinate)
   * 
   * @param pixel_value 
   * @return double 
   */
  double convertPixelToCmYPosition(const double pixel_value);
  
  /**
   * @brief Get the Angle Difference
   * 
   * @return double 
   */
  double getAngleDifference();
  
  /**
   * @brief Set the Object Polar Coordinates
   * 
   * @param object_polar_x 
   * @param object_polar_y 
   * @param color_object 
   */
  void setObjectPolarCoordinates(double& object_polar_x, double& object_polar_y, std::shared_ptr<ColorObject> color_object);

private:
  /**
   * @brief 
   * 
   */
  cv::VideoCapture m_cap;
  
  /**
   * @brief 
   * 
   */
  cv::Mat m_frame;
  
  /**
   * @brief 
   * 
   */
  cv::Mat m_filtered_frame;
  
  /**
   * @brief 
   * 
   */
  cv::Mat m_drawing_frame;
  
  /**
   * @brief 
   * 
   */
  cv::Mat m_color_mask;

  
  /**
   * @brief 
   * 
   */
  ros::Publisher m_publisher;
  
  /**
   * @brief 
   * 
   */
  ros::NodeHandle m_node_handle;

  
  /**
   * @brief 
   * 
   */
  ros::ServiceClient m_client;

  
  /**
   * @brief 
   * 
   */
  std::shared_ptr<ColorObject> m_color_object;
  
  /**
   * @brief 
   * 
   */
  std::shared_ptr<ColorObject> m_destination_object;
  
  /**
   * @brief Starts the calibration tool 
   * 
   */
  Calibration m_calibrator;
  
  /**
   * @brief 
   * 
   */
  std::atomic<bool> m_user_input_correct;

  
  /**
   * @brief Get the Color Scale
   * 
   * @param color 
   * @return ColorScale 
   */
  ColorScale getColorScale(std::string& color);

  
  /**
   * @brief Starts the object detector
   * 
   */
  ObjectDetector m_object_detector;
  
  /**
   * @brief 
   * 
   */
  bool m_coordinates_sended;
};

#endif 
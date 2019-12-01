#ifndef VISIONCONTROLLER_H
#define VISIONCONTROLLER_H

#include "ObjectDetector.h"
#include <thread>
#include "Calibration.h"
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
   * @brief splits user input based on regex
   * 
   * @param message 
   */
  void splitString(std::string message);
 
  /**
   * @brief Function to check the values of  
   * 
   * @param figure 
   * @param color 
   */
  void checkStringValues(const std::string& figure, const std::string& color);
  
  /**
   * @brief the main loop of the application
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
   * @brief sends the object values to the robotcontroller
   * 
   */
  void sendObjectCoordinates();
  
  /**
   * @brief clones the live frame to the other mat objects
   * 
   */
  void cloneFrames();
  
  /**
   * @brief loop for find the object
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
   * @brief for reading the user input on a other thread than the main thread
   * 
   * @return std::thread 
   */
  std::thread readInputThread();
  
  /**
   * @brief thread for reading the camera on a other thread than the main thread
   * 
   * @return std::thread 
   */
  std::thread videoCamThread();
  
  /**
   * @brief reads the video camera
   * 
   */
  void readVideoCam();
  
  /**
   * @brief Function for converting pixels to centimeters (x-coordinate)
   * 
   * @param pixel_value 
   * @return double x position in cm
   */
  double convertPixelToCmXPosition(const double pixel_value);
  
  /**
   * @brief Function for converting pixels to centimeters (y-coordinate)
   * 
   * @param pixel_value 
   * @return double y position in cm
   */
  double convertPixelToCmYPosition(const double pixel_value);
  
  /**
   * @brief Get the Angle Difference
   * 
   * @return double with the angle difference
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
   * @brief the opencv videocapture object
   * 
   */
  cv::VideoCapture m_cap;
  
  /**
   * @brief the live frame of the camera
   * 
   */
  cv::Mat m_frame;
  
  /**
   * @brief contains the filtered frame 
   * 
   */
  cv::Mat m_filtered_frame;
  
  /**
   * @brief contains the frame that will be drawed
   * 
   */
  cv::Mat m_drawing_frame;
  
  /**
   * @brief contains the mask of the color
   * 
   */
  cv::Mat m_color_mask;

  /**
   * @brief is an object which represents the ROS node
   * 
   */
  ros::NodeHandle m_node_handle;

  
  /**
   * @brief client service for sending data
   * 
   */
  ros::ServiceClient m_client;

  
  /**
   * @brief contains information of the object that has to be picked up
   * 
   */
  std::shared_ptr<ColorObject> m_color_object;
  
  /**
   * @brief contains information of the destination location of the arm
   * 
   */
  std::shared_ptr<ColorObject> m_destination_object;
  
  /**
   * @brief Starts the calibration tool 
   * 
   */
  Calibration m_calibrator;
  
  /**
   * @brief checks if the user input is correct
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
   * @brief wether or not the message is sended
   * 
   */
  bool m_coordinates_sended;
};

#endif //VISIONCONTROLLER_H
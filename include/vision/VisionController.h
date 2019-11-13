#include <ros/ros.h>
#include "ObjectDetector.h"
#include <thread>
#include "vision/Calibration.h"
#include "robot_kinematica/found_object.h"

class VisionController
{
public:
  VisionController();
  ~VisionController();

  void splitString(std::string message);
  void splitAndStoreLinesBasedOnRegex();
  void checkStringValues(const std::string& figure, const std::string& color);
  void visionControllerLoop();
  void setFilteredFrame(const cv::Mat& filtered_frame);
  void readCommandLineInput();
  void findColorAndShape(const std::string& input_color, const std::string& input_figure);
  void startApplication();
  void sendObjectCoordinates();
  void cloneFrames();
  void findObjectLoop(std::shared_ptr<ColorObject>& color_object);

  std::thread readInputThread();
  double convertPixelToCmXPosition(const double pixel_value);
  double convertPixelToCmYPosition(const double pixel_value);
  double getAngleDifference();

private:
  cv::VideoCapture m_cap;
  cv::Mat m_frame;
  cv::Mat m_filtered_frame;
  cv::Mat m_drawing_frame;

  ros::Publisher m_publisher;
  ros::NodeHandle m_node_handle;

  std::shared_ptr<ColorObject> m_color_object;
  std::shared_ptr<ColorObject> m_destination_object;

  Calibration m_calibrator;
  std::atomic<bool> m_user_input_correct;

  ColorScale getColorScale(std::string& color);

  ObjectDetector m_object_detector;
  bool m_coordinates_sended;
};
#include "vision/VisionController.h"
#include <thread>
#include <regex>

namespace
{
const std::vector<std::string> COLORS = { "groen", "blauw", "rood", "wit", "zwart", "geel" };
const std::vector<std::string> FIGURES = { "vierkant", "rechthoek", "cirkel", "halve cirkel", "driehoek" };
const uint8_t ROS_LOOP_RATE = 20;
const int CAMERA_ID = 1;
const double SCREEN_HEIGHT = 480;
const double SCREEN_WIDTH = 640;
const double SCREEN_WIDTH_CM = 47.5;
const double SCREEN_HEIGHT_CM = 31.65;
const double BASE_GROUND_OFFSET = -8.0;
const int NUMBER_OF_LOOPS = 100;
const uint16_t QUEUE_SIZE = 1000;
}  // namespace

VisionController::VisionController() : m_user_input_correct(false), m_coordinates_sended(false)
{
  m_publisher = m_node_handle.advertise<robot_kinematica::found_object>("found_object", QUEUE_SIZE);
}

VisionController::~VisionController()
{
}

std::thread VisionController::readInputThread()
{
  return std::thread([=] { readCommandLineInput(); });
}

void VisionController::splitString(std::string str)
{
  size_t index = str.find_last_of(" ");
  // checkString(str);
  if (index != std::string::npos)
  {
    // Last substring for the color.
    std::string color = str.substr(index + 1, str.length());
    // remove whitespaces
    color.erase(color.find_last_not_of(" \t\n\r\f\v") + 1);
    // remove color.
    str.erase(index + 1, str.length());
    // remaining the figure
    std::string figure = str;
    // remove whitespaces
    figure.erase(figure.find_last_not_of(" \t\n\r\f\v") + 1);
    findColorAndShape(color, figure);
  }
}

void VisionController::findColorAndShape(const std::string& input_color, const std::string& input_figure)
{
  std::vector<std::string>::const_iterator color =
      std::find_if(COLORS.begin(), COLORS.end(), [&](const auto& color) { return color.compare(input_color) == 0; });

  std::vector<std::string>::const_iterator figure = std::find_if(
      FIGURES.begin(), FIGURES.end(), [&](const auto& figure) { return figure.compare(input_figure) == 0; });

  if (color != std::end(COLORS) && figure != std::end(FIGURES))
  {
    m_color_object = std::make_shared<ColorObject>(*figure, *color);
    m_color_object->setColorScale(m_calibrator.getColorScale(*color));
    m_destination_object = std::make_shared<ColorObject>("cirkel", "wit");
    m_destination_object->setColorScale(m_calibrator.getColorScale("wit"));
    m_user_input_correct = true;
  }
  else
  {
    m_user_input_correct = false;
  }
}

ColorScale VisionController::getColorScale(std::string& color)
{
  return m_calibrator.getColorScale(color);
}

void VisionController::readCommandLineInput()
{
  std::cout << "Voer een vorm en een kleur in met als format: [vorm][whitespace][kleur]" << std::endl;

  std::string input = "";
  ros::Rate rate(ROS_LOOP_RATE);

  while (ros::ok())
  {
    if (!m_user_input_correct)
    {
      input = "";
      // clean cin input
      std::cin >> std::ws;
      std::getline(std::cin, input);
      splitString(input);
    }
  }
}

void VisionController::sendObjectCoordinates()
{
  robot_kinematica::found_object found_object_message;

  double object_x_dimension = convertPixelToCmXPosition(m_color_object->getXDimension());
  double object_y_dimension = convertPixelToCmYPosition(m_color_object->getYDimension());
  double object_origin_x_location = convertPixelToCmXPosition(m_color_object->getCenterXPos());
  double object_origin_y_location = convertPixelToCmXPosition(m_color_object->getCenterYPos());
  double destination_x_location = convertPixelToCmXPosition(m_destination_object->getCenterXPos());
  double destination_y_location = convertPixelToCmYPosition(m_destination_object->getCenterYPos());

  ROS_INFO_STREAM("\nFound object dimensions"
                  << " x: " << object_x_dimension << " y: " << object_y_dimension << "\nFound object origin location"
                  << " x: " << object_origin_x_location << " y: " << object_origin_y_location
                  << " \nDestination origin location"
                  << " x: " << destination_x_location << " y: " << destination_y_location);

  found_object_message.dimension_x = object_x_dimension;
  found_object_message.dimension_y = object_y_dimension;
  found_object_message.dimension_z = BASE_GROUND_OFFSET;

  found_object_message.origin_x = object_origin_x_location;
  found_object_message.origin_y = object_origin_y_location;
  found_object_message.origin_z = BASE_GROUND_OFFSET;

  found_object_message.destination_x = destination_x_location;
  found_object_message.destination_y = destination_y_location;
  found_object_message.destination_z = BASE_GROUND_OFFSET;

  m_publisher.publish(found_object_message);

  m_coordinates_sended = true;
}

double VisionController::convertPixelToCmYPosition(const double pixel_value)
{
  double scale = SCREEN_HEIGHT_CM / SCREEN_HEIGHT;
  return scale * pixel_value;
}

double VisionController::convertPixelToCmXPosition(const double pixel_value)
{
  double scale = SCREEN_WIDTH_CM / SCREEN_WIDTH;
  return scale * pixel_value;
}

void VisionController::cloneFrames()
{
  m_filtered_frame = m_frame.clone();
  m_drawing_frame = m_frame.clone();
}

void VisionController::findObjectLoop(std::shared_ptr<ColorObject>& color_object)
{
  for (int i = 0; i < NUMBER_OF_LOOPS; i++)
  {
    m_cap >> m_frame;
    cv::waitKey(30);
    cloneFrames();
    m_object_detector.filterFrame(m_filtered_frame);
    m_object_detector.filterColor(color_object, m_filtered_frame);
    m_object_detector.findShape(color_object, m_drawing_frame);

    if (color_object->getObjectDetected())
    {
      break;
    }
  }
}

void VisionController::visionControllerLoop()
{
  m_cap.open(CAMERA_ID);

  while (true)
  {
    m_cap >> m_frame;
    imshow("live", m_frame);
    cv::waitKey(30);
    if (m_user_input_correct)
    {
      if (!m_color_object->getObjectDetected() && !m_destination_object->getObjectDetected())
      {
        findObjectLoop(m_destination_object);
        findObjectLoop(m_color_object);
      }

      if (m_color_object->getObjectDetected() && m_destination_object->getObjectDetected())
      {
        if (!m_coordinates_sended)
        {
          sendObjectCoordinates();
        }
      }
      else
      {
        std::cout << "Voer een vorm en een kleur in met als format: [vorm][whitespace][kleur]" << std::endl;
        m_user_input_correct = false;
      }

      imshow("object detector", m_drawing_frame);
    }
  }
  cv::waitKey(0);
}

void VisionController::startApplication()
{
  m_calibrator.startCalibration();
  std::thread userInputThread = readInputThread();
  visionControllerLoop();
  userInputThread.join();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "VisionController");
  ros::Time::init();
  VisionController controller;
  controller.startApplication();
  ros::spin();

  return 0;
}
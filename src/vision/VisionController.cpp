#include "vision/VisionController.h"
#include <thread>
#include <regex>

namespace
{
const std::vector<std::string> COLORS = { "groen", "blauw", "rood", "wit", "zwart", "geel" };
const std::vector<std::string> FIGURES = { "vierkant", "rechthoek", "cirkel", "halve cirkel", "driehoek" };
const uint8_t ROS_LOOP_RATE = 20;
const int CAMERA_ID = 0;
const double SCREEN_HEIGHT = 480;
const double SCREEN_WIDTH = 640;
const double SCREEN_WIDTH_CM = 46.8;
const double SCREEN_HEIGHT_CM = 35.5;
const double BASE_CARTESIAN_X = 336;
const double BASE_CARTESIAN_Y = 467;
const double BASE_GROUND_OFFSET = 0.0;
const int NUMBER_OF_LOOPS = 100;
const uint16_t QUEUE_SIZE = 1000;
}  // namespace

VisionController::VisionController() : m_user_input_correct(false), m_coordinates_sended(false)
{
  m_client = m_node_handle.serviceClient<robot_kinematica::found_object>("found_object");
  m_drawing_frame = cv::Mat::zeros(cv::Size(1, 49), CV_64FC1);
  m_filtered_frame = cv::Mat::zeros(cv::Size(1, 49), CV_64FC1);
  m_color_mask = cv::Mat::zeros(cv::Size(1, 49), CV_64FC1);

  while (!m_client.waitForExistence(ros::Duration(5.0)))
  {
  }
}

VisionController::~VisionController()
{
}

std::thread VisionController::readInputThread()
{
  return std::thread([=] { readCommandLineInput(); });
}

void VisionController::readCommandLineInput()
{
  ROS_INFO_STREAM("Voer een vorm en een kleur in met als format: [vorm][whitespace][kleur]");

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

std::thread VisionController::videoCamThread()
{
  return std::thread([=] { readVideoCam(); });
}

void VisionController::readVideoCam()
{
  while (true)
  {
    m_cap >> m_frame;
    cv::waitKey(30);
    imshow("live", m_frame);
    imshow("object detector", m_drawing_frame);
    imshow("detection", m_color_mask);
  }
  cv::waitKey(0);
}

double VisionController::getObjectCorrectedXPosition(double object_center_x)
{
  double object_corrected_x = 0;

  const double a = 0.540126931;
  const double b = -0.0355280635;
  const double c = 0.011841286;

  object_corrected_x = a * (std::pow(object_center_x, 2) + b * object_center_x + c);

  if (object_center_x < 0)
  {
    object_corrected_x *= -1;
  }

  return object_corrected_x;
}

void VisionController::splitString(std::string str)
{
  size_t index = str.find_last_of(" ");

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

double VisionController::getAngleDifference()
{
  double angleDifference = 0;

  if (m_color_object->m_approx[3].x < BASE_CARTESIAN_X)
  {
    double xDev1 = BASE_CARTESIAN_X - m_color_object->m_approx[3].x;
    double yDev1 = BASE_CARTESIAN_Y - m_color_object->m_approx[3].y;

    double hypotenuse1 = std::hypot(xDev1, yDev1);
    double angle1 = std::asin(xDev1 / hypotenuse1);

    double xDev2 = BASE_CARTESIAN_X - m_color_object->m_approx[0].x;
    double yDev2 = BASE_CARTESIAN_Y - m_color_object->m_approx[0].y;

    double hypotenuse2 = std::hypot(xDev2, yDev2);
    double angle2 = std::asin(xDev2 / hypotenuse2);

    angleDifference = ((angle2 - angle1) * 180 / M_PI) * -1;
  }
  else if (m_color_object->m_approx[0].x >= BASE_CARTESIAN_X)
  {
    double xDev1 = m_color_object->m_approx[0].x - BASE_CARTESIAN_X;
    double yDev1 = BASE_CARTESIAN_Y - m_color_object->m_approx[0].y;

    double hypotenuse1 = std::hypot(xDev1, yDev1);
    double angle1 = std::asin(xDev1 / hypotenuse1);

    double xDev2 = m_color_object->m_approx[3].x - BASE_CARTESIAN_X;
    double yDev2 = BASE_CARTESIAN_Y - m_color_object->m_approx[3].y;

    double hypotenuse2 = std::hypot(xDev2, yDev2);
    double angle2 = std::asin(xDev2 / hypotenuse2);

    angleDifference = (angle2 - angle1) * 180 / M_PI;
  }
  return angleDifference;
}

void VisionController::setObjectPolarCoordinates(double& object_polar_x, double& object_polar_y,
                                                 std::shared_ptr<ColorObject> color_object)
{
  if (color_object->getCenterXPos() < BASE_CARTESIAN_X)
  {
    object_polar_x = convertPixelToCmXPosition((BASE_CARTESIAN_X - color_object->getCenterXPos()) * -1) / 100;
    object_polar_y = convertPixelToCmYPosition(BASE_CARTESIAN_Y - color_object->getCenterYPos()) / 100;
  }
  else if (color_object->getCenterXPos() >= BASE_CARTESIAN_X)
  {
    object_polar_x = convertPixelToCmXPosition(color_object->getCenterXPos() - BASE_CARTESIAN_X) / 100;
    object_polar_y = convertPixelToCmYPosition(BASE_CARTESIAN_Y - color_object->getCenterYPos()) / 100;
  }
}

void VisionController::sendObjectCoordinates()
{
  robot_kinematica::found_object found_object_message;

  double object_polar_x = 0;
  double object_polar_y = 0;
  double destination_polar_x = 0;
  double destination_polar_y = 0;

  setObjectPolarCoordinates(object_polar_x, object_polar_y, m_color_object);
  setObjectPolarCoordinates(destination_polar_x, destination_polar_y, m_destination_object);

  double correction_x_position = getObjectCorrectedXPosition(object_polar_x);

  ROS_INFO_STREAM("\nFound object dimensions"
                  << " x: "
                  << "Object_x_dimension"
                  << " y: "
                  << "object_y_dimension"
                  << "\nFound object origin location"
                  << " x: " << correction_x_position << " y: " << object_polar_y << " \nDestination origin location"
                  << " x: " << destination_polar_x << " y: " << destination_polar_y);

  found_object_message.request.origin_x = correction_x_position + object_polar_x;
  found_object_message.request.origin_y = object_polar_y;
  found_object_message.request.origin_cartesian_x = m_color_object->getCenterXPos();
  found_object_message.request.origin_cartesian_y = m_color_object->getCenterYPos();

  correction_x_position = getObjectCorrectedXPosition(destination_polar_x);
  found_object_message.request.destination_x = destination_polar_x + correction_x_position;
  found_object_message.request.destination_y = destination_polar_y;

  if ((!m_color_object->getFigure().compare("cirkel") == 0) && (!m_color_object->getFigure().compare("driehoek") == 0))
  {
    setApproxValues(found_object_message);
  }

  m_coordinates_sended = true;
  if (m_client.call(found_object_message))
  {
    if (found_object_message.response.finished)
    {
      ROS_INFO_STREAM("Robot movement finished");
      ROS_INFO_STREAM("Voer een vorm en een kleur in met als format: [vorm][whitespace][kleur]");
    }
    else
    {
      ROS_INFO_STREAM("Robot movement failed");
      ROS_INFO_STREAM("Voer een vorm en een kleur in met als format: [vorm][whitespace][kleur]");
    }

    m_user_input_correct = false;
    m_color_object->setObjectDetected(false);
    m_destination_object->setObjectDetected(false);
    m_coordinates_sended = false;
  }
}

void VisionController::setApproxValues(robot_kinematica::found_object& found_object_message)
{
  found_object_message.request.approx_0_x = m_color_object->m_approx[0].x;
  found_object_message.request.approx_0_y = m_color_object->m_approx[0].y;
  found_object_message.request.approx_1_x = m_color_object->m_approx[1].x;
  found_object_message.request.approx_1_y = m_color_object->m_approx[1].y;
  found_object_message.request.approx_3_x = m_color_object->m_approx[3].x;
  found_object_message.request.approx_3_y = m_color_object->m_approx[3].y;
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
    m_object_detector.filterColor(color_object, m_filtered_frame, m_color_mask);
    m_object_detector.findShape(color_object, m_drawing_frame);

    if (color_object->getObjectDetected())
    {
      break;
    }
  }
}

void VisionController::applicationLoop()
{
  m_cap.open(CAMERA_ID);
  m_calibrator.startCalibration(m_cap);
  std::thread user_input_thread = readInputThread();
  std::thread video_cam_thread = videoCamThread();

  while (true)
  {
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
        ROS_INFO_STREAM("Voer een vorm en een kleur in met als format: [vorm][whitespace][kleur]");
        m_user_input_correct = false;
      }
    }
  }
  user_input_thread.join();
  video_cam_thread.join();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "VisionController");
  ros::Time::init();
  try
  {
    VisionController controller;
    controller.applicationLoop();
    ros::spin();
  }
  catch (const std::exception& e)
  {
    std::cout << e.what() << std::endl;
  }

  return 0;
}
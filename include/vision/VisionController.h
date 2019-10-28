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
    void checkStringValues(const std::string &figure, const std::string &color);
    void visionControllerLoop();
    void setFilteredFrame(const cv::Mat &filtered_frame);
    void readCommandLineInput();
    void findColorAndShape(const std::string& input_color, const std::string& input_figure);
    void startApplication();
    void sendObjectCoordinates(std::shared_ptr<ColorObject>& found_object);

    std::thread readInputThread();
    double convertPixelToCmXPosition(const double pixel_value);


  private:
    cv::VideoCapture m_cap;
    cv::Mat m_frame;
    cv::Mat m_filtered_frame;

    ros::Publisher message_publisher;
    //ros::NodeHandle node_handle; veroorzaakt crash..
    
    std::shared_ptr<ObjectDetector> m_detector;
    std::shared_ptr<ColorObject> m_color_object;
    Calibration m_calibrator;
    std::atomic<bool> m_found_shape_object;

    ColorScale getColorScale(std::string& color);
};
#include <ros/ros.h>
#include "ObjectDetector.hpp"
#include <regex>
#include <iostream>
#include <fstream>
#include <mutex>
#include <thread>
#include "vision/Calibration.hpp"


class VisionController
{
  public:
    /**
     * @brief Construct a new VisionController object
     * 
     * @param aInputData string that contains the data of a batch file.
     */
    VisionController();

    /**
     * @brief Destroy the VisionController object
     * 
     */
    ~VisionController();

    /**
     * @brief Splits the string in two seperate words based on the format.
     * 
     * @param str string contains a single commando's of the user or batch file
     */
    void splitString(std::string str);

    /**
     * @brief splits batch file line by line based of the regex.
     * 
     */
    void splitAndStoreLinesBasedOnRegex();

    /**
     * @brief Checks the string values based of the static figures and colors.
     * 
     * @param figure string that contains a figure. 
     * @param color string that contains a color.
     */
    void checkStringValues(const std::string &figure, const std::string &color);

    /**
     * @brief The main while loop of the VisionController.
     * 
     * @param boolean for the mode of the VisionController.
     * 0 for interactive modus.
     * 1 for batch modus.
     */
    void visionControllerLoop();

    /**
     * @brief Set the Filtered Frame object
     * 
     * @param aFiltereFrame 
     */
    void setFilteredFrame(const cv::Mat &aFiltereFrame);


    /**
     * @brief Function that reads the user input.
     * 
     */
    void readCin();
    
    void checkString(const std::string & str);

    void findColorAndShape(const std::string& inputColor, const std::string& inputFigure);
    
    static std::atomic<bool> foundShapeObject;
    //static std::atomic<bool> c;

    std::thread readInputThread() {
          return std::thread([=] { readCin(); });
      }



    Calibration m_calibrator;

  private:
    cv::VideoCapture cap;
    cv::Mat frame;
    cv::Mat filteredFrame;
    std::string inputData;
    char input[100];
    bool inputFromUser;
    std::shared_ptr<ObjectDetector> detecter;
    std::shared_ptr<ColorObject> m_colorObject;
    std::atomic<bool> m_foundShapeObject;
    ColorScale getColorScale(std::string& color);
};
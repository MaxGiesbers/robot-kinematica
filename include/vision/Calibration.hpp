#include <ros/ros.h>
#include "opencv2/core.hpp"
#include <opencv2/opencv.hpp>
#include "vision/ColorScale.hpp"

class Calibration
{
    public: 
    Calibration();
    ~Calibration();
    void startCalibration();
    ColorScale getColorScale(std::string color) const;

    private:
    cv::Mat BrightnessAndContrastAuto(const cv::Mat& frame, double clipHistPercent);
    void updateSetRange();

    static void calibrate(int v, void *ptr)
    {
        v += 1; // To prevent wExtra complains

        Calibration* calibration = static_cast<Calibration*>(ptr);
        calibration->updateSetRange();
    }

    int m_iLowH;
    int m_iLowS;
    int m_iLowV;
    int m_iHighH;
    int m_iHighS;
    int m_iHighV;
    cv::Mat m_treshold;
    cv::Mat m_src;
    std::vector<ColorScale> m_colorScales;
    cv::VideoCapture m_cap;
    cv::Mat m_captureWindow;
};
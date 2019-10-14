#include <ros/ros.h>
#include "ColorObject.h"

class ObjectDetector
{
    public:

    explicit ObjectDetector(cv::Mat frame, cv::Mat filtered_frame);
    ~ObjectDetector();

    void setText(std::shared_ptr<ColorObject>& color_object);
    void setCenterPoint(const std::shared_ptr<ColorObject> &color_object,std::vector<cv::Point> &contour);
    void findShape(std::shared_ptr<ColorObject>& color_object);
    void filterColor(std::shared_ptr<ColorObject>& shape);
    void DrawImageContours(const std::vector<std::vector<cv::Point>> &contour, 
    const std::shared_ptr<ColorObject>& color_object, const int contour_number);

    cv::Mat BrightnessAndContrastAuto(const cv::Mat& frame, double clip_hist_percent);
    bool checkSquareAndRectangle(std::shared_ptr<ColorObject>& color_object, std::vector <cv::Point>& approx);
    bool checkCircle(std::shared_ptr<ColorObject>& color_object,
                                    std::vector<std::vector<cv::Point>>& contours, int element);
    bool semiCircle(std::shared_ptr<ColorObject>& color_object,
                                    std::vector<std::vector<cv::Point>>& contours, int element);

    private:
    cv::Mat m_frame;
    cv::Mat m_filtered_frame;
};



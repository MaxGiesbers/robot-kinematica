#include "ColorScale.h"
#include <ros/ros.h>
#include <vector>
#include "opencv2/core.hpp"
#include <opencv2/opencv.hpp>

class ColorObject{

    public:

    explicit ColorObject(const std::string& input_figure, const std::string& color);
    ~ColorObject();
    void setColorMask(const cv::Mat& color_mask); 
    void printColorObject();
    void setArea(const double aArea);
    void setCenterXPos(const double center_x_pos);
    void setCenterYPos(const double center_y_pos);

    void setXDimension(const double x_dimension);
    void setYDimension(const double y_dimension);
    void setZDimension(const double z_dimension);

    void setXOrigin(const double x_origin);
    void setYOrigin(const double y_origin);
    void setZOrigin(const double z_origin);

    void setXDestination(const double x_destination);
    void setYDestination(const double y_destination);
    void setZDestination(const double y_destination);
    void setObjectDetected(bool object_detected);

    void setColorScale(ColorScale color_scale);
    void setFigure(const std::string& figure);
    std::string getFigure() const;

    double getCenterXPos() const;
    double getCenterYPos() const;

    double getXDimension() const;
    double getYDimension() const;
    double getZDimension() const;

    double getXOrigin() const;
    double getYOrigin() const;
    double getZOrigin() const;

    double getXDestination() const;
    double getYDestination() const;
    double getZDestination() const;

    double getArea() const;
    bool getObjectDetected() const;

    const std::string getShapeName() const;
    const std::string getInputFigure() const;
    const std::string getColor() const;
    const cv::Mat getColorMask() const;
    const ColorScale& getColorScale() const;
    

    private:
    ColorScale m_color_scale;

    std::string m_input_figure;
    std::string m_color;
    cv::Mat m_color_mask;
    std::string m_figure;
    double m_center_x_pos;
    double m_center_y_pos;
    double m_area;

    double m_x_dimension;
    double m_y_dimension;
    double m_z_dimension;

    double m_x_origin;
    double m_y_origin;
    double m_z_origin;

    double m_x_destination;
    double m_y_destination;
    double m_z_destination;
    bool m_object_detected;
};
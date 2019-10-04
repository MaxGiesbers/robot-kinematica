#include "ColorScale.hpp"
#include <vector>
#include "opencv2/core.hpp"
#include <opencv2/opencv.hpp>
#include <ros/ros.h>


class ColorObject{

    public:
    /**
     * @brief Construct a new Color Object object.
     * 
     * @param aInputFigure string that contains the user input figure.
     * @param aColor string that contains the user input color.
     */
    explicit ColorObject(const std::string &aInputFigure, const std::string &aColor);

    /**
     * @brief Destroy the Color Object object.
     * 
     */
    ~ColorObject();

    /**
     * @brief setter that sets the colormask of a certain color range.
     * 
     * @param aColorMask Mat object that contains a threshold of a certain color.
     */
    void setColorMask(const cv::Mat& aColorMask); 

    /**
     * @brief prints some member variables of the class.
     * 
     */
    void printColorObject();

   /**
    * @brief Set the Area object.
    * 
    * @param aArea double that contains the area of a shape.
    */
    void setArea(const double aArea);

    /**
     * @brief Set the Center x pos object.
     * 
     * @param aCenter_x_pos double that contains the center coordinate of the x pos.
     */
    void setCenter_x_pos(const double aCenter_x_pos);

    /**
     * @brief Set the Center y Pos object.
     * 
     * @param aCenter_y_pos double that contains the center coordinate of the y pos.
     */
    void setCenter_y_Pos(const double aCenter_y_pos);

    /**
     * @brief Set the Color object.
     * 
     * @param aColor string that contains the color of the shape.
     */
    void setColorScale(ColorScale colorScale);

    /**
     * @brief Set the Figure object.
     * 
     * @param aFigure. 
     */
    void setFigure(const std::string& aFigure);

    /**
     * @brief Get the Figure object.
     * 
     * @return std::string contains the figure of the shape.
     */
    std::string getFigure() const;

    /**
     * @brief Get the Center x pos object.
     * 
     * @return double that contains the center_x coordinate of the shape.
     */
    double getCenter_x_pos()const;

    /**
     * @brief Get the Center y pos object.
     * 
     * @return double that contains the center_y coordinate of the shape.
     */
    double getCenter_y_pos() const;

    /**
     * @brief Get the Area object.
     * 
     * @return double that contains the area of the shape.
     */
    double getArea() const;

    /**
     * @brief Get the Shape Name object.
     * 
     * @return const std::string that contains the shape name.
     */
    const std::string getShapeName() const;

    /**
     * @brief Get the Input Figure object.
     * 
     * @return const std::string that contains user input figure.
     */
    const std::string getInputFigure() const;

    /**
     * @brief Get the Color object.
     * 
     * @return const std::string that contains the color.
     */
    const std::string getColor() const;

    /**
     * @brief Get the Color Mask object.
     * 
     * @return const cv::Mat contains the mask of a certain color range.
     */
    const cv::Mat getColorMask() const;
    
    ColorScale m_colorScale;

    private:
    std::string inputFigure;
    std::string color;
    cv::Mat colorMask;
    std::string figure;
    double center_x_pos;
    double center_y_pos;
    double area;
};
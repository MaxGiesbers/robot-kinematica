#include <iostream>
#include "ColorObject.hpp"
#include <cmath>
#include <ros/ros.h>


class ObjectDetector
{
    public:
    /**
     * @brief Construct a new Object Detecter object.
     * 
     * @param aFrame Mat object that contains the image of the camera.
     */
    explicit ObjectDetector(cv::Mat aFrame, cv::Mat aFilteredFrame);

    /**
     * @brief Destroy the Object Detecter object
     * 
     */
    ~ObjectDetector();

    /**
     * @brief puts texts in the frame.
     * 
     * @param colorObject that has a mat Object for placing text in the mat. 
     * @param mode for interactive or batch mode. 
     */
    void setText(std::shared_ptr<ColorObject>& colorObject);
    
    /**
     * @brief Set the Center x and y coordinates of the colorObject.
     * 
     * @param colorObject contains the x and y coordinates for the setter.
     * @param contour contains the contour of the shape.
     */
    void setCenterPoint(const std::shared_ptr<ColorObject> &ColorObject,std::vector<cv::Point> &contour);

    /**
     * @brief Searches for the shapes in the Mat Object.
     * 
     * @param colorObject ColorObject that contains the mat object and all the setter variables.
     * @param mode for interactive of batch mode.
     */
    void findShape(std::shared_ptr<ColorObject>& colorObject);

    /**
     * @brief filters a certain color mask based of the color. 
     * 
     * @param shape that contains the color
     */
    void filterColor(std::shared_ptr<ColorObject>& shape);

    /**
     * @brief draws the contours of a certain shape.
     * 
     * @param contour that contains the contours of a certain shape.
     * @param colorObject contains the mat object for drawing contours.
     * @param contourNumber the number of the contour that has to be drawn
     */
    void DrawImageContours(const std::vector<std::vector<cv::Point>> &contour, 
    const std::shared_ptr<ColorObject>& colorObject, const int contourNumber);

    /**
     * @brief enhances the contrast color of the mat object.
     * 
     * @param frame the frame that has to be copied.
     * @param clipHistPercent percentage of the filter
     * @return cv::Mat 
     */
    cv::Mat BrightnessAndContrastAuto(const cv::Mat& frame, double clipHistPercent);

    /**
     * @brief checks if the contour of the shape is the same as a square or a rectangle.
     * 
     * @param colorObject that contains the mat object with the threshold.
     * @param approx vector of points which contains the number angles.
     * @return true if the shape is the same as a square.
     * @return false if the shape is not the same as square.
     */
    bool checkSquareAndRectangle(std::shared_ptr<ColorObject>& colorObject, std::vector <cv::Point>& approx);

    /**
     * @brief Checks if the contour of the shape is the same as a circle.  
     * 
     * @param colorObject contains a object that contains the mat object with the threshold of the color range.
     * @param contours a vector of vectors that contains the points from the contours.
     * @param element int that contains the right index number of the right contour.
     * @return true if the shape is the same as a circle.
     * @return false if the shape is not the same as a circle.
     */
    bool checkCircle(std::shared_ptr<ColorObject>& colorObject,
                                    std::vector<std::vector<cv::Point>>& contours, int element);

    /**
     * @brief Checks if the contour of the shape is the same as a semicircle   
     * 
     * @param colorObject contains a object that contains the mat object with the threshold of the color range.
     * @param contours a vector of vectors that contains the points from the contours.
     * @param element int that contains the right index number of the right contour.
     * @return true if the shape is the same as a semicircle.
     * @return false if the shape is not the same as a semicircle.
     */
    bool semiCircle(std::shared_ptr<ColorObject>& colorObject,
                                    std::vector<std::vector<cv::Point>>& contours, int element);

    private:
    cv::Mat frame;
    cv::Mat filteredFrame;
};



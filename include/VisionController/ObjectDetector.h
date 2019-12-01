#ifndef OBJECTDETECTOR_H
#define OBJECTDETECTOR_H

#include "ColorObject.h"

class ObjectDetector
{
public:
  /**
   * @brief Construct a new Object Detector object
   *
   */
  explicit ObjectDetector();

  /**
   * @brief Destroy the Object Detector object
   *
   */
  ~ObjectDetector();

  /**
   * @brief Set the Text
   *
   * @param color_object The detected object
   * @param drawing_frame The window in which the text should be set
   */
  void setText(std::shared_ptr<ColorObject>& color_object, cv::Mat& drawing_frame);

  /**
   * @brief Set the Center Point
   *
   * @param color_object The detected object
   * @param contour The contour data for the detected object
   */
  void setCenterPoint(const std::shared_ptr<ColorObject>& color_object, std::vector<cv::Point>& contour);

  /**
   * @brief function to find the requested shape
   *
   * @param color_object contains all the information of the found object
   * @param drawing_frame this frame will be drawed with contour lines of the found object
   */
  void findShape(std::shared_ptr<ColorObject>& color_object, cv::Mat& drawing_frame);

  /**
   * @brief filters the color based on the color input of the user
   *
   * @param color_object The detected shape
   * @param filtered_frame The window on which the filtered frame should be displayed
   * @param color_mask The mask that is used for filtering the color
   */
  void filterColor(const std::shared_ptr<ColorObject>& color_object, cv::Mat& filtered_frame, cv::Mat& color_mask);

  /**
   * @brief Function to draw the contours of the detected shape
   *
   * @param contour The contour data of the detected shape
   * @param color_object The detected shape
   * @param contour_number the number of the contour that has to be drawn
   * @param drawing_frame The window in which the contour should be drawn
   */
  void DrawImageContours(const std::vector<std::vector<cv::Point>>& contour,
                         const std::shared_ptr<ColorObject>& color_object, const int contour_number,
                         cv::Mat& drawing_frame);

  /**
   * @brief enhances the contrast color of the mat object.
   *
   * @param frame the frame that has to be copied.
   * @param clipHistPercent percentage of the filter
   * @return cv::Mat
   */
  cv::Mat BrightnessAndContrastAuto(const cv::Mat& frame, double clip_hist_percent);

  /**
   * @brief Function to check whether the detected shape is a square or rectangle
   *
   * @param color_object The detected shape
   * @param approx Vecor containing the contour data
   * @return true if the shape is the same as rectangle or square
   * @return false if the shape is not the sames as a rectangle or square
   */
  bool checkSquareAndRectangle(std::shared_ptr<ColorObject>& color_object, std::vector<cv::Point>& approx);

  /**
   * @brief Function to check whether the detected shape is a circle
   *
   * @param color_object The deteccted shape
   * @param contours Vector containing the contour data
   * @param element int that contains the right index number of the right contour.
   * @return true if the shape is the same as a circle
   * @return false if the shape is not the same as a circle
   */
  bool checkCircle(std::shared_ptr<ColorObject>& color_object, std::vector<std::vector<cv::Point>>& contours,
                   int element);

  /**
   * @brief Function to check whether the detected shape is a semicircle
   *
   * @param color_object The detected shape
   * @param contours Vector containing the contour data
   * @param element int that contains the right index number of the right contour.
   * @return true when the shape is the same as a semi circle
   * @return false when the shape is not the same a semi circle
   */
  bool semiCircle(std::shared_ptr<ColorObject>& color_object, std::vector<std::vector<cv::Point>>& contours,
                  int element);

  /**
   * @brief filters the matrix frame based on the color
   *
   * @param filtered_frame The matrix containing the filtered frame
   */
  void filterFrame(cv::Mat& filtered_frame);
};

#endif  // OBJECTDETECTOR_H

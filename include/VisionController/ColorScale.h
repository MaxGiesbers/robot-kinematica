#ifndef COLORSCALE_H
#define COLORSCALE_H
#include <string>

/**
 * @brief A struct that contains the names of the HSV values
 *
 */
struct ColorScale
{
  int iLowH;
  int iHighH;
  int iLowS;
  int iHighS;
  int iLowV;
  int iHighV;
  std::string color;
};
#endif  // COLORSCALE_H
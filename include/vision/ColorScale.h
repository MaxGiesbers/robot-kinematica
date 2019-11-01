#ifndef COLORSCALE_H
#define COLORSCALE_H
#include <string>
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
#endif
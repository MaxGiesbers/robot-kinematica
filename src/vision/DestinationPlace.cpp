#include "vision/DestinationPlace.h"

double DestinationPlace::getXDestination() const
{
    return m_destination_x;
}
double DestinationPlace::getYDestination() const
{
    return m_destination_y;
}
double DestinationPlace::getZDestination() const
{
    return m_destination_z;
}

void DestinationPlace::setXDestination(const double x_destination)
{
    m_destination_x = x_destination;
}

void DestinationPlace::setYDestination(const double y_destination)
{   
    m_destination_y = y_destination;
}
void DestinationPlace::setZDestination(const double z_destination)
{
    m_destination_z = z_destination;
}
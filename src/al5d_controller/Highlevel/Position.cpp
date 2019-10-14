#include "al5d_controller/Highlevel/Position.h"
namespace Highlevel{
Position::Position() : name("unknown position"), time(0)
{

}

Position::Position(const std::string& name) : name(name), time(0)
{

}

Position::~Position()
{

}

// Position::Position(const Position& obj) : name(obj.name), time(obj.time)
// {

// }

std::vector<short> Position::getDegrees() const
{
    return degreesList;
}

void Position::addDegrees(short degrees)
{
    degreesList.push_back(degrees);
}

const std::string& Position::getName() const
{
    return name;
}

void Position::setName(const std::string& name)
{
    this->name = name;
}

unsigned short Position::getTime() const
{
    return time;
}

void Position::setTime(unsigned short time)
{
    this->time = time;
}
}
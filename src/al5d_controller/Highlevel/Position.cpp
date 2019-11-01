#include "al5d_controller/Highlevel/Position.h"

Position::Position() : m_name("unknown position"), m_time(0)
{
}

Position::Position(const std::string& name) : m_name(name), m_time(0)
{
}

Position::~Position()
{
}

std::vector<int16_t> Position::getDegrees() const
{
  return m_degrees_list;
}

void Position::addDegrees(int16_t degrees)
{
  m_degrees_list.push_back(degrees);
}

const std::string& Position::getName() const
{
  return m_name;
}

void Position::setName(const std::string& name)
{
  m_name = name;
}

uint16_t Position::getTime() const
{
  return m_time;
}

void Position::setTime(uint16_t time)
{
  m_time = time;
}

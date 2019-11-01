#ifndef POSITION_H_
#define POSITION_H_

#include <string>
#include <vector>
class Position
{
public:
  Position();
  Position(const std::string& name);
  virtual ~Position();
  std::vector<int16_t> getDegrees() const;
  void addDegrees(int16_t degrees);
  const std::string& getName() const;
  void setName(const std::string& name);
  uint16_t getTime() const;
  void setTime(uint16_t time);

private:
  std::string m_name;
  std::vector<int16_t> m_degrees_list;
  uint16_t m_time;
};

#endif  // POSITION_H_
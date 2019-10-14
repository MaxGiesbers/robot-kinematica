#ifndef POSITION_H_
#define POSITION_H_

/**
 * @brief The pre-programmed Positions that are parsed from ProgrammedPositions.csv into the High level interface.
 * 
 * @file Position.h
 * @author Joris van Zeeland, Max Giesbers
 * @date 2018-09-25
 */

#include <string>
#include <vector>
namespace Highlevel{
class Position 
{
public:
    /**
     * @brief Construct a new Position object
     */
    Position();
    /**
     * @brief Construct a new Position object
     * 
     * @param name The name of the pre-programmed Position
     */
    Position(const std::string& name);
    /**
     * @brief Destroy the Position object
     * 
     */
    virtual ~Position();
    // /**
    //  * @brief Copy-constructor of Position object
    //  * 
    //  * @param obj the instance of Position that needs to be copied
    //  */
    // Position(const Position& obj);
    /**
     * @brief Get the degrees from the list of degrees of the pre-programmed position
     * 
     * @return std::vector<short> 
     */
    std::vector<short> getDegrees() const;
    /**
    * @brief add degrees to list of doegrees
    */
    void addDegrees(short degrees);
    /**
     * @brief Get the name of the pre-programmed position
     * 
     * @return const std::string& 
     */
    const std::string& getName() const;
    /**
     * @brief Set the name of the pre-programmed position
     * 
     * @param name 
     */
    void setName(const std::string& name);
    /**
     * @brief Get the time of the movement of the arm
     * 
     * @return unsigned short the time
     */
    unsigned short getTime() const;
    /**
     * @brief Set the velocity/time of the movement of arm
     * 
     * @param time The time in which the movement has to be executed
     */
    void setTime(unsigned short time);

private:
    std::string name;
    std::vector<short> degreesList;
    unsigned short time;
    
};
}

#endif //POSITION_H_
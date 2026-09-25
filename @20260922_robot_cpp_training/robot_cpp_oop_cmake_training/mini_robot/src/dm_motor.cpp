#include "robot/dm_motor.hpp"

#include <iomanip>
#include <iostream>

namespace robot {

DMMotor::DMMotor(int id)
    : id_(id)
{
}

void DMMotor::enable()
{
    std::cout << "DM Motor " << id_ << " enabled." << std::endl;
}

void DMMotor::disable()
{
    std::cout << "DM Motor " << id_ << " disabled." << std::endl;
}

void DMMotor::setPosition(double position)
{
    position_ = position;
    std::cout << "DM Motor " << id_ << " -> target = " << std::fixed
              << std::setprecision(2) << position_ << " rad" << std::endl;
}

double DMMotor::getPosition() const
{
    return position_;
}

}  // namespace robot

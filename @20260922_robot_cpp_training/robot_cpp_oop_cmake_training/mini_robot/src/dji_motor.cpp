#include "robot/dji_motor.hpp"

#include <iomanip>
#include <iostream>

namespace robot {

DJIMotor::DJIMotor(int id)
    : id_(id)
{
}

void DJIMotor::enable()
{
    std::cout << "DJI Motor " << id_ << " enabled." << std::endl;
}

void DJIMotor::disable()
{
    std::cout << "DJI Motor " << id_ << " disabled." << std::endl;
}

void DJIMotor::setPosition(double position)
{
    position_ = position;
    std::cout << "DJI Motor " << id_ << " -> target = " << std::fixed
              << std::setprecision(2) << position_ << " rad" << std::endl;
}

double DJIMotor::getPosition() const
{
    return position_;
}

}  // namespace robot

#include "robot/unitree_motor.hpp"

#include <iomanip>
#include <iostream>

namespace robot {

UnitreeMotor::UnitreeMotor(int id)
    : id_(id)
{
}

void UnitreeMotor::enable()
{
    std::cout << "Unitree Motor " << id_ << " enabled." << std::endl;
}

void UnitreeMotor::disable()
{
    std::cout << "Unitree Motor " << id_ << " disabled." << std::endl;
}

void UnitreeMotor::setPosition(double position)
{
    position_ = position;
    std::cout << "Unitree Motor " << id_ << " -> target = " << std::fixed
              << std::setprecision(2) << position_ << " rad" << std::endl;
}

double UnitreeMotor::getPosition() const
{
    return position_;
}

}  // namespace robot

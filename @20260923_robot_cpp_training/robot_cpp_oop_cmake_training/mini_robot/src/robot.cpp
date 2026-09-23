#include "robot/robot.hpp"

#include <iomanip>
#include <iostream>

namespace robot {

Robot::Robot(Motor& left_motor, Motor& right_motor)
    : left_motor_(left_motor), right_motor_(right_motor)
{
}

void Robot::initialize()
{
    std::cout << "Robot initialization..." << std::endl << std::endl;
    left_motor_.enable();
    right_motor_.enable();
}

void Robot::move(double position)
{
    std::cout << std::endl << "Set robot target position: " << std::fixed
              << std::setprecision(2) << position << " rad" << std::endl << std::endl;
    left_motor_.setPosition(position);
    right_motor_.setPosition(position);
    std::cout << std::endl << "Robot motion command finished." << std::endl;
}

void Robot::printStatus() const
{
    std::cout << std::endl << "Left motor position: " << std::fixed
              << std::setprecision(2) << left_motor_.getPosition() << " rad" << std::endl;
    std::cout << "Right motor position: " << std::fixed
              << std::setprecision(2) << right_motor_.getPosition() << " rad" << std::endl;
}

}  // namespace robot

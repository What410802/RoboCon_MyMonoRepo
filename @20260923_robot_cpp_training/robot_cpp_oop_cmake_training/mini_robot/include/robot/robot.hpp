#pragma once

#include "robot/motor.hpp"

namespace robot {

class Robot {
public:
    Robot(Motor& left_motor, Motor& right_motor);

    void initialize();
    void move(double position);
    void printStatus() const;

private:
    Motor& left_motor_;
    Motor& right_motor_;
};

}  // namespace robot

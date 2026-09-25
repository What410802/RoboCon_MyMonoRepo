#pragma once

#include "robot/motor.hpp"

namespace robot {

class DJIMotor : public Motor {
public:
    explicit DJIMotor(int id);

    void enable() override;
    void disable() override;
    void setPosition(double position) override;
    double getPosition() const override;

private:
    int id_;
    double position_ = 0.0;
};

}  // namespace robot

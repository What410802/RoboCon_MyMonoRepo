#pragma once

#include "robot/motor.hpp"

namespace robot {

class DMMotor : public Motor {
public:
    explicit DMMotor(int id);

    void enable() override;
    void setPosition(double position) override;
    double getPosition() const override;

private:
    int id_;
    double position_ = 0.0;
};

}  // namespace robot

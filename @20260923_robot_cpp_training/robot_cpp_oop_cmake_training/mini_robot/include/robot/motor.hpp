#pragma once

namespace robot {

class Motor {
public:
    virtual void enable() = 0;
    virtual void setPosition(double position) = 0;
    virtual double getPosition() const = 0;
    virtual ~Motor() = default;
};

}  // namespace robot

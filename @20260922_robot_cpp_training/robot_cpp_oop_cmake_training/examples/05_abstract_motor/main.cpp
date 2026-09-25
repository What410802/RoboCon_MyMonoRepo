#include <iostream>

class Motor {
public:
    virtual void enable() = 0;
    virtual void setPosition(double position) = 0;
    virtual double getPosition() const = 0;
    virtual ~Motor() = default;
};

class SimulatedMotor : public Motor {
public:
    void enable() override
    {
        enabled_ = true;
        std::cout << "Simulated motor enabled." << std::endl;
    }

    void setPosition(double position) override
    {
        position_ = position;
        std::cout << "Simulated target: " << position_ << " rad" << std::endl;
    }

    double getPosition() const override
    {
        return position_;
    }

private:
    bool enabled_ = false;
    double position_ = 0.0;
};

int main()
{
    SimulatedMotor motor;
    motor.enable();
    motor.setPosition(1.0);
    std::cout << "Position: " << motor.getPosition() << " rad" << std::endl;
    return 0;
}

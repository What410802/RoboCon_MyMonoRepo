#include <iomanip>
#include <iostream>

class Motor {
public:
    virtual void enable() = 0;
    virtual void setPosition(double position) = 0;
    virtual double getPosition() const = 0;
    virtual ~Motor() = default;
};

class DMMotor : public Motor {
public:
    explicit DMMotor(int id) : id_(id) {}
    void enable() override { std::cout << "DM Motor " << id_ << " enabled." << std::endl; }
    void setPosition(double position) override { position_ = position; std::cout << "DM Motor " << id_ << " -> target = " << std::fixed << std::setprecision(2) << position_ << " rad" << std::endl; }
    double getPosition() const override { return position_; }
private:
    int id_;
    double position_ = 0.0;
};

class UnitreeMotor : public Motor {
public:
    explicit UnitreeMotor(int id) : id_(id) {}
    void enable() override { std::cout << "Unitree Motor " << id_ << " enabled." << std::endl; }
    void setPosition(double position) override { position_ = position; std::cout << "Unitree Motor " << id_ << " -> target = " << std::fixed << std::setprecision(2) << position_ << " rad" << std::endl; }
    double getPosition() const override { return position_; }
private:
    int id_;
    double position_ = 0.0;
};

void controlMotor(Motor& motor)
{
    motor.enable();
    motor.setPosition(1.5);
}

int main()
{
    DMMotor dm_motor(1);
    UnitreeMotor unitree_motor(2);
    controlMotor(dm_motor);
    controlMotor(unitree_motor);
    return 0;
}

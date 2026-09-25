#include <iostream>

class Motor {
public:
    void setPosition(double position)
    {
        if (position > 2.0) {
            position = 2.0;
        }
        if (position < -2.0) {
            position = -2.0;
        }
        position_ = position;
    }

    double getPosition() const
    {
        return position_;
    }

    void printStatus() const
    {
        std::cout << "Motor position: " << position_ << " rad" << std::endl;
    }

private:
    double position_ = 0.0;
};

int main()
{
    Motor motor;
    motor.setPosition(4.0);
    motor.printStatus();
    std::cout << "Read by getter: " << motor.getPosition() << " rad" << std::endl;
    return 0;
}

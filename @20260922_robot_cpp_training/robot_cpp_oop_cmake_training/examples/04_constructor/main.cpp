#include <iostream>

class Motor {
public:
    Motor(int id) : id_(id), position_(0.0)
    {
    }

    void setPosition(double position)
    {
        position_ = position;
    }

    void printStatus() const
    {
        std::cout << "Motor " << id_ << " position: " << position_ << " rad" << std::endl;
    }

private:
    int id_;
    double position_;
};

int main()
{
    Motor motor(1);
    motor.setPosition(1.25);
    motor.printStatus();
    return 0;
}

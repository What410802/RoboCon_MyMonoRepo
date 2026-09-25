#include <iostream>

class Motor {
public:
    int id;
    double position;
    double velocity;
    double torque;
    double temperature;
};
 
int main()
{
    Motor motor1;
    motor1.id = 1;
    motor1.position = 0.0;
    motor1.velocity = 0.0;
    motor1.torque = 0.0;
    motor1.temperature = 25.0;

    Motor motor2;
    motor2.id = 2;
    motor2.position = 0.5;
    motor2.velocity = 0.1;
    motor2.torque = 1.2;
    motor2.temperature = 26.5;

    std::cout << "Motor " << motor1.id << " position: " << motor1.position << " rad" << std::endl;
    std::cout << "Motor " << motor2.id << " position: " << motor2.position << " rad" << std::endl;
    return 0;
}

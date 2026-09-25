#include "robot/dji_motor.hpp"
#include "robot/dm_motor.hpp"
#include "robot/robot.hpp"
#include "robot/unitree_motor.hpp"

#include <iostream>

int main()
{
    std::cout << "=== Mini Robot Demo ===" << std::endl << std::endl;

    robot::DJIMotor right_motor(3);
    robot::DMMotor left_motor(1);
    // robot::UnitreeMotor right_motor(2);
    robot::Robot robot(left_motor, right_motor);

    robot.initialize();
    robot.move(2.0);
    robot.printStatus();
    robot.shutdown();
    return 0;
}

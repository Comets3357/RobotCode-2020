#include "Robot.h"

void Robot::RobotInit()
{
    timer.RobotInit(robotData.timerData);
    gyro.RobotInit(robotData.gyroData);

    drivebase.RobotInit();
}

void Robot::RobotPeriodic()
{
    timer.RobotPeriodic(robotData.timerData);
    gyro.RobotPeriodic(robotData.gyroData);

    if (IsEnabled())
    {
        otherComponents.RobotPeriodic(robotData.otherComponentsData);

        drivebase.RobotPeriodic(robotData, robotData.drivebaseData);
    }
}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}
void Robot::TeleopInit() {}

void Robot::TeleopPeriodic()
{
    controller.TeleopPeriodic(robotData, robotData.controllerData);
}

void Robot::DisabledInit()
{
    timer.DisabledInit();

    drivebase.DisabledInit();
}

void Robot::DisabledPeriodic() {}
void Robot::TestInit() {}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif
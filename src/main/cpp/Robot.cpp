#include "Robot.h"

void Robot::RobotInit()
{
    timer.RobotInit(robotData.timerData);
    auton.RobotInit();

    drivebase.RobotInit();
    intake.RobotInit();
    shooter.RobotInit();
    
}

void Robot::RobotPeriodic()
{
    timer.RobotPeriodic(robotData.timerData);

    if (IsEnabled())
    {
        otherComponents.RobotPeriodic(robotData.otherComponentsData);

        drivebase.RobotPeriodic(robotData, robotData.drivebaseData);
        intake.RobotPeriodic(robotData, robotData.intakeData);
        shooter.RobotPeriodic(robotData, robotData.shooterData);
    }
}

void Robot::AutonomousInit() 
{
    auton.AutonomousInit(robotData, robotData.autonData);
}
void Robot::AutonomousPeriodic() 
{
    auton.AutonomousPeriodic(robotData, robotData.autonData, robotData.controllerData, robotData.drivebaseData);
}
void Robot::TeleopInit() 
{
    controller.TeleopInit(robotData.controllerData);
}

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
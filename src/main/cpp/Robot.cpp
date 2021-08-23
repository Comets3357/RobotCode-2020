#include "Robot.h"

void Robot::RobotInit()
{
    timer.RobotInit(robotData.timerData);
    gyro.RobotInit(robotData.gyroData);
    drivebase.RobotInit();
    shooter.RobotInit();
}

void Robot::RobotPeriodic()
{
    timer.RobotPeriodic(robotData.timerData);
    gyro.RobotPeriodic(robotData.gyroData);
    limelight.RobotPeriodic(robotData, robotData.limelightData);


    if (IsEnabled())
    {
        otherComponents.RobotPeriodic(robotData.otherComponentsData);

        drivebase.RobotPeriodic(robotData, robotData.drivebaseData);
        indexer.RobotPeriodic(robotData, robotData.indexerData);
        intake.RobotPeriodic(robotData, robotData.intakeData);
        shooter.RobotPeriodic(robotData, robotData.shooterData);
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
    shooter.DisabledInit();
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
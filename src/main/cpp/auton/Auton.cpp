#include "auton/Auton.h"
#include "RobotData.h"

void Auton::RobotInit()
{
    //auton selector
    autonSelector.AddOption("Potato", AutonSelect::autonSelect_potato);
    autonSelector.AddOption("Exit Init Line Towards Driver Station", AutonSelect::autonSelect_exitInitLineDriverStation);
    autonSelector.AddOption("Exit Init Line Towards Rendezvous", AutonSelect::autonSelect_exitInitLineRendezvous);
    autonSelector.AddOption("Shoot and Drive Towards Driver Station", AutonSelect::autonSelect_shootAndDriveToDriverStation);
    autonSelector.AddOption("Shoot and Drive Towards Rendezvous", AutonSelect::autonSelect_shootAndDriveToRendezvous);
    autonSelector.SetDefaultOption("Potato", AutonSelect::autonSelect_potato);

    frc::SmartDashboard::PutData("Auto", &autonSelector);
}

void Auton::AutonomousInit(const RobotData &robotData, AutonData &autonData)
{
    // making sure the auton sequences start at the right step and setting the auton choice based on dashboard selection
    autonData.autonStep = 0;
    autonData.autonSelection = autonSelector.GetSelected();
}

void Auton::AutonomousPeriodic(const RobotData &robotData, AutonData &autonData, ControllerData &controllerData, DrivebaseData &drivebaseData)
{
    // for debugging
    frc::SmartDashboard::PutNumber("autonStep", autonData.autonStep);
    frc::SmartDashboard::PutNumber("autonSelection", autonData.autonSelection);

    // for autons, the robot MUST be in semi auto mode

    switch (autonData.autonSelection)
    {
    case autonSelect_potato:
        endAllTasks(robotData, controllerData);
        break;

    case autonSelect_exitInitLineDriverStation:

        switch (autonData.autonStep)
        {
        case 0:
            // drivebaseData.desiredDistance = -20;
            // controllerData.driveMode = driveMode_driveStraight;
            driveStraight(-20, robotData, drivebaseData, controllerData, autonData);
            break;
        default:
            endAllTasks(robotData, controllerData);
            break;
        }
        break;

    case autonSelect_exitInitLineRendezvous:

        switch (autonData.autonStep)
        {
        case 0:
            // drivebaseData.desiredDistance = 20;
            // controllerData.driveMode = driveMode_driveStraight;
            driveStraight(20, robotData, drivebaseData, controllerData, autonData);
            break;
        // case 1:
        //     // robotData.driveMode = driveMode_driveStraight;
        //     break;
        default:
            endAllTasks(robotData, controllerData);
            break;
        }
        break;

    case autonSelect_shootAndDriveToDriverStation:

        switch (autonData.autonStep)
        {
        case 0:
            //run up flywheel?
            controllerData.driveMode = driveMode_potato;
            controllerData.shootingMode = true;
            startDelay(10, robotData);
            autonData.autonStep++;
            break;
        case 1:
            checkDelay(robotData, autonData);
            break;
        case 2:
            controllerData.shootingMode = false;
            autonData.autonStep++;
            break;
        case 3:
            drivebaseData.desiredDistance = -20;
            controllerData.driveMode = driveMode_driveStraight;
            checkDriveStraight(robotData, autonData);
            break;

        default:
            endAllTasks(robotData, controllerData);
            break;
        }
        break;

    case autonSelect_shootAndDriveToRendezvous:

        switch (autonData.autonStep)
        {
        case 0:
            controllerData.driveMode = driveMode_potato;
            controllerData.shootingMode = true;
            startDelay(10, robotData);
            autonData.autonStep++;
            break;
        case 1:
            checkDelay(robotData, autonData);
            break;
        case 2:
            //turn shooter off before driving away
            controllerData.shootingMode = false;
            autonData.autonStep++;
            break;
        case 3:
            drivebaseData.desiredDistance = 20;
            controllerData.driveMode = driveMode_driveStraight;
            break;
        default:
            endAllTasks(robotData, controllerData);
            break;
        }
        break;

    default:
        endAllTasks(robotData, controllerData);
        break;
    }
}

void Auton::startDelay(double duration, const RobotData &robotData)
{
    delayFinal = robotData.timerData.secSinceEnabled + duration;
}

void Auton::checkDelay(const RobotData &robotData, AutonData &autonData)
{
    if (robotData.timerData.secSinceEnabled > delayFinal)
    {
        autonData.autonStep++;
    }
}

void Auton::endAllTasks(const RobotData &robotData, ControllerData &controllerData)
{
    controllerData.shootingMode = false;
    controllerData.driveMode = driveMode_potato;
}

void Auton::driveStraight(double distance, const RobotData &robotData, DrivebaseData &drivebaseData, ControllerData &controllerData, AutonData &autonData)
{
    drivebaseData.desiredDistance = distance;
    frc::SmartDashboard::PutNumber("desired distance", drivebaseData.desiredDistance);
    controllerData.driveMode = driveMode_driveStraight;
    frc::SmartDashboard::PutNumber("drivemode", controllerData.driveMode);
    checkDriveStraight(robotData, autonData);
}

void Auton::checkDriveStraight(const RobotData &robotData, AutonData &autonData)
{
    if (robotData.drivebaseData.driveStraightCompleted)
    {
        autonData.autonStep++;
    }
}
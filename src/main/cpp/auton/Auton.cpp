#include "auton/Auton.h"
#include "RobotData.h"

void Auton::RobotInit()
{
    //auton selector
    autonSelector.AddOption("Potato", AutonSelect::autonSelect_potato);
    autonSelector.SetDefaultOption("Potato", AutonSelect::autonSelect_potato);
}

void Auton::AutonomousInit(const RobotData &robotData, AutonData &autonData)
{
    // making sure the auton sequences start at the right step and setting the auton choice based on dashboard selection
    autonData.autonStep = 0;
    autonData.autonSelection = autonSelector.GetSelected();
    
}

void Auton::AutonomousPeriodic(const RobotData &robotData, AutonData &autonData, ControllerData &controllerData)
{
    // for debugging
    // frc::SmartDashboard::PutNumber("autonStep", autonData.autonStep);
    // frc::SmartDashboard::PutNumber("autonSelection", autonData.autonSelection);

    // for autons, the robot MUST be in semi auto mode

    switch(autonData.autonSelection)
    {
    case autonSelect_potato:
        // endAllTasks(robotData, controllerData);
        break;

    case autonSelect_exitInitLineDriverStation:

        switch (autonData.autonStep)
        {
        case 0:
            // robotData.desiredDBDist = -20;
            // robotData.driveMode = driveMode_initDriveStraight;
            break;
        case 1:
            // robotData.driveMode = driveMode_driveStraight;
            break;
        default:
            // endAllTasks(robotData);
            break;
        }
        break;

    case autonSelect_exitInitLineRendezvous:

        switch (autonData.autonStep)
        {
        case 0:
            // robotData.desiredDBDist = 20;
            // robotData.driveMode = driveMode_initDriveStraight;
            break;
        case 1:
            // robotData.driveMode = driveMode_driveStraight;
            break;
        default:
            // endAllTasks(robotData);
            break;
        }
        break;

    case autonSelect_shootAndDriveToDriverStation:

        switch (autonData.autonStep)
        {
        case 0:
            // robotData.sBBtn = true;
            // startDelay(10, robotData);
            // robotData.shootingMode = true;
            // robotData.driveMode = driveMode_potato;
            // robotData.autonStep++;
            break;
        case 1:
            // checkDelay(robotData);
            break;
        case 2:
            // robotData.shootingMode = false;
            // robotData.autonStep++;
            break;
        case 3:
            // robotData.desiredDBDist = -20;
            // robotData.driveMode = driveMode_initDriveStraight;
            break;
        case 4:
            // robotData.driveMode = driveMode_driveStraight;
            break;
        default:
            // endAllTasks(robotData);
            break;
        }
        break;

    case autonSelect_shootAndDriveToRendezvous:

        switch (autonData.autonStep)
        {
        case 0:
            // robotData.sBBtn = true;
            // startDelay(10, robotData);
            // robotData.shootingMode = true;
            // robotData.driveMode = driveMode_potato;
            // robotData.autonStep++;
            break;
        case 1:
            // checkDelay(robotData);
            break;
        case 2:
            //turn shooter off before driving away
            // robotData.shootingMode = false;
            // robotData.autonStep++;
            break;
        case 3:
            // robotData.desiredDBDist = 20;
            // robotData.driveMode = driveMode_initDriveStraight;
            break;
        case 4:
            // robotData.driveMode = driveMode_driveStraight;
            break;
        default:
            // endAllTasks(robotData);
            break;
        }
        break;

        default:
            // endAllTasks(robotData);
            break;
    }

}


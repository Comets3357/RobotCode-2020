#pragma once

#include <frc/smartdashboard/SendableChooser.h> 
#include <frc/smartdashboard/SmartDashboard.h>

struct RobotData;
struct ControllerData;
struct DrivebaseData;


enum AutonSelect
{
    autonSelect_potato,
    autonSelect_exitInitLineRendezvous,
    autonSelect_exitInitLineDriverStation,
    autonSelect_shootAndDriveToRendezvous,
    autonSelect_shootAndDriveToDriverStation
};

struct AutonData
{
    int autonStep = 0;
    AutonSelect autonSelection = autonSelect_potato;

    // drive straight
    bool driveStraightInitialized = false;
    int desiredDistance = 0;
    double initialLDBPos;
    double initialRDBPos;
    double initialAngle;

    

};

class Auton
{
public:
    void RobotInit();
    void AutonomousInit(const RobotData &robotData, AutonData &autonData);
    void AutonomousPeriodic(const RobotData &robotData, AutonData &autonData, ControllerData &controllerData, DrivebaseData &drivebaseData);

private:
    void startDelay(double duration, const RobotData &robotData);
    void checkDelay(const RobotData &robotData);
    void endAllTasks(const RobotData &robotData, ControllerData &controllerData);
    

    frc::SendableChooser<AutonSelect> autonSelector;


};
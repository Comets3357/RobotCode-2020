#pragma once

#include <frc/smartdashboard/SendableChooser.h> 
#include <frc/smartdashboard/SmartDashboard.h>

struct RobotData;
struct ControllerData;


enum AutonSelect
{
    autonSelect_potato,
    autonSelect_exitInitLineRendezvous,
    autonSelect_exitInitLineDriverStation,
    autonSelect_shootAndDriveToRendezvous,
    autonSelect_shootAndDriveToDriverStation
};

enum DriveMode
{
    driveMode_teleop,
    driveMode_potato
};

struct AutonData
{
    int autonStep = 0;
    AutonSelect autonSelection = autonSelect_potato;
};

class Auton
{
public:
    void RobotInit();
    void AutonomousInit(const RobotData &robotData, AutonData &autonData);
    void AutonomousPeriodic(const RobotData &robotData, AutonData &autonData, ControllerData &controllerData);

private:
    void startDelay(double duration, const RobotData &robotData);
    void checkDelay(const RobotData &robotData);
    void endAllTasks(const RobotData &RobotData, ControllerData &controllerData);

    frc::SendableChooser<AutonSelect> autonSelector;


};
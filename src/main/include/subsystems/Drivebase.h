#pragma once

#include "Constants.h"

#include <frc/DriverStation.h>
#include <frc/PowerDistributionPanel.h>
#include <frc/TimedRobot.h>
#include <rev/CANSparkMax.h>
#include <rev/CANEncoder.h>

struct RobotData;
struct AutonData;

enum DriveMode
{
    driveMode_teleop,
    driveMode_potato, 
    driveMode_driveStraight
};

struct DrivebaseData
{
    double currentLDBPos;
    double currentRDBPos;

    //read from the encoders
    double lDriveVel;
    double rDriveVel;

    //for autons
    DriveMode driveMode = driveMode_teleop;
    double setLVelocity;
    double setRVelocity;

    bool driveStraightInitialized = false;
    int desiredDistance = 0;
    double initialLDBPos;
    double initialRDBPos;
    double initialAngle;
};

class Drivebase
{

public:
    void RobotInit();
    void RobotPeriodic(const RobotData &robotData, DrivebaseData &drivebaseData, AutonData &autonData);
    void DisabledInit();

    //set during autons
    double lDrive;
    double rDrive;

private:
    const double cStraight = 1;
    const double cTurn = 0.4;

    void updateData(const RobotData &robotData, DrivebaseData &drivebaseData);
    void teleopControl(const RobotData &robotData); //vbus
    void autonControl(const RobotData &robotData); //velocity

    void potato(const RobotData &robotData);
    void driveStraight(const RobotData &robotData, DrivebaseData &drivebaseData, AutonData &autonData);

    void courseCorrection(bool isForward, const RobotData &robotData, DrivebaseData &drivebaseData);

    rev::CANSparkMax dbLM{leftLeadDeviceID,
                          rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax dbRM{rightLeadDeviceID,
                          rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax dbLS{leftFollowDeviceID,
                          rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax dbRS{rightFollowDeviceID,
                          rev::CANSparkMax::MotorType::kBrushless};

    rev::CANEncoder dbRMEncoder = dbRM.GetEncoder();
    rev::CANEncoder dbLMEncoder = dbLM.GetEncoder();
    rev::CANEncoder dbRSEncoder = dbRS.GetEncoder();
    rev::CANEncoder dbLSEncoder = dbLS.GetEncoder();
    rev::CANPIDController dbRMPID = dbRM.GetPIDController();
    rev::CANPIDController dbLMPID = dbLM.GetPIDController();
};
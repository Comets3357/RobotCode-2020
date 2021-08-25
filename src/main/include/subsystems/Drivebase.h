#pragma once

#include "Constants.h"

#include <frc/DriverStation.h>
#include <frc/PowerDistributionPanel.h>
#include <frc/TimedRobot.h>
#include <rev/CANSparkMax.h>
#include <rev/CANEncoder.h>

struct RobotData;

struct DrivebaseData
{
    double currentLDBPos;
    double currentRDBPos;

    double lDriveVel;
    double rDriveVel;

    //for autons

    double setLVelocity;
    double setRVelocity;

    bool driveStraightInitialized = false;
    bool driveStraightCompleted = false;
    int desiredDistance = 0;
    double initialLDBPos;
    double initialRDBPos;
    double initialAngle;
};

class Drivebase
{

public:
    void RobotInit();
    void RobotPeriodic(const RobotData &robotData, DrivebaseData &drivebaseData);
    void DisabledInit();

private:
    // double setPoint; //for pid tuning

    //set during autons
    double lDrive;
    double rDrive;

    const double cStraight = 1;
    const double cTurn = 0.4;

    double setPoint; //for pid tuning

    void updateData(const RobotData &robotData, DrivebaseData &drivebaseData);
    void teleopControl(const RobotData &robotData);
    void autonControl(const RobotData &robotData); //velocity

    // void pidTuningControl(const RobotData &robotData);

    void potato(const RobotData &robotData);
    void driveStraight(const RobotData &robotData, DrivebaseData &drivebaseData);

    void courseCorrection(bool isForward, const RobotData &robotData, DrivebaseData &drivebaseData);

    rev::CANSparkMax dbLM{leftLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax dbRM{rightLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax dbLS{leftFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax dbRS{rightFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless};

    rev::CANEncoder dbRMEncoder = dbRM.GetEncoder();
    rev::CANEncoder dbLMEncoder = dbLM.GetEncoder();
    rev::CANEncoder dbRSEncoder = dbRS.GetEncoder();
    rev::CANEncoder dbLSEncoder = dbLS.GetEncoder();
    rev::CANPIDController dbRMPID = dbRM.GetPIDController();
    rev::CANPIDController dbLMPID = dbLM.GetPIDController();
};
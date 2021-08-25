#pragma once

// includes other files' data
#include "Controller.h"
#include "common/OtherComponents.h"
#include "common/Gyro.h"
#include "common/Limelight.h"
#include "common/Timer.h"

#include "auton/Auton.h"

#include "subsystems/Drivebase.h"
#include "subsystems/Shooter.h"
#include "subsystems/Intake.h"
#include "subsystems/Indexer.h"


//could be separated into all separate files for the data *from* each subsystem
//commented out variables are not in use
struct RobotData
{
    ControllerData controllerData;
    OtherComponentsData otherComponentsData;
    GyroData gyroData;
    LimelightData limelightData;
    TimerData timerData;

    AutonData autonData;

    DrivebaseData drivebaseData;
    IntakeData intakeData;
    ShooterData shooterData;
    IndexerData indexerData;
};
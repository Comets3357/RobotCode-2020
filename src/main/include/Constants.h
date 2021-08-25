#pragma once

// drivebase
const int
    leftLeadDeviceID = 1,
    leftFollowDeviceID = 2,
    rightLeadDeviceID = 4,
    rightFollowDeviceID = 5;

// intake
const int
    intakePivotID = 11,
    intakeWheelsID = 12;
const double
    pkP = 0.02, pkI = 0, pkD = 0, pkIz = 0, pkFF = 0, pkMaxOutput = 1, pkMinOutput = -1,
    wkP = 1, wkI = 0, wkD = 0, wkIz = 0, wkFF = 0, wkMaxOutput = 1, wkMinOutput = -1;

// indexer
const int
    indexerBeltsID = 16,
    indexerFirstSensorDIO = 2,
    indexerGapSensorDIO = 3;

// shooter
const int
    shooterWheelMID = 21,
    shooterWheelSID = 22,
    shooterHoodID = 23,
    shooterKickID = 24,
    shooterTurretID = 25;

// climb
const int
    climbLiftID = 32;
    // probably not using this but maybe
    // climbLockPWM = 2;
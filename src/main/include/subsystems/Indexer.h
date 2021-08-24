#pragma once

#include "Constants.h"

#include <frc/TimedRobot.h>
#include <frc/DigitalInput.h>
#include <rev/CANSparkMax.h>
#include <rev/CANEncoder.h>

struct RobotData;

struct IndexerData
{
    int powerCellCount;
    int sensorTransition;

    double currentIndexerPos;
    double currentIndexerVel;

    bool currentFirstSensorState;
    bool currentGapSensorState;

    bool isFull = false;
};

class Indexer
{

public:
    void RobotInit();
    void RobotPeriodic(const RobotData &robotData, IndexerData &indexerData);

private:
    const double indexerBeltsSpeed = 0.25;
    const double ejectBallSpeed = 0.5;

    void updateData(const RobotData &robotData, IndexerData &indexerData);
    void manual(const RobotData &robotData, IndexerData &indexerData);
    void semiAuto(const RobotData &robotData, IndexerData &indexerData);

    frc::DigitalInput indexerFirstSensor{indexerFirstSensorDIO};
    frc::DigitalInput indexerGapSensor{indexerGapSensorDIO};

    rev::CANSparkMax indexerBelts{indexerBeltsID, rev::CANSparkMax::MotorType::kBrushless};

    rev::CANEncoder indexerBeltsEncoder = indexerBelts.GetEncoder();
    rev::CANPIDController indexerBeltsPID = indexerBelts.GetPIDController();
};
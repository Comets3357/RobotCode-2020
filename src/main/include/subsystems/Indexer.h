#pragma once

#include "Constants.h"

#include <frc/TimedRobot.h>
#include <rev/CANSparkMax.h>
#include <rev/CANEncoder.h>

struct RobotData;

struct IndexerData
{
    double currentIndexerPos;
    double currentIndexerVel;
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

    rev::CANSparkMax indexerBelts{indexerBeltsID, rev::CANSparkMax::MotorType::kBrushless};

    rev::CANEncoder indexerBeltsEncoder = indexerBelts.GetEncoder();
    rev::CANPIDController indexerBeltsPID = indexerBelts.GetPIDController();
};
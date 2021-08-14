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
    void updateData(const RobotData &robotData, IndexerData &indexerData);
    void teleopControl(const RobotData &robotData);

    rev::CANSparkMax indexerBelt{indexerBeltsID, rev::CANSparkMax::MotorType::kBrushless};

    rev::CANEncoder indexerBeltEncoder = indexerBelt.GetEncoder();
    rev::CANPIDController indexerBeltPID = indexerBelt.GetPIDController();
};
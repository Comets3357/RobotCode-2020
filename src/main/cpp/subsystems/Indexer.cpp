#include "subsystems/Indexer.h"
#include "RobotData.h"

void Indexer::RobotInit()
{
    indexerBelt.RestoreFactoryDefaults();

    indexerBelt.SetInverted(false);

    indexerBelt.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);

    indexerBelt.SetSmartCurrentLimit(45);
}

void Indexer::RobotPeriodic(const RobotData &robotData, IndexerData &indexerData)
{
    updateData(robotData, indexerData);
}

void Indexer::updateData(const RobotData &robotData, IndexerData &indexerData)
{
    indexerData.currentIndexerPos = indexerBeltEncoder.GetPosition();

    indexerData.currentIndexerVel = indexerBeltEncoder.GetVelocity();
}

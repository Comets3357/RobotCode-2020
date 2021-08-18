#include "subsystems/Indexer.h"
#include "RobotData.h"

void Indexer::RobotInit()
{
    indexerBelts.RestoreFactoryDefaults();

    indexerBelts.SetInverted(false);

    indexerBelts.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);

    indexerBelts.SetSmartCurrentLimit(45);
}

void Indexer::RobotPeriodic(const RobotData &robotData, IndexerData &indexerData)
{
    updateData(robotData, indexerData);

    if (robotData.controllerData.manualMode)
    {
        manual(robotData, indexerData);
    }
    else
    {
        semiAuto(robotData, indexerData);
    }
}

void Indexer::updateData(const RobotData &robotData, IndexerData &indexerData)
{
    indexerData.currentIndexerPos = indexerBeltsEncoder.GetPosition();

    indexerData.currentIndexerVel = indexerBeltsEncoder.GetVelocity();
}

void Indexer::manual(const RobotData &robotData, IndexerData &indexerData)
{
    if (robotData.controllerData.mIndexer)
    {
        indexerBelts.Set(indexerBeltsSpeed);
    }
    else if (robotData.controllerData.mIndexerBackwards)
    {
        indexerBelts.Set(-indexerBeltsSpeed);
    }
    else
    {
        indexerBelts.Set(0);
    }
}

void Indexer::semiAuto(const RobotData &robotData, IndexerData &indexerData)
{
    if (robotData.controllerData.saEjectBallsBackwards)
    {
        indexerBelts.Set(-ejectBallSpeed);
    }
}
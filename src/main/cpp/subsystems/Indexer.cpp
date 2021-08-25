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

    indexerData.currentFirstSensorState = indexerFirstSensor.Get();
    indexerData.currentGapSensorState = indexerGapSensor.Get();

    frc::SmartDashboard::PutNumber("ball count", indexerData.powerCellCount);
    frc::SmartDashboard::PutBoolean("is full", indexerData.isFull);
    frc::SmartDashboard::PutNumber("sensor transition", indexerData.sensorTransition);
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
        indexerData.isFull = false;
        indexerData.powerCellCount = 0;
        indexerBelts.Set(-ejectBallSpeed);
    }
    else if (robotData.shooterData.readyShoot)
    {
        indexerData.isFull = false;
        indexerData.powerCellCount = 0;
        indexerBelts.Set(indexerBeltsSpeed);
    }
    else if ((!indexerData.currentFirstSensorState || !indexerData.currentGapSensorState) && !indexerData.isFull)
    {
        indexerBelts.Set(indexerBeltsSpeed);

        if (!indexerData.currentFirstSensorState && indexerData.currentGapSensorState && indexerData.sensorTransition == 0)
        {
            indexerData.sensorTransition = 1;
        }
        else if (!indexerData.currentFirstSensorState && !indexerData.currentGapSensorState && indexerData.sensorTransition == 1)
        {
            indexerData.sensorTransition = 2;
        }
        else if (indexerData.currentFirstSensorState && !indexerData.currentGapSensorState && indexerData.sensorTransition == 2)
        {
            indexerData.sensorTransition = 3;
        }
        else if (indexerData.sensorTransition == 3)
        {
            indexerData.sensorTransition = 0;

            if (indexerData.powerCellCount < 3)
            {
                indexerData.powerCellCount++;
            }
            if (indexerData.powerCellCount == 3)
            {
                indexerData.isFull = true;
            }
        }
    }
    else
    {
        indexerData.sensorTransition = 0;
        indexerBelts.Set(0);
    }
}
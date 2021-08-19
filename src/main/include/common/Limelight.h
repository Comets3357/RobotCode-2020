#pragma once

#include "Constants.h"

#include <frc/TimedRobot.h>

struct RobotData;

struct LimelightData
{
    double xOffset;
    double yOffset;
    int targetValue;
    double calcHoodPos;
    bool validTarget;
    double calcTurretPos;
    int pipeline; //for LED power
};

class Limelight
{

public:
    void RobotInit();
    void RobotPeriodic(const RobotData &robotData, LimelightData &limelightData);
    double getHorizontalOffset();
    double getVerticalOffset();
    int getTarget();
    double calcHoodPOS(double verticalOffset, const RobotData &robotData);
    double calcTurretPOS(double horOffset);
    int getPipeline(double verticalOffset);

private:

};

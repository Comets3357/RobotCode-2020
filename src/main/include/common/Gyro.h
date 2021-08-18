#pragma once

#include <adi/ADIS16470_IMU.h>

struct GyroData
{
    double rawYaw = 0;
    double yaw = 0;

    double rawPitch = 0;
    double pitch = 0;

    double rawRoll = 0;
    double roll = 0;
};

class Gyro
{

public:
    void RobotInit(GyroData &gyroData);
    void RobotPeriodic(GyroData &gyroData);



private:
    frc::ADIS16470_IMU gyro{};
    void updateData(GyroData &gyroData);

};
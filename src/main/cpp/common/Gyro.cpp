#include "common/Gyro.h"


void Gyro::RobotInit(GyroData &gyroData){
    gyro.Calibrate();
    // gyro.SetYawAxis(frc::ADIS16470_IMU::kZ);
}

void Gyro::RobotPeriodic(GyroData &gyroData){
    updateData(gyroData);
}

void Gyro::updateData(GyroData &gyroData){
    // don't know which functions to use yet
    // continuous angle
    gyroData.rawYaw = gyro.GetGyroInstantZ();
    // gyroData.rawYaw = gyro.GetYawAxis();

    // 0 - 360 angle
    double tempRobotAngle = gyroData.rawYaw;
    while(tempRobotAngle >= 360){
        tempRobotAngle -= 360;
    }
    while(tempRobotAngle <= 0){
        tempRobotAngle += 360;
    }
    gyroData.yaw = tempRobotAngle;

}
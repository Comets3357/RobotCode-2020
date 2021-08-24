#pragma once

#include <rev/CANSparkMax.h>
#include <frc/Solenoid.h>
#include <frc/DoubleSolenoid.h>
#include <adi/ADIS16448_IMU.h>

#include "RobotData.h"

#include <frc/smartdashboard/SmartDashboard.h>

class Climb {
    
    public:

        void RobotInit();
        void Periodic(const RobotData &robotData);

        void semiAutoMode(const RobotData &robotData);
        void manualMode(const RobotData &robotData);

        bool climbInitiated;
        bool climbRunning;

        // Change SparkMax IDs
        static const int climbLiftID = 32;

        rev::CANSparkMax climbLift{32, rev::CANSparkMax::MotorType::kBrushless};

        rev::CANEncoder climbLiftPos = climbLift.GetEncoder();
};
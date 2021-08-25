#pragma once

#include "Constants.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/CANEncoder.h>
#include <rev/CANSparkMax.h>
#include <rev/CANPIDController.h>

struct RobotData;

struct IntakeData
{
};

class Intake
{

public:
    void RobotInit();
    void RobotPeriodic(const RobotData &robotData, IntakeData &intakeData);

private:
    const double intakePivotSpeed = 0.3;
    const double intakeRollersSpeed = 0.3;
    const double intakeRollersEjectSpeed = 0.5;

    void manual(const RobotData &robotData, IntakeData &intakeData);
    void semiAuto(const RobotData &robotData, IntakeData &intakeData);

    rev::CANSparkMax intakeRollers{intakeWheelsID, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax intakePivot{intakePivotID, rev::CANSparkMax::MotorType::kBrushless};

    rev::CANEncoder intakePivotEncoder = intakePivot.GetEncoder();
    rev::CANEncoder intakeRollersEncoder = intakeRollers.GetEncoder();

    rev::CANPIDController intakePivot_pidController = intakePivot.GetPIDController();
    rev::CANPIDController intakeRollers_pidController = intakeRollers.GetPIDController();
};
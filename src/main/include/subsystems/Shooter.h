#pragma once

#include "Constants.h"

#include <frc/PowerDistributionPanel.h>
#include <frc/TimedRobot.h>
#include <rev/CANSparkMax.h>
#include <rev/CANEncoder.h>

struct RobotData;

struct ShooterData
{
    bool shooting = false;

    int secondaryPOVArrayInput;
    int turretSequence = 0;
    int shootPOV;
    double turretSnapshot = 0;

    bool intakeEncoderPositionZero;

    double hoodPosition;
    double turretPosition;
    double flywheelVelocity;
    static const int shootingBtn = 0;
    int targetVelocity = 0;
    bool readyShoot = false; //when flywheel reaches velocity and everything is aimed
    int roughAim;
    int roughHood;
    bool stopAntiJam = false;
    bool isZero = false;

};

class Shooter
{

public:
    void RobotInit();
    void RobotPeriodic(const RobotData &robotData, ShooterData &shooterData);
    void DisabledInit();
        
    


private:
    void updateData(const RobotData &robotData, ShooterData &shooterData);
    //void teleopControl(const RobotData &robotData);
    double getHoodPos();
    double getTurretPos();
    double getWheelPos();
    double getWheelVel();
    double getHoodOffset();
    bool getTurretLimitSwitch();
    bool getHoodLimitSwitch();

    double turretSnapshot;

    void semiAutoMode(const RobotData &robotData, ShooterData &shooterData);
    void manualMode(const RobotData &robotData);

    void setShooterPID(rev::CANPIDController motor, int pidSlot, double p, double i, double d, double ff);
    void setHood(double power);
    void setTurret(double power);
    void setWheel(double power);
    void setHoodPos(double pos);
    void setTurretPos(double pos); 
    
    //void updateDiagnostics(DiagnosticsData &diagnosticsData);

    void semiAutoMode(RobotData &robotData);
    void manualMode(RobotData &robotData);   


    rev::CANSparkMax shooterFlywheelM{shooterWheelMID, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax shooterFlywheelS{shooterWheelSID, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax shooterHood{shooterHoodID, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax shooterKick{shooterKickID, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax shooterTurret{shooterTurretID, rev::CANSparkMax::MotorType::kBrushless};

    rev::CANEncoder shooterHoodPOS = shooterHood.GetEncoder();
    rev::CANEncoder shooterTurretPOS = shooterTurret.GetEncoder();
    rev::CANEncoder shooterWheelMPOS = shooterFlywheelM.GetEncoder();
    rev::CANEncoder shooterWheelSPOS = shooterFlywheelS.GetEncoder();
    rev::CANEncoder shooterKickPOS = shooterKick.GetEncoder();

    //limit switches;
        rev::CANDigitalInput turretReverseLimit = shooterTurret.GetReverseLimitSwitch(rev::CANDigitalInput::LimitSwitchPolarity::kNormallyClosed);
        rev::CANDigitalInput hoodReverseLimit = shooterHood.GetReverseLimitSwitch(rev::CANDigitalInput::LimitSwitchPolarity::kNormallyClosed);

    rev::CANPIDController shooterFlywheelM_pidController = shooterFlywheelM.GetPIDController();
    rev::CANPIDController shooterFlywheelS_pidController = shooterFlywheelS.GetPIDController();
    rev::CANPIDController shooterHood_pidController = shooterHood.GetPIDController();
    rev::CANPIDController shooterTurret_pidController = shooterTurret.GetPIDController();
    rev::CANPIDController shooterKick_pidController = shooterKick.GetPIDController();
};
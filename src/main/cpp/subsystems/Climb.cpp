#include "subsystems/Climb.h"
#include "Robot.h"


void Climb::RobotInit(){
    //zeroing encoders
    climbLiftPos.SetPosition(0);

    climbLift.RestoreFactoryDefaults();
    climbLift.SetInverted(true);
    climbLift.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    climbLift.SetSmartCurrentLimit(45);
    climbLift.Set(0);

}

void Climb::Periodic(const RobotData &robotData){

    if (robotData.controllerData.climbMode){
        if (robotData.controllerData.manualMode) {
            manualMode(robotData);
        } else {
            semiAutoMode(robotData);
        }
    }
    if (robotData.controllerData.manualMode) {
            manualMode(robotData);
        } else {
            semiAutoMode(robotData);
        }
}

void Climb::manualMode(const RobotData &robotData){

    climbLift.Set(robotData.controllerData.sLYStick);
    frc::SmartDashboard::PutBoolean("climb manual", true);
    frc::SmartDashboard::PutNumber("climb pos", climbLiftPos.GetPosition());

}

void Climb::semiAutoMode(const RobotData &robotData){
    frc::SmartDashboard::PutBoolean("climb auto", true);
    frc::SmartDashboard::PutNumber("climb pos", climbLiftPos.GetPosition());
    //climbLift.Set(robotData.controllerData.sLYStick);
    frc::SmartDashboard::PutBoolean("climbrun", climbRunning);
    if (robotData.controllerData.sXBtn) {
        climbRunning = true;
        frc::SmartDashboard::PutBoolean("climb", true);
    }
    if (climbRunning && climbInitiated) {
        if (climbLiftPos.GetPosition() > -210) {
            climbLift.Set(-1);
        } else {
            climbLift.Set(0);
            climbInitiated = false;
            climbRunning = false;
        }
    } else if (!climbInitiated && climbRunning){
        if (climbLiftPos.GetPosition() < 0) {
            climbLift.Set(1);
        } else {
            climbLift.Set(0);
            climbInitiated = true;
            climbRunning = false;
        }
    }
        
}
#include "subsystems/Climb.h"
#include "Robot.h"


void ClimbSubsystem::RobotInit(){
    //zeroing encoders
    climbLiftPos.SetPosition(0);

    climbLift.RestoreFactoryDefaults();
    climbLift.SetInverted(true);
    climbLift.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    climbLift.SetSmartCurrentLimit(45);
    climbLift.Set(0);
}

void ClimbSubsystem::Periodic(const RobotData &robotData){
    
    if (robotData.controllerData.manualMode) {
        manualMode(robotData);
    } else {
        semiAutoMode(robotData);
    }


}

void ClimbSubsystem::manualMode(const RobotData &robotData){



}

void ClimbSubsystem::semiAutoMode(const RobotData &robotData){

    if (robotData.controllerData.sXBtn) {
        climbRunning = true;
    }
    if (climbRunning && climbInitiated) {
        if (climbLiftPos.GetPosition() > 10) {
            climbLift.Set(-0.1);
        } else {
            climbLift.Set(0);
            climbInitiated = false;
            climbRunning = false;
        }
    } else if (!climbInitiated && climbRunning){
        if (climbLiftPos.GetPosition() < 100) {
            climbLift.Set(0.1);
        } else {
            climbLift.Set(0);
            climbInitiated = true;
            climbRunning = false;
        }
    }
    if (robotData.controllerData.sYBtn && climbInitiated) {
        if (climbLiftPos.GetPosition() < 50) {
            climbLift.Set(0.1);
        } else {
            climbLift.Set(0);
        }
    } else {
        climbLift.Set(0);
    }
        
}
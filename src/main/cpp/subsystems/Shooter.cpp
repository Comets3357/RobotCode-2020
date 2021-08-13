#include "subsystems/Shooter.h"
#include "RobotData.h"

#include <frc/Driverstation.h>
#include <frc/smartdashboard/SmartDashboard.h>

void Shooter::RobotInit()
{
    shooterFlywheelM.RestoreFactoryDefaults();
    shooterFlywheelS.RestoreFactoryDefaults();
    shooterHood.RestoreFactoryDefaults();
    shooterTurret.RestoreFactoryDefaults();

    shooterFlywheelS.SetInverted(true);
    shooterFlywheelM.SetInverted(true);
    shooterTurret.SetInverted(false);
    shooterHood.SetInverted(true);

    /**
     * note:
     * the shooter hood isn't inverted the right way
     * a positive set value moves the hood down rather than up
     * but don't change it because it'll mess up the limit switch 
     * easier to just do it this way then rewiring the entire limitswitch
     * 
     * update:: could now be false i dont know anymore
     */

    shooterFlywheelS.Follow(shooterFlywheelM);

    shooterFlywheelM.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    shooterFlywheelS.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    shooterHood.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    shooterTurret.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    shooterFlywheelM.SetSmartCurrentLimit(30);

    shooterFlywheelS.SetSmartCurrentLimit(30);
    shooterHood.SetSmartCurrentLimit(45);
    shooterTurret.SetSmartCurrentLimit(45);

    // setShooterPID(shooterWheelMPID, 0, 0.0012, 0, 0.02, 0.0002); //first pid for high velocity shooting
    // setShooterPID(shooterWheelSPID, 1, 0, 0, 0, 0.0002); //second pid for low constant velocity 
    // setShooterPID(shooterHoodPID, 0, 0.1, 0, 0, 0);
    // setShooterPID(shooterTurretPID, 0, 0.09, 0, 1, 0);

    shooterHoodPOS.SetPosition(0);
    shooterTurretPOS.SetPosition(0);
    shooterWheelMPOS.SetPosition(0);
    shooterWheelSPOS.SetPosition(0);

    shooterFlywheelM.BurnFlash();
    shooterFlywheelS.BurnFlash();
    shooterHood.BurnFlash();
    shooterTurret.BurnFlash();

    //rev::CANSparkMaxLowLevel::EnableExternalUSBControl(true);

    //zeros hood at the begining
    setHood(-0.1);
    if(getHoodLimitSwitch()){
        setHoodPos(0);
        setHood(0);
    }
} 


void Shooter::RobotPeriodic(const RobotData &robotData, ShooterData &shooterData)
{
    updateData(robotData, shooterData);

    // frc::SmartDashboard::PutNumber("hood Position",  getHoodPos()); 
    // frc::SmartDashboard::PutNumber("turret Position",  getTurretPos()); 
    // frc::SmartDashboard::PutNumber("calc hood pos",  robotData.calcHoodPos); 

    updateData(robotData, shooterData);
    semiAutoMode(robotData, shooterData);
    // if(!robotData.climbMode){
    //     if(robotData.manualMode){
    //         manualMode(robotData);
    //     } else {
    //         semiAutoMode(robotData);
    //     }
    // }else{
    //     setHood(0);
    //     setTurret(0);
    //     setWheel(0);
    // }

    //updateDiagnostics(diagnosticsData);


}

//updates the robotData struct with the flywheel velocity, turret position, and hood position
void Shooter::updateData(const RobotData &robotData, ShooterData &shooterData){
    shooterData.flywheelVelocity = getWheelVel();
    shooterData.turretPosition = getTurretPos();
    shooterData.hoodPosition = getHoodPos();
}


void Shooter::semiAutoMode(const RobotData &robotData, ShooterData &shooterData){

   //retreive controller input

    //if you're shooting 
    if(getTurretLimitSwitch()){ //for the beginning of the math zero the turret 
        setTurretPos(0);
        shooterData.isZero = true;
    }else if(!robotData.shooterData.isZero){
        setTurret(-0.1);
    }else if(robotData.shooterData.isZero){
        
        //adding the two left/right pov buttons to turn the turret left/right
        if (robotData.controllerData.shootingMode){ 
            turretSnapshot = getTurretPos();

            //if we're close to the target the velocity doesn't need to be as high, gets us shooting faster
            if(robotData.limelightData.yOffset > 5){
                shooterData.targetVelocity = 2400;
            }else{
                shooterData.targetVelocity = 3000;
            }
        
            //if the bot can see a target
            if(robotData.limelightData.targetValue != 0){

                //Use PID to set Hood and Turret based off limelight values
                shooterHood_pidController.SetReference(robotData.limelightData.calcHoodPos, rev::ControlType::kPosition);
                shooterTurret_pidController.SetReference(robotData.limelightData.calcTurretPos + getTurretPos(), rev::ControlType::kPosition);
                
                //uses PID to get the shooter wheel up to speed and stay there
                shooterFlywheelM_pidController.SetReference(3400, rev::ControlType::kVelocity);
                
                if (getWheelVel() > robotData.shooterData.targetVelocity - 0)
                {
                    shooterData.stopAntiJam = true;
                }
                else {
                    shooterData.stopAntiJam = false;
                }

                //once the shooter has high enough velocity and is aimed correctly tell robot to begin shooting (start indexer)
                if ((getWheelVel() > robotData.shooterData.targetVelocity) && (std::abs(getTurretPos() - (turretSnapshot + robotData.limelightData.calcTurretPos)) <= 1) && (std::abs(getHoodPos() - robotData.limelightData.calcHoodPos) <= 2) ){
                    shooterData.readyShoot = true;
                }else{
                    shooterData.readyShoot = false;
                }

            }
            //robotData.isZero = false;
            

        } else {  //not shooting

            //set the turret to face forward
            shooterTurret_pidController.SetReference(12 + (robotData.shooterData.roughAim*4.5), rev::ControlType::kPosition);
            //spins up flywheel beforehand
            if(robotData.controllerData.sBBtn){
                shooterFlywheelM_pidController.SetReference(3400, rev::ControlType::kVelocity);
            }else{
                if(getWheelVel() < 1200){ //once the flywheel reaches a low enough velocity begin constant velociy
                    shooterFlywheelM_pidController.SetReference(1000, rev::ControlType::kVelocity, 1); //uses second pid
                }else{
                    setWheel(0); //starts the shooting wheel slowing down
                }


            }

            

            shooterData.readyShoot = false;

            //zeros the hood after
            setHood(-0.2);
            if(getHoodLimitSwitch()){
                setHoodPos(0);
                setHood(0);
            }

        }



    }

}

void Shooter::manualMode(RobotData &robotData){
    
    // frc::SmartDashboard::PutNumber("turret pos", getTurretPos());

    //make hood and turret moveable by joystick
    setTurret(robotData.controllerData.sLYStick*.1);

    if(robotData.controllerData.sBBtn){
        //spins the flywheel up beforehand
        shooterFlywheelM_pidController.SetReference(3400, rev::ControlType::kVelocity);
    }else{
        setWheel(0);
        setHood(robotData.controllerData.sRYStick*.1);
    }
}

void Shooter::setHoodPos(double pos){
    shooterHoodPOS.SetPosition(pos);
}
void Shooter::setTurretPos(double pos){
    shooterTurretPOS.SetPosition(pos);
}

double Shooter::getHoodPos(){
    return shooterHoodPOS.GetPosition();
}
double Shooter::getTurretPos(){
    return shooterTurretPOS.GetPosition();
}
double Shooter::getWheelPos(){
    return shooterWheelMPOS.GetPosition();
} 

bool Shooter::getTurretLimitSwitch(){
    return turretReverseLimit.Get();
}
bool Shooter::getHoodLimitSwitch(){
    return hoodReverseLimit.Get();
}

void Shooter::setHood(double power){
    shooterHood.Set(power);
}
void Shooter::setTurret(double power){
    shooterTurret.Set(power);
}
void Shooter::setWheel(double power){
    shooterFlywheelM.Set(power);
}

double Shooter::getWheelVel(){
    return shooterWheelMPOS.GetVelocity();
}





// void ShooterSubsystem::updateDiagnostics(DiagnosticsData &diagnosticsData)
// {
//     /**
//      * turret rotate 23
//      * hood 22
//      * shooter m 20
//      * shooter s 21
//      * 
//      * limit switches
//      */

//     diagnosticsData.mControlCurrents.at(23) = shooterTurret.GetOutputCurrent();
//     diagnosticsData.mControlVoltages.at(23) = shooterTurret.GetBusVoltage();
//     diagnosticsData.mControlTemps.at(23) = shooterTurret.GetMotorTemperature();

//     diagnosticsData.mControlPositions.at(23) = shooterTurretPOS.GetPosition();
//     diagnosticsData.mControlVelocities.at(23) = shooterTurretPOS.GetVelocity();

//     diagnosticsData.mControlFaults.at(23) = shooterTurret.GetFaults();


//     diagnosticsData.mControlCurrents.at(22) = shooterHood.GetOutputCurrent();
//     diagnosticsData.mControlVoltages.at(22) = shooterHood.GetBusVoltage();
//     diagnosticsData.mControlTemps.at(22) = shooterHood.GetMotorTemperature();

//     diagnosticsData.mControlPositions.at(22) = shooterHoodPOS.GetPosition();
//     diagnosticsData.mControlVelocities.at(22) = shooterHoodPOS.GetVelocity();

//     diagnosticsData.mControlFaults.at(22) = shooterHood.GetFaults();


//     diagnosticsData.mControlCurrents.at(20) = shooterWheelM.GetOutputCurrent();
//     diagnosticsData.mControlVoltages.at(20) = shooterWheelM.GetBusVoltage();
//     diagnosticsData.mControlTemps.at(20) = shooterWheelM.GetMotorTemperature();

//     diagnosticsData.mControlPositions.at(20) = shooterWheelMPOS.GetPosition();
//     diagnosticsData.mControlVelocities.at(20) = shooterWheelMPOS.GetVelocity();

//     diagnosticsData.mControlFaults.at(20) = shooterWheelM.GetFaults();


//     diagnosticsData.mControlCurrents.at(21) = shooterWheelS.GetOutputCurrent();
//     diagnosticsData.mControlVoltages.at(21) = shooterWheelS.GetBusVoltage();
//     diagnosticsData.mControlTemps.at(21) = shooterWheelS.GetMotorTemperature();

//     diagnosticsData.mControlPositions.at(21) = shooterWheelSPOS.GetPosition();
//     diagnosticsData.mControlVelocities.at(21) = shooterWheelSPOS.GetVelocity();

//     diagnosticsData.mControlFaults.at(21) = shooterWheelS.GetFaults();


//     diagnosticsData.turretLSwitch = getTurretLimitSwitch();
//     diagnosticsData.hoodLSwitch = getHoodLimitSwitch();
// }

void Shooter::DisabledInit(){
    setHood(0);
    setTurret(0);
    setWheel(0);
}

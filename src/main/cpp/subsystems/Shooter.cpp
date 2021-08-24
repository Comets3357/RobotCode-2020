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
    shooterKick.RestoreFactoryDefaults();


    shooterFlywheelS.SetInverted(true);
    shooterFlywheelM.SetInverted(true);
    shooterTurret.SetInverted(false);
    shooterHood.SetInverted(true);
    shooterKick.SetInverted(false);

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
    shooterKick.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    shooterFlywheelM.SetSmartCurrentLimit(30);
    shooterFlywheelS.SetSmartCurrentLimit(30);
    shooterHood.SetSmartCurrentLimit(45);
    shooterTurret.SetSmartCurrentLimit(45);
    shooterKick.SetSmartCurrentLimit(40);



    setShooterPID(shooterFlywheelM_pidController, 0, 0.0007, 0, 0, 0.00025); //first pid for high velocity shooting
    setShooterPID(shooterFlywheelM_pidController, 1, 0, 0, 0, 0.0002); //second pid for low constant velocity 
    setShooterPID(shooterHood_pidController, 0, 0.085, 0, 0.0001, 0);
    setShooterPID(shooterTurret_pidController, 0, 0.3, 0, 1, 0);
    setShooterPID(shooterKick_pidController, 0, 0, 0, 0, 0);

    shooterHoodPOS.SetPosition(0);
    shooterTurretPOS.SetPosition(0);
    shooterWheelMPOS.SetPosition(0);
    shooterWheelSPOS.SetPosition(0);
    shooterKickPOS.SetPosition(0);

    shooterFlywheelM.BurnFlash();
    shooterFlywheelS.BurnFlash();
    shooterHood.BurnFlash();
    shooterTurret.BurnFlash();
    shooterKick.BurnFlash();

    //rev::CANSparkMaxLowLevel::EnableExternalUSBControl(true);

    setHoodPos(0);
    setTurretPos(0);

} 


void Shooter::RobotPeriodic(const RobotData &robotData, ShooterData &shooterData)
{
    updateData(robotData, shooterData);
    frc::SmartDashboard::PutNumber("hood Position",  getHoodPos()); 
    frc::SmartDashboard::PutNumber("turret Position",  getTurretPos()); 
    frc::SmartDashboard::PutNumber("x axis",  robotData.limelightData.xOffset); 

    //frc::SmartDashboard::PutNumber("calc hood pos",  robotData.calcHoodPos); 

    // if(!robotData.controllerData.climbMode){
        
    // }else{
    //     setHood(0);
    //     setTurret(0);
    //     setWheel(0);
    //     setKick(0);

    // }

    if(robotData.controllerData.manualMode){
        manualMode(robotData);
    } else {
        semiAutoMode(robotData, shooterData);
    }

}

//updates the robotData struct with the flywheel velocity, turret position, and hood position
void Shooter::updateData(const RobotData &robotData, ShooterData &shooterData){
    shooterData.flywheelVelocity = getWheelVel();
    shooterData.turretPosition = getTurretPos();
    shooterData.hoodPosition = getHoodPos();
    frc::SmartDashboard::PutNumber("Turret Potentiometer", getTurretPot());

}


void Shooter::semiAutoMode(const RobotData &robotData, ShooterData &shooterData){

   //retreive controller input

    if (robotData.controllerData.shootingMode){ 
        //turretSnapshot = getTurretPos();

        if(robotData.limelightData.yOffset > 5){
            shooterData.targetVelocity = -2400;
        }else{
            shooterData.targetVelocity = -3000;
        }
    
        //if the bot can see a target
        if(robotData.limelightData.targetValue != 0){

            //Use PID to set Hood and Turret based off limelight values
            //shooterHood_pidController.SetReference(robotData.limelightData.calcHoodPos, rev::ControlType::kPosition);
            //shooterTurret_pidController.SetReference(robotData.limelightData.calcTurretPos + getTurretPos(), rev::ControlType::kPosition);
            if(robotData.limelightData.xOffset > 3){
                setTurret(-0.1);
            }else if (robotData.limelightData.xOffset < 1){
                setTurret(0.1);
            }else{
                //setTurret(0);
                shooterTurret_pidController.SetReference(robotData.limelightData.calcTurretPos + getTurretPos(), rev::ControlType::kPosition);


                shooterHood_pidController.SetReference(robotData.limelightData.calcHoodPos, rev::ControlType::kPosition);
                shooterFlywheelM_pidController.SetReference(-3400, rev::ControlType::kVelocity);

                //once the shooter has high enough velocity and is aimed correctly tell robot to begin shooting (start indexer)
                if ((getWheelVel() > robotData.shooterData.targetVelocity) && (std::abs(getTurretPos() - (turretSnapshot + robotData.limelightData.calcTurretPos)) <= 2) && (std::abs(getHoodPos() - robotData.limelightData.calcHoodPos) <= 2)){
                    shooterData.readyShoot = true;
                    setKick(0.5);
                }else{
                    shooterData.readyShoot = false;
                    setKick(0);
                }
            }
                 

        }
        shooterData.isZero = false;            




    } else {  //not shooting
        setWheel(0);
        setKick(0);
        setTurret(0);



        //set the turret to face forward
        //shooterTurret_pidController.SetReference(12 + (robotData.shooterData.roughAim*4.5), rev::ControlType::kPosition);
        //spins up flywheel beforehand
        if(robotData.controllerData.mShooterFlyWheel){
            shooterFlywheelM_pidController.SetReference(-3400, rev::ControlType::kVelocity);
        }else{
            if(spinup){
                if(getWheelVel() > -1200){ //once the flywheel reaches a low enough velocity begin constant velociy
                    shooterFlywheelM_pidController.SetReference(-1000, rev::ControlType::kVelocity, 1); //uses second pid
                }else{
                    setWheel(0); //starts the shooting wheel slowing down
                }
            }
            else{
                setWheel(0); //starts the shooting wheel slowing down
            }
        }
    
        shooterData.readyShoot = false;

        //zeros the hood after
        
        if(!robotData.shooterData.isZero){
            if(std::abs(currentSnapshot - getHoodPos()) < 0.1){
                if(tickcount > 20){
                    setHoodPos(0);
                    setHood(0);
                    shooterData.isZero = true;
                }
            }else{
                setHood(-0.05);
                currentSnapshot = getHoodPos();
            }
            tickcount = (tickcount+1);
            
        }else{
            tickcount = 0;
        }

        // setHood(-0.05);
        // if(getHoodPos() < 1){
        //     setHood(0);
        //     setHoodPos(0);
        // }


        

    }

    

}

void Shooter::manualMode(const RobotData &robotData){
    
    //make hood and turret moveable by joystick
    setTurret(robotData.controllerData.mSetTurret*.2);

    if(robotData.controllerData.mShooterFlyWheel){
        //spins the flywheel up beforehand
        shooterFlywheelM_pidController.SetReference(-3400, rev::ControlType::kVelocity);
    }else{
        setWheel(0);
        setHood(robotData.controllerData.mSetHood*.2);
    }

    if(robotData.controllerData.sABtn){
        setTurretPos(0);
        setHoodPos(0);
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
void Shooter::setHood(double power){
    shooterHood.Set(power);
}
void Shooter::setTurret(double power){
    shooterTurret.Set(power);
}
void Shooter::setWheel(double power){
    shooterFlywheelM.Set(power);
}
void Shooter::setKick(double power){
    shooterKick.Set(power);
}
double Shooter::getWheelVel(){
    return shooterWheelMPOS.GetVelocity();
}
double Shooter::getTurretPot(){
    return turretPot.Get();
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

/**
 * Sets all the PID values for specific motor
 * @param motor name of the PID controller
 */
void Shooter::setShooterPID(rev::CANPIDController motor, int pidSlot, double p, double i, double d, double ff){
    motor.SetP(p, pidSlot);
    motor.SetI(i, pidSlot);
    motor.SetD(d, pidSlot);
    motor.SetFF(ff, pidSlot);
}

void Shooter::DisabledInit(){
    setHood(0);
    setTurret(0);
    setWheel(0);
    setKick(0);

}
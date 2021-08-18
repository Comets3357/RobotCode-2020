#include "subsystems/Drivebase.h"
#include "RobotData.h"

void Drivebase::RobotInit()
{
    dbLM.RestoreFactoryDefaults();
    dbRM.RestoreFactoryDefaults();

    dbLS.RestoreFactoryDefaults();
    dbRS.RestoreFactoryDefaults();

    dbLM.SetInverted(true);
    dbRM.SetInverted(false);

    dbLS.Follow(dbLM);
    dbRS.Follow(dbRM);

    dbRM.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    dbRS.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    dbLM.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    dbLS.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);

    dbLM.SetSmartCurrentLimit(45);
    dbRM.SetSmartCurrentLimit(45);
    dbLS.SetSmartCurrentLimit(45);
    dbRS.SetSmartCurrentLimit(45);

    dbLM.Set(0);
    dbRM.Set(0);

    //again, used for pid testing
    dbLM.BurnFlash();
    dbRM.BurnFlash();
    dbLS.BurnFlash();
    dbRS.BurnFlash();
}

void Drivebase::RobotPeriodic(const RobotData &robotData, DrivebaseData &drivebaseData, AutonData &autonData)
{
    updateData(robotData, drivebaseData);

    if (frc::DriverStation::GetInstance().IsEnabled())
    {
        dbRM.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
        dbRS.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
        dbLM.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
        dbLS.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    }

    
    
    switch (drivebaseData.driveMode)
    {
        case driveMode_teleop:
            teleopControl(robotData);
            break;
        case driveMode_potato:
            potato(robotData);
            break;
        case driveMode_driveStraight:
            driveStraight(robotData, drivebaseData, autonData);
            break;
        default:
            potato(robotData);
            break;
    }
}

void Drivebase::DisabledInit()
{
    dbLM.Set(0);
    dbRM.Set(0);
    dbRM.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    dbRS.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    dbLM.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    dbLS.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
}

// updates encoder and gyro values
void Drivebase::updateData(const RobotData &robotData, DrivebaseData &drivebaseData)
{
    //add back wheel encoders at some point
    drivebaseData.currentLDBPos = dbLMEncoder.GetPosition();
    drivebaseData.currentRDBPos = dbRMEncoder.GetPosition();

    drivebaseData.lDriveVel = dbRMEncoder.GetVelocity();
    drivebaseData.rDriveVel = dbLMEncoder.GetVelocity();
}
// driving functions:

// adjusts for the deadzone and converts joystick input to velocity values for PID
void Drivebase::teleopControl(const RobotData &robotData)
{
    double tempLDrive = robotData.controllerData.lDrive;
    double tempRDrive = robotData.controllerData.rDrive;

    // converts from tank to arcade drive, limits the difference between left and right drive
    double frontBack = cStraight * (tempLDrive + tempRDrive) / 2;
    double leftRight = cTurn * (tempRDrive - tempLDrive) / 2;

    //deadzone NOT needed for drone controller
    if (tempLDrive <= -0.08 || tempLDrive >= 0.08)
    {
        tempLDrive = (frontBack - leftRight);
    }
    else
    {
        tempLDrive = 0;
    }

    if (tempRDrive <= -0.08 || tempRDrive >= 0.08)
    {
        tempRDrive = (frontBack + leftRight);
    }
    else
    {
        tempRDrive = 0;
    }

    if (robotData.controllerData.dbInverted)
    {
        tempLDrive *= -1;
        tempRDrive *= -1;
    }

    //set as percent vbus
    dbLM.Set(tempLDrive);
    dbRM.Set(tempRDrive);

}

void Drivebase::autonControl(const RobotData &robotData){
    dbLMPID.SetReference(lDrive, rev::ControlType::kVelocity);
    dbRMPID.SetReference(rDrive, rev::ControlType::kVelocity);  
}

void Drivebase::potato(const RobotData &robotData){
    lDrive = 0;
    rDrive = 0;
    autonControl(robotData);
}

void Drivebase::driveStraight(const RobotData &robotData, DrivebaseData &drivebaseData, AutonData &autonData){
    if(!drivebaseData.driveStraightInitialized){
        drivebaseData.initialLDBPos = drivebaseData.currentLDBPos;
        drivebaseData.initialRDBPos = drivebaseData.currentRDBPos;
        drivebaseData.initialAngle = robotData.gyroData.rawYaw;
        drivebaseData.driveStraightInitialized = true;
    }

    double lDistLeft = drivebaseData.desiredDistance - (robotData.drivebaseData.currentLDBPos - drivebaseData.initialLDBPos);
    double rDistLeft = drivebaseData.desiredDistance - (robotData.drivebaseData.currentRDBPos - drivebaseData.initialRDBPos);


    if (drivebaseData.desiredDistance > 0){ // if driving forward
        if(lDistLeft > 0){
            if(lDistLeft * 100 < 5000){
                lDrive = lDistLeft * 100;
            } else {
                lDrive = 5000;
            }
            if (lDrive < 400)
            {
                lDrive = 400;
            }
        } else {
            lDrive = 0;
        }

        if(rDistLeft > 0){
            if(rDistLeft * 100 < 5000){
                rDrive = rDistLeft * 100;
            } else {
                rDrive = 5000;
            } 
            if (rDrive < 400)
            {
                rDrive = 400;
            }
        } else {
            rDrive = 0;
        }

        courseCorrection(true, robotData, drivebaseData);

        if (lDistLeft <= .5 && rDistLeft <= .5) {
            drivebaseData.driveStraightInitialized = false;
            autonData.autonStep++;
        }
    } else {
         if(lDistLeft < 0){
            if(lDistLeft * 170 > -5000){
                lDrive = lDistLeft * 170;
            } else {
                lDrive = -5000;
            } 
            if (lDrive > -400)
            {
                lDrive = -400;
            }
        } else {
            lDrive = 0;
        }

        if(rDistLeft < 0){
            if(rDistLeft * 170 > -5000){
                rDrive = rDistLeft * 170;
            } else {
                rDrive = -5000;
            }
            if (rDrive > -400)
            {
                rDrive = -400;
            }
        } else {
            rDrive = 0;
        }

        courseCorrection(false, robotData, drivebaseData);

        if (lDistLeft >= -.5 && rDistLeft >= -.5) {
            autonData.driveStraightInitialized = false;
            autonData.autonStep++;
        }
    }
}

void Drivebase::courseCorrection(bool isForward, const RobotData &robotData, DrivebaseData &drivebaseData){
    if(isForward){ // if you're going forward
        if(robotData.gyroData.rawYaw > drivebaseData.initialAngle){ // if you're facing too far right

            // modify drive 
            if(robotData.gyroData.rawYaw - drivebaseData.initialAngle > 10){ 
                lDrive *= .7;
                rDrive *= 1.1;
            } else if(robotData.gyroData.rawYaw - drivebaseData.initialAngle > 5){
                lDrive *= .8;
                rDrive *= 1.05;
            } else if (robotData.gyroData.rawYaw - drivebaseData.initialAngle > 2){
                lDrive *= .8;
            } else {
                lDrive *= .9;
            }
        } else if (robotData.gyroData.rawYaw < drivebaseData.initialAngle) { // if you're facing too far left
            if(drivebaseData.initialAngle - robotData.gyroData.rawYaw > 10){
                rDrive *= .7;
                lDrive *= 1.1;
            } else if(drivebaseData.initialAngle - robotData.gyroData.rawYaw > 5){
                rDrive *= .8;
                lDrive *= 1.05;
            } else if (drivebaseData.initialAngle - robotData.gyroData.rawYaw > 2){
                rDrive *= .8;
            } else {
                rDrive *= .9;
            }
        }
    } else {
        if(robotData.gyroData.rawYaw > drivebaseData.initialAngle){

            if(robotData.gyroData.rawYaw - drivebaseData.initialAngle > 10){
                rDrive *= .7;
                lDrive *= 1.1;
            } else if(robotData.gyroData.rawYaw - drivebaseData.initialAngle > 5){
                rDrive *= .8;
                lDrive *= 1.05;
            } else if (robotData.gyroData.rawYaw - drivebaseData.initialAngle > 2){
                rDrive *= .8;
            } else {
                rDrive *= .9;
            }
        } else if (robotData.gyroData.rawYaw < drivebaseData.initialAngle) {
            if(drivebaseData.initialAngle - robotData.gyroData.rawYaw > 10){
                lDrive *= .7;
                rDrive *= 1.1;
            } else if(drivebaseData.initialAngle - robotData.gyroData.rawYaw > 5){
                lDrive *= .8;
                rDrive *= 1.05;
            } else if (drivebaseData.initialAngle - robotData.gyroData.rawYaw > 2){
                lDrive *= .8;
            } else {
                lDrive *= .9;
            }
        }
    }
}
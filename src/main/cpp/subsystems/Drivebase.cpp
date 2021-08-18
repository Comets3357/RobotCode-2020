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

void Drivebase::RobotPeriodic(const RobotData &robotData, DrivebaseData &drivebaseData)
{
    updateData(robotData, drivebaseData);

    if (frc::DriverStation::GetInstance().IsEnabled())
    {
        dbRM.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
        dbRS.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
        dbLM.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
        dbLS.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    }

    teleopControl(robotData);
    // to use, commment out teleopControl
    // pidTuningControl(robotData);
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

//this is the function for tuning drivebase pids
//to use it, you HAVE to comment out teleopControl in periodic so that the motor controllers don't receive conflicting commands
//the robot will drive forward as you use rev client. BE IN SEMI AUTO MODE
//to drive the robot back to start over again, TOGGLE TO MANUAL MODE ON SECONDARY
void Drivebase::pidTuningControl(const RobotData &robotData){
    if(robotData.controllerData.manualMode){
        teleopControl(robotData);
    } else {
        double setPoint;
        if(robotData.gyroData.yaw > 180){
                setPoint = (360 - robotData.gyroData.yaw) * 30;
            } else {
                setPoint = 0;
            }
            dbLMPID.SetReference(setPoint, rev::ControlType::kVelocity);
    }
    
}


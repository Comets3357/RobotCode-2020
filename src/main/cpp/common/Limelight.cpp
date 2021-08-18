#include "common/Limelight.h"
#include "RobotData.h"


#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <cmath>
#include <frc/Driverstation.h>
#include <frc/smartdashboard/SmartDashboard.h>


void Limelight::RobotInit(){}

/**
 * calculates hood position needed 
 * @param verticalOffset vertical offset from the target from limelight in degrees
 * @return desired encoder position of Shooter Hood
 */
double Limelight::calcHoodPOS(double verticalOffset, const RobotData& robotData){ 
    double x = verticalOffset;

    if(verticalOffset == 0){
        return 0;
    }else{
        return ((-0.0000680977*std::pow(x,4.0))+(.00141765*std::pow(x,3.0))+(-0.00521063*std::pow(x,2.0))+(-0.170811*x) + 15.1117) + robotData.shooterData.roughHood; 
    }
}

/**
 * calculates turret position needed 
 * @param horOffset horizontal offset from the target from limelight in degrees
 * @return desired encoder position of Shooter turret
 */
double Limelight::calcTurretPOS(double horOffset){ 
    double x = horOffset;
    return (x*0.144034);
}

/**
 * @return horizontal offset angle from limelight
 */
double Limelight::getHorizontalOffset(){
    std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight"); //opens up the table
    return (table->GetNumber("tx",0.0)) + 0.5; //offset
}

/**
 * @return vertical offset angle from limelight
 */
double Limelight::getVerticalOffset(){
    std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight"); //opens up the table
    return table->GetNumber("ty",0.0);
}

/**
 * @return if a target is seen or not 0 or 1
 */
int Limelight::getTarget(){
    std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight"); //opens up the table
    return table->GetNumber("tv",0.0);
}


/**
 * @param verticalOffset y offset from limelight
 * @return needed pipeline based off how close to the target the bot is
 * 
 * pipeline 0 = off
 * pipeline 1 = 40 power
 * pipeline 2 = 60 power
 * pipeline 3 = 80 power
 * pipeline 4 = 100 power
 * at indiana below
 * pipline 5 = backup for 1
 * pipeline 6 = backup for 2
 * pipeline 7 = backup for 3
 * pipeline 8 = backup for 4
 */
int Limelight::getPipeline(double verticalOffset){

    int pipeline;

    if(verticalOffset > 14){
        // pipeline = 1;
        pipeline = 5;
    }else if(verticalOffset > 9){
        // pipeline = 2;
        pipeline = 6;
    }else if(verticalOffset > 6){
        // pipeline = 3;
        pipeline = 7;
    }else if(verticalOffset > 1){
        // pipeline = 4;
        pipeline = 8;
    }else{
        pipeline = 1;

    }

    //basically if you can see the target turn on the limelight otherwise don't
    return pipeline;
}


void Limelight::RobotPeriodic(const RobotData &robotData, LimelightData &limelightData){

   
   std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight"); //opens up the networktable

    //updating data
    limelightData.xOffset = getHorizontalOffset();
    limelightData.yOffset = getVerticalOffset();
    limelightData.targetValue = getTarget();
    limelightData.calcHoodPos = calcHoodPOS(robotData.limelightData.yOffset, robotData);

    limelightData.calcTurretPos = calcTurretPOS(robotData.limelightData.xOffset);
    limelightData.validTarget = table->GetNumber("tv", 0.0);


    if(robotData.controllerData.shootingMode){
        table->PutNumber("pipeline", getPipeline(robotData.limelightData.yOffset)); //set the pipeline based on y offset
        // frc::SmartDashboard::PutBoolean("shooting", true);

    }else{
        table->PutNumber("pipeline",0); //set the limelight to off
        // frc::SmartDashboard::PutBoolean("shooting", false);

    }


}


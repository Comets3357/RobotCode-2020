#pragma once

#include <frc/DriverStation.h>
#include <frc/Joystick.h>
#include <frc/TimedRobot.h>

struct RobotData;

enum DriveMode
{
    driveMode_teleop,
    driveMode_potato,
    driveMode_driveStraight
};

struct ControllerData
{
    // controls data:

    // states:
    bool manualMode = false;
    bool climbMode = false;
    bool shift = false;
    bool shootingMode = false;

    // drivebase:
    double lDrive = 0;
    double rDrive = 0;
    bool turnResponsive = false;
    bool dbInverted = false;
    double maxStraight = 1;
    double maxTurn = 0.4;
    DriveMode driveMode = driveMode_potato;

    // indexer:
    bool mIndexer;
    bool mIndexerBackwards;
    bool saEjectBallsBackwards;

    // intake:
    bool mIntakeDown;
    bool mIntakeRollers;
    bool mIntakeRollersBackward;
    bool saIntake;
    bool saIntakeBackward;

    //shooter:
    bool mShooterFlyWheel;
    double mSetTurret;
    double mSetHood;

    //limelight
    int roughHood = 0;
    int roughTurret = 0;

    // btn data:
    // L = left, R = right, p = primary, s = secondary, Btn = button

    // primary:

    double pLXStick = 0;
    double pLYStick = 0;
    double pRXStick = 0;
    double pRYStick = 0;

    bool pLShoulderSwitch = false;
    bool pRShoulderSwitch = false;

    // secondary:

    double sLXStick = 0;
    double sLYStick = 0;
    double sRXStick = 0;
    double sRYStick = 0;

    bool sLStickBtn = false;
    bool sRStickBtn = false;

    double sLTrigger = 0;
    double sRTrigger = 0;
    bool sLBumper = false;
    bool sRBumper = false;

    bool sXBtn = false;
    bool sYBtn = false;
    bool sABtn = false;
    bool sBBtn = false;

    bool sABtnToggled = false;
    bool sBBtnToggled = false;
    bool sXBtnToggled = false;
    bool sYBtnToggled = false;

    bool sLCenterBtn = false;
    bool sRCenterBtn = false;

    bool sLCenterBtnToggled = false;
    bool sRCenterBtnToggled = false;

    int sDPad = -1;
};

class Controller
{

public:
    void TeleopPeriodic(const RobotData &robotData, ControllerData &controllerData);
    void TeleopInit(ControllerData &controllerData);

private:
    void updateBtnData(ControllerData &controllerData);
    void updateControlsData(ControllerData &controllerData);

    // basic btn getters:
    bool getBtn(int js, int index);
    bool getBtnToggled(int js, int index);
    int getPOV(int js, int index);
    double getAxis(int js, int index);

    frc::Joystick primary{0};
    frc::Joystick secondary{1};
};
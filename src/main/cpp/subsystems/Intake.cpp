#include "subsystems/Intake.h"

#include "RobotData.h"

void Intake::RobotInit()
{
    intakeRollers.RestoreFactoryDefaults();
    intakePivot.RestoreFactoryDefaults();

    intakeRollers.SetInverted(true);
    intakePivot.SetInverted(false);

    intakePivot.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    intakeRollers.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    intakePivot_pidController.SetP(pkP);
    intakePivot_pidController.SetI(pkI);
    intakePivot_pidController.SetD(pkD);
    intakePivot_pidController.SetIZone(pkIz);
    intakePivot_pidController.SetFF(pkFF);
    intakePivot_pidController.SetOutputRange(pkMinOutput, pkMaxOutput);

    intakeRollers_pidController.SetP(wkP);
    intakeRollers_pidController.SetI(wkI);
    intakeRollers_pidController.SetD(wkD);
    intakeRollers_pidController.SetIZone(wkIz);
    intakeRollers_pidController.SetFF(wkFF);
    intakeRollers_pidController.SetOutputRange(wkMinOutput, wkMaxOutput);

    intakePivot.SetSmartCurrentLimit(45);
    intakeRollers.SetSmartCurrentLimit(45);

    intakePivotEncoder.SetPosition(0);
    intakeRollersEncoder.SetPosition(0);
}

void Intake::RobotPeriodic(const RobotData &robotData, IntakeData &intakeData)
{
    if (robotData.controllerData.manualMode)
    {
        manual(robotData, intakeData);
    }
    else
    {
        semiAuto(robotData, intakeData);
    }
}

void Intake::manual(const RobotData &robotData, IntakeData &intakeData)
{
    if (robotData.controllerData.mIntakeDown)
    {
        // pivot down
        if (intakePivotEncoder.GetPosition() < 12)
        {
            intakePivot.Set(intakePivotSpeed);
            
        }
        // once you're down
        else
        {
            intakePivot.Set(0);
        }
    }
    // otherise, bring the intake back up slowly
    else
    {
        
        if (intakePivotEncoder.GetPosition() > 0)
        {
            intakePivot.Set(-intakePivotSpeed);
        }
        else
        {
            intakePivot.Set(0);
        }
    }

    if (robotData.controllerData.mIntakeRollers)
    {
        intakeRollers.Set(intakeRollersSpeed);
    }
    else if (robotData.controllerData.mIntakeRollersBackward)
    {
        intakeRollers.Set(-intakeRollersSpeed);
    }
    else
    {
        intakeRollers.Set(0);
    }
    
}

void Intake::semiAuto(const RobotData &robotData, IntakeData &intakeData)
{
    if (robotData.controllerData.saIntake)
    {
        // pivot down
        if (intakePivotEncoder.GetPosition() < 12)
        {
            intakePivot.Set(intakePivotSpeed);
            intakeRollers.Set(intakeRollersSpeed);
        }
        // once you're down
        else
        {
            intakePivot.Set(0);
        }
    }
    // otherise, bring the intake back up slowly
    else if (robotData.controllerData.saIntakeBackward)
    {
        intakeRollers.Set(-intakeRollersSpeed);
    }
    else
    {
        intakeRollers.Set(0);
        if (intakePivotEncoder.GetPosition() > 0)
        {
            intakePivot.Set(-intakePivotSpeed);
        }
        else
        {
            intakePivot.Set(0);
        }
    }
}
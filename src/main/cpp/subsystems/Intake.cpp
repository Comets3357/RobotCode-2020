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

    frc::SmartDashboard::PutNumber("intakePivotEncoder", intakePivotEncoder.GetPosition());
}

void Intake::manual(const RobotData &robotData, IntakeData &intakeData)
{
    if (robotData.controllerData.mIntakeDown)
    {
        // pivot down
        if (intakePivotEncoder.GetPosition() < 13)
        {
            intakePivot.Set(0.1);
            
        }
        // once you're down
        else
        {
            intakePivot.Set(0);
            // delete this if we are using prox sensor for indexer belt suck
            /* if (intakeRollersEncoder.GetVelocity() < 1150)
            {
                index.setIndexerBelt(0.36);
            }
            else
            {
                index.setIndexerBelt(0);
            } */
        }
    }
    // otherise, bring the intake back up slowly
    else
    {
        
        if (intakePivotEncoder.GetPosition() > 0)
        {
            intakePivot.Set(-0.1);
        }
        else
        {
            intakePivot.Set(0);
        }
    }

    if (robotData.controllerData.mIntakeRollers)
    {
        intakeRollers.Set(0.3);
    }
    else
    {
        intakeRollers.Set(0);
    }

    if (robotData.controllerData.mIntakeRollersBackward)
    {
        intakeRollers.Set(-0.3);
    }
    else
    {
        intakeRollers.Set(0);
    }
    
}

void Intake::semiAuto(const RobotData &robotData, IntakeData &intakeData)
{
    // eject balls backward
    /* if (index.getGoBack()){
        goBack = true;
        intakeRollers.Set(-0.25);
    } else if (sStick.GetRawButton(1)){
        goBack = false;
        intakeRollers.Set(0);
        index.setIndexerBelt(0);
    } */


    if (robotData.controllerData.saIntake)
    {
        // pivot down
        if (intakePivotEncoder.GetPosition() < 13)
        {
            intakePivot.Set(0.1);
            intakeRollers.Set(0.3);
        }
        // once you're down
        else
        {
            intakePivot.Set(0);
            // delete this if we are using prox sensor for indexer belt suck
            /* if (intakeRollersEncoder.GetVelocity() < 1150)
            {
                index.setIndexerBelt(0.36);
            }
            else
            {
                index.setIndexerBelt(0);
            } */
        }
    }
    // otherise, bring the intake back up slowly
    else if (robotData.controllerData.saIntakeBackward)
    {
        intakeRollers.Set(-0.3);
    }
    else
    {
        intakeRollers.Set(0);
        if (intakePivotEncoder.GetPosition() > 0)
        {
            intakePivot.Set(-0.1);
        }
        else
        {
            intakePivot.Set(0);
        }
    }
}
#include "common/Timer.h"

void Timer::RobotInit(TimerData &timerData)
{
    timer.Reset();
    timer.Start();
}

void Timer::RobotPeriodic(TimerData &timerData)
{
    if (!enabledSPointSet && frc::DriverStation::GetInstance().IsEnabled())
    {
        enabledStartPoint = timer.Get();
        
        enabledSPointSet = true;
    }

    timerData.secSinceInit = timer.Get();
    timerData.secSinceEnabled = timer.Get() - enabledStartPoint;
}

void Timer::DisabledInit()
{
    enabledSPointSet = false;
}
#pragma once
#include "hydrv_tim_low.hpp"

extern "C"
{
#include "hydrolib_return_codes.hpp"
}

namespace hydrv::thruster
{

class Thruster
{
public:
    static constexpr int speed_null{0};
    static constexpr int max_speed{100};

    static constexpr unsigned tim_prescaler{1679};
    static constexpr unsigned tim_counter_period{40000};
    static constexpr unsigned tim_clock_mhz{168};

    static constexpr unsigned pwm_null{3000};

public:
    constexpr Thruster(unsigned thruster_tim_channel,
             hydrv::timer::TimerLow &thruster_tim,
             hydrv::GPIO::GPIOLow &thruster_tim_pin);

    void Init();

    hydrolib::ReturnCode SetSpeed(int speed);
    int GetSpeed();

private:
    static constexpr unsigned SpeedToPWM_(int speed);

private:
    int speed;

    timer::TimerLow &tim;
    unsigned tim_channel;
    GPIO::GPIOLow &tim_pin;
};

inline constexpr Thruster::Thruster(unsigned thruster_tim_channel,
                          hydrv::timer::TimerLow &thruster_tim,
                          hydrv::GPIO::GPIOLow &thruster_tim_pin)
    : speed(speed_null), tim(thruster_tim),
      tim_channel(thruster_tim_channel),
      tim_pin(thruster_tim_pin)
{
}

inline void Thruster::Init()
{
    tim.Init();
    tim.ConfigurePWM(tim_channel, tim_pin);
    tim.SetCaptureCompare(tim_channel, SpeedToPWM_(speed_null));
    tim.StartTimer();
}

inline hydrolib::ReturnCode Thruster::SetSpeed(int thruster_speed)
{
    if (thruster_speed <= max_speed && thruster_speed >= -max_speed)
    {
        speed = thruster_speed;
        tim.SetCaptureCompare(tim_channel, SpeedToPWM_(speed));
        return hydrolib::ReturnCode::OK;
    }
    else
    {
        return hydrolib::ReturnCode::ERROR;
    }
}

inline int Thruster::GetSpeed() { return speed; }

inline constexpr unsigned Thruster::SpeedToPWM_(int speed)
{
    return pwm_null + speed;
}

} // namespace hydrv::thruster

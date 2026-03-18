#pragma once

#include <cstdint>

extern "C"
{
#include "stm32f4xx.h"
}

#include "hydrv_gpio_low.hpp"

namespace hydrv::timer
{
class TimerLow
{
public:
    struct TimerPreset
    {
        const uint32_t TIMx;
        const uint8_t GPIO_alt_func;

        const uint32_t RCC_APBENR_TIMxEN;
        const uint32_t RCC_address;
    };

public:
    static constexpr TimerPreset TIM5_low{TIM5_BASE, 2, RCC_APB1ENR_TIM5EN,
                                          RCC_BASE +
                                              offsetof(RCC_TypeDef, APB1ENR)};

public:
    consteval TimerLow(TimerPreset preset, unsigned prescaler, uint32_t period);

public:
    void Init();
    void ConfigurePWM(unsigned channel, hydrv::GPIO::GPIOLow &pin);

    void StartTimer();
    void StopTimer();

    void SetCaptureCompare(unsigned channel, uint32_t value);

private:
    static constexpr uint32_t CountCR1Mask_();
    static inline void EnableTimerClock_(uint32_t rcc_address, uint32_t en_bit);

private:
    const uint32_t TIMx_;
    unsigned prescaler_;
    uint32_t period_;
    const uint32_t RCC_APBENR_TIMxEN_;
    const uint32_t RCC_address_;
    const uint8_t GPIO_alt_func_;
    const uint32_t cr1_;
};

consteval TimerLow::TimerLow(TimerPreset preset, unsigned prescaler,
                             uint32_t period)
    : TIMx_(preset.TIMx),
      prescaler_(prescaler),
      period_(period),
      RCC_APBENR_TIMxEN_(preset.RCC_APBENR_TIMxEN),
      RCC_address_(preset.RCC_address),
      GPIO_alt_func_(preset.GPIO_alt_func),
      cr1_(CountCR1Mask_())
{
}

inline void TimerLow::Init()
{
    EnableTimerClock_(RCC_address_, RCC_APBENR_TIMxEN_);

    reinterpret_cast<TIM_TypeDef *>(TIMx_)->CR1 = cr1_;

    reinterpret_cast<TIM_TypeDef *>(TIMx_)->ARR = period_;
    reinterpret_cast<TIM_TypeDef *>(TIMx_)->PSC = prescaler_ - 1;

    SET_BIT(reinterpret_cast<TIM_TypeDef *>(TIMx_)->EGR, TIM_EGR_UG);
}
// TODO move to init, pass to constructor by array of channel presets
inline void TimerLow::ConfigurePWM(unsigned channel, hydrv::GPIO::GPIOLow &pin)
{
    CLEAR_BIT(reinterpret_cast<TIM_TypeDef *>(TIMx_)->CCER,
              0x1UL << (channel * 4));

    uint32_t ccer = reinterpret_cast<TIM_TypeDef *>(TIMx_)->CCER;
    CLEAR_BIT(ccer, 0x1UL << ((channel * 4) + 1));
    CLEAR_BIT(ccer, 0x1UL << ((channel * 4) + 3));
    reinterpret_cast<TIM_TypeDef *>(TIMx_)->CCER = ccer;

    uint32_t ccmr = (channel < 2)
                        ? reinterpret_cast<TIM_TypeDef *>(TIMx_)->CCMR1
                        : reinterpret_cast<TIM_TypeDef *>(TIMx_)->CCMR2;
    MODIFY_REG(ccmr, 0x3UL << ((channel % 2) * 8), 0x0UL);
    SET_BIT(ccmr, 0x7UL << ((channel % 2) * 8 + 3)); // Autoreload preload
    MODIFY_REG(ccmr, 0x7UL << ((channel % 2) * 8 + 4),
               0x6UL << ((channel % 2) * 8 + 4));
    if (channel < 2)
    {
        reinterpret_cast<TIM_TypeDef *>(TIMx_)->CCMR1 = ccmr;
    }
    else
    {
        reinterpret_cast<TIM_TypeDef *>(TIMx_)->CCMR2 = ccmr;
    }

    SET_BIT(reinterpret_cast<TIM_TypeDef *>(TIMx_)->CCER,
            0x1UL << (channel * 4));

    pin.Init(GPIO_alt_func_);
}

inline void TimerLow::StartTimer()
{
    SET_BIT(reinterpret_cast<TIM_TypeDef *>(TIMx_)->CR1, TIM_CR1_CEN);
}
inline void TimerLow::StopTimer()
{
    CLEAR_BIT(reinterpret_cast<TIM_TypeDef *>(TIMx_)->CR1, TIM_CR1_CEN);
}

inline void TimerLow::SetCaptureCompare(unsigned channel, uint32_t value)
{
    switch (channel)
    {
    case 0:
        reinterpret_cast<TIM_TypeDef *>(TIMx_)->CCR1 = value;
        break;
    case 1:
        reinterpret_cast<TIM_TypeDef *>(TIMx_)->CCR2 = value;
        break;
    case 2:
        reinterpret_cast<TIM_TypeDef *>(TIMx_)->CCR3 = value;
        break;
    case 3:
        reinterpret_cast<TIM_TypeDef *>(TIMx_)->CCR4 = value;
        break;
    }
}

inline constexpr uint32_t TimerLow::CountCR1Mask_()
{
    uint32_t cr1 = 0;
    CLEAR_BIT(cr1, TIM_CR1_DIR);     // Upcounting
    MODIFY_REG(cr1, TIM_CR1_CMS, 0); // No center alingment
    MODIFY_REG(cr1, TIM_CR1_CKD, 0); // No internal clock divider
    SET_BIT(cr1, TIM_CR1_URS);       // No update because of pulse change
    SET_BIT(cr1, TIM_CR1_ARPE);

    return cr1;
}

inline void TimerLow::EnableTimerClock_(uint32_t rcc_address, uint32_t en_bit)
{
    __IO uint32_t tmpreg = 0x00U;
    SET_BIT(*(volatile uint32_t *)rcc_address,
            en_bit); /* Delay after an RCC peripheral clock enabling */
    tmpreg = READ_BIT(*(volatile uint32_t *)rcc_address, en_bit);
    (void)tmpreg;
}

} // namespace hydrv::timer

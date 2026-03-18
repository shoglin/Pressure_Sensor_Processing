#pragma once

#include <cstddef>
#include <cstdint>

extern "C"
{
#include "hydrolib_common.h"
#include "stm32f1xx.h"
}

namespace hydrv::GPIO
{
class GPIOLow
{

private:
    enum class Mode : uint32_t
    {
        kInput = 0x00,
        kOutput10MHz = 0x01,
        kOutput2MHz = 0x02,
        kOutput50MHz = 0x03
    };

    enum class Configure : uint32_t // TODO: Make check Input-Output from mode
    {
        kAnalogInput = 0x00,
        kFloatingInput = 0x01,
        kPullUpDownInput = 0x02,
        kGeneralPurposePushPullOutput = 0x00,
        kGeneralPurposeOpenDrainOutput = 0x01,
        kAlternateFunctionPushPullOutput = 0x02,
        kAlternateFunctionOpenDrainOutput = 0x03
    };

public:
    struct GPIOPort
    {
    public:
        static constexpr std::size_t PIN_COUNT = 16;

    public:
        const uint32_t GPIOx;
        const uint32_t RCC_APB2ENR_IOPxEN;

        bool *const inited_pins_;
    };

    struct GPIOPreset
    {
        Mode MODE;
        Configure CNF;
    };

private:
    static constinit bool GPIOA_inited_pins_[GPIOPort::PIN_COUNT];
    static constinit bool GPIOB_inited_pins_[GPIOPort::PIN_COUNT];
    static constinit bool GPIOC_inited_pins_[GPIOPort::PIN_COUNT];
    static constinit bool GPIOD_inited_pins_[GPIOPort::PIN_COUNT];

public:
    static constexpr GPIOPort GPIOA_port{GPIOA_BASE, RCC_APB2ENR_IOPAEN,
                                         GPIOA_inited_pins_};
    static constexpr GPIOPort GPIOB_port{GPIOB_BASE, RCC_APB2ENR_IOPBEN,
                                         GPIOB_inited_pins_};
    static constexpr GPIOPort GPIOC_port{GPIOC_BASE, RCC_APB2ENR_IOPCEN,
                                         GPIOC_inited_pins_};
    static constexpr GPIOPort GPIOD_port{GPIOD_BASE, RCC_APB2ENR_IOPDEN,
                                         GPIOD_inited_pins_};
    static constexpr GPIOPreset GPIO_Output{
        Mode::kOutput2MHz, Configure::kGeneralPurposePushPullOutput};
    static constexpr GPIOPreset GPIO_UART_TX{
        Mode::kOutput10MHz, Configure::kAlternateFunctionPushPullOutput};
    static constexpr GPIOPreset GPIO_UART_RX{Mode::kInput,
                                             Configure::kFloatingInput};

public:
    consteval GPIOLow(const GPIOPort &GPIO_group, unsigned pin,
                      GPIOPreset preset);

public:
    hydrolib_ReturnCode Init(uint32_t altfunc);

    bool IsInited();

    void Set();

    void Reset();

private:
    bool &is_inited_;
    const uint32_t GPIOx_;
    const unsigned pin_;
    const uint32_t RCC_APB2ENR_IOPxEN_;

    const uint32_t cr_reg_mask_;
    const uint32_t cr_reg_value_;

    const uint32_t set_reg_mask_;
    const uint32_t reset_reg_mask_;

private:
    static void EnableGPIOxClock_(const uint32_t RCC_AHB1ENR_GPIOxEN);
};

inline bool GPIOLow::GPIOA_inited_pins_[GPIOPort::PIN_COUNT] = {};
inline bool GPIOLow::GPIOB_inited_pins_[GPIOPort::PIN_COUNT] = {};
inline bool GPIOLow::GPIOC_inited_pins_[GPIOPort::PIN_COUNT] = {};
inline bool GPIOLow::GPIOD_inited_pins_[GPIOPort::PIN_COUNT] = {};

consteval inline GPIOLow::GPIOLow(const GPIOPort &GPIO_group, unsigned pin,
                                  GPIOPreset preset)
    : is_inited_(GPIO_group.inited_pins_[pin]),
      GPIOx_(GPIO_group.GPIOx),
      pin_(pin),
      RCC_APB2ENR_IOPxEN_(GPIO_group.RCC_APB2ENR_IOPxEN),
      cr_reg_mask_(0xFUL << (4 * (pin % 8))),
      cr_reg_value_(static_cast<uint32_t>(preset.MODE) << (4 * (pin % 8)) |
                    static_cast<uint32_t>(preset.CNF) << (4 * (pin % 8) + 2)),
      set_reg_mask_(0x1UL << pin),
      reset_reg_mask_(0x1UL << (pin + GPIO_BSRR_BR0_Pos))
{
}

inline hydrolib_ReturnCode GPIOLow::Init([[maybe_unused]] uint32_t altfunc = 0)
{
    if (is_inited_)
    {
        return HYDROLIB_RETURN_FAIL;
    }

    EnableGPIOxClock_(RCC_APB2ENR_IOPxEN_);

    if (pin_ > 7)
    {
        MODIFY_REG(reinterpret_cast<GPIO_TypeDef*>(GPIOx_)->CRH, cr_reg_mask_, cr_reg_value_);
    }
    else
    {
        MODIFY_REG(reinterpret_cast<GPIO_TypeDef*>(GPIOx_)->CRL, cr_reg_mask_, cr_reg_value_);
    }
    is_inited_ = true;

    return HYDROLIB_RETURN_OK;
}

inline bool GPIOLow::IsInited() { return is_inited_; }

inline void GPIOLow::Set() { SET_BIT(reinterpret_cast<GPIO_TypeDef*>(GPIOx_)->BSRR, set_reg_mask_); }

inline void GPIOLow::Reset() { SET_BIT(reinterpret_cast<GPIO_TypeDef*>(GPIOx_)->BSRR, reset_reg_mask_); }

inline void GPIOLow::EnableGPIOxClock_(const uint32_t RCC_AHB1ENR_GPIOxEN)
{
    __IO uint32_t tmpreg = 0x00U;
    SET_BIT(RCC->APB2ENR, RCC_AHB1ENR_GPIOxEN); /* Delay after an RCC peripheral
                                                   clock enabling */
    tmpreg = READ_BIT(RCC->APB2ENR, RCC_AHB1ENR_GPIOxEN);
    (void)tmpreg;
}

} // namespace hydrv::GPIO

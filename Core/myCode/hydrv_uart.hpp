#pragma once

#include <cstring>
#include "hydrolib_common.h"
#include "hydrolib_func_concepts.hpp"
#include "hydrv_uart_low.hpp"

namespace hydrv::UART
{
template <int RX_BUFFER_CAPACITY, int TX_BUFFER_CAPACITY,
          typename CallbackType = decltype(&hydrolib::concepts::func::DummyFunc<void>)>
requires hydrolib::concepts::func::FuncConcept<CallbackType, void>
class UART
{
public:

//   bool IsTxBusy() {  
//     return (tx_head_ != tx_tail_) || !UART_handler_.IsTxDone();
// }

    static constexpr unsigned REAL_RX_BUFFER_CAPACITY_ = RX_BUFFER_CAPACITY + 1;
    static constexpr unsigned REAL_TX_BUFFER_CAPACITY_ = TX_BUFFER_CAPACITY + 1;

    consteval UART(const UARTLow::UARTPreset &UART_preset,
                   hydrv::GPIO::GPIOLow &rx_pin, hydrv::GPIO::GPIOLow &tx_pin,
                   unsigned IRQ_priority,
                   CallbackType rx_callback = hydrolib::concepts::func::DummyFunc<void>)
        : UART_handler_(UART_preset, rx_pin, tx_pin, IRQ_priority),
          rx_head_(0), 
          rx_tail_(0),
          rx_buffer_{},
          tx_buffer_{},
          tx_head_(0), 
          tx_tail_(0), 
          rx_callback_(rx_callback) {}

    void Init() { UART_handler_.Init(); }
    
    void ClearRx() { 
        rx_head_ = 0;
        rx_tail_ = 0;
    }

    void Push(const uint8_t* data, uint16_t length) {
        if (data == nullptr || length == 0) return;
        
        for (uint16_t i = 0; i < length; i++) {
            unsigned next_tail = (rx_tail_ + 1) % REAL_RX_BUFFER_CAPACITY_;
            if (next_tail != rx_head_) {
                rx_buffer_[rx_tail_] = data[i];
                rx_tail_ = next_tail;
            }
        }
        if (rx_callback_) rx_callback_(); 
    }

    void IRQCallback() { 
        ProcessRx_();  // Added this call to process RX data
        ProcessTx_(); 
    }

    int Read(void *data, unsigned data_length) {
        unsigned available = GetRxLength();
        if (data_length > available) data_length = available;
        if (data_length == 0 || data == nullptr) return 0;

        unsigned current_head = rx_head_;
        unsigned forward_length = REAL_RX_BUFFER_CAPACITY_ - current_head;
        
        if (data_length > forward_length) {
            std::memcpy(data, &rx_buffer_[current_head], forward_length);
            std::memcpy(static_cast<uint8_t*>(data) + forward_length, &rx_buffer_[0], data_length - forward_length);
        } else {
            std::memcpy(data, &rx_buffer_[current_head], data_length);
        }
        
        rx_head_ = (current_head + data_length) % REAL_RX_BUFFER_CAPACITY_;
        return (int)data_length;
    }

    int Transmit(const void *data, unsigned data_length) {
        unsigned used = GetTxLength();
        unsigned available_space = TX_BUFFER_CAPACITY - used;
        if (data_length > available_space) data_length = available_space;
        if (data_length == 0 || data == nullptr) return 0;

        unsigned current_tail = tx_tail_;
        unsigned forward_length = REAL_TX_BUFFER_CAPACITY_ - current_tail;
        
        if (forward_length >= data_length) {
            std::memcpy(&tx_buffer_[current_tail], data, data_length);
        } else {
            std::memcpy(&tx_buffer_[current_tail], data, forward_length);
            std::memcpy(&tx_buffer_[0], static_cast<const uint8_t*>(data) + forward_length, data_length - forward_length);
        }
        
        tx_tail_ = (current_tail + data_length) % REAL_TX_BUFFER_CAPACITY_;
        UART_handler_.EnableTxInterruption();
        return (int)data_length;
    }

    unsigned GetRxLength() const {
        unsigned h = rx_head_;
        unsigned t = rx_tail_;
        return (t >= h) ? (t - h) : (t + REAL_RX_BUFFER_CAPACITY_ - h);
    }

    unsigned GetTxLength() const {
        unsigned h = tx_head_;
        unsigned t = tx_tail_;
        return (t >= h) ? (t - h) : (t + REAL_TX_BUFFER_CAPACITY_ - h);
    }

private:
    void ProcessTx_() {
        if (!UART_handler_.IsTxDone()) return;
        
        unsigned h = tx_head_;
        if (h == tx_tail_) {
            UART_handler_.DisableTxInterruption();
            return;
        }
        
        UART_handler_.SetTx(tx_buffer_[h]);
        tx_head_ = (h + 1) % REAL_TX_BUFFER_CAPACITY_;
    }

    void ProcessRx_() {
        if (!UART_handler_.IsRxDone()) return;

        unsigned next_tail = (rx_tail_ + 1) % REAL_RX_BUFFER_CAPACITY_;

        if (next_tail == rx_head_) {
            (void)UART_handler_.GetRx();  // Discard byte on overrun
            // Optional: set error flag or log
            return;
        }

        rx_buffer_[rx_tail_] = UART_handler_.GetRx();
        rx_tail_ = next_tail;

        if (rx_callback_) rx_callback_();
    }

    UARTLow UART_handler_;
    volatile unsigned rx_head_;
    volatile unsigned rx_tail_;  
    uint8_t rx_buffer_[REAL_RX_BUFFER_CAPACITY_] __attribute__((aligned(4)));

    uint8_t tx_buffer_[REAL_TX_BUFFER_CAPACITY_] __attribute__((aligned(4)));
    volatile unsigned tx_head_;
    unsigned tx_tail_;
    CallbackType rx_callback_;
};

/** * ADL Helper functions for Hydrolib Concepts 
 * These MUST stay in the same namespace as the UART class
 */
template <int R, int T, typename C>
int read(UART<R, T, C> &stream, void *dest, unsigned len) { 
    return stream.Read(dest, len); 
}

template <int R, int T, typename C>
int write(UART<R, T, C> &stream, const void *src, unsigned len) { 
    return stream.Transmit(src, len); 
}

} // namespace hydrv::UART
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>

#ifndef HAL_RTCTRL_ENABLED
#define HAL_RTCTRL_ENABLED !HAL_MINIMIZE_FEATURES
#endif

#if HAL_RTCTRL_ENABLED

#define INCOMING_FLOAT_COUNT 5
#define INCOMING_PACKET_LENGTH 2 + INCOMING_FLOAT_COUNT * 4
#define OUTGOING_PACKET_LENGTH 10

#define REALTIME_FREQ 100

#define PACKET_HEADER 0x0F

class AP_Realtime_Control {
public:
    AP_Realtime_Control();
    AP_Realtime_Control(const AP_Realtime_Control &other) = delete;
    AP_Realtime_Control &operator=(const AP_Realtime_Control&) = delete;

    static AP_Realtime_Control *get_singleton(void) {
        return singleton;
    }

    void init(void);
private:
    static AP_Realtime_Control *singleton;

    AP_HAL::UARTDriver *uart;

    void update(void);
    uint8_t read_packet(uint8_t *buffer, uint8_t length, float *float_packet);
    uint8_t packet_crc(uint8_t *buffer);
    float byte_to_float(uint8_t *fl_buf);
    void float_to_byte(float in, uint8_t *out);
};

namespace AP {
    AP_Realtime_Control *rt_control();
};

#endif // HAL_RTCTRL_ENABLED
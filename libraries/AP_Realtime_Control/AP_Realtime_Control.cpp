#include "AP_Realtime_Control.h"

#if HAL_RTCTRL_ENABLED
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

AP_Realtime_Control *AP_Realtime_Control::singleton;

AP_Realtime_Control::AP_Realtime_Control(void)
{
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (singleton != nullptr) {
        AP_HAL::panic("AP_Realtime_Control must be singleton");
    }
#endif
    singleton = this;
}

void AP_Realtime_Control::init()
{
    const AP_SerialManager &serial_manager = AP::serialmanager();
    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Realtime_Control, 0);
    if (uart)
    {
        if (!hal.scheduler->thread_create(  FUNCTOR_BIND_MEMBER(&AP_Realtime_Control::update, void),
                                            "RTCtrl",
                                            1024, AP_HAL::Scheduler::PRIORITY_BOOST, 1)) {
            hal.console->printf("Failed to create RTCtrl thread\n");
        }
    }
}

void AP_Realtime_Control::update(void)
{
    uart->begin(921600, 128, 128);
    uart->set_unbuffered_writes(true);
    uart->set_blocking_writes(true);
    uint8_t buf[128];
    const uint32_t micros_step = 1000000;//(100000.0 / REALTIME_FREQ);
    uint32_t last_micros = 0;
    float incoming_packet[INCOMING_FLOAT_COUNT];
    for (int ii = 0; ii < INCOMING_FLOAT_COUNT; ii++)
    {
        incoming_packet[ii] = 0;
    }

    uart->read();
    uart->available();
    while (true)
    {
        // read info from PI then write ekf data
        uint32_t now_micros = AP_HAL::micros();
        if (now_micros - last_micros > micros_step)
        {
            uint8_t avail = uart->available();
            if (avail >= INCOMING_PACKET_LENGTH)
            {
                hal.console->printf("serial available: %u\n", avail);
                uart->read(buf, avail);

                if (read_packet(buf, avail, incoming_packet))
                {
                    hal.console->printf("inc: ");
                    for (uint8_t ii = 0; ii < INCOMING_FLOAT_COUNT - 1; ii++)
                    {
                        hal.console->printf("%f, ", incoming_packet[ii]);
                    }
                    hal.console->printf("%f\n", incoming_packet[INCOMING_FLOAT_COUNT - 1]);
                    // xyz pos, xyz angle, xyz vel, xyz ang vel, xyz acc, xyz ang acc, motor rpm x4?
					// 22 floats * 4 + header + crc bytes = 90 bytes
					float_to_byte(10.4, buf);
                    uart->write(buf, 4);
                }
                
                uart->flush();
            }
            last_micros = now_micros;
        }
    }
}

uint8_t AP_Realtime_Control::read_packet(uint8_t *buffer, uint8_t length, float *float_packet)
{
    uint8_t *buf_window;
    for (uint8_t ii = 0; ii <= length - INCOMING_PACKET_LENGTH; ii++)
    {
        buf_window = buffer + ii;
        hal.console->printf("window: %u. ", ii);
        if (buf_window[0] == PACKET_HEADER && packet_crc(buffer))
        {
            for (uint8_t jj = 0; jj < INCOMING_FLOAT_COUNT; jj++)
            {
                float_packet[jj] = byte_to_float(buf_window + (jj * 4 + 1));
            }
            return 1;
        }
    }
    return 0;
}

uint8_t AP_Realtime_Control::packet_crc(uint8_t *buffer)
{
    return 1;
}

float AP_Realtime_Control::byte_to_float(uint8_t *fl_buf)
{
    float test;
    uint8_t bytes[] = {fl_buf[0], fl_buf[1], fl_buf[2], fl_buf[3]};
    memcpy(&test, &bytes, sizeof(test));
    return test;
}

void AP_Realtime_Control::float_to_byte(float in, uint8_t *out)
{
    uint32_t fbits = 0;
    memcpy(&fbits, &in, sizeof(fbits));
    out[0] = fbits & 0xFF;
    out[1] = (fbits >> 8) & 0xFF;
    out[2] = (fbits >> 16) & 0xFF;
    out[3] = (fbits >> 24) & 0xFF;
    hal.console->printf("hex: %x %x %x %x\n", out[0], out[1], out[2], out[3]);
}


namespace AP {
    AP_Realtime_Control *rt_control() {
        return AP_Realtime_Control::get_singleton();
    }
};

#endif // HAL_RTCTRL_ENABLED
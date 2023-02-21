/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  AP_Periph main firmware

  To flash this firmware on Linux use:

     st-flash write build/f103-periph/bin/AP_Periph.bin 0x8006000

 */
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include "AP_Periph.h"
#include <stdio.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#include <AP_HAL_ChibiOS/hwdef/common/stm32_util.h>
#include <AP_HAL_ChibiOS/hwdef/common/watchdog.h>
#include <AP_HAL_ChibiOS/I2CDevice.h>
#endif

#ifndef HAL_PERIPH_HWESC_SERIAL_PORT
#define HAL_PERIPH_HWESC_SERIAL_PORT 3
#endif

extern const AP_HAL::HAL &hal;

AP_Periph_FW periph;

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
void stm32_watchdog_init() {}
void stm32_watchdog_pat() {}
#endif

void setup(void)
{
    
    hal.serial(0)->begin(115200, 128, 128);
    hal.serial(1)->begin(115200, 128, 128);

    periph.init();
}

void loop(void)
{
    periph.update();
}

AP_Periph_FW::AP_Periph_FW()
{
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_Periph_FW must be singleton");
    }
    _singleton = this;
}

void AP_Periph_FW::init()
{
    
    stm32_watchdog_pat();

    // hal.serial(0)->begin(AP_SERIALMANAGER_CONSOLE_BAUD, 32, 32);

    load_parameters();

    stm32_watchdog_pat();

    can_start();

    serial_manager.init();

    stm32_watchdog_pat();

}

void AP_Periph_FW::update()
{
    hal.serial(0)->printf("hi on serial 0 x3\n");
    can_update();

    // hal.serial(1)->printf("hi on serial 1 is back in black");
    hal.scheduler->delay(50);



}


#ifdef HAL_PERIPH_LISTEN_FOR_SERIAL_UART_REBOOT_CMD_PORT
// check for uploader.py reboot command
void AP_Periph_FW::check_for_serial_reboot_cmd(const int8_t serial_index)
{
    // These are the string definitions in uploader.py
    //            NSH_INIT        = bytearray(b'\x0d\x0d\x0d')
    //            NSH_REBOOT_BL   = b"reboot -b\n"
    //            NSH_REBOOT      = b"reboot\n"

    // This is the command sequence that is sent from uploader.py
    //            self.__send(uploader.NSH_INIT)
    //            self.__send(uploader.NSH_REBOOT_BL)
    //            self.__send(uploader.NSH_INIT)
    //            self.__send(uploader.NSH_REBOOT)

    for (uint8_t i=0; i<hal.num_serial; i++) {
        if (serial_index >= 0 && serial_index != i) {
            // a specific serial port was selected but this is not it
            continue;
        }

        auto *uart = hal.serial(i);
        if (uart == nullptr || !uart->is_initialized()) {
            continue;
        }

        uint32_t available = MIN(uart->available(), 1000U);
        while (available-- > 0) {
            const char reboot_string[] = "\r\r\rreboot -b\n\r\r\rreboot\n";
            const char reboot_string_len = sizeof(reboot_string)-1; // -1 is to remove the null termination
            static uint16_t index[hal.num_serial];

            const int16_t data = uart->read();
            if (data < 0 || data > 0xff) {
                // read error
                continue;
            }
            if (index[i] >= reboot_string_len || (uint8_t)data != reboot_string[index[i]]) {
                // don't have a perfect match, start over
                index[i] = 0;
                continue;
            }
            index[i]++;
            if (index[i] == reboot_string_len) {
                // received reboot msg. Trigger a reboot and stay in the bootloader
                prepare_reboot();
                hal.scheduler->reboot(true);
            }
        }
    }
}
#endif // HAL_PERIPH_LISTEN_FOR_SERIAL_UART_REBOOT_CMD_PORT



// prepare for a safe reboot where PWMs and params are gracefully disabled
// This is copied from AP_Vehicle::reboot(bool hold_in_bootloader) minus the actual reboot
void AP_Periph_FW::prepare_reboot()
{
#ifdef HAL_PERIPH_ENABLE_RC_OUT
        // force safety on
        hal.rcout->force_safety_on();
#endif

        // flush pending parameter writes
        AP_Param::flush();

        // do not process incoming mavlink messages while we delay:
        hal.scheduler->register_delay_callback(nullptr, 5);

        // delay to give the ACK a chance to get out, the LEDs to flash,
        // the IO board safety to be forced on, the parameters to flush,
        hal.scheduler->delay(40);
}

AP_Periph_FW *AP_Periph_FW::_singleton;

AP_Periph_FW& AP::periph()
{
    return *AP_Periph_FW::get_singleton();
}

AP_HAL_MAIN();
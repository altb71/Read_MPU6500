/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include "mpu6500_spi.h"

// Blinking rate in milliseconds
#define BLINKING_RATE     500ms
SPI spi(PA_12,PA_11,PA_1);

static BufferedSerial serial_port(USBTX, USBRX);


int main()
{
    serial_port.set_baud(115200);
    serial_port.set_format(8,BufferedSerial::None,1);
    serial_port.set_blocking(false); // force to send whenever possible and data is there
    // Initialise the digital pin LED1 as an output
    DigitalOut led(LED1);
    mpu6500_spi imu(spi,PB_0);
    imu.init_inav();
    while (true) {
        led = !led;
        imu.readAcc();
        ThisThread::sleep_for(BLINKING_RATE);
        printf("%d %d %d \r\n",imu.accX_raw,imu.accY_raw,imu.accZ_raw);
    }
}

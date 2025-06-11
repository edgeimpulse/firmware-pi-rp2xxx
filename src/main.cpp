/* The Clear BSD License
 *
 * Copyright (c) 2025 EdgeImpulse Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 *   * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "FreeRTOS.h"
#include "ei_accelerometer.h"
#include "ei_analogsensor.h"
#include "ei_at_handlers.h"
#include "ei_classifier_porting.h"
#include "ei_device_raspberry_rp2xxx.h"
#include "ei_dht11sensor.h"
#include "ei_inertialsensor.h"
#include "ei_rp2xxx_internal_temperature.h"
#include "ei_run_impulse.h"
#include "ei_ultrasonicsensor.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "task.h"
#include <stdio.h>
#include <time.h>

#if defined(RASPBERRYPI_PICO2_W) || defined(RASPBERRYPI_PICO_W)
#include "pico/cyw43_arch.h"
#pragma message("Including WiFi support for Raspberry Pi Pico 2 W")
#endif

EiDeviceInfo *EiDevInfo = dynamic_cast<EiDeviceInfo *>(EiDeviceRP2xxx::get_device());
static ATServer *at;

void ei_init(void)
{
    EiDeviceRP2xxx *dev = static_cast<EiDeviceRP2xxx *>(EiDeviceRP2xxx::get_device());

    ei_sleep(2000); // Wait for the serial port to be ready

    ei_printf(
        "Hello from Edge Impulse\r\n"
        "Compiled on %s %s\r\n",
        __DATE__,
        __TIME__);

    // Setup internal temperature sensor on RP2350/RP2040
    ei_rp2xxxtemp_sensor_init();

    // Setup ADXL345 Accelerometer
    if (ei_accelerometer_init() == false) {
        ei_printf("ADXL345 initialization failed");
    }

    // Setup the inertial sensor
    if (ei_inertial_sensor_init() == false) {
        ei_printf("Inertial sensor communication error occurred\r\n");
    }

    // Setup the temp&humidity sensor
    if (ei_dht11_sensor_init() == false) {
        ei_printf("DHT11 initialization failed\r\n");
    }
    else {
        ei_printf("DHT11 initialization successful");
    }

    // Setup the ultrasonic sensor
    if (ei_ultrasonic_sensor_init() == false) {
        ei_printf("Ultrasonic ranger initialization failed\r\n");
    }

    if (ei_analog_sensor_init() == false) {
        ei_printf("ADC sensor initialization failed\r\n");
    }

    // cannot init device id before main() started on RP2XXX
    dev->init_device_id();
    dev->load_config();
    dev->set_state(eiStateFinished);

    // init AT command parser
    at = ei_at_init(dev);
    ei_printf("Type AT+HELP to see a list of commands.\r\n");
    at->print_prompt();
}

void ei_main(void *pvParameters)
{
    /* Initialize Edge Impulse sensors and commands */
    ei_init();

    while (true) {
        /* handle command comming from uart */
        ei_sleep(5);
        char data = ei_get_serial_byte();

        while (data != 0xFF) {
            at->handle(data);
            data = ei_get_serial_byte();
        }
    }
}

/* To verify FreeRTOS is working across both Pico targets */
void test_task(void *pvParameters)
{
    while (1) {
#if defined(RASPBERRYPI_PICO2_W) || defined(RASPBERRYPI_PICO_W)

        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1); // Turn LED on
        sleep_ms(2050); // Wait for 2.05 seconds
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0); // Turn LED off
        sleep_ms(1250); // Wait for 1.25 seconds

#elif defined(RASPBERRYPI_PICO2) || defined(RASPBERRYPI_PICO)

        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        sleep_ms(2050);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        sleep_ms(1250);

#endif
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second before repeating
    }
}

void vLaunch(void)
{
    TaskHandle_t task;

#if configUSE_CORE_AFFINITY && configNUMBER_OF_CORES > 1
    // we must bind the main task to one core (well at least while the init is called)
    vTaskCoreAffinitySet(task, 1);
#endif

    /* Start the tasks and timer running. */
    vTaskStartScheduler();
}

int main(void)
{

    stdio_init_all();

#ifdef RASPBERRYPI_PICO2_W
    if (cyw43_arch_init()) {
        ei_printf("Wi-Fi init failed\n");
        return -1;
    }
#endif

    xTaskCreate(ei_main, "ei_main", 1024, NULL, (tskIDLE_PRIORITY + 1), NULL);

    vTaskStartScheduler();

    while (1) { }

    return 0;
}
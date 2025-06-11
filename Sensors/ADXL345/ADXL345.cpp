/*
 *
 * Description: ADXL345 Library for Edge Impulse Ingestion Sampler and RP2XXX
 *
 * Author: Peter Ing
 * Date: 25 February 2025
 *
 *
 *
 * Licensed under the MIT License (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://opensource.org/licenses/MIT
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "ADXL345.h"

#define SENSITIVITY_FACTOR_2G (1.0f / 256.0f)
#define ADXL_I2C_TIMEOUT_US   (100 * 1000) // 10 milliseconds timeout

ADXL345::ADXL345(i2c_inst_t *i2cport)
    : _i2c(i2cport)
{
}

bool ADXL345::adxl345_init()
{

    uint8_t adxl_data[1], cmd_register_devid_base[1] = { 0x00 };

    i2c_init(_i2c, 400 * 1000);

    gpio_set_function(I2C1_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C1_SCL_PIN, GPIO_FUNC_I2C);

    int retval = i2c_write_timeout_us(
        _i2c,
        ADXL_I2C_ADDR,
        cmd_register_devid_base,
        1,
        true,
        ADXL_I2C_TIMEOUT_US);

    if (retval == PICO_ERROR_TIMEOUT) {

        return false; // catch the failed init and return
    }

    (void)i2c_read_blocking(_i2c, ADXL_I2C_ADDR, adxl_data, 1, false);
    // Should not get here if initial read failed, below left in case you wish to use async reads

    // int retval_r = i2c_read_timeout_us(_i2c, ADXL_I2C_ADDR, adxl_data, 1, false, ADXL_I2C_TIMEOUT_US);

    // if (retval_r == PICO_ERROR_TIMEOUT  ) {
    //     return false;
    // }

    if (adxl_data[0] != 0xE5) {
        return false; // result of initialization will be handled by sensors/ei_accelerometer
    }
    else {

        adxl345_setmode_measure();
        sleep_ms(2500);

        return true;
    }
}

bool ADXL345::adxl345_setmode_measure()
{

    const uint8_t cmd_measure_on[2] = { 0x2D, 0x08 };

    uint8_t res[2]; // can be used later

    res[0] = i2c_write_blocking(_i2c, ADXL_I2C_ADDR, cmd_measure_on, 2, true);
    (void)i2c_write_blocking(_i2c, ADXL_I2C_ADDR, cmd_measure_on, 1, true);
    (void)i2c_read_blocking(_i2c, ADXL_I2C_ADDR, (res + 1), 1, false);

    if (res[0] == 0) {
        return true;
    }
    else {
        return false;
    }
}

int ADXL345::adxl345_read_acceleration_XYZ(float &accel_x, float &accel_y, float &accel_z)
{

    uint8_t accel_data[6]; // for raw data
    int16_t temp_x, temp_y, temp_z;
    int num_bytes;

    const uint8_t cmd_register_datax0_base[1] = { 0x32 };

    i2c_write_blocking(_i2c, ADXL_I2C_ADDR, cmd_register_datax0_base, 1, true);
    num_bytes = i2c_read_blocking(_i2c, ADXL_I2C_ADDR, accel_data, 6, false);

    temp_x = (int16_t)((accel_data[1] << 8) | accel_data[0]);
    temp_y = (int16_t)((accel_data[3] << 8) | accel_data[2]);
    temp_z = (int16_t)((accel_data[5] << 8) | accel_data[4]);

    accel_x = temp_x * SENSITIVITY_FACTOR_2G;
    accel_y = temp_y * SENSITIVITY_FACTOR_2G;
    accel_z = temp_z * SENSITIVITY_FACTOR_2G;

    return num_bytes;
}

ADXL345 adxl_accel(i2c1); // using I2C1 to allow I2C0 to be available for Arduino Wire based libs

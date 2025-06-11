/*
 *
 * Description: ADXL345 Library for Edge Impulse and RP2XXX
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

#ifndef ADXL345_h
#define ADXL345_h

#include <hardware/i2c.h>
#include <pico/stdlib.h>
#include <stdio.h>

#define ADXL_I2C_ADDR 0x53
#define I2C1_SDA_PIN  18
#define I2C1_SCL_PIN  19

class ADXL345 {
public:
    ADXL345(i2c_inst_t *i2cport);
    bool adxl345_init(void);
    bool adxl345_setmode_measure(void);
    int adxl345_read_acceleration_XYZ(float &x, float &y, float &z);

private:
    i2c_inst_t *_i2c;
};
extern ADXL345 adxl_accel;
#endif
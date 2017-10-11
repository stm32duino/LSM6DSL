/**
 ******************************************************************************
 * @file    DISCO_IOT_LSM6DSL_DataLogTerminal.ino
 * @author  WI6LABS from AST
 * @version V1.0.0
 * @date    7 September 2017
 * @brief   Arduino test application for the STMicrolectronics STM32 DISCO_IOT
 *          MEMS Inertial and Environmental sensor expansion board.
 *          This application makes use of C++ classes obtained from the C
 *          components' drivers.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */


// Includes.
#include <LSM6DSLSensor.h>

#define SerialPort Serial
#define I2C2_SCL    PB10
#define I2C2_SDA    PB11

// Components.
LSM6DSLSensor *AccGyr;
TwoWire *dev_i2c;

void setup() {
  // Led.
  pinMode(LED_BUILTIN, OUTPUT);
  // Initialize serial for output.
  SerialPort.begin(9600);

  // Initialize I2C bus.
  dev_i2c = new TwoWire(I2C2_SDA, I2C2_SCL);
  dev_i2c->begin();

  // Initlialize components.
  AccGyr = new LSM6DSLSensor(dev_i2c, LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW);
  AccGyr->Enable_X();
  AccGyr->Enable_G();
}

void loop() {
  // Led blinking.
  digitalWrite(LED_BUILTIN, HIGH);
  delay(250);
  digitalWrite(LED_BUILTIN, LOW);
  delay(250);

  // Read accelerometer and gyroscope.
  int32_t accelerometer[3];
  int32_t gyroscope[3];
  AccGyr->Get_X_Axes(accelerometer);
  AccGyr->Get_G_Axes(gyroscope);

  // Output data.
  SerialPort.print("Acc[mg]: ");
  SerialPort.print(accelerometer[0]);
  SerialPort.print(" ");
  SerialPort.print(accelerometer[1]);
  SerialPort.print(" ");
  SerialPort.print(accelerometer[2]);
  SerialPort.print(" | Gyr[mdps]: ");
  SerialPort.print(gyroscope[0]);
  SerialPort.print(" ");
  SerialPort.print(gyroscope[1]);
  SerialPort.print(" ");
  SerialPort.println(gyroscope[2]);
}

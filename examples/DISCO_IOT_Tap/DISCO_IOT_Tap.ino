/**
 ******************************************************************************
 * @file    DISCO_IOT_Tap.ino
 * @author  WI6LABS from AST
 * @version V1.0.0
 * @date    7 September 2017
 * @brief   Arduino test application for the STMicrolectronics STM32 IOT Discovery Kit.
 *          MEMS Inertial and Environmental sensor expansion board.
 *          This application detects tap event through the LSM6DSL sensor.
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
#define INT1        PD11

// Components.
LSM6DSLSensor *AccGyr;
TwoWire *dev_i2c;

//Interrupts.
volatile int mems_event = 0;

void INT1Event_cb();

void setup() {
  // Led.
  pinMode(LED_BUILTIN, OUTPUT);

  // Initialize serial for output.
  SerialPort.begin(9600);

  // Initialize I2C bus.
  dev_i2c = new TwoWire(I2C2_SDA, I2C2_SCL);
  dev_i2c->begin();

  //Interrupts.
  attachInterrupt(INT1, INT1Event_cb, RISING);

  // Initlialize Components.
  AccGyr = new LSM6DSLSensor(dev_i2c, LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW);
  AccGyr->Enable_X();

  // Enable Single Tap Detection.
  AccGyr->Enable_Single_Tap_Detection();
}

void loop() {
  if (mems_event) {
    mems_event = 0;
    LSM6DSL_Event_Status_t status;
    AccGyr->Get_Event_Status(&status);
    if (status.TapStatus)
    {
      // Led blinking.
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);

      // Output data.
      SerialPort.println("Single Tap Detected!");
    }
  }
}

void INT1Event_cb()
{
  mems_event = 1;
}

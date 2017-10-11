/**
 ******************************************************************************
 * @file    DISCO_IOT__Pedometer.ino
 * @author  WI6LABS from AST
 * @version V1.0.0
 * @date    7 September 2017
 * @brief   Arduino test application for the STMicrolectronics STM32 IOT Discovery Kit.
 *          MEMS Inertial and Environmental sensor expansion board.
 *          This application detects step event through the LSM6DSL sensor.
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

uint32_t previous_tick = 0;
uint32_t current_tick = 0;
uint16_t step_count = 0;
char report[256];

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

  // Enable Pedometer.
  AccGyr->Enable_Pedometer();

  previous_tick = millis();
}

void loop() {
  if (mems_event)
  {
    mems_event = 0;
    LSM6DSL_Event_Status_t status;
    AccGyr->Get_Event_Status(&status);
    if (status.StepStatus)
    {
      // New step detected, so print the step counter
      AccGyr->Get_Step_Counter(&step_count);
      snprintf(report, sizeof(report), "Step counter: %d", step_count);
      SerialPort.println(report);

      // Led blinking.
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
    }
  }

  // Print the step counter in any case every 3000 ms
  current_tick = millis();
  if((current_tick - previous_tick) >= 3000)
  {
    AccGyr->Get_Step_Counter(&step_count);
    snprintf(report, sizeof(report), "Step counter: %d", step_count);
    SerialPort.println(report);
    previous_tick = millis();
  }
}

void INT1Event_cb()
{
  mems_event = 1;
}

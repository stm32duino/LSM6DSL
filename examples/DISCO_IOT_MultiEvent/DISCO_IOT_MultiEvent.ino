/**
 ******************************************************************************
 * @file    DISCO_IOT_MultiEvent.ino
 * @author  WI6LABS from AST
 * @version V1.0.0
 * @date    7 September 2017
 * @brief   Arduino test application for the STMicrolectronics STM32 IOT Discovery Kit.
 *          MEMS Inertial and Environmental sensor expansion board.
 *          This application detects free fall, tap, double tap, tilt, wake up,
 *          6D Orientation and step events through the LSM6DSL sensor.
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

/** NOTE
  The interrupt pin INT2 of the LSM6DSL is not connected on the Discovery L475VG
  IoT board. So the wakeup event is not used in this sketch to not decrease
  performances. If wakeup event is enabled on INT1 with all other events, this
  could cause some troubles.
*/

// Includes.
#include <LSM6DSLSensor.h>

#define SerialPort Serial
#define I2C2_SCL    PB10
#define I2C2_SDA    PB11
#define INT1        PD11

// Components.
LSM6DSLSensor *AccGyr;

//Interrupts.
volatile int mems_event = 0;

uint16_t step_count = 0;
char report[256];

void INT1Event_cb();
void sendOrientation();
TwoWire *dev_i2c;

void setup() {
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

  // Enable all HW events.
  AccGyr->Enable_Pedometer();
  AccGyr->Enable_Tilt_Detection();
  AccGyr->Enable_Free_Fall_Detection();
  AccGyr->Enable_Single_Tap_Detection();
  AccGyr->Enable_Double_Tap_Detection();
  AccGyr->Enable_6D_Orientation();
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
    }

    if (status.FreeFallStatus)
    {
      // Output data.
      SerialPort.println("Free Fall Detected!");
    }

    if (status.TapStatus)
    {
      // Output data.
      SerialPort.println("Single Tap Detected!");
    }

    if (status.DoubleTapStatus)
    {
      // Output data.
      SerialPort.println("Double Tap Detected!");
    }

    if (status.TiltStatus)
    {
      // Output data.
      SerialPort.println("Tilt Detected!");
    }

    if (status.D6DOrientationStatus)
    {
      // Send 6D Orientation
	    sendOrientation();
    }
  }
}

void INT1Event_cb()
{
  mems_event = 1;
}

void sendOrientation()
{
  uint8_t xl = 0;
  uint8_t xh = 0;
  uint8_t yl = 0;
  uint8_t yh = 0;
  uint8_t zl = 0;
  uint8_t zh = 0;

  AccGyr->Get_6D_Orientation_XL(&xl);
  AccGyr->Get_6D_Orientation_XH(&xh);
  AccGyr->Get_6D_Orientation_YL(&yl);
  AccGyr->Get_6D_Orientation_YH(&yh);
  AccGyr->Get_6D_Orientation_ZL(&zl);
  AccGyr->Get_6D_Orientation_ZH(&zh);

  if ( xl == 0 && yl == 0 && zl == 0 && xh == 0 && yh == 1 && zh == 0 )
  {
    sprintf( report, "\r\n  ________________  " \
                      "\r\n |                | " \
                      "\r\n |  *             | " \
                      "\r\n |                | " \
                      "\r\n |                | " \
                      "\r\n |                | " \
                      "\r\n |                | " \
                      "\r\n |________________| \r\n" );
  }

  else if ( xl == 1 && yl == 0 && zl == 0 && xh == 0 && yh == 0 && zh == 0 )
  {
    sprintf( report, "\r\n  ________________  " \
                      "\r\n |                | " \
                      "\r\n |             *  | " \
                      "\r\n |                | " \
                      "\r\n |                | " \
                      "\r\n |                | " \
                      "\r\n |                | " \
                      "\r\n |________________| \r\n" );
  }

  else if ( xl == 0 && yl == 0 && zl == 0 && xh == 1 && yh == 0 && zh == 0 )
  {
    sprintf( report, "\r\n  ________________  " \
                      "\r\n |                | " \
                      "\r\n |                | " \
                      "\r\n |                | " \
                      "\r\n |                | " \
                      "\r\n |                | " \
                      "\r\n |  *             | " \
                      "\r\n |________________| \r\n" );
  }

  else if ( xl == 0 && yl == 1 && zl == 0 && xh == 0 && yh == 0 && zh == 0 )
  {
    sprintf( report, "\r\n  ________________  " \
                      "\r\n |                | " \
                      "\r\n |                | " \
                      "\r\n |                | " \
                      "\r\n |                | " \
                      "\r\n |                | " \
                      "\r\n |             *  | " \
                      "\r\n |________________| \r\n" );
  }

  else if ( xl == 0 && yl == 0 && zl == 0 && xh == 0 && yh == 0 && zh == 1 )
  {
    sprintf( report, "\r\n  __*_____________  " \
                      "\r\n |________________| \r\n" );
  }

  else if ( xl == 0 && yl == 0 && zl == 1 && xh == 0 && yh == 0 && zh == 0 )
  {
    sprintf( report, "\r\n  ________________  " \
                      "\r\n |________________| " \
                      "\r\n    *               \r\n" );
  }

  else
  {
    sprintf( report, "None of the 6D orientation axes is set in LSM6DSL - accelerometer.\r\n" );
  }

  SerialPort.print(report);
}

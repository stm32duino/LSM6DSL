/**
 ******************************************************************************
 * @file    LSM6DSLSensor.h
 * @author  AST
 * @version V1.0.0
 * @date    7 September 2017
 * @brief   Abstract Class of an LSM6DSL Inertial Measurement Unit (IMU) 6 axes
 *          sensor.
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


/* Prevent recursive inclusion -----------------------------------------------*/

#ifndef __LSM6DSLSensor_H__
#define __LSM6DSLSensor_H__


/* Includes ------------------------------------------------------------------*/

#include "Wire.h"
#include "LSM6DSL_ACC_GYRO_Driver.h"

/* Defines -------------------------------------------------------------------*/

#define LSM6DSL_ACC_SENSITIVITY_FOR_FS_2G   0.061  /**< Sensitivity value for 2 g full scale [mg/LSB] */
#define LSM6DSL_ACC_SENSITIVITY_FOR_FS_4G   0.122  /**< Sensitivity value for 4 g full scale [mg/LSB] */
#define LSM6DSL_ACC_SENSITIVITY_FOR_FS_8G   0.244  /**< Sensitivity value for 8 g full scale [mg/LSB] */
#define LSM6DSL_ACC_SENSITIVITY_FOR_FS_16G  0.488  /**< Sensitivity value for 16 g full scale [mg/LSB] */

#define LSM6DSL_GYRO_SENSITIVITY_FOR_FS_125DPS   04.375  /**< Sensitivity value for 125 dps full scale [mdps/LSB] */
#define LSM6DSL_GYRO_SENSITIVITY_FOR_FS_245DPS   08.750  /**< Sensitivity value for 245 dps full scale [mdps/LSB] */
#define LSM6DSL_GYRO_SENSITIVITY_FOR_FS_500DPS   17.500  /**< Sensitivity value for 500 dps full scale [mdps/LSB] */
#define LSM6DSL_GYRO_SENSITIVITY_FOR_FS_1000DPS  35.000  /**< Sensitivity value for 1000 dps full scale [mdps/LSB] */
#define LSM6DSL_GYRO_SENSITIVITY_FOR_FS_2000DPS  70.000  /**< Sensitivity value for 2000 dps full scale [mdps/LSB] */

#define LSM6DSL_PEDOMETER_THRESHOLD_LOW       0x00  /**< Lowest  value of pedometer threshold */
#define LSM6DSL_PEDOMETER_THRESHOLD_MID_LOW   0x07
#define LSM6DSL_PEDOMETER_THRESHOLD_MID       0x0F
#define LSM6DSL_PEDOMETER_THRESHOLD_MID_HIGH  0x17
#define LSM6DSL_PEDOMETER_THRESHOLD_HIGH      0x1F  /**< Highest value of pedometer threshold */

#define LSM6DSL_WAKE_UP_THRESHOLD_LOW       0x01  /**< Lowest  value of wake up threshold */
#define LSM6DSL_WAKE_UP_THRESHOLD_MID_LOW   0x0F
#define LSM6DSL_WAKE_UP_THRESHOLD_MID       0x1F
#define LSM6DSL_WAKE_UP_THRESHOLD_MID_HIGH  0x2F
#define LSM6DSL_WAKE_UP_THRESHOLD_HIGH      0x3F  /**< Highest value of wake up threshold */

#define LSM6DSL_TAP_THRESHOLD_LOW       0x01  /**< Lowest  value of wake up threshold */
#define LSM6DSL_TAP_THRESHOLD_MID_LOW   0x08
#define LSM6DSL_TAP_THRESHOLD_MID       0x10
#define LSM6DSL_TAP_THRESHOLD_MID_HIGH  0x18
#define LSM6DSL_TAP_THRESHOLD_HIGH      0x1F  /**< Highest value of wake up threshold */

#define LSM6DSL_TAP_SHOCK_TIME_LOW       0x00  /**< Lowest  value of wake up threshold */
#define LSM6DSL_TAP_SHOCK_TIME_MID_LOW   0x01
#define LSM6DSL_TAP_SHOCK_TIME_MID_HIGH  0x02
#define LSM6DSL_TAP_SHOCK_TIME_HIGH      0x03  /**< Highest value of wake up threshold */

#define LSM6DSL_TAP_QUIET_TIME_LOW       0x00  /**< Lowest  value of wake up threshold */
#define LSM6DSL_TAP_QUIET_TIME_MID_LOW   0x01
#define LSM6DSL_TAP_QUIET_TIME_MID_HIGH  0x02
#define LSM6DSL_TAP_QUIET_TIME_HIGH      0x03  /**< Highest value of wake up threshold */

#define LSM6DSL_TAP_DURATION_TIME_LOW       0x00  /**< Lowest  value of wake up threshold */
#define LSM6DSL_TAP_DURATION_TIME_MID_LOW   0x04
#define LSM6DSL_TAP_DURATION_TIME_MID       0x08
#define LSM6DSL_TAP_DURATION_TIME_MID_HIGH  0x0C
#define LSM6DSL_TAP_DURATION_TIME_HIGH      0x0F  /**< Highest value of wake up threshold */

/* Typedefs ------------------------------------------------------------------*/
typedef enum
{
  LSM6DSL_STATUS_OK = 0,
  LSM6DSL_STATUS_ERROR,
  LSM6DSL_STATUS_TIMEOUT,
  LSM6DSL_STATUS_NOT_IMPLEMENTED
} LSM6DSLStatusTypeDef;

typedef enum
{
  LSM6DSL_INT1_PIN,
  LSM6DSL_INT2_PIN
} LSM6DSL_Interrupt_Pin_t;

typedef struct
{
  unsigned int FreeFallStatus : 1;
  unsigned int TapStatus : 1;
  unsigned int DoubleTapStatus : 1;
  unsigned int WakeUpStatus : 1;
  unsigned int StepStatus : 1;
  unsigned int TiltStatus : 1;
  unsigned int D6DOrientationStatus : 1;
} LSM6DSL_Event_Status_t;

/* Class Declaration ---------------------------------------------------------*/

/**
 * Abstract class of an LSM6DSL Inertial Measurement Unit (IMU) 6 axes
 * sensor.
 */
class LSM6DSLSensor
{
  public:
    LSM6DSLSensor                                     (TwoWire *i2c);
    LSM6DSLSensor                                     (TwoWire *i2c, uint8_t address);
    LSM6DSLStatusTypeDef Enable_X                     (void);
    LSM6DSLStatusTypeDef Enable_G                     (void);
    LSM6DSLStatusTypeDef Disable_X                    (void);
    LSM6DSLStatusTypeDef Disable_G                    (void);
    LSM6DSLStatusTypeDef ReadID                       (uint8_t *p_id);
    LSM6DSLStatusTypeDef Get_X_Axes                   (int32_t *pData);
    LSM6DSLStatusTypeDef Get_G_Axes                   (int32_t *pData);
    LSM6DSLStatusTypeDef Get_X_Sensitivity            (float *pfData);
    LSM6DSLStatusTypeDef Get_G_Sensitivity            (float *pfData);
    LSM6DSLStatusTypeDef Get_X_AxesRaw                (int16_t *pData);
    LSM6DSLStatusTypeDef Get_G_AxesRaw                (int16_t *pData);
    LSM6DSLStatusTypeDef Get_X_ODR                    (float *odr);
    LSM6DSLStatusTypeDef Get_G_ODR                    (float *odr);
    LSM6DSLStatusTypeDef Set_X_ODR                    (float odr);
    LSM6DSLStatusTypeDef Set_G_ODR                    (float odr);
    LSM6DSLStatusTypeDef Get_X_FS                     (float *fullScale);
    LSM6DSLStatusTypeDef Get_G_FS                     (float *fullScale);
    LSM6DSLStatusTypeDef Set_X_FS                     (float fullScale);
    LSM6DSLStatusTypeDef Set_G_FS                     (float fullScale);
    LSM6DSLStatusTypeDef Enable_Free_Fall_Detection   (void);
    LSM6DSLStatusTypeDef Enable_Free_Fall_Detection   (LSM6DSL_Interrupt_Pin_t int_pin);
    LSM6DSLStatusTypeDef Disable_Free_Fall_Detection  (void);
    LSM6DSLStatusTypeDef Set_Free_Fall_Threshold      (uint8_t thr);
    LSM6DSLStatusTypeDef Enable_Pedometer             (void);
    LSM6DSLStatusTypeDef Disable_Pedometer            (void);
    LSM6DSLStatusTypeDef Get_Step_Counter             (uint16_t *step_count);
    LSM6DSLStatusTypeDef Reset_Step_Counter           (void);
    LSM6DSLStatusTypeDef Set_Pedometer_Threshold      (uint8_t thr);
    LSM6DSLStatusTypeDef Enable_Tilt_Detection        (void);
    LSM6DSLStatusTypeDef Enable_Tilt_Detection        (LSM6DSL_Interrupt_Pin_t int_pin);
    LSM6DSLStatusTypeDef Disable_Tilt_Detection       (void);
    LSM6DSLStatusTypeDef Enable_Wake_Up_Detection     (void);
    LSM6DSLStatusTypeDef Enable_Wake_Up_Detection     (LSM6DSL_Interrupt_Pin_t int_pin);
    LSM6DSLStatusTypeDef Disable_Wake_Up_Detection    (void);
    LSM6DSLStatusTypeDef Set_Wake_Up_Threshold        (uint8_t thr);
    LSM6DSLStatusTypeDef Enable_Single_Tap_Detection  (void);
    LSM6DSLStatusTypeDef Enable_Single_Tap_Detection  (LSM6DSL_Interrupt_Pin_t int_pin);
    LSM6DSLStatusTypeDef Disable_Single_Tap_Detection (void);
    LSM6DSLStatusTypeDef Enable_Double_Tap_Detection  (void);
    LSM6DSLStatusTypeDef Enable_Double_Tap_Detection  (LSM6DSL_Interrupt_Pin_t int_pin);
    LSM6DSLStatusTypeDef Disable_Double_Tap_Detection (void);
    LSM6DSLStatusTypeDef Set_Tap_Threshold            (uint8_t thr);
    LSM6DSLStatusTypeDef Set_Tap_Shock_Time           (uint8_t time);
    LSM6DSLStatusTypeDef Set_Tap_Quiet_Time           (uint8_t time);
    LSM6DSLStatusTypeDef Set_Tap_Duration_Time        (uint8_t time);
    LSM6DSLStatusTypeDef Enable_6D_Orientation        (void);
    LSM6DSLStatusTypeDef Enable_6D_Orientation        (LSM6DSL_Interrupt_Pin_t int_pin);
    LSM6DSLStatusTypeDef Disable_6D_Orientation       (void);
    LSM6DSLStatusTypeDef Get_6D_Orientation_XL        (uint8_t *xl);
    LSM6DSLStatusTypeDef Get_6D_Orientation_XH        (uint8_t *xh);
    LSM6DSLStatusTypeDef Get_6D_Orientation_YL        (uint8_t *yl);
    LSM6DSLStatusTypeDef Get_6D_Orientation_YH        (uint8_t *yh);
    LSM6DSLStatusTypeDef Get_6D_Orientation_ZL        (uint8_t *zl);
    LSM6DSLStatusTypeDef Get_6D_Orientation_ZH        (uint8_t *zh);
	LSM6DSLStatusTypeDef Get_Event_Status             (LSM6DSL_Event_Status_t *status);
    LSM6DSLStatusTypeDef ReadReg                      (uint8_t reg, uint8_t *data);
    LSM6DSLStatusTypeDef WriteReg                     (uint8_t reg, uint8_t data);
	
	/**
     * @brief Utility function to read data.
     * @param  pBuffer: pointer to data to be read.
     * @param  RegisterAddr: specifies internal address register to be read.
     * @param  NumByteToRead: number of bytes to be read.
     * @retval 0 if ok, an error code otherwise.
     */
    uint8_t IO_Read(uint8_t* pBuffer, uint8_t RegisterAddr, uint16_t NumByteToRead)
    {
      dev_i2c->beginTransmission(((uint8_t)(((address) >> 1) & 0x7F)));
      dev_i2c->write(RegisterAddr);
      dev_i2c->endTransmission(false);

      dev_i2c->requestFrom(((uint8_t)(((address) >> 1) & 0x7F)), (byte) NumByteToRead);

      int i=0;
      while (dev_i2c->available())
      {
        pBuffer[i] = dev_i2c->read();
        i++;
      }

      return 0;
    }
    
    /**
     * @brief Utility function to write data.
     * @param  pBuffer: pointer to data to be written.
     * @param  RegisterAddr: specifies internal address register to be written.
     * @param  NumByteToWrite: number of bytes to write.
     * @retval 0 if ok, an error code otherwise.
     */
    uint8_t IO_Write(uint8_t* pBuffer, uint8_t RegisterAddr, uint16_t NumByteToWrite)
    {
      dev_i2c->beginTransmission(((uint8_t)(((address) >> 1) & 0x7F)));

      dev_i2c->write(RegisterAddr);
      for (int i = 0 ; i < NumByteToWrite ; i++)
        dev_i2c->write(pBuffer[i]);

      dev_i2c->endTransmission(true);

      return 0;
    }

  private:
    LSM6DSLStatusTypeDef Set_X_ODR_When_Enabled(float odr);
    LSM6DSLStatusTypeDef Set_G_ODR_When_Enabled(float odr);
    LSM6DSLStatusTypeDef Set_X_ODR_When_Disabled(float odr);
    LSM6DSLStatusTypeDef Set_G_ODR_When_Disabled(float odr);

    /* Helper classes. */
    TwoWire *dev_i2c;

    /* Configuration */
    uint8_t address;

    uint8_t X_isEnabled;
    float X_Last_ODR;
    uint8_t G_isEnabled;
    float G_Last_ODR;
};

#ifdef __cplusplus
extern "C" {
#endif
uint8_t LSM6DSL_IO_Write( void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite );
uint8_t LSM6DSL_IO_Read( void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead );
#ifdef __cplusplus
}
#endif

#endif
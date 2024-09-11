/**
  ******************************************************************************
  * @file         I2C_HAL.h
  * @brief        File containing tools needed when working with I2S
  *               functions in the HAL library
  * @attention    This file is not included in the library and is connected
  *               by the user separately
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include "PCA9685.h"
#include "stm32f1xx_hal.h"

extern I2C_HandleTypeDef hi2c1; //Insert your I2S channel
/* Initialisation functions --------------------------------------------------*/
PCA9685_Stat M_RX(unsigned char *data_ptr, unsigned short size_of_data);
PCA9685_Stat M_TX(unsigned char reset, unsigned char *data_ptr, unsigned short size_of_data);

/* Exported structure ---------------------------------------------------------*/
/** @brief function containing executable methods*/
PCA9685_Func PCA = {.M_RX = M_RX, .M_TX = M_TX};
/** @brief structure containing the data necessary for working with the library
 *         about the device of the system used*/
PCA9685_TypeDef PCA_1 = {.methods_typedef = &PCA, .dev_addr=PCA9685_ADDRESS};
/* Exported functions --------------------------------------------------------*/
/**
  * @brief  Pointer to the Master_Receive function
  * @param  data_ptr Pointer to data buffer
  * @param  size_of_data Amount of data to be sent
  */
PCA9685_Stat M_RX(unsigned char *data_ptr, unsigned short size_of_data)
{
    if(HAL_I2C_Master_Receive(&hi2c1, (uint16_t )((PCA_1.dev_addr<<1)|0x01), data_ptr, size_of_data, 1000) != HAL_OK)
    {
        return PCA9685_ERROR;
    }
    return PCA9685_OK;
}
/**
  * @brief  Pointer to the Master_Transmit function
  * @param  data_ptr Pointer to data buffer
  * @param  size_of_data Amount of data to be sent
  */
PCA9685_Stat M_TX(unsigned char reset, unsigned char *data_ptr, unsigned short size_of_data)
{
    if (reset)
    {
        if(HAL_I2C_Master_Transmit(&hi2c1, (uint16_t )0x00, data_ptr, size_of_data, 1000) != HAL_OK)
        {
            return PCA9685_ERROR;
        }
    }
    else{
    if(HAL_I2C_Master_Transmit(&hi2c1, (uint16_t )(PCA_1.dev_addr<<1), data_ptr, size_of_data, 1000) != HAL_OK)
    {
        return PCA9685_ERROR;
    }}
    return PCA9685_OK;
}
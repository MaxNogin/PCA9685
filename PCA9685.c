/**
  ******************************************************************************
  * @file         PCA9685.c
  * @brief        This file provides code for the creating PWM signal by PCA9685
  *               controller. Includes the implementation of functions that control
  *               the operation of this board
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "PCA9685.h"
/* Exported functions --------------------------------------------------------*/
/**
  * @brief  Function that changes the value of a specific bit in a register
  * @param  Struct User-created structure that contains the solution methods
  *                and the device address
  * @param  Register The address of the internal memory containing the bits
  *                  responsible for the operation of the device
  * @param  Bit Variable register bit
  * @param  Value Contains the mutable state of the bit being used
  */
PCA9685_Stat PCA9685_SetRegBit(PCA9685_TypeDef *Struct, unsigned char Register, unsigned char Bit, unsigned char Value)
{
    unsigned char Data[2];
    if (Value) Value = 1;
    Data[0] = Register;
    Struct->methods_typedef->M_RX(Data, 2);
    if (Value == 0) Data[1] &= ~(1 << Bit);
    else Data[1] |= (1 << Bit);
    Struct->methods_typedef->M_TX(0,Data, 2);
    return PCA9685_OK;
}
/**
  * @brief  Provides a software reset of the device
 * @param  Struct User-created structure that contains the solution methods
  *                and the device address
  */
PCA9685_Stat PCA9685_Reset(PCA9685_TypeDef *Struct)
{
    unsigned char cmd = 0x6;
    Struct->methods_typedef->M_TX(1,&cmd, 1);
    return PCA9685_OK;
}
/**
  * @brief  Restart all of the previously active PWM channels
  * @param  Struct User-created structure that contains the solution methods
  *                and the device address
  * @param  Enable Means the value of the register
  */
PCA9685_Stat PCA9685_RestartMode(PCA9685_TypeDef *Struct, unsigned char Enable)
{
    return PCA9685_SetRegBit(Struct, PCA9685_MODE1, PCA9685_MODE1_RESTART_BIT, Enable);
}
/**
  * @brief  Function that causes the control register to be automatically
  *         incremented after a read or write.
 * @param  Struct User-created structure that contains the solution methods
  *                and the device address
  * @param  Enable means the value of the register
  */
PCA9685_Stat PCA9685_AutoIncrement(PCA9685_TypeDef *Struct, unsigned char Enable)
{
    return PCA9685_SetRegBit(Struct,PCA9685_MODE1, PCA9685_MODE1_AI_BIT, Enable);
}
/**
  * @brief  Switches the controller to power saving mode and turns off the
  *         internal generator
 * @param  Struct User-created structure that contains the solution methods
  *                and the device address
  * @param  Enable means the value of the register
  */
PCA9685_Stat PCA9685_SleepMode(PCA9685_TypeDef *Struct, unsigned char Enable)
{
    return PCA9685_SetRegBit(Struct,PCA9685_MODE1, PCA9685_MODE1_SLEEP_BIT, Enable);
}
/**
  * @brief  Allows the controller to respond to an additional address of one
  *         of the available buses
 * @param  Struct User-created structure that contains the solution methods
  *                and the device address
  * @param  Enable means the value of the register
  */
PCA9685_Stat PCA9685_SubAddress_Respond(PCA9685_TypeDef *Struct, SubAddrBit SubAddress, unsigned char Enable)
{
    return PCA9685_SetRegBit(Struct,PCA9685_MODE1, SubAddress, Enable);
}
/**
  * @brief  Allows the controller to respond to the I2C bus shared call address
 * @param  Struct User-created structure that contains the solution methods
  *                and the device address
  * @param  Enable means the value of the register
  */
PCA9685_Stat PCA9685_AllCallRespond(PCA9685_TypeDef *Struct, unsigned char Enable)
{
    return PCA9685_SetRegBit(Struct,PCA9685_MODE1, PCA9685_MODE1_ALL_CALL_BIT, Enable);
}
/**
  * @brief  Enable inverting of the output signal
 * @param  Struct User-created structure that contains the solution methods
  *                and the device address
  * @param  Enable means the value of the register
  */
PCA9685_Stat PCA9685_InvertSign(PCA9685_TypeDef *Struct, unsigned char Enable)
{
    return PCA9685_SetRegBit(Struct,PCA9685_MODE2, PCA9685_MODE2_INVRT_BIT, Enable);
}
/**
  * @brief  Sets the value of the PWM frequency
 * @param  Struct User-created structure that contains the solution methods
  *               and the device address
  * @param  Frequency frequency of PWM operation
  */
PCA9685_Stat PCA9685_SetPWMFrequency(PCA9685_TypeDef *Struct, unsigned short Frequency)
{
    unsigned char Data[2];
    Data[0] = PCA9685_PRE_SCALE;
    if(Frequency >= PCA9685_MAX_FREQ) Data[1] = 0x03;
    else if(Frequency <= PCA9685_MIN_FREQ) Data[1] = 0xFF;
    else Data[1] = CLOCK_FREQ / (4096 * Frequency);//Timer clock is 25 MHz

    /*To change the frequency, PCA9685 have to be in Sleep mode*/
    PCA9685_SleepMode(Struct,1);
    Struct->methods_typedef->M_TX(0, Data, 2);
    PCA9685_SleepMode(Struct,0);
    PCA9685_RestartMode(Struct,1);
    return PCA9685_OK;
}
/**
  * @brief  Setting up the PWM operation
 * @param  Struct User-created structure that contains the solution methods
  *               and the device address
  * @param  Channel Is the value of the PWM output specified on the device
  * @param  OnTime pulse duration
  * @param  OffTime pulse delay time
  */
PCA9685_Stat PCA9685_SetPWM(PCA9685_TypeDef *Struct, unsigned char Channel, unsigned short OnTime, unsigned short OffTime)
{
    unsigned char pwm[5];
    pwm[0] = PCA9685_LED0_ON_L + (4 * Channel);
    pwm[1] = OnTime & 0xFF;
    pwm[2] = OnTime>>8;
    pwm[3] = OffTime & 0xFF;
    pwm[4] = OffTime>>8;
    Struct->methods_typedef->M_TX(0, pwm, 5);
    return PCA9685_OK;
}
#ifdef PCA9685_SERVO_MODE
/**
  * @brief  Set the rotation of servo
 * @param  Struct User-created structure that contains the solution methods
  *               and the device address
  * @param  Channel Is the value of the PWM output specified on the device
  * @param  ServoAngle Max rotation angle of servo in degrees
  * @param  MaxAngle Max angle of rotation for servo, when the PWM frequency is set
  * @param  MinAngle Min angle of rotation for servo, when the PWM frequency is set
  * @param  Rotation Desired servo rotation angle
  */
PCA9685_Stat PCA9685_ServoAngle(PCA9685_TypeDef *Struct, unsigned char Channel, unsigned short ServoAngle, unsigned short MaxAngle, unsigned short MinAngle, unsigned short Rotation) {
    unsigned short Value;
    if (Rotation > ServoAngle) Rotation = ServoAngle;
    if (Rotation < 0) Rotation = 0;
    Value = (Rotation * (MaxAngle - MinAngle) / ServoAngle) + MinAngle;
    return PCA9685_SetPWM(Struct, Channel, Value, 0);
}
#endif
/**
  * @brief  Initializing the PWM operation
 * @param  Struct User-created structure that contains the solution methods
  *               and the device address
  * @param  Frequency frequency of PWM operation
  */
PCA9685_Stat PCA9685_Init(PCA9685_TypeDef *Struct, unsigned short Frequency)
{
    PCA9685_Reset(Struct);
    PCA9685_SetPWMFrequency(Struct, Frequency);
    PCA9685_AutoIncrement(Struct,1);
    return PCA9685_OK;
}
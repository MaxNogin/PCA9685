/**
  ******************************************************************************
  * @file         PCA9685.h
  * @brief        This file provides code for the Initialization codes for
  *               creating PWM signal by PCA9685 controller. Is the main library
  *               file, including the board registers.
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef PCA9685_LIBRARY_H
#define PCA9685_LIBRARY_H

#ifdef __cplusplus
extern "C" {
#endif

/*Comment or uncomment the defines you need depending on the tools
 * and libraries you use*/
#define PCA9685_SERVO_MODE    /**< Includes functions necessary when working with a servo drive*/

/* Exported types ------------------------------------------------------------*/
/**
 * @brief  PCA9685 structure containing the statuses of library functions
 */
typedef enum
{
    PCA9685_OK 		= 0,
    PCA9685_ERROR	= 1,
}PCA9685_Stat;
/**
  * @brief  A structure containing links to functions that work
  *         with this library
  */
typedef struct
{
    PCA9685_Stat (*M_RX)(unsigned char *, unsigned short);
    PCA9685_Stat (*M_TX)(unsigned char, unsigned char *, unsigned short);
}PCA9685_Func;
/**
  * @brief  A structure containing the address values of the PCA9685 board
  *         and functions that you will need to set when working with this
  *         library, or connect one of the attached files, depending on the
  *         system
  */
typedef struct
{
    PCA9685_Func * methods_typedef;
    unsigned char dev_addr;
    /* NOTE : In the main file, you need to create a structure of this type,
        * containing your function declaration
    */
}PCA9685_TypeDef;

/* Exported macro ------------------------------------------------------------*/


#define PCA9685_ADDRESS 0x40            /**< Default device address*/

/* Registers addresses from data sheet*/
#define PCA9685_MODE1 0x00             /**< Mode Register 1*/
#define PCA9685_MODE2 0x01             /**< Mode Register 2*/
#define PCA9685_SUBADR1 0x02           /**< I2C-bus subaddress 1*/
#define PCA9685_SUBADR2 0x03           /**< I2C-bus subaddress 2*/
#define PCA9685_SUBADR3 0x04           /**< I2C-bus subaddress 3*/
#define PCA9685_ALLCALLADR 0x05        /**< LED All call I2C-bus address*/

/* All channels registers addresses*/
#define PCA9685_LED0_ON_L 0x06         /**< LED0 on tick, low bite */
#define PCA9685_LED0_ON_H 0x07         /**< LED0 on tick, high bite */
#define PCA9685_LED0_OFF_L 0x08        /**< LED0 off tick, low bite */
#define PCA9685_LED0_OFF_H 0x09        /**< LED0 off tick, high bite */

#define PCA9685_LED1_ON_L 0x0A         /**< LED1 on tick, low bite */
#define PCA9685_LED1_ON_H 0x0B         /**< LED1 on tick, high bite */
#define PCA9685_LED1_OFF_L 0x0C        /**< LED1 off tick, low bite */
#define PCA9685_LED1_OFF_H 0x0D        /**< LED1 off tick, high bite */

#define PCA9685_LED2_ON_L 0x0E         /**< LED2 on tick, low bite */
#define PCA9685_LED2_ON_H 0x0F         /**< LED2 on tick, high bite */
#define PCA9685_LED2_OFF_L 0x10        /**< LED2 off tick, low bite */
#define PCA9685_LED2_OFF_H 0x11        /**< LED2 off tick, high bite */

#define PCA9685_LED3_ON_L 0x12         /**< LED3 on tick, low bite */
#define PCA9685_LED3_ON_H 0x13         /**< LED3 on tick, high bite */
#define PCA9685_LED3_OFF_L 0x14        /**< LED3 off tick, low bite */
#define PCA9685_LED3_OFF_H 0x15        /**< LED3 off tick, high bite */

#define PCA9685_LED4_ON_L 0x16         /**< LED4 on tick, low bite */
#define PCA9685_LED4_ON_H 0x17         /**< LED4 on tick, high bite */
#define PCA9685_LED4_OFF_L 0x18        /**< LED4 off tick, low bite */
#define PCA9685_LED4_OFF_H 0x19        /**< LED4 off tick, high bite */

#define PCA9685_LED5_ON_L 0x1A         /**< LED5 on tick, low bite */
#define PCA9685_LED5_ON_H 0x1B         /**< LED5 on tick, high bite */
#define PCA9685_LED5_OFF_L 0x1C        /**< LED5 off tick, low bite */
#define PCA9685_LED5_OFF_H 0x1D        /**< LED5 off tick, high bite */

#define PCA9685_LED6_ON_L 0x1E         /**< LED6 on tick, low bite */
#define PCA9685_LED6_ON_H 0x1F         /**< LED6 on tick, high bite */
#define PCA9685_LED6_OFF_L 0x20        /**< LED6 off tick, low bite */
#define PCA9685_LED6_OFF_H 0x21        /**< LED6 off tick, high bite */

#define PCA9685_LED7_ON_L 0x22         /**< LED7 on tick, low bite */
#define PCA9685_LED7_ON_H 0x23         /**< LED7 on tick, high bite */
#define PCA9685_LED7_OFF_L 0x24        /**< LED7 off tick, low bite */
#define PCA9685_LED7_OFF_H 0x25        /**< LED7 off tick, high bite */

#define PCA9685_LED8_ON_L 0x26         /**< LED8 on tick, low bite */
#define PCA9685_LED8_ON_H 0x27         /**< LED8 on tick, high bite */
#define PCA9685_LED8_OFF_L 0x28        /**< LED8 off tick, low bite */
#define PCA9685_LED8_OFF_H 0x29        /**< LED8 off tick, high bite */

#define PCA9685_LED9_ON_L 0x2A         /**< LED9 on tick, low bite */
#define PCA9685_LED9_ON_H 0x2B         /**< LED9 on tick, high bite */
#define PCA9685_LED9_OFF_L 0x2C        /**< LED9 off tick, low bite */
#define PCA9685_LED9_OFF_H 0x2D        /**< LED9 off tick, high bite */

#define PCA9685_LED10_ON_L 0x2E        /**< LED10 on tick, low bite */
#define PCA9685_LED10_ON_H 0x2F        /**< LED10 on tick, high bite */
#define PCA9685_LED10_OFF_L 0x30       /**< LED10 off tick, low bite */
#define PCA9685_LED10_OFF_H 0x31       /**< LED10 off tick, high bite */

#define PCA9685_LED11_ON_L 0x32        /**< LED11 on tick, low bite */
#define PCA9685_LED11_ON_H 0x33        /**< LED11 on tick, high bite */
#define PCA9685_LED11_OFF_L 0x34       /**< LED11 off tick, low bite */
#define PCA9685_LED11_OFF_H 0x35       /**< LED11 off tick, high bite */

#define PCA9685_LED12_ON_L 0x36        /**< LED12 on tick, low bite */
#define PCA9685_LED12_ON_H 0x37        /**< LED12 on tick, high bite */
#define PCA9685_LED12_OFF_L 0x38       /**< LED12 off tick, low bite */
#define PCA9685_LED12_OFF_H 0x39       /**< LED12 off tick, high bite */

#define PCA9685_LED13_ON_L 0x3A        /**< LED13 on tick, low bite */
#define PCA9685_LED13_ON_H 0x3B        /**< LED13 on tick, high bite */
#define PCA9685_LED13_OFF_L 0x3C       /**< LED13 off tick, low bite */
#define PCA9685_LED13_OFF_H 0x3D       /**< LED13 off tick, high bite */

#define PCA9685_LED14_ON_L 0x3E        /**< LED14 on tick, low bite */
#define PCA9685_LED14_ON_H 0x3F        /**< LED14 on tick, high bite */
#define PCA9685_LED14_OFF_L 0x40       /**< LED14 off tick, low bite */
#define PCA9685_LED14_OFF_H 0x41       /**< LED14 off tick, high bite */

#define PCA9685_LED15_ON_L 0x42        /**< LED15 on tick, low bite */
#define PCA9685_LED15_ON_H 0x43        /**< LED15 on tick, high bite */
#define PCA9685_LED15_OFF_L 0x44       /**< LED15 off tick, low bite */
#define PCA9685_LED15_OFF_H 0x45       /**< LED15 off tick, high bite */

#define PCA9685_ALL_LED_ON_L 0xFA      /**< load all the LEDn_ON registers, low*/
#define PCA9685_ALL_LED_ON_H 0xFB      /**< load all the LEDn_ON registers, high*/
#define PCA9685_ALL_LED_OFF_L 0xFC     /**< load all the LEDn_OFF registers, low*/
#define PCA9685_ALL_LED_OFF_H 0xFD     /**< load all the LEDn_OFF registers, high*/


#define PCA9685_PRE_SCALE 0xFE         /**< Prescaler for PWM output frequency*/
#define PCA9685_TEST_MODE 0xFF         /**< defines the test mode to be entered*/


/*Setting register MODE1 bits*/
#define PCA9685_MODE1_ALL_CALL_BIT 0    /**< Respond to LED All Call I2C-bus address*/
typedef enum
{
    PCA9685_MODE1_SUB3_BIT 	= 3,        /**< Respond to I2C-bus subaddress 3*/
    PCA9685_MODE1_SUB2_BIT	= 2,        /**< Respond to I2C-bus subaddress 2*/
    PCA9685_MODE1_SUB1_BIT	= 1         /**< Respond to I2C-bus subaddress 1*/
}SubAddrBit;
#define PCA9685_MODE1_SLEEP_BIT 4       /**< Low power mode*/
#define PCA9685_MODE1_AI_BIT 5          /**< Auto-increment*/
#define PCA9685_MODE1_EXT_CLK_BIT 6     /**< Use clock source*/
#define PCA9685_MODE1_RESTART_BIT 7     /**< Restart*/


/*Setting register MODE2 bits*/
#define PCA9685_MODE2_OUT_NE_BIT 00
#define PCA9685_MODE2_OUT_DV_BIT 2
#define PCA9685_MODE2_OCH_BIT 3         /*outputs change*/
#define PCA9685_MODE2_INVRT_BIT 4      /*inverting the output signal*/

/*LED output frequency MIN and MAX values*/
#define PCA9685_MIN_FREQ 24             /**< Min frequency for control PWM*/
#define PCA9685_MAX_FREQ 1526           /**< Max frequency for control PWM*/
#define CLOCK_FREQ 25000000

/* Initialization functions --------------------------------------------------------*/
PCA9685_Stat PCA9685_SetRegBit(PCA9685_TypeDef *Struct, unsigned char Register, unsigned char Bit, unsigned char Value);
PCA9685_Stat PCA9685_Reset(PCA9685_TypeDef *Struct);
PCA9685_Stat PCA9685_RestartMode(PCA9685_TypeDef *Struct, unsigned char Enable);
PCA9685_Stat PCA9685_AutoIncrement(PCA9685_TypeDef *Struct,unsigned char Enable);
PCA9685_Stat PCA9685_SubAddress_Respond(PCA9685_TypeDef *Struct,SubAddrBit SubAddress, unsigned char Enable);
PCA9685_Stat PCA9685_AllCallRespond(PCA9685_TypeDef *Struct,unsigned char Enable);
PCA9685_Stat PCA9685_SleepMode(PCA9685_TypeDef *Struct,unsigned char Enable);
PCA9685_Stat PCA9685_InvertSign(PCA9685_TypeDef *Struct,unsigned char Enable);

PCA9685_Stat PCA9685_SetPWMFrequency(PCA9685_TypeDef *Struct,unsigned short frequency);

PCA9685_Stat PCA9685_SetPWM(PCA9685_TypeDef *Struct,unsigned char Channel, unsigned short OnTime, unsigned short OffTime);
#ifdef PCA9685_SERVO_MODE
PCA9685_Stat PCA9685_ServoAngle(PCA9685_TypeDef *Struct,unsigned char Channel, unsigned short ServoAngle, unsigned short MaxAngle, unsigned short MinAngle, unsigned short Rotation);
#endif
PCA9685_Stat PCA9685_Init(PCA9685_TypeDef *Struct, unsigned short Frequency);

#ifdef __cplusplus
}
#endif

#endif /*PCA9685_LIBRARY_H*/
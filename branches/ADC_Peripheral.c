/*********************************************************************
 * INCLUDES
 */

#include"app_include.h"



/*********************************************************************
 * GLOBAL VARIABLES
 */



/*********************************************************************
 * FUNCTIONS
 *********************************************************************/

/*********************************************************************
 * @fn     
 *
 * @brief   
 *
 * @param   
 *
 * @return  none
 */


/*********************************************************************
*********************************************************************/

void adc_out_value(uint8 *pdata,uint16 addr)
{
  uint8 senddata[4];
  senddata[AF_SIG_STIUATION] = AF_DIRECT_TRAN_SIG;
  if(pdata[0] == 0)
  {
    senddata[1] =  0x1;
    senddata[2] =  readTemp();
    senddata[3] =  readVoltage();
    TRAN_NETWORK_SYSTEM_TO_NODE(addr,senddata,4,NETWORK_SYSTEM_CLUSTERID_ADC_ORDER_NETWORK);
  }
   
}


int8 readTemp(void)
{
  static uint16 voltageAtTemp22;
  static uint8 bCalibrate=TRUE; // Calibrate the first time the temp sensor is read
  uint16 value;
  int8 temp;

  #if defined (HAL_MCU_CC2530)
  ATEST = 0x01;
  TR0  |= 0x01; 
  
  /* Clear ADC interrupt flag */
  ADCIF = 0;

  ADCCON3 = (HAL_ADC_REF_125V | HAL_ADC_DEC_512 | HAL_ADC_CHN_TEMP);

  /* Wait for the conversion to finish */
  while ( !ADCIF );

  /* Get the result */
  value = ADCL;
  value |= ((uint16) ADCH) << 8;

  // Use the 12 MSB of adcValue
  value >>= 4;
  
  /*
   * These parameters are typical values and need to be calibrated
   * See the datasheet for the appropriate chip for more details
   * also, the math below may not be very accurate
   */
    /* Assume ADC = 1480 at 25C and ADC = 4/C */
  #define VOLTAGE_AT_TEMP_25        1480
  #define TEMP_COEFFICIENT          4

  // Calibrate for 22C the first time the temp sensor is read.
  // This will assume that the demo is started up in temperature of 22C
  if(bCalibrate) {
    voltageAtTemp22=value;
    bCalibrate=FALSE;
  }
  
  temp = 22 + ( (value - voltageAtTemp22) / TEMP_COEFFICIENT );
  
  // Set 0C as minimum temperature, and 100C as max
  if( temp >= 100) 
  {
    return 100;
  }
  else if (temp <= 0) {
    return 0;
  }
  else { 
    return temp;
  }
  // Only CC2530 is supported
  #else
  return 0;
  #endif
}

/******************************************************************************
 * @fn          readVoltage
 *
 * @brief       read voltage from ADC
 *
 * @param       none
 *              
 * @return      voltage
 */
uint8 readVoltage(void)
{
  #if defined (HAL_MCU_CC2530)
  uint16 value;

  // Clear ADC interrupt flag 
  ADCIF = 0;

  ADCCON3 = (HAL_ADC_REF_125V | HAL_ADC_DEC_128 | HAL_ADC_CHN_VDD3);

  // Wait for the conversion to finish 
  while ( !ADCIF );

  // Get the result
  value = ADCL;
  value |= ((uint16) ADCH) << 8;

  
  // value now contains measurement of Vdd/3
  // 0 indicates 0V and 32767 indicates 1.25V
  // voltage = (value*3*1.25)/32767 volts
  // we will multiply by this by 10 to allow units of 0.1 volts
  value = value >> 6;   // divide first by 2^6
  value = (uint16)(value * 37.5);
  value = value >> 9;   // ...and later by 2^9...to prevent overflow during multiplication

  return value;
  #else
  return 0;
  #endif // CC2530
}
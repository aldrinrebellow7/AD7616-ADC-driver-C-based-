/*
 * Drv_AD7616.h
 *
 *  Created on: May 18, 2025
 *      Author: aldrin.Rebellow
 */

#ifndef SRC_DRV_AD7616_H_
#define SRC_DRV_AD7616_H_

#define TIME_RESET_WAIT 					(200U)/*1 Ms*/
#define TIMEOUT_AD7616_BUSY 				(10U)/*1MS*/
#define AD7616_MAX_NUM_CHANNEL 				(16U)
#define AD7616_MAX_BYTES_TO_READ_1_SW		(AD7616_MAX_NUM_CHANNEL * AD7616_LEN_PER_CHANNEL_IN_BYTES)
#define AD7616_LEN_PER_CHANNEL_IN_BYTES 	(2U)
#define AD7616_CRC_ENABLED_FLAG				(FALSE)
#define AD7616_SIZE_OF_CRC					(1U)
#define AD7616_CRC_INDEX					(32U)
#define AD7616_SOC_TIMER_MIN_PERIOD_US      (15UL)
#define AD7616_SOC_TIMER_MAX_PERIOD_US      (2000000UL)
typedef enum
{
	RANGE_SW_MODE = 0x00,
	RANGE_SEL_PM_2_5V = 0x01,
	RANGE_SEL_PM_5V = 0x02,
	RANGE_SEL_PM_10V = 0x03,
}HW_RANGE_SEL;

typedef enum
{
	REG_READ = 0U,
	REG_WRITE = 1U,
}REG_RW_STATE;

typedef enum
{
	AD7616_CHAB0 = 0U,
	AD7616_CHAB1,
	AD7616_CHAB2,
	AD7616_CHAB3,
	AD7616_CHAB4,
	AD7616_CHAB5,
	AD7616_CHAB6,
	AD7616_CHAB7,
	AD7616_CHMAX
}AD7616_CHANNEL;

typedef enum
{
	ADDRS_REG_CONFIGURATION 	= 0x02,/*Configuration register*/
	ADDRS_REG_CHANNEL 			= 0x03,/*Channel register*/
	ADDRS_REG_INPUT_RANGE_A1 	= 0x04,/*Input Range A1*/
	ADDRS_REG_INPUT_RANGE_A2 	= 0x05,/*Input Range A2*/
	ADDRS_REG_INPUT_RANGE_B1 	= 0x06,/*Input Range B1*/
	ADDRS_REG_INPUT_RANGE_B2 	= 0x07,/*Input Range B2*/
	ADDRS_REG_STATUS			= 0x80/*Status Register*/
}AD7616_REGISTER_ADDRS_MAP;

typedef enum
{
	en_AD7616_IDLE = 0U,
	en_AD7616_READING_CHANNEL,
}AD7616_STATE;

void Drv_AD7616_Init(void);
void Drv_AD7616_AdjustConversionPeriod(uint32_t u32Period_us);
void ISRCallback_Ad7616_TriggerAdcConverison(void);
void ISRCallback_Ad7616_Busy(void);
void Callback_AD7616RxComplete(void);
void Callback_AD7616TxComplete(void);
void Drv_AD7616_SelectChannel(AD7616_CHANNEL m_Ch);
void Drv_AD7616_SelectHWInputVoltageRange(HW_RANGE_SEL m_inpVR);
void Drv_AD7616_TriggerAdcConvst(void);
uint8_t Drv_AD7616_GetStatus_DeviceConvCmplte(void);
uint8_t Drv_AD7616_GetInstanceAdcBuffer(int16_t **pChA , int16_t **pChB);
uint8_t Drv_AD7616_GetStatus_TX_Complete(void);
uint8_t Drv_AD7616_GetStatus_RX_Available(void);
uint8_t* Drv_AD7616_ReadSpiADC_1W(void);
void Drv_AD7616_TriggerReadRegisterSpi_1W(void);
void Drv_AD7616_TriggerReadADCSpi_1W(void);
void Drv_AD7616_WriteSpiRegister_1W(uint8_t u8RegAddrs , uint16_t u16RegData , REG_RW_STATE u8RW);
uint8_t Drv_AD7616_ReadSpiRegister_1W(uint8_t *u8RegAddrs , uint16_t *pu8RegData);
void Drv_AD7616_Handler(void);
uint8_t Drv_AD7616_GetState(void);
#endif /* SRC_DRV_AD7616_H_ */

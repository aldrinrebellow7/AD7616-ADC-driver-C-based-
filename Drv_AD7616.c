/*
 * Drv_AD7616.c
 *
 *  Created on: May 18, 2025
 *      Author: aldrin.Rebellow
 */
#include "main.h"
#include "string.h"
#include "Appl_Timer.h"
#include "Drv_AD7616.h"

uint8_t g_u8AdcConvComplteFlag = FALSE;/*varibale to flag ADC conversion is completed*/
uint8_t g_u8AD7616DataAvailFlag = FALSE;
uint8_t g_u8SPI_WrCmplte = FALSE;/*varibale to flag SPI tarnsmission is completed*/
uint8_t g_u8SPI_RdCmplte = FALSE;/*varibale to flag SPI reception is completed*/

#if (AD7616_CRC_ENABLED_FLAG)
uint8_t g_u8RxCrc = 0U;
uint8_t g_u8RxCrcValidity = FALSE;
uint64_t u64CrcValidCnt = 0U;
uint64_t u64CrcInvalidCnt = 0U;
#endif
/*
 * Buffer to recieve SPI data
 */
#if (AD7616_CRC_ENABLED_FLAG)
uint8_t g_u16SpiReadBuffer[AD7616_SIZE_OF_CRC + (AD7616_MAX_NUM_CHANNEL * AD7616_LEN_PER_CHANNEL_IN_BYTES)];
#else
uint8_t g_u16SpiReadBuffer[(AD7616_MAX_NUM_CHANNEL * AD7616_LEN_PER_CHANNEL_IN_BYTES)];
#endif
/*
 * Buffer to read ADC data from register of AD7616
 */
int16_t g_u16ADCDataVal_ChannelA[AD7616_CHMAX];
int16_t g_u16ADCDataVal_ChannelB[AD7616_CHMAX];

AD7616_STATE m_State = en_AD7616_IDLE;/*State machine state variable*/
AD7616_CHANNEL m_MaxChannelScan = AD7616_CHAB0;/*variable to store id of last channel to be scanned in each cycle*/
#if (AD7616_CRC_ENABLED_FLAG)
static uint8_t crc_step(uint16_t data, uint8_t crc_in);
static uint8_t get_bit(uint16_t word, uint8_t bit_pos);
static uint8_t ad7616_calculate_crc_all(const uint16_t buffer_a[AD7616_CHMAX], const uint16_t buffer_b[AD7616_CHMAX]);
static uint8_t ad7616_validate_crc(const uint16_t buffer_a[AD7616_CHMAX],
											const uint16_t buffer_b[AD7616_CHMAX],
											uint16_t crc_word);
#endif
/*********************.HAL_GPIO_EXTI_Callback().*****************************
 .Purpose        : Callback for BUSY interrupt PIN - Rising and falling
 .Returns        :  RETURN_ERROR
					RETURN_SUCCESS
 .Note           :
 ****************************************************************************/
inline void ISRCallback_Ad7616_Busy(void)
{
	/*
	 * If BUSY pin LOW means , AD7616 starts conversion of SAR , now make CONV signal LOW
	 */
	HAL_GPIO_WritePin(AD7616_CONV_GPIO_Port , AD7616_CONV_Pin , GPIO_PIN_RESET);
	/*If BUSY is LOW means , ADC conversion is completed*/
	/*IF BUSY signal is low (END of converison) -> READ CHANNEL*/
	Drv_AD7616_TriggerReadADCSpi_1W();/*Inititate reading*/
	m_State = en_AD7616_READING_CHANNEL;
	g_u8AdcConvComplteFlag = TRUE;
}
/*********************.ISRCallback_Ad7616_Busy().*****************************
 .Purpose        : 	Callback for tigger converison start for ADC
 .Returns        :  RETURN_ERROR
					RETURN_SUCCESS
 .Note           :
 ****************************************************************************/
inline void ISRCallback_Ad7616_TriggerAdcConverison(void)
{
	/*
	 * Trigger conversion start
	 */
	Drv_AD7616_TriggerAdcConvst();/*TRIGGER SCAN CONV*/
}
/*********************.HAL_GPIO_EXTI_Callback().*****************************
 .Purpose        : Callback for GPIO interrupt Rising and falling
 .Returns        :  RETURN_ERROR
					RETURN_SUCCESS
 .Note           :
 ****************************************************************************/
inline void Callback_AD7616RxComplete(void)
{
	HAL_GPIO_WritePin(AD7616_CS__GPIO_Port , AD7616_CS__Pin , GPIO_PIN_SET);/*Make CS pin HIGH*/
	g_u8SPI_RdCmplte = TRUE;/*Flag read complete*/
}
/*********************.HAL_GPIO_EXTI_Callback().*****************************
 .Purpose        : Callback for GPIO interrupt Rising and falling
 .Returns        :  RETURN_ERROR
					RETURN_SUCCESS
 .Note           :
 ****************************************************************************/
inline void Callback_AD7616TxComplete(void)
{
	HAL_GPIO_WritePin(AD7616_CS__GPIO_Port , AD7616_CS__Pin , GPIO_PIN_SET);/*Make CS pin HIGH*/
	g_u8SPI_WrCmplte = TRUE;/*Flag write complete*/
}
/*********************.HAL_GPIO_EXTI_Callback().*****************************
 .Purpose        : Callback for GPIO interrupt Rising and falling
 .Returns        :  RETURN_ERROR
					RETURN_SUCCESS
 .Note           :
 ****************************************************************************/
void Drv_AD7616_Init(void)
{
	/*
	 * Perform RESET
	 */
	HAL_GPIO_WritePin(AD7616_RESET_GPIO_Port , AD7616_RESET_Pin , GPIO_PIN_SET);/*Make RESET pin HIGH*/
	HAL_GPIO_WritePin(AD7616_CS__GPIO_Port , AD7616_CS__Pin , GPIO_PIN_SET);/*Make CS pin HIGH*/
	Drv_AD7616_SelectChannel(m_MaxChannelScan = AD7616_CHAB7/*MAX Channel to scan*/);/*Sequencer will read from channel 0 - Configured channel*/
	Drv_AD7616_SelectHWInputVoltageRange(RANGE_SEL_PM_5V);/*Select HARDWARE MODE and INP voltage range*/
	HAL_GPIO_WritePin(AD7616_CONV_GPIO_Port , AD7616_CONV_Pin , GPIO_PIN_RESET);/*Make CONV pin RESET*/

	HAL_Delay(TIME_RESET_WAIT);
	HAL_GPIO_WritePin(AD7616_RESET_GPIO_Port , AD7616_RESET_Pin , GPIO_PIN_RESET);/*Make RESET pin LOW : RESET STATE*/

	HAL_Delay(TIME_RESET_WAIT);
	HAL_GPIO_WritePin(AD7616_RESET_GPIO_Port , AD7616_RESET_Pin , GPIO_PIN_SET);/*Make RESET pin HIGH*/

	HAL_Delay(TIME_RESET_WAIT);
	HAL_TIM_Base_Start_IT(GetInstance_AD7616SOC_TIM5());
}
/*********************.Drv_AD7616_TriggerAdcConvst().*****************************
 .Purpose        : Function to get address of buffer to application layer and to get validity of adc data.
 .Returns        :  RETURN_ERROR
					RETURN_SUCCESS
 .Note           :
 ****************************************************************************/
uint8_t Drv_AD7616_GetInstanceAdcBuffer(int16_t **pChA , int16_t **pChB)
{
	uint8_t u8ValidityFlag = TRUE;/*Copy default validity flag*/
	*pChA = (&g_u16ADCDataVal_ChannelA[0U]);/*Copy address of ADC buffer channel A*/
	*pChB = (&g_u16ADCDataVal_ChannelB[0U]);/*Copy address of ADC buffer channel B*/
#if (AD7616_CRC_ENABLED_FLAG)
	u8ValidityFlag = g_u8RxCrcValidity;/*Copy adc data validity flag*/
#endif
	return u8ValidityFlag;
}
/*********************.Drv_AD7616_TriggerAdcConvst().*****************************
 .Purpose        : Function to trigger start of converion of ADC
 .Returns        :  RETURN_ERROR
					RETURN_SUCCESS
 .Note           :
 ****************************************************************************/
inline uint8_t Drv_AD7616_GetStatus_DeviceConvCmplte(void)
{
	return g_u8AdcConvComplteFlag;/*get status of conversion complete*/
}
/*********************.Drv_AD7616_TriggerAdcConvst().*****************************
 .Purpose        : Function to trigger start of converion of ADC
 .Returns        :  RETURN_ERROR
					RETURN_SUCCESS
 .Note           :
 ****************************************************************************/
inline uint8_t Drv_AD7616_GetStatus_TX_Complete(void)
{
	return g_u8SPI_WrCmplte;/*Get status of spi write complete*/
}
/*********************.Drv_AD7616_TriggerAdcConvst().*****************************
 .Purpose        : Function to trigger start of converion of ADC
 .Returns        :  RETURN_ERROR
					RETURN_SUCCESS
 .Note           :
 ****************************************************************************/
inline uint8_t Drv_AD7616_GetStatus_RX_Available(void)
{
	return g_u8SPI_RdCmplte;/*Get status of spi read complete*/
}
/*********************.HAL_GPIO_EXTI_Callback().*****************************
 .Purpose        : Callback for GPIO interrupt Rising and falling
 .Returns        :  RETURN_ERROR
					RETURN_SUCCESS
 .Note           :
 ****************************************************************************/
void Drv_AD7616_SelectChannel(AD7616_CHANNEL m_Ch)
{
	uint8_t u8Channel = (uint8_t)m_Ch;
	HAL_GPIO_WritePin(AD7616_CHSEL2_GPIO_Port, AD7616_CHSEL2_Pin, (u8Channel & 0x01));
	HAL_GPIO_WritePin(AD7616_CHSEL1_GPIO_Port, AD7616_CHSEL1_Pin, 0x01 & (u8Channel >> 1U));
	HAL_GPIO_WritePin(AD7616_CHSEL0_GPIO_Port, AD7616_CHSEL0_Pin, 0x01 & (u8Channel >> 2U));
}
/*********************.HAL_GPIO_EXTI_Callback().*****************************
 .Purpose        : Callback for GPIO interrupt Rising and falling
 .Returns        :  RETURN_ERROR
					RETURN_SUCCESS
 .Note           :
 ****************************************************************************/
void Drv_AD7616_SelectHWInputVoltageRange(HW_RANGE_SEL m_inpVR)
{
	uint8_t u8Buff = (uint8_t)m_inpVR;

	HAL_GPIO_WritePin(AD7616_HW_RNGSEL1_GPIO_Port , AD7616_HW_RNGSEL1_Pin , (u8Buff & 0x01));
	HAL_GPIO_WritePin(AD7616_HW_RNGSEL0_GPIO_Port , AD7616_HW_RNGSEL0_Pin , (0x01 & (u8Buff >> 1U)));
}
/*********************.Drv_AD7616_TriggerAdcConvst().*****************************
 .Purpose        : Function to trigger start of converion of ADC
 .Returns        :  RETURN_ERROR
					RETURN_SUCCESS
 .Note           :
 ****************************************************************************/
void Drv_AD7616_TriggerAdcConvst(void)
{
	g_u8AD7616DataAvailFlag = FALSE;
	g_u8AdcConvComplteFlag = FALSE;
	HAL_GPIO_WritePin(AD7616_CONV_GPIO_Port , AD7616_CONV_Pin , GPIO_PIN_SET);
}

/*********************.Drv_AD7616_TriggerAdcConvst().*****************************
 .Purpose        : Function to trigger start of converion of ADC
 .Returns        :  RETURN_ERROR
					RETURN_SUCCESS


 .Note           :
 ****************************************************************************/
inline void Drv_AD7616_TriggerReadADCSpi_1W(void)
{
	g_u8SPI_RdCmplte = FALSE;
	HAL_GPIO_WritePin(AD7616_CS__GPIO_Port , AD7616_CS__Pin , GPIO_PIN_RESET);
#if (AD7616_CRC_ENABLED_FLAG)
	HAL_SPI_Receive_IT(GetInstance_SPI1(), &g_u16SpiReadBuffer[0U],
			/*Read size of bytes*/(AD7616_MAX_BYTES_TO_READ_1_SW + AD7616_SIZE_OF_CRC/*SIZE of CRC BYTE*/);
#else
	HAL_SPI_Receive_IT(GetInstance_SPI1(), &g_u16SpiReadBuffer[0U],
							(AD7616_MAX_BYTES_TO_READ_1_SW)/*Read size of bytes*/);
#endif

}
/*********************.Drv_AD7616_TriggerAdcConvst().*****************************
 .Purpose        : Function to trigger start of converion of ADC
 .Returns        :  RETURN_ERROR
					RETURN_SUCCESS
 .Note           :
 ****************************************************************************/
uint8_t* Drv_AD7616_ReadSpiADC_1W(void)
{
	return (&g_u16SpiReadBuffer[0U]);
}
/*********************.Drv_AD7616_TriggerAdcConvst().*****************************
 .Purpose        : Function to trigger start of converion of ADC
 .Returns        :  RETURN_ERROR
					RETURN_SUCCESS
 .Note           :
 ****************************************************************************/
uint8_t Drv_AD7616_GetState(void)
{
	uint8_t u8Flag = g_u8AD7616DataAvailFlag;
	g_u8AD7616DataAvailFlag = FALSE;/*Clear new data avail flag after reading*/
	return (u8Flag);
}
/*********************.Drv_AD7616_TriggerAdcConvst().*****************************
 .Purpose        : Function to trigger start of converion of ADC
 .Returns        :  RETURN_ERROR
					RETURN_SUCCESS
 .Note           :
 ****************************************************************************/
void Drv_AD7616_AdjustConversionPeriod(uint32_t u32Period_us)
{
    TIM_HandleTypeDef *phTim = GetInstance_AD7616SOC_TIM5();

    if (u32Period_us < AD7616_SOC_TIMER_MIN_PERIOD_US)
    {
    	u32Period_us = AD7616_SOC_TIMER_MIN_PERIOD_US;
    }
    else if (u32Period_us > AD7616_SOC_TIMER_MAX_PERIOD_US)
    {
    	u32Period_us = AD7616_SOC_TIMER_MAX_PERIOD_US;
    }
    else
    {
        /*NOP*/
    }

    __disable_irq();

    if (phTim->Instance != NULL)
    {
        __HAL_TIM_SET_AUTORELOAD(phTim, u32Period_us - 1UL);
        __HAL_TIM_SET_COUNTER(phTim, 0UL);
    }

    __enable_irq();
}
#if (AD7616_CRC_ENABLED_FLAG)
/*********************.Drv_AD7616_TriggerAdcConvst().*****************************
 .Purpose        : Function to trigger start of converion of ADC
 .Returns        :  RETURN_ERROR
					RETURN_SUCCESS
 .Note           :
 ****************************************************************************/
/* Extract a single bit from a 16-bit word */
static uint8_t get_bit(uint16_t word, uint8_t bit_pos)
{
    return (uint8_t)((word >> bit_pos) & (uint16_t)1U);
}
/*********************.Drv_AD7616_TriggerAdcConvst().*****************************
 .Purpose        : Function to trigger start of converion of ADC
 .Returns        :  RETURN_ERROR
					RETURN_SUCCESS
 .Note           :
 ****************************************************************************/
/* Compute CRC for a single 16-bit word and current CRC byte */
static uint8_t crc_step(uint16_t data, uint8_t crc_in)
{
    uint8_t data_bit[16] = {0U};
    uint8_t crc_bit[8] = {0U};
    uint8_t i;
    uint8_t crc_out_val = 0U;

    /* Extract data bits */
    for (i = 0U; i < 16U; ++i)
    {
        data_bit[i] = get_bit(data, i);
    }

    /* Extract CRC bits */
    for (i = 0U; i < 8U; ++i)
    {
        crc_bit[i] = get_bit((uint16_t)crc_in, i);
    }

    /* Compute each bit of the CRC output */
    crc_out_val |= (uint8_t)((data_bit[14] ^ data_bit[12] ^ data_bit[8] ^ data_bit[7] ^
                              data_bit[6] ^ data_bit[0] ^ crc_bit[0] ^ crc_bit[4] ^ crc_bit[6]) << 0U);

    crc_out_val |= (uint8_t)((data_bit[15] ^ data_bit[14] ^ data_bit[13] ^ data_bit[12] ^
                              data_bit[9] ^ data_bit[6] ^ data_bit[1] ^ data_bit[0] ^ crc_bit[1] ^
                              crc_bit[4] ^ crc_bit[5] ^ crc_bit[6] ^ crc_bit[7]) << 1U);

    crc_out_val |= (uint8_t)((data_bit[15] ^ data_bit[13] ^ data_bit[12] ^ data_bit[10] ^
                              data_bit[8] ^ data_bit[6] ^ data_bit[2] ^ data_bit[1] ^ data_bit[0] ^
                              crc_bit[0] ^ crc_bit[2] ^ crc_bit[4] ^ crc_bit[5] ^ crc_bit[7]) << 2U);

    crc_out_val |= (uint8_t)((data_bit[14] ^ data_bit[13] ^ data_bit[11] ^ data_bit[9] ^
                              data_bit[7] ^ data_bit[3] ^ data_bit[2] ^ data_bit[1] ^ crc_bit[1] ^
                              crc_bit[3] ^ crc_bit[5] ^ crc_bit[6]) << 3U);

    crc_out_val |= (uint8_t)((data_bit[15] ^ data_bit[14] ^ data_bit[12] ^ data_bit[10] ^
                              data_bit[8] ^ data_bit[4] ^ data_bit[3] ^ data_bit[2] ^ crc_bit[0] ^
                              crc_bit[2] ^ crc_bit[4] ^ crc_bit[6] ^ crc_bit[7]) << 4U);

    crc_out_val |= (uint8_t)((data_bit[15] ^ data_bit[13] ^ data_bit[11] ^ data_bit[9] ^
                              data_bit[5] ^ data_bit[4] ^ data_bit[3] ^ crc_bit[1] ^ crc_bit[3] ^
                              crc_bit[5] ^ crc_bit[7]) << 5U);

    crc_out_val |= (uint8_t)((data_bit[14] ^ data_bit[12] ^ data_bit[10] ^ data_bit[6] ^
                              data_bit[5] ^ data_bit[4] ^ crc_bit[2] ^ crc_bit[4] ^ crc_bit[6]) << 6U);

    crc_out_val |= (uint8_t)((data_bit[15] ^ data_bit[13] ^ data_bit[11] ^ data_bit[7] ^
                              data_bit[6] ^ data_bit[5] ^ crc_bit[3] ^ crc_bit[5] ^ crc_bit[7]) << 7U);

    return crc_out_val;
}
/*********************.Drv_AD7616_TriggerAdcConvst().*****************************
 .Purpose        : Function to trigger start of converion of ADC
 .Returns        :  RETURN_ERROR
					RETURN_SUCCESS
 .Note           :
 ****************************************************************************/
/* Calculate CRC for 16-word A buffer followed by 16-word B buffer */
uint8_t ad7616_calculate_crc_all(const uint16_t buffer_a[AD7616_CHMAX], const uint16_t buffer_b[AD7616_CHMAX])
{
    uint8_t crc = 0U;
    for (uint8_t u8nI = 0U; u8nI < AD7616_CHMAX; ++u8nI)
    {
        crc = crc_step(buffer_a[u8nI], crc);
    }

    for (uint8_t u8nI = 0U; u8nI < AD7616_CHMAX; ++u8nI)
    {
        crc = crc_step(buffer_b[u8nI], crc);
    }

    return crc;
}
/*********************.Drv_AD7616_TriggerAdcConvst().*****************************
 .Purpose        : Function to trigger start of converion of ADC
 .Returns        :  RETURN_ERROR
					RETURN_SUCCESS
 .Note           :
 ****************************************************************************/
/* Validate CRC from ADC against calculated CRC */
uint8_t ad7616_validate_crc(const uint16_t buffer_a[AD7616_CHMAX],
								const uint16_t buffer_b[AD7616_CHMAX],
								uint16_t crc_word)
{
    volatile uint8_t received_crc = (uint8_t)(crc_word & 0x00FFU); /* CRC is LSB */
    volatile uint8_t expected_crc = ad7616_calculate_crc_all(buffer_a, buffer_b);

    return (received_crc == expected_crc) ? TRUE : FALSE;
}
#endif
/*********************.Drv_AD7616_TriggerAdcConvst().*****************************
 .Purpose        : Function to trigger start of converion of ADC
 .Returns        :  RETURN_ERROR
					RETURN_SUCCESS
 .Note           :
 ****************************************************************************/
void Drv_AD7616_Handler(void)
{
	volatile uint8_t *pBuff = 0U;
#if (AD7616_CRC_ENABLED_FLAG)
	volatile uint16_t u16BuffChA[AD7616_CHMAX] = {0U};
	volatile uint16_t u16BuffChB[AD7616_CHMAX] = {0U};
#endif
	switch (m_State)
	{
		case (en_AD7616_IDLE):
		{

		}break;
		case (en_AD7616_READING_CHANNEL):
		{
			if(TRUE == Drv_AD7616_GetStatus_RX_Available())
			{
				pBuff = Drv_AD7616_ReadSpiADC_1W();

				for(uint8_t u8Channel = 0U , u8Index = 0U; u8Channel <= m_MaxChannelScan ; ++u8Channel)
				{
					u8Index = u8Channel * 4U;
#if (AD7616_CRC_ENABLED_FLAG)
					u16BuffChA[u8Channel] = (pBuff[(u8Index) + 0U]);
					u16BuffChA[u8Channel] |= (pBuff[(u8Index) + 1U] << 8U);

					u16BuffChB[u8Channel] = (pBuff[((u8Index + 2U)) + 0U]);
					u16BuffChB[u8Channel] |= (pBuff[((u8Index + 2U)) + 1U] << 8U);
#else
					g_u16ADCDataVal_ChannelA[u8Channel] = 0U;
					g_u16ADCDataVal_ChannelB[u8Channel] = 0U;
					g_u16ADCDataVal_ChannelA[u8Channel] = 0x00FF & (pBuff[(u8Index) + 1U]);
					g_u16ADCDataVal_ChannelA[u8Channel] |= 0xFF00 & (pBuff[(u8Index) + 0U] << 8U);


					g_u16ADCDataVal_ChannelB[u8Channel] = 0x00FF & (pBuff[((u8Index + 2U)) + 1U]);
					g_u16ADCDataVal_ChannelB[u8Channel] |= 0xFF00 & (pBuff[((u8Index + 2U)) + 0U] << 8U);
#endif
#if (FALSE)
					/*
					 * Work around - issue : some time 0xFFFF is comming as random when signal value is ZERO
					 */
					if(0xFFFF <= g_u16ADCDataVal_ChannelA[u8Channel])
					{
						g_u16ADCDataVal_ChannelA[u8Channel] = 0U;
					}
					if(0xFFFF <= g_u16ADCDataVal_ChannelB[u8Channel])
					{
						g_u16ADCDataVal_ChannelB[u8Channel] = 0U;
					}
					/*
					 * Work around - issue : some time 0xFFFF is comming as random when signal value is ZERO
					 */
#endif
				}
#if (AD7616_CRC_ENABLED_FLAG)
				g_u8RxCrc = pBuff[AD7616_CRC_INDEX];/*Get CRC byte data from recieved buffer*/
				if(TRUE == ad7616_validate_crc((uint16_t*)u16BuffChA , (uint16_t*)u16BuffChB , (uint8_t)g_u8RxCrc))
				{
					memcpy((uint16_t*)g_u16ADCDataVal_ChannelA , (uint16_t*)u16BuffChA , sizeof(g_u16ADCDataVal_ChannelA));
					memcpy((uint16_t*)g_u16ADCDataVal_ChannelB , (uint16_t*)u16BuffChB , sizeof(g_u16ADCDataVal_ChannelB));
					g_u8RxCrcValidity = TRUE;
					u64CrcValidCnt++;
				}
				else
				{
					g_u8RxCrcValidity = FALSE;
					u64CrcInvalidCnt++;
				}
#endif
				/*If all channel got readed*/
				g_u8AD7616DataAvailFlag = TRUE;/*SET read complete flag*/
				m_State = en_AD7616_IDLE;/*Switch back to IDLE state*/
			}
		}break;
		default:
		{
			/*NOP*/
		}break;
	}
}



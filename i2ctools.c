#include <stdint.h>
#include <stm32f1xx_hal.h>

#include "def.h"
#include "SEGGER/SEGGER_RTT.h"

extern I2C_HandleTypeDef hi2c1;
extern DMA_HandleTypeDef hdma_i2c1_tx;
extern DMA_HandleTypeDef hdma_i2c1_rx;

extern uint16_t CRC_EN;

#define mBUFFER_SIZE	16
#define mTimeOut 	1000

uint8_t i2c1_tx_buffer[mBUFFER_SIZE];
uint8_t i2c1_rx_buffer[mBUFFER_SIZE];

uint8_t wwBuffer[mBUFFER_SIZE];

uint32_t forceResetI2c = 0;

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	
}



/// 读取mcf8316的寄存器
void mccReadRegister()
{
	uint32_t txLen = 3;
	uint32_t rxLen = mccEvm.DLEN + CRC_EN;
	
	uint16_t sid = mccEvm.SLAVE_ID << 1; 
	
	/*
	if (HAL_I2C_IsDeviceReady(&hi2c1, sid, 3, 100) != HAL_OK)
	{
		forceResetI2c = 1;
	}
	*/
	
	uint32_t timeout = HAL_MAX_DELAY;
	if (forceResetI2c)
	{
		forceResetI2c = 0;
		ErrLog("fore reset i2c");
		HAL_I2C_DeInit(&hi2c1);
		HAL_I2C_Init(&hi2c1);
	}
	
	///
	HAL_StatusTypeDef hstatus = HAL_OK;
	
	i2c1_tx_buffer[0] = mccEvm.DATAWR_ARRAY[1];
	i2c1_tx_buffer[1] = mccEvm.DATAWR_ARRAY[2];
	i2c1_tx_buffer[2] = mccEvm.DATAWR_ARRAY[3];
	//pMccOPH_T poph = (pMccOPH_T)&mccEvm.DATAWR_ARRAY[0];
	
	//HAL_I2C_Master_Transmit()
	
	//SEGGER_RTT_printf(0, "read8136 CMD:\n");
	//dump(mccEvm.DATAWR_ARRAY, mccEvm.MCC_Packet_Size);
	

	if (HAL_I2C_Master_Seq_Transmit_DMA(&hi2c1, sid, i2c1_tx_buffer, txLen, I2C_FIRST_FRAME) != HAL_OK)
	{
		//SEGGER_RTT_printf(2, "[%s]HAL_I2C_Master_Seq_Transmit_DMA error\n", __func__);
		forceResetI2c = 1;
		return;
	}
	//wait for tx complete
	timeout = HAL_MAX_DELAY;
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
	{
		if (timeout == 0) {
			forceResetI2c = 1;
			return;			
		};
		timeout--;
	}

	hstatus = HAL_I2C_Master_Seq_Receive_DMA(&hi2c1, sid, i2c1_rx_buffer, rxLen, I2C_LAST_FRAME);
	if (hstatus != HAL_OK)
	{
		//uint32_t errcode = HAL_DMA_GetError(&hi2c1.hdmarx);
		//SEGGER_RTT_printf(0, "HAL_I2C_Master_Seq_Receive_DMA error [%d]\n", hstatus);
		//return;
		forceResetI2c = 1;
		return;
	}
	
	//wait for rx complete
	timeout = HAL_MAX_DELAY;
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
	{
		if (timeout == 0) {
			forceResetI2c = 1;
			return;			
		}
		;
		timeout--;
	}
	//mccEvm.DATARD_ARRAY[5]
	for (int i = 0; i < rxLen; i++)
	{
		mccEvm.DATARD_ARRAY[i + 5] = i2c1_rx_buffer[i];
	}
	

	mccEvm.MEMADDR_PROG = 10;
	mccEvm.RXCOUNT_I2C = 0;

	
}

void mccWriteRegister()
{
	uint32_t timeout = HAL_MAX_DELAY;
	if (forceResetI2c)
	{
		forceResetI2c = 0;
		ErrLog("fore reset i2c");
		HAL_I2C_DeInit(&hi2c1);
		HAL_I2C_Init(&hi2c1);
		//HAL_I2C_IsDeviceReady()
	}
	uint32_t txLen = 3 + mccEvm.DLEN + CRC_EN;
	uint16_t sid = mccEvm.SLAVE_ID << 1; 
	HAL_StatusTypeDef hstatus = HAL_OK;
	
	//wwBuffer[0] = mccEvm.DATAWR_ARRAY[1];
	//wwBuffer[1] = mccEvm.DATAWR_ARRAY[2];
	//wwBuffer[2] = mccEvm.DATAWR_ARRAY[3];
	
	for (int i = 0; i < txLen; i++)
	{
		wwBuffer[i] = mccEvm.DATAWR_ARRAY[i+1];
	}
	
	hstatus = HAL_I2C_Master_Transmit_DMA(&hi2c1, sid, wwBuffer, txLen);
	if (hstatus != HAL_OK)
	{
		forceResetI2c = 1;
		return;
	}

	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
	{
		if (timeout == 0) {
			forceResetI2c = 1;
			return;			
		}
		;
		timeout--;
	}
	
	//mccEvm.TXData_I2C = 0;
	mccEvm.EndOfWrite = 1;
	mccEvm.MEMADDR_PROG = 2;
	
}
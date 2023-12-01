#include <string.h>
//#include <stdio.h>
#include <stdatomic.h>
#include "stm32f1xx.h"
#include "def.h"

#include "SEGGER/SEGGER_RTT.h"


//uint8_t aRxBuffer[BUFFER_SIZE];
//volatile uint8_t aRxBufferIdx = 0;
//volatile uint8_t aRxBufferCursor = 0;

// 超时设置
#define SYSTEM_TIMEOUT 	2000

//volatile uint32_t m2_SystemTimeout;



uint8_t aTxBuffer[256];
volatile uint8_t aTxBufferIdx;


//uint8_t gInCmdBuffer[CMD_BUFFER_SIZE];

volatile uint8_t gInCmdBufferIdx;

//volatile uint8_t gInCmdBufferReqLen;

//volatile uint8_t ginCmdBufferCursor = 0;


spinlock_t gInCmdBufferLock;

volatile int cmd_received = 0;

mccEvm_t mccEvm;


uint16_t MCC_RW = 0;
uint16_t CRC_EN = 0;

/*
 		 n_gInCmdBuffer = 0x00,
		 n_NVMData = 0x44,
		 n_DATARD_ARRAY = 0x84,
		 n_DATAWR_ARRAY = 0x94,
		 n_DLEN = 0xA2,
		 n_EndOfWrite = 0xA4,
		 n_MCC_Packet_Size = 0xA6,
		 n_MCC_RW = 0xA8,
		 n_MEMADDR_PROG = 0xAA,
		 n_NVMAddress = 0xAC,
		 n_NVMWrite = 0xAE,
		 n_RXCOUNT_I2C = 0xB0,
		 n_cmd_received = 0xB2,
		 n_gInCmdSkipCount = 0xB4,
		 n_SLAVE_ID = 0xB6,
 **/
void dumpAddName(uint32_t v)
{
	char* fName = "not name";
	char* pname = fName;
	uint32_t mSet = v;
	
	if (v >= n_gInCmdBuffer)
	{
		fName = "gInCmdBuffer";
		mSet = v - n_gInCmdBuffer;
	}
	if (v >= n_NVMData)
	{
		fName = "NVMData";
		mSet = v - n_NVMData;
	}
	if (v >= n_DATARD_ARRAY)
	{
		fName = "DATARD_ARRAY";
		mSet = v - n_DATARD_ARRAY;
	}
	if (v >= n_DATAWR_ARRAY)
	{
		fName = "DATAWR_ARRAY";
		mSet = v - n_DATAWR_ARRAY;
	}
	if (v >= n_DLEN)
	{
		fName = "DLEN";
		mSet = v - n_DLEN;
	}
	if (v >= n_EndOfWrite)
	{
		fName = "EndOfWrite";
		mSet = v - n_EndOfWrite;
	}
	if (v >= n_MCC_Packet_Size)
	{
		fName = "MCC_Packet_Size";
		mSet = v - n_MCC_Packet_Size;
	}
	if (v >= n_MCC_RW)
	{
		fName = "MCC_RW";
		mSet = v - n_MCC_RW;
	}
	if (v >= n_MEMADDR_PROG)
	{
		fName = "MEMADDR_PROG";
		mSet = v - n_MEMADDR_PROG;
	}
	if (v >= n_NVMAddress)
	{
		fName = "NVMAddress";
		mSet = v - n_NVMAddress;
	}
	if (v >= n_NVMWrite)
	{
		fName = "NVMWrite";
		mSet = v - n_NVMWrite;
	}
	if (v >= n_RXCOUNT_I2C)
	{
		fName = "RXCOUNT_I2C";
		mSet = v - n_RXCOUNT_I2C;
	}
	if (v >= n_cmd_received)
	{
		fName = "cmd_received";
		mSet = v - n_cmd_received;
	}
	if (v >= n_gInCmdSkipCount)
	{
		fName = "gInCmdSkipCount";
		mSet = v - n_gInCmdSkipCount;
	}
	if (v >= n_SLAVE_ID)
	{
		fName = "SLAVE_ID";
		mSet = v - n_SLAVE_ID;
	}
	
	if (v > sizeof(mccEvm_t))
	{
		fName = pname;
		mSet = 0;
	}
	
	
	/*
	-if (v == n_gInCmdBuffer) fName = "gInCmdBuffer";	
	-if(v == n_NVMData) fName = "NVMData";	
	-if(v == n_DATARD_ARRAY) fName = "DATARD_ARRAY";
	-if(v == n_DATAWR_ARRAY) fName= "DATAWR_ARRAY";
	-if(v == n_DLEN) fName = "DLEN";
	-if(v == n_EndOfWrite) fName = "EndOfWrite";
	-if(v == n_MCC_Packet_Size) fName = "MCC_Packet_Size";
	-if(v == n_MCC_RW) fName = "MCC_RW";
	-if(v == n_MEMADDR_PROG) fName = "MEMADDR_PROG";
	-if(v == n_NVMAddress) fName = "NVMAddress";
	if(v == n_NVMWrite) fName = "NVMWrite";
	if(v == n_RXCOUNT_I2C) fName = "RXCOUNT_I2C";
	if(v == n_cmd_received) fName = "cmd_received";
	if(v == n_gInCmdSkipCount) fName = "gInCmdSkipCount";
	if(v == n_SLAVE_ID) fName = "SLAVE_ID";
	*/
	
	/*
	SEGGER_RTT_printf(0,
			"Name:%s\toffset:0x%02x\n",
			fName
			,v);
	*/

	
	InfoLog("ValueName:%s\t index:0x%02x offset:0x%02x", fName,v, mSet);
	

}


void InitBuffer()
{
	memset(&mccEvm, 0x00,sizeof(mccEvm_t));
	
	MCC_RW = 0;
	CRC_EN = 0;

	//m2_SystemTimeout = 0;
	
	
	
	//memset(aRxBuffer, 0x00, sizeof(aRxBuffer));
	//memset(aTxBuffer, 0x00, sizeof(aTxBuffer));
	//memset(gInCmdBuffer, 0x00, sizeof(gInCmdBuffer));
	
	//aRxBufferCursor = 0;
	//aRxBufferIdx = 0;
	
	gInCmdBufferIdx = 0;
	//gInCmdBufferReqLen = 0;
	//ginCmdBufferCursor = 0;
	spin_lock_init(&gInCmdBufferLock);
	
	
	mccEvm.MEMADDR_PROG = 2;
	
	//printf("InitBuffer done\n");
	
	
	InfoLog("Init memory");
	

}




// 发送8位字节到串口
void Write8bitByteToCOM(unsigned char c)
{
	// uartTxByte(c & 0xff); -> Need to insert transmit code here
	//EUSCI_A_UART_transmitData(EUSCI_BASE, c & 0xff); // why is &ff needed?

}

// 获取8位字节占用宽度
int GetSizeOfMAUIn8bitByte()
{
	unsigned char maxMAUValue = (unsigned char)(-1);
	switch (maxMAUValue)
	{
	case 0xff:
		return 1;
	case 0xffff:
		return 2;
	default:
		return 0;
	}
}



/// @brief  写一个字节到指令缓冲区


int GetTransferSizeInMAU()
{
	uint8_t val = mccEvm.gInCmdBuffer[0];
	uint8_t ret = val & 0x3f;
	return ret;
}

/**
  * @brief 验证输入命令头,错误返回非0
  */
int VerifyInputCmdHeaders()
{
	uint8_t val = mccEvm.gInCmdBuffer[0];
	
	return ((val & 0x80) == 0x80) ? 0 : 1;
}

/**
  *	@brief 验证输入命令类型
  */
int GetInputCmdType()
{
	uint8_t val = mccEvm.gInCmdBuffer[0];
	return (val & 0x80);
}

/// @brief 获取读写标志 0:写 1:读
int GetRWFlag()//equivalent to endianness on the MAU in transmission
{
	uint8_t val = mccEvm.gInCmdBuffer[0];
	int ret = ((val >> 6) & 0x1); //
	return ret;
	//	return READ;
}

/// @brief  获取目标地址
/// @return 
uint8_t* GetInCmdAddress()
{
	uint8_t* addr = 0;
	uint8_t* Fixaddr = 0;
	//unsigned long addr_value = 0;
	uint32_t addr_value = 0;
	uint32_t fixAddr_value = 0;
	
	int i = 0;
	int addressSize = 4; // always use 32bit address
	for (; i < addressSize; i++)
	{
		uint32_t midx = 1 + i;
		uint32_t mpos = 8 * (addressSize - 1 - i);
		uint8_t val = mccEvm.gInCmdBuffer[midx];
		//addr_value |= (unsigned long)(gInCmdBuffer[midx] << 8 * i); //little endian
		addr_value |= (uint32_t)(val << mpos); //big endian
		//addr_value |= (unsigned long)(gInCmdBuffer[ginCmdBufferCursor + 1 + i] << 8 * (addressSize - 1 - i)); //big endian
	}

	addr = (uint8_t*) addr_value;
	fixAddr_value = addr_value & 0xfff;
	
	Fixaddr = (uint8_t*)(fixAddr_value + (uint32_t)&mccEvm);
	
	dumpAddName(fixAddr_value);
	
	
	//return addr;
	return Fixaddr;
}

int GetTargetEndianness()
{
	return LITTLE_ENDIAN;
}

unsigned char GetWriteCmdDataMAU(int idx)
{
	unsigned char startIdx = 1 + 4;

	unsigned char val = 0;
	int MAUSize = GetSizeOfMAUIn8bitByte();
	int byteOffset = idx*MAUSize;

	switch (MAUSize)
	{
	case 1:
		val = mccEvm.gInCmdBuffer[startIdx + byteOffset];
		break;
	case 2:
		if (GetTargetEndianness() == LITTLE_ENDIAN)
		{
			val = (mccEvm.gInCmdBuffer[startIdx + byteOffset + 1] << 8) | mccEvm.gInCmdBuffer[startIdx + byteOffset];
		}
		else {
			val = (mccEvm.gInCmdBuffer[startIdx + byteOffset] | mccEvm.gInCmdBuffer[startIdx + byteOffset + 1] << 8);
		}
		break;
	default://only handles 8bit, 16bit MAU
		ErrLog("not case 0x%02x",MAUSize);
		break;
	}

	return val;
}

/// @brief  清理缓冲所有指令参数, 输入要清理的字节数
void ClearBufferRelatedParam(uint8_t seek)
{
	spin_lock(&gInCmdBufferLock);
	if (seek < __LDRBT(&gInCmdBufferIdx))
	{
		memmove(mccEvm.gInCmdBuffer, &(mccEvm.gInCmdBuffer[seek]), __LDRBT(&gInCmdBufferIdx) - seek);
		__STRBT(__LDRBT(&gInCmdBufferIdx) - seek, &gInCmdBufferIdx);
	}else{
		__STRBT(0, &gInCmdBufferIdx);
	}
	__STRHT(0, &(mccEvm.gInCmdSkipCount));
	
	spin_unlock(&gInCmdBufferLock);
}

/// 写一个字节到发送缓冲区
void WriteByteToTxBuffer(unsigned char d)
{
	aTxBuffer[aTxBufferIdx++] = d & 0xff;
}

void MemAccessCmd(int mrw)
{
	uint8_t *addr = GetInCmdAddress();
	uint16_t MAUsToRead = GetTransferSizeInMAU();
	uint8_t datachar = 0x00;
	//uint8_t* addrfix = (uint8_t*)&mccEvm;
	
	WriteByteToTxBuffer(mccEvm.gInCmdBuffer[0]);
	// 修正内存访问
	//if (((uint32_t)addr) == 0x2000) addr = (uint8_t*)gInCmdBuffer;
	 //addr = (uint8_t*)gInCmdBuffer;
	//addr = (uint8_t*)(((uint32_t)oaddr - 0x2000) + (uint32_t)addrfix);
	
	
	//SEGGER_RTT_printf(0, "RW size:%d\n", MAUsToRead);
	//ErrLog("RW size:%d", MAUsToRead);
	
	
	for (int i = 0; i < MAUsToRead; i++)
	{
		if (mrw == m2_READ)
		{
			uint8_t* tprt = (addr + i);
			datachar = *tprt;
			WriteByteToTxBuffer(datachar);
		}
		else
		{
			datachar = GetWriteCmdDataMAU(i);
			uint8_t* tprt = (addr + i);
			(*tprt) = datachar;
		}
	}
	
}

// 处理指令缓冲区
void ProcessCommand()
{
	/*
	if ((m2_SystemTimeout + SYSTEM_TIMEOUT) < HAL_GetTick())
	{
		ClearBufferRelatedParam(CLEAR_ALL);
		return;
	}
	
	*/
	
	
	if (gInCmdBufferIdx == 0) return;
	//uint8_t sIdx = __LDRBT(&gInCmdBufferIdx);
	//uint32_t sCursor = 0;
	
	while (VerifyInputCmdHeaders())
	{
		ClearBufferRelatedParam(1);
		if (__LDRBT(&gInCmdBufferIdx) == 0) return;
	}
	
		
	int rwFlag = GetRWFlag();

	if (rwFlag == m2_WRITE)
	{
		mccEvm.gInCmdSkipCount = 5 + GetTransferSizeInMAU()*GetSizeOfMAUIn8bitByte();
	}
	else
	{
		mccEvm.gInCmdSkipCount = 5;
	}
	
	
	
	// 修正数量
	if (mccEvm.gInCmdSkipCount > gInCmdBufferIdx)
	{
		mccEvm.gInCmdSkipCount -= gInCmdBufferIdx;
	}
	else
	{
		mccEvm.gInCmdSkipCount = 0;
	}
	
	/// 数据不完整,等待下次接收
	if (mccEvm.gInCmdSkipCount > 0)
	{
		return;
	}
	
	
	//InfoLog("ProcessCommand:Read\t RW[%s]", rwFlag ? "read" : "write");
	
	if(GetInputCmdType() == m2_RW_CMD){
		MemAccessCmd(rwFlag);
	}
	
	
	
	
	//WriteByteToTxBuffer(gInCmdBuffer[ginCmdBufferCursor]);
	
	mccEvm.cmd_received = 1;
	mccEvm.EndOfWrite = 1;
	
	
	ClearBufferRelatedParam(CLEAR_ALL);
	
}

void MCC_Packet_Byte1Decode()
{
	CRC_EN = ((mccEvm.DATAWR_ARRAY[1] & 0x40) == 0x40) ? 1 : 0;
	MCC_RW = ((mccEvm.DATAWR_ARRAY[1] & 0x80) == 0x80) ? 1 : 0; //1->Read, 0->Write
	
	uint8_t svid = (mccEvm.DATAWR_ARRAY[0] & 0xFE) >> 1;
	
	uint8_t sv = mccEvm.DATAWR_ARRAY[1] & 0x30;
	uint8_t s_memsec = mccEvm.DATAWR_ARRAY[1] & 0x0f;
	uint8_t s_mempage = mccEvm.DATAWR_ARRAY[2] & 0xf0;
	uint16_t s_memaddr = ((mccEvm.DATAWR_ARRAY[2] & 0x0f) << 8) | mccEvm.DATAWR_ARRAY[3];

	switch (mccEvm.DATAWR_ARRAY[1] & 0x30) // decoding length
	{
	case 0x00:	// 16bit
		mccEvm.DLEN = 2;
		break;

	case 0x10:	// 32bit
		mccEvm.DLEN = 4;
		break;

	case 0x20:	// 64bit
		mccEvm.DLEN = 8;
		break;
	default:
		ErrLog("MCC_Packet_Byte1Decode not case[0x%02x]", sv);
		
		break;

	}
	mccEvm.SLAVE_ID =  (mccEvm.DATAWR_ARRAY[0] & 0xFE) >> 1;
	// info
	InfoLog("info: ADDR:0x%02x OPRW:%s CRCEN:%d DLEN:%d MEMSEC:%d MEMPAGE:%d MEMADDR:0x%04x",
		svid,
		MCC_RW?"read":"write",
		CRC_EN,
		mccEvm.DLEN,
		s_memsec,
		s_mempage,
		s_memaddr);
	
	//SEGGER_RTT_printf(0, "MCF8316 :CRC_EN[%d]\tMCC_RW[%s]\tDLEN[%d]\tSLAVE_ID[0x%02x]\n", CRC_EN, MCC_RW ? "Read" : "Write", mccEvm.DLEN, mccEvm.SLAVE_ID);

}
void MCC_I2C_Write()
{
	///写目标ID
	mccWriteRegister();
	
	mccEvm.EndOfWrite = 1;
	//mccEvm.MEMADDR_PROG = 2;
}
void MCC_I2C_Read()
{
	MCC_RW = m2_WRITE;
	mccEvm.MCC_Packet_Size = 1 + 1 + 1 + 1;
	mccEvm.DATARD_ARRAY[0] = mccEvm.DATAWR_ARRAY[0];
	mccEvm.DATARD_ARRAY[1] = mccEvm.DATAWR_ARRAY[1];
	mccEvm.DATARD_ARRAY[2] = mccEvm.DATAWR_ARRAY[2];
	mccEvm.DATARD_ARRAY[3] = mccEvm.DATAWR_ARRAY[3];
	mccEvm.DATARD_ARRAY[4] = (mccEvm.DATAWR_ARRAY[0]+1);
	mccReadRegister();
	
	//mccEvm.MEMADDR_PROG = 2;
	
	MCC_RW = m2_READ;

	mccEvm.EndOfWrite = 1;
	
	
}

void m2_mcf8136RW()
{
	MCC_Packet_Byte1Decode();
	
	switch (MCC_RW)
	{
		case 0x01:
			{
				MCC_I2C_Read();
			}break;
		case 0x00:
			{
				mccEvm.MCC_Packet_Size = 1 + 1 + 1 + 1 + mccEvm.DLEN + CRC_EN;
				MCC_I2C_Write();
				
			}break;
		default: 
			{
				ErrLog("m2_mcf8136RW not case[0x%02x]", MCC_RW);
			}
			break;
	}
}



void MCC_Decode()
{
	switch (mccEvm.MEMADDR_PROG)
	{
		/// mfc8316
		case 0x07: {
			m2_mcf8136RW();
			
		}break;
		case 0x02: {
			// server ???
		}
		break;
		default:
		{
			//SEGGER_RTT_printf(0, "MCC_Decod not case:[0x%02x]\n", mccEvm.MEMADDR_PROG);
			ErrLog("MCC_Decod not case:[0x%02x]", mccEvm.MEMADDR_PROG);
		}
		break;
	}
	
}



//------------------spinlock

//#include <stdbool.h>
//#include <stdint.h>


void spin_lock_init(spinlock_t* lock)
{
	lock->lock = 0;
}


uint32_t spin_lock_trylock(spinlock_t* lock)
{
	return (__LDREXW(&lock->lock) == 0) && (__STREXW(1, &lock->lock) == 0);
}


void spin_lock(spinlock_t* lock)
{
	while (!spin_lock_trylock(lock)) {
		__CLREX();
	}
}
void spin_unlock(spinlock_t* lock)
{
	lock->lock = 0;
}
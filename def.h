#pragma once

#ifndef _DEF_H_
#define _DEF_H_


// 定义缓冲区大小
#define BUFFER_SIZE (1 << 4)
//定义缓冲区大小掩码
#define BUFFER_SIZE_MASK ( BUFFER_SIZE -1)

// 定义指令缓冲区大小
#define CMD_BUFFER_SIZE 68

//RW CMD TYPE
#define m2_READ 1
#define m2_WRITE 0
#define     m2_RW_CMD           0x80

#define LITTLE_ENDIAN 0

#define ENUM_ToString(x) #x

#define ErrLog(fmt, ...) do{SEGGER_RTT_printf(0,"%s [%s:%d] ERROR\t",__func__,__FILE__,__LINE__);SEGGER_RTT_printf(0, fmt, ##__VA_ARGS__);SEGGER_RTT_printf(0,"\n");}while(0)
#define InfoLog(fmt,...) do{SEGGER_RTT_printf(0,"%s [%s:%d] INFO \t",__func__,__FILE__,__LINE__);SEGGER_RTT_printf(0, fmt, ##__VA_ARGS__);SEGGER_RTT_printf(0,"\n");}while(0)
//SEGGER_RTT_printf(2, "[%s]HAL_I2C_Master_Seq_Transmit_DMA error\n", __func__);

// 发送数据到mcf8316
#define PROG_TX8316 2


#define CLEAR_ALL 0xff



//extern volatile uint32_t m2_SystemTimeout;







typedef struct
{	
	uint32_t  TargetID:7;
	uint32_t  I2c_RW : 1;
	uint32_t  RW : 1;
	uint32_t  CRCEN : 1;
	uint32_t  DLEN : 2;
	uint32_t  MemSec : 4;
	uint32_t  RegPage : 4;
	uint32_t  RegAddress : 12;
	
}MccOPH_T,*pMccOPH_T;
_Static_assert(sizeof(MccOPH_T) == 4, "int must be 4 bytes");

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */
	 
	 typedef enum
	 {
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
		 
	 } addrName_Type;
	 
 
	 
	 
	  typedef struct {
		 volatile uint32_t lock;
	 } spinlock_t;
	 
	 typedef struct
	 {
		 uint8_t  gInCmdBuffer[68];
		 uint8_t  NVMData[64];
		 uint8_t DATARD_ARRAY[16];
		 uint8_t DATAWR_ARRAY[14];
		 
		 short  DLEN;
		 
		 short EndOfWrite;
		 short MCC_Packet_Size;
		 short MCC_RW;
		 short MEMADDR_PROG; // 偏移 0xAA
		 short NVMAddress; // 偏移 0xAC
		 
		 short NVMWrite; // 偏移 0xAE
		 short RXCOUNT_I2C; // 偏移 0xB0
		 short cmd_received; // 偏移 0xB2
		 
		 volatile uint16_t gInCmdSkipCount; // 偏移 0xB4
		 uint8_t SLAVE_ID; // 偏移 0xB6
		 uint8_t SLAVE_ID2; // 偏移 0xB7

	 }mccEvm_t;
	 
	 
	 _Static_assert(sizeof(mccEvm_t) == 184, "int must be 184 bytes");
	 
	 extern mccEvm_t mccEvm;

	//#define spin_lock_init(_lck) do{_lck->lock=0;}while(0)
	 void spin_lock_init(spinlock_t* lock);
	//#define spin_unlock(_lck) do{_lck->lock=0;}while(0)
	 void spin_unlock(spinlock_t* lock);
	 //uint32_t spin_lock_trylock(spinlock_t* lock) __attribute__((always_inline));
	 uint32_t spin_lock_trylock(spinlock_t* lock);
	//#define spin_lock(_lck) do{while(!spin_lock_trylock(_lck)){};}while(0)
	 void spin_lock(spinlock_t* lock);

	 // 定义全局变量
	 
		 //用于串口通讯读取缓冲区,68个字节
	 //extern uint8_t aRxBuffer[BUFFER_SIZE];
	 //extern volatile uint8_t aRxBufferIdx;
	 //extern volatile uint8_t aRxBufferCursor;
	 
		 // 用于穿孔通讯发送缓冲区,68个字节
	 extern uint8_t aTxBuffer[256];
	 extern volatile uint8_t aTxBufferIdx;
	 
		 // 指令缓冲区
	 // extern uint8_t gInCmdBuffer[CMD_BUFFER_SIZE];
	 // 预测指令最小长度
	 //extern volatile uint8_t gInCmdBufferReqLen;
	 //extern volatile uint8_t gInCmdBufferLock;
	 extern spinlock_t gInCmdBufferLock;
	 
	// 指令缓冲区游标
	 extern volatile uint8_t gInCmdBufferIdx;
	 
	 
	 extern UART_HandleTypeDef huart1;
	 
		 // 有新指令到达
	 extern volatile int cmd_received;
	 
	 
	
	 

// 定义自建函数
	 
	 static void ITM_Writes(const char *pString, uint32_t iLen);
	 // 获取数据大小端顺序
	 
	 // 写8位数据到串口
	 
	 void ResetInCmdBuffer();
	 
	 void InitBuffer();
	 
	 int WriteByteToInCmdBuffer(unsigned char d);
	 
	 //void SyncReceivedDataToCmdBuffer();
	 
	 
	 /// 处理缓冲区内的指令
	 void ProcessCommand();
	 
	 void MCC_Decode();

	 extern void dump(void* addr, int len);
	 
	 extern uint8_t crc8(uint8_t *data, uint32_t len);
	 
	 void mccReadRegister();
	 
	 void mccWriteRegister();
	 
	 
	 
#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif

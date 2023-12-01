替代原厂 MCF8316AEVM 的 MSP430 固件，移植（重写）了适用于 STM32F401 的代码
mps430代码来源由 www.ti.com 上下载,原来的固件源代码是msp430的MCU
现在已经移植到stm32f401,经过测试当前可以与 "MCF8316A_GUI 1.1.8" 进行通讯,
可以实现GUI控制EVM的功能,但是还有一些问题,比如:stm32的代码是异步完成,导致串口速度无法调整(默认9600)
串口通讯芯片需要特定的串行芯片 (FTDI)以保证 "MCF8316A_GUI"  可以识别到串行通讯端口

项目由"visualgdb"建立,使用了"STM32CubeMX"进行配置.

测试用硬件
https://oshwhub.com/syscjx/mcf8316_demo

固件地址
https://github.com/sdvcn/mcf8316_gui_stm32
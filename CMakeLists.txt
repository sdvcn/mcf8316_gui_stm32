cmake_minimum_required(VERSION 3.15)

project(EmbeddedProject1 LANGUAGES C CXX ASM)

find_bsp(
	ID com.sysprogs.project_importers.stm32.cubemx
	SOURCE_PROJECT EmbeddedProject1.ioc
	HWREGISTER_LIST_FILE STM32F103.mcudef)

add_bsp_based_executable(
	NAME EmbeddedProject1
	GENERATE_BIN
	GENERATE_MAP
	SOURCES
		${BSP_ROOT}/m2.c
		${BSP_ROOT}/def.h
		${BSP_ROOT}/SEGGER/SEGGER_RTT.c
		${BSP_ROOT}/SEGGER/SEGGER_RTT_printf.c
		${BSP_ROOT}/SEGGER/SEGGER_RTT_Syscalls_SES.c
		${BSP_ROOT}/crc8.c
		${BSP_ROOT}/i2ctools.c)


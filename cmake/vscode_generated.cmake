# This is converter generated file, and shall not be touched by user
#
# Use CMakeLists.txt to apply user changes
cmake_minimum_required(VERSION 3.22)

set(STM32F10x_StdPeriph_Lib "${CMAKE_SOURCE_DIR}/../STM32F10x_StdPeriph_Lib_V3.6.0" CACHE STRING "STM32F10x_StdPeriph_Lib root path")
message(">>> STM32F10x_StdPeriph_Lib: " ${STM32F10x_StdPeriph_Lib})

set(STM32_TYPE "STM32F10X_MD" CACHE STRING "STM32 types can be: STM32F10X_LD, STM32F10X_MD, STM32F10X_HD, STM32F10X_CL, STM32F10X_XL, STM32F10X_LD_VL, STM32F10X_MD_VL, STM32F10X_HD_VL, STM32F10X_CL_VL, STM32F10X_XL_VL")

if (STM32_TYPE STREQUAL "STM32F10X_LD")
    add_definitions(-DSTM32F10X_LD)
    set(STM32_Startup startup_stm32f10x_ld.s)
elseif (STM32_TYPE STREQUAL "STM32F10X_MD")
    add_definitions(-DSTM32F10X_MD)
    set(STM32_Startup startup_stm32f10x_md.s)
elseif (STM32_TYPE STREQUAL "STM32F10X_HD")
    add_definitions(-DSTM32F10X_HD)
    set(STM32_Startup startup_stm32f10x_hd.s)
elseif (STM32_TYPE STREQUAL "STM32F10X_CL")
    add_definitions(-DSTM32F10X_CL)
    set(STM32_Startup startup_stm32f10x_cl.s)
elseif (STM32_TYPE STREQUAL "STM32F10X_XL")
    add_definitions(-DSTM32F10X_XL)
    set(STM32_Startup startup_stm32f10x_xl.s)
elseif (STM32_TYPE STREQUAL "STM32F10X_LD_VL")
    add_definitions(-DSTM32F10X_LD_VL)
    set(STM32_Startup startup_stm32f10x_ld_vl.s)
elseif (STM32_TYPE STREQUAL "STM32F10X_MD_VL")
    add_definitions(-DSTM32F10X_MD_VL)
    set(STM32_Startup startup_stm32f10x_md_vl.s)
elseif (STM32_TYPE STREQUAL "STM32F10X_HD_VL")
    add_definitions(-DSTM32F10X_HD_VL)
    set(STM32_Startup startup_stm32f10x_hd_vl.s)
else()
    message(FATAL_ERROR "Unknown STM32 type: " ${STM32_TYPE})
endif()

message(STATUS "=== STM32_TYPE: ${STM32_TYPE}")

add_definitions(-DUSE_STDPERIPH_DRIVER)
option(ENABLE_STDPERIPH_ADC OFF)
option(ENABLE_STDPERIPH_BKP OFF)
option(ENABLE_STDPERIPH_CAN OFF)
option(ENABLE_STDPERIPH_CEC OFF)
option(ENABLE_STDPERIPH_CRC OFF)
option(ENABLE_STDPERIPH_DAC OFF)
option(ENABLE_STDPERIPH_DBGMCU OFF)
option(ENABLE_STDPERIPH_DMA OFF)
option(ENABLE_STDPERIPH_EXTI OFF)
option(ENABLE_STDPERIPH_FLASH OFF)
option(ENABLE_STDPERIPH_FSMC OFF)
option(ENABLE_STDPERIPH_GPIO ON)
option(ENABLE_STDPERIPH_I2C OFF)
option(ENABLE_STDPERIPH_IWDG OFF)
option(ENABLE_STDPERIPH_PWR OFF)
option(ENABLE_STDPERIPH_RCC ON)
option(ENABLE_STDPERIPH_RTC OFF)
option(ENABLE_STDPERIPH_SDIO OFF)
option(ENABLE_STDPERIPH_SPI OFF)
option(ENABLE_STDPERIPH_TIM OFF)
option(ENABLE_STDPERIPH_USART OFF)
option(ENABLE_STDPERIPH_WWDG OFF)
option(ENABLE_STDPERIPH_FRAMEWORK ON)
option(ENABLE_FULL_ASSERT OFF)

set(STM32_STD_PERIPH_LIBRARY)

if(ENABLE_STDPERIPH_ADC)
    add_definitions(-DRTE_DEVICE_STDPERIPH_ADC)
    list(APPEND STM32_STD_PERIPH_LIBRARY ${STM32F10x_StdPeriph_Lib}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_adc.c)
    message(STATUS "=== ENABLE_STDPERIPH_ADC: ON")
endif()

if(ENABLE_STDPERIPH_BKP)
    add_definitions(-DRTE_DEVICE_STDPERIPH_BKP)
    list(APPEND STM32_STD_PERIPH_LIBRARY ${STM32F10x_StdPeriph_Lib}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_bkp.c)
    message(STATUS "=== ENABLE_STDPERIPH_BKP: ON")
endif()

if(ENABLE_STDPERIPH_CAN)
    add_definitions(-DRTE_DEVICE_STDPERIPH_CAN)
    list(APPEND STM32_STD_PERIPH_LIBRARY ${STM32F10x_StdPeriph_Lib}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_can.c)
    message(STATUS "=== ENABLE_STDPERIPH_CAN: ON")
endif()

if(ENABLE_STDPERIPH_CEC)
    add_definitions(-DRTE_DEVICE_STDPERIPH_CEC)
    list(APPEND STM32_STD_PERIPH_LIBRARY ${STM32F10x_StdPeriph_Lib}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_cec.c)
    message(STATUS "=== ENABLE_STDPERIPH_CEC: ON")
endif()

if(ENABLE_STDPERIPH_CRC)
    add_definitions(-DRTE_DEVICE_STDPERIPH_CRC)
    list(APPEND STM32_STD_PERIPH_LIBRARY ${STM32F10x_StdPeriph_Lib}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_crc.c)
    message(STATUS "=== ENABLE_STDPERIPH_CRC: ON")
endif()

if(ENABLE_STDPERIPH_DAC)
    add_definitions(-DRTE_DEVICE_STDPERIPH_DAC)
    list(APPEND STM32_STD_PERIPH_LIBRARY ${STM32F10x_StdPeriph_Lib}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_dac.c)
    message(STATUS "=== ENABLE_STDPERIPH_DAC: ON")
endif()

if(ENABLE_STDPERIPH_DBGMCU)
    add_definitions(-DRTE_DEVICE_STDPERIPH_DBGMCU)
    list(APPEND STM32_STD_PERIPH_LIBRARY ${STM32F10x_StdPeriph_Lib}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_dbgmcu.c)
    message(STATUS "=== ENABLE_STDPERIPH_DBGMCU: ON")
endif()

if(ENABLE_STDPERIPH_DMA)
    add_definitions(-DRTE_DEVICE_STDPERIPH_DMA)
    list(APPEND STM32_STD_PERIPH_LIBRARY ${STM32F10x_StdPeriph_Lib}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_dma.c)
    message(STATUS "=== ENABLE_STDPERIPH_DMA: ON")
endif()

if(ENABLE_STDPERIPH_EXTI)
    add_definitions(-DRTE_DEVICE_STDPERIPH_EXTI)
    list(APPEND STM32_STD_PERIPH_LIBRARY ${STM32F10x_StdPeriph_Lib}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_exti.c)
    message(STATUS "=== ENABLE_STDPERIPH_EXTI: ON")
endif()

if(ENABLE_STDPERIPH_FLASH)
    add_definitions(-DRTE_DEVICE_STDPERIPH_FLASH)
    list(APPEND STM32_STD_PERIPH_LIBRARY ${STM32F10x_StdPeriph_Lib}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_flash.c)
    message(STATUS "=== ENABLE_STDPERIPH_FLASH: ON")
endif()

if(ENABLE_STDPERIPH_FSMC)
    add_definitions(-DRTE_DEVICE_STDPERIPH_FSMC)
    list(APPEND STM32_STD_PERIPH_LIBRARY ${STM32F10x_StdPeriph_Lib}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_fsmc.c)
    message(STATUS "=== ENABLE_STDPERIPH_FSMC: ON")
endif()

if(ENABLE_STDPERIPH_GPIO)
    add_definitions(-DRTE_DEVICE_STDPERIPH_GPIO)
    list(APPEND STM32_STD_PERIPH_LIBRARY ${STM32F10x_StdPeriph_Lib}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_gpio.c)
    message(STATUS "=== ENABLE_STDPERIPH_GPIO: ON")
endif()

if(ENABLE_STDPERIPH_I2C)
    add_definitions(-DRTE_DEVICE_STDPERIPH_I2C)
    list(APPEND STM32_STD_PERIPH_LIBRARY ${STM32F10x_StdPeriph_Lib}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_i2c.c)
    message(STATUS "=== ENABLE_STDPERIPH_I2C: ON")
endif()

if(ENABLE_STDPERIPH_IWDG)
    add_definitions(-DRTE_DEVICE_STDPERIPH_IWDG)
    list(APPEND STM32_STD_PERIPH_LIBRARY ${STM32F10x_StdPeriph_Lib}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_iwdg.c)
    message(STATUS "=== ENABLE_STDPERIPH_IWDG: ON")
endif()

if(ENABLE_STDPERIPH_PWR)
    add_definitions(-DRTE_DEVICE_STDPERIPH_PWR)
    list(APPEND STM32_STD_PERIPH_LIBRARY ${STM32F10x_StdPeriph_Lib}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_pwr.c)
    message(STATUS "=== ENABLE_STDPERIPH_PWR: ON")
endif()

if(ENABLE_STDPERIPH_RCC)
    add_definitions(-DRTE_DEVICE_STDPERIPH_RCC)
    list(APPEND STM32_STD_PERIPH_LIBRARY ${STM32F10x_StdPeriph_Lib}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_rcc.c)
    message(STATUS "=== ENABLE_STDPERIPH_RCC: ON")
endif()

if(ENABLE_STDPERIPH_RTC)
    add_definitions(-DRTE_DEVICE_STDPERIPH_RTC)
    list(APPEND STM32_STD_PERIPH_LIBRARY ${STM32F10x_StdPeriph_Lib}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_rtc.c)
    message(STATUS "=== ENABLE_STDPERIPH_RTC: ON")
endif()

if(ENABLE_STDPERIPH_SDIO)
    add_definitions(-DRTE_DEVICE_STDPERIPH_SDIO)
    list(APPEND STM32_STD_PERIPH_LIBRARY ${STM32F10x_StdPeriph_Lib}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_sdio.c)
    message(STATUS "=== ENABLE_STDPERIPH_SDIO: ON")
endif()

if(ENABLE_STDPERIPH_SPI)
    add_definitions(-DRTE_DEVICE_STDPERIPH_SPI)
    list(APPEND STM32_STD_PERIPH_LIBRARY ${STM32F10x_StdPeriph_Lib}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_spi.c)
    message(STATUS "=== ENABLE_STDPERIPH_SPI: ON")
endif()

if(ENABLE_STDPERIPH_TIM)
    add_definitions(-DRTE_DEVICE_STDPERIPH_TIM)
    list(APPEND STM32_STD_PERIPH_LIBRARY ${STM32F10x_StdPeriph_Lib}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_tim.c)
    message(STATUS "=== ENABLE_STDPERIPH_TIM: ON")
endif()

if(ENABLE_STDPERIPH_USART)
    add_definitions(-DRTE_DEVICE_STDPERIPH_USART)
    list(APPEND STM32_STD_PERIPH_LIBRARY ${STM32F10x_StdPeriph_Lib}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_usart.c)
    message(STATUS "=== ENABLE_STDPERIPH_USART: ON")
endif()

if(ENABLE_STDPERIPH_WWDG)
    add_definitions(-DRTE_DEVICE_STDPERIPH_WWDG)
    list(APPEND STM32_STD_PERIPH_LIBRARY ${STM32F10x_StdPeriph_Lib}/Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_wwdg.c)
    message(STATUS "=== ENABLE_STDPERIPH_WWDG: ON")
endif()

if(ENABLE_STDPERIPH_FRAMEWORK)
    add_definitions(-DRTE_DEVICE_STDPERIPH_FRAMEWORK)
    list(APPEND STM32_STD_PERIPH_LIBRARY ${STM32F10x_StdPeriph_Lib}/Libraries/STM32F10x_StdPeriph_Driver/src/misc.c)
    message(STATUS "=== ENABLE_STDPERIPH_FRAMEWORK: ON")
endif()

# Core MCU flags, CPU, instruction set and FPU setup
set(cpu_PARAMS ${cpu_PARAMS}
    -mthumb

    # Other parameters
    # -mcpu, -mfloat, -mfloat-abi, ...
    -mcpu=cortex-m3
)

# Linker script
set(linker_script_SRC ${linker_script_SRC}
    ${CMAKE_CURRENT_SOURCE_DIR}/stm32f103c8tx_FLASH.ld
)

set(STM32_STARTUP_DIR "${CMAKE_CURRENT_SOURCE_DIR}/startup")

# Sources
set(sources_SRCS ${sources_SRCS}
    ${CMAKE_CURRENT_SOURCE_DIR}/source/main.c
    ${STM32_STARTUP_DIR}/core_cm3.c
    ${STM32_STARTUP_DIR}/core_cm3.h
    ${STM32_STARTUP_DIR}/system_stm32f10x.c
    ${STM32_STARTUP_DIR}/system_stm32f10x.h
    ${STM32_STARTUP_DIR}/${STM32_Startup}
    ${STM32_STD_PERIPH_LIBRARY}
)

# Include directories
set(include_c_DIRS ${include_c_DIRS}
    ${STM32_STARTUP_DIR}
    ${STM32_STARTUP_DIR}/STM32F10x
    ${STM32F10x_StdPeriph_Lib}/Libraries/STM32F10x_StdPeriph_Driver/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
set(include_cxx_DIRS ${include_cxx_DIRS}
    ${STM32_STARTUP_DIR}
    ${STM32_STARTUP_DIR}/STM32F10x
    ${STM32F10x_StdPeriph_Lib}/Libraries/STM32F10x_StdPeriph_Driver/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
set(include_asm_DIRS ${include_asm_DIRS}
    # ...existing code...
)

# Symbols definition
set(symbols_c_SYMB ${symbols_c_SYMB}
    # ...existing code...
)
set(symbols_cxx_SYMB ${symbols_cxx_SYMB}
    # ...existing code...
)
set(symbols_asm_SYMB ${symbols_asm_SYMB}
    # ...existing code...
)

# Link directories
set(link_DIRS ${link_DIRS}
    # ...existing code...
)

# Link libraries
set(link_LIBS ${link_LIBS}
    # ...existing code...
)

# Compiler options
set(compiler_OPTS ${compiler_OPTS})

# Linker options
set(linker_OPTS ${linker_OPTS})

cmake_minimum_required(VERSION 2.8)

project(uart-test C CXX ASM)

set(MCU atmega328p)
set(F_CPU 16000000UL)
set(PORT /dev/ttyUSB0)
set(PROGRAMMER arduino)

set(AVRCC	avr-gcc)
set(AVRCPP	avr-g++)
set(AVROBJ	avr-objcopy)
set(AVRDUDE	avrdude)

set(CMAKE_C_COMPILER ${AVRCC})
set(CMAKE_CXX_COMPILER ${AVRCPP})
set(CMAKE_ASM_COMPILER ${AVRCC})
set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS "")
set(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "")

set(BASE_PATH	"${${PROJECT_NAME}_SOURCE_DIR}")
set(SRC_PATH	"${BASE_PATH}/src")

file(GLOB SRC_FILES "${SRC_PATH}/*.cc"
					"${SRC_PATH}/*.c")

# Compiler flags
set(CSTANDARD "-std=c99")
set(CWARN     "-Wall")
set(CTUNING   "-funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -ffunction-sections -fdata-sections")
set(COPT      "-Os")
set(CMCU      "-mmcu=${MCU}")
set(CDEFS     "-DF_CPU=${F_CPU}")

set(CFLAGS   "${CMCU} ${CDEFS} ${COPT} ${CWARN} ${CSTANDARD} ${CTUNING}")
set(CXX_FLAGS "${CMCU} ${CDEFS} ${COPT} ${CWARN} ${CTUNING}")

set(CMAKE_C_FLAGS ${CFLAGS})
set(CMAKE_CXX_FLAGS ${CXX_FLAGS})
set(CMAKE_ASM_FLAGS ${CFLAGS})

include_directories("/usr/avr/include" )
add_executable(${PROJECT_NAME} ${SRC_FILES})
set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME "${PROJECT_NAME}.elf")

add_custom_target(hex   ALL     ${AVROBJ} -R .eeprom -O ihex "${PROJECT_NAME}.elf" "${PROJECT_NAME}.hex" DEPENDS ${PROJECT_NAME})
add_custom_target(upload         ${AVRDUDE} -v -c ${PROGRAMMER} -p ${MCU} -P ${PORT} -b 57600 -U flash:w:${PROJECT_NAME}.hex DEPENDS hex)

message("> ")
message("> Project Name:\t${PROJECT_NAME}")
message("> Project Source:\t${SRC_PATH}")
message("> Project Include:\t${INC_PATH}")
message("> Library Include:\t${LIB_INC_PATH}")
message("> ")
message("> Project Source Files:\t${SRC_FILES}")
message("> Library Source Files:\t${LIB_SRC_FILES}")
message("> ")
message("> C Flags:\t${CMAKE_C_FLAGS}")
message("> CXX Flags:\t${CMAKE_C_FLAGS}")
message("> ")

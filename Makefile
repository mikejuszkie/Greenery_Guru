#******************************************************************************
#
# Makefile - Rules for building the project example.
#
# Copyright (c) 2013-2017 Texas Instruments Incorporated.  All rights reserved.
# Software License Agreement
# 
#   Redistribution and use in source and binary forms, with or without
#   modification, are permitted provided that the following conditions
#   are met:
# 
#   Redistributions of source code must retain the above copyright
#   notice, this list of conditions and the following disclaimer.
# 
#   Redistributions in binary form must reproduce the above copyright
#   notice, this list of conditions and the following disclaimer in the
#   documentation and/or other materials provided with the  
#   distribution.
# 
#   Neither the name of Texas Instruments Incorporated nor the names of
#   its contributors may be used to endorse or promote products derived
#   from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# 
# This is part of revision 2.1.4.178 of the Tiva Firmware Development Package.
#
#******************************************************************************


# Set project name based off current directory
MKFILE_PATH := $(abspath $(lastword $(MAKEFILE_LIST)))
MKFILE_DIR_NAME := $(notdir $(patsubst %/,%,$(dir $(MKFILE_PATH))))
PROJECT_NAME := $(MKFILE_DIR_NAME)


#
# Defines the part type that this project uses.
#
PART=TM4C123GH6PM

#
# The base directory for TivaWare.
#
ROOT=./

#
# Include the common make definitions.
#
include ${ROOT}/makedefs

#
# Where to find source files that do not live in this directory.
#
VPATH=./utils
VPATH+=./third_party
VPATH+=./third_party/fatfs/src/option
VPATH+=./third_party/fatfs/port
VPATH+=./third_party/fatfs/src

#
# Where to find header files that do not live in the source directory.
#
IPATH=./
IPATH+=./third_party
IPATH+=./third_party/fatfs/src/option
IPATH+=./third_party/fatfs/port
IPATH+=./third_party/fatfs/src

#
# The default rule, which causes the project example to be built.
#
all: ${COMPILER}
all: ${COMPILER}/${PROJECT_NAME}.axf

#
# The rule to flash the program to the chip
#
flash:
	lm4flash gcc/${PROJECT_NAME}.bin 


debug: CFLAGS += -D DEBUG
debug: all


#
# The rule to clean out all the build products.
#
clean:
	@rm -rf ${COMPILER} ${wildcard *~};clear

#
# The rule to create the target directory.
#
${COMPILER}:
	@mkdir -p ${COMPILER}

#
#Rules for building guru.o
#
${COMPILER}/guru.o: ${ROOT}/driverlib/${COMPILER}/libdriver.a
${COMPILER}/guru.o: ${ROOT}/driverlib/${COMPILER}/ssi.o
${COMPILER}/guru.o: ${ROOT}/driverlib/${COMPILER}/i2c.o
${COMPILER}/guru.o: ${ROOT}/driverlib/${COMPILER}/uart.o
${COMPILER}/guru.o: ${ROOT}/driverlib/${COMPILER}/gpio.o



#
# Rules for building the ${PROJECT_NAME} example.
#
${COMPILER}/${PROJECT_NAME}.axf: ${COMPILER}/main.o
${COMPILER}/${PROJECT_NAME}.axf: ${COMPILER}/startup_${COMPILER}.o
${COMPILER}/${PROJECT_NAME}.axf: ${ROOT}/driverlib/${COMPILER}/libdriver.a	
${COMPILER}/${PROJECT_NAME}.axf: ${COMPILER}/uartstdio.o
${COMPILER}/${PROJECT_NAME}.axf: ${COMPILER}/ff.o
${COMPILER}/${PROJECT_NAME}.axf: ${COMPILER}/mmc-dk-tm4c123g.o
${COMPILER}/${PROJECT_NAME}.axf: ${COMPILER}/guru.o
${COMPILER}/${PROJECT_NAME}.axf: project.ld
SCATTERgcc_${PROJECT_NAME}=project.ld
ENTRY_${PROJECT_NAME}=ResetISR
CFLAGSgcc=-DTARGET_IS_TM4C123_RB1


#
# Include the automatically generated dependency files.
#
ifneq (${MAKECMDGOALS},clean)
-include ${wildcard ${COMPILER}/*.d} __dummy__
endif

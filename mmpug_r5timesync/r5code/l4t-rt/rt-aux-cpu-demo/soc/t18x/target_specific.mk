# Copyright (c) 2018-2020, NVIDIA CORPORATION. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#  * Neither the name of NVIDIA CORPORATION nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
# EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
# PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
# OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

FREERTOS_COMMON_INCLUDE_SOC_DIR := $(FREERTOS_COMMON_DIR)/include-soc/t186
FREERTOS_COMMON_CODE_SOC_DIR := $(FREERTOS_COMMON_DIR)/code-soc/t186
HW_INC_PUBL_DIR := $(RT_AUX_DIR)/../hwinc-t18x

TARGET_INCLUDES := \
	$(SPE_TARGET_DIR)/include/ \
	$(SPE_TARGET_DIR)/config/

TARGET_SRCS := \
	$(SPE_TARGET_DIR)/src/clk.c \
	$(SPE_TARGET_DIR)/src/init_padctrl.c \
	$(SPE_TARGET_DIR)/src/spe-pm-sequencer.c

# Enable = 1/Disable = 0 GPCDMA functionality
ENABLE_GPCDMA_FUNC := 1

# Enable = 1/Disable = 0 timer sample app
ENABLE_TIMER_APP := 0

# Enable = 1/Disable = 0 GPIO sample app
ENABLE_GPIO_APP := 0

# Enable = 1/Disable = 0 GTE sample app
ENABLE_GTE_APP := 0

# GTE app requires GPIO app to be compiled as well
ifeq ($(ENABLE_GTE_APP), 1)
	ENABLE_GPIO_APP := 1
endif

ifeq ($(ENABLE_GPIO_APP), 1)
	TARGET_SRCS += \
		$(FREERTOS_COMMON_CODE_SOC_DIR)/tegra186-gpio-ids.c
endif

# Enable = 1/Disable = 0 I2C sample app
ENABLE_I2C_APP := 0

# Enable = 1/Disable = 0 UART sample app
ENABLE_UART_APP := 0

ifeq ($(ENABLE_UART_APP), 1)
	ENABLE_UART_SRC := 1
endif

# Enable = 1/Disable = 0 SPI sample app
ENABLE_SPI_APP := 0

# Enable = 1/Disable = 0 CAN sample app
ENABLE_CAN_APP := 0
# Fine tuning based on platform, follow can-app user guide for more information
ENABLE_CAN_RX := 1
ENABLE_CAN_TX := 1

ifeq ($(ENABLE_CAN_APP), 1)
  ifeq ($(ENABLE_CAN_RX), 0)
    ifeq ($(ENABLE_CAN_TX), 0)
      $(error "At least CAN RX and/or TX need to be enabled")
    endif
  endif
endif

# Enable = 1/Disable = 0 AODMIC sample app
ENABLE_AODMIC_APP := 0

# Copyright (c) 2016-2020, NVIDIA CORPORATION. All rights reserved.
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


# Macro for checking that each variable given in list $(1) is both defined
# and has a non empty value in it.
define NV_BUILD_VARIABLES_SANITY_CHECK
$(foreach var,$(1),$(eval \
  ifndef $(var)
    _missing_vars := 1 $$(info ERROR: variable not set or empty: $(var))
  endif)) \
  $(if $(_missing_vars),$(error Environment sanity check failed))
endef
$(call NV_BUILD_VARIABLES_SANITY_CHECK,FREERTOS_DIR FREERTOS_COMMON_DIR \
	CROSS_COMPILE)

RT_AUX_DIR := $(CURDIR)
OUTDIR := $(RT_AUX_DIR)/out
SOC_PATH := $(RT_AUX_DIR)/soc
DOC_PATH := $(RT_AUX_DIR)/doc
DOC_BUILD_PATH := $(OUTDIR)/docs
CC := $(CROSS_COMPILE)gcc
OBJCOPY := $(CROSS_COMPILE)objcopy
OBJDUMP := $(CROSS_COMPILE)objdump

ALL_BUILD_TARGETS := bin_t18x bin_t19x docs
ALL_CLEAN_TARGETS := clean_t18x clean_t19x clean_docs

# This must be the first rule
.PHONY: all
all: $(ALL_BUILD_TARGETS)

hide := @
show := @test '${hide}' != @ || echo '   '

show_compile = ${show} CC ${notdir $<}
define cmd_compile
	$(CC) -c $(CFLAGS) -MD -MP -MF $(_tgt_depname) $< -o $@
endef

show_ldscript_cpp = ${show} CPP ${notdir $<}
define cmd_ldscript_cpp
	$(CC) -E $(CPPFLAGS) -MD -MP -MF $(_tgt_depname) -MT $@ -P -D __ASSEMBLY__ \
		-x assembler-with-cpp $< -o $@
endef

show_link = ${show} LINK ${notdir $@}
define cmd_link
	$(CC) $(LDFLAGS) -nostartfiles \
		-Xlinker -M -Xlinker -Map=$(patsubst %.elf,%.map,$@) \
		-Xlinker --gc-sections \
		-Xlinker -e_stext \
		$(filter-out FORCE %.ld,$^) \
		-T $(filter %.ld,$^) \
		-o $@ -lc
endef

show_elf2bin = ${show} OBJCOPY ${notdir $@}
define cmd_elf2bin
	$(OBJCOPY) -O binary $< $@
endef

show_disassemble = ${show} DISASM ${notdir $@}
define cmd_disassemble
	$(OBJDUMP) -d $< > $@
endef

# This gives false-positives if x$1x appears multiple times in x$2x. However,
# since we use this to compare command-lines, the chance of that happening is
# remote, so we ignore this issue.
cmds_differ = $(subst x$(strip $1)x,,x$(strip $2)x)

need_rebuild = $$(filter-out FORCE,$$?)$$(call cmds_differ,$$(cmd_$2),$$(_tgt_cmd_$1))

.PHONY: FORCE
FORCE: | $(OBJDIR)

# The echo _tgt_cmd_... command below and/or the parsing of $(_depname) by
# make later will fail if $(cmd_$2) contains characters that are special to the
# shell (echo) or make (parsing). This can be worked around by escaping the
# data. However, a simpler suggestion is to keep $(cmd_*) simple, i.e "don't
# do that".
define gen_rule_body
_depname := $1.d
-include $$(_depname)

$1: _tgt_depname := $$(_depname)
$1: $3 FORCE
	@set -e
	@$$(if $(need_rebuild),rm -f $$(_tgt_depname))
	@$$(if $(need_rebuild),$$(show_$2))
	${hide}$$(if $(need_rebuild),$$(cmd_$2))
	@$$(if $(need_rebuild),echo '_tgt_cmd_$$@ := $$(cmd_$2)' >> $$(_tgt_depname))
endef

gen_rule = $(eval $(gen_rule_body))

define build_target

SOC_TARGET := $(subst bin_,,$1)
SPE_TARGET_DIR := $(SOC_PATH)/$(SOC_TARGET)
include $(SPE_TARGET_DIR)/target_specific.mk

OBJDIR := $(OUTDIR)/$(SOC_TARGET)
ELF := $(OBJDIR)/spe.elf
BIN := $(OBJDIR)/spe.bin
DIS := $(OBJDIR)/spe.dis
LDSCRIPT := $(OBJDIR)/spe.ld

INCLUDES := \
	. \
	./include \
	$(TARGET_INCLUDES) \
	$(FREERTOS_DIR)/include \
	$(FREERTOS_DIR)/portable/GCC/ARM_R5 \
	$(HW_INC_PUBL_DIR) \
	$(FREERTOS_COMMON_DIR)/include-common \
	$(FREERTOS_COMMON_INCLUDE_SOC_DIR)-aon \
	$(FREERTOS_COMMON_INCLUDE_SOC_DIR)

CPPFLAGS := \
	$(addprefix -I,$(abspath $(INCLUDES))) \

CFLAGS := \
	$(CPPFLAGS) \
	$(TARGET_CFLAGS) \
	-mcpu=cortex-r5 -mthumb-interwork \
	-fdata-sections -ffunction-sections -mlong-calls -g3 \
	-mfloat-abi=softfp -mfpu=vfpv3-d16 \
	-Wall -std=c99 -pedantic \
	-Wstrict-prototypes -Wmissing-prototypes \
	-Werror -Wpointer-arith \
	-Wfloat-equal -Wundef -Wshadow -Wbad-function-cast \
	-Wwrite-strings -Waggregate-return -Wmissing-declarations \
	-Wmissing-format-attribute -Wno-deprecated-declarations -Wpacked \
	-Wredundant-decls -Wnested-externs -Wunreachable-code

ifeq ($(ENABLE_SPE_FOR_NX), 1)
	CFLAGS += -DENABLE_SPE_FOR_NX
endif

ifeq ($(ENABLE_COMBINED_UART_SERVER), 1)
	CFLAGS += -DENABLE_COMBINED_UART_SERVER
endif

ifeq ($(ENABLE_GPCDMA_FUNC), 1)
	CFLAGS += -DENABLE_GPCDMA_FUNC
endif

ifeq ($(ENABLE_GPIO_APP), 1)
	CFLAGS += -DENABLE_GPIO_APP
endif

ifeq ($(ENABLE_TIMER_APP), 1)
	CFLAGS += -DENABLE_TIMER_APP
endif

ifeq ($(ENABLE_I2C_APP), 1)
	CFLAGS += -DENABLE_I2C_APP
endif

ifeq ($(ENABLE_UART_APP), 1)
	CFLAGS += -DENABLE_UART_APP
endif

ifeq ($(ENABLE_SPI_APP), 1)
	CFLAGS += -DENABLE_SPI_APP
endif

ifeq ($(ENABLE_GTE_APP), 1)
	CFLAGS += -DENABLE_GTE_APP
endif

ifeq ($(ENABLE_CAN_APP), 1)
	CFLAGS += -DENABLE_CAN_APP
   ifeq ($(ENABLE_CAN_TX), 1)
	CFLAGS += -DENABLE_CAN_TX
   endif
   ifeq ($(ENABLE_CAN_RX), 1)
	CFLAGS += -DENABLE_CAN_RX
   endif
endif

ifeq ($(ENABLE_AODMIC_APP), 1)
	CFLAGS += -DENABLE_AODMIC_APP
endif

LDFLAGS := \
	$(CFLAGS)

SRCS := \
	$(TARGET_SRCS) \
	$(FREERTOS_COMMON_DIR)/code-common/armv7-exceptions.S \
	$(FREERTOS_COMMON_DIR)/code-common/newlib-syscalls.c \
	$(FREERTOS_COMMON_DIR)/code-common/err-hook-printf.c \
	$(FREERTOS_COMMON_DIR)/code-common/arm-vic.c \
	$(FREERTOS_COMMON_DIR)/code-common/irqapi-vic.c \
	$(FREERTOS_COMMON_DIR)/code-common/irqapi-cortex-r5.c \
	$(FREERTOS_COMMON_DIR)/code-common/tke-tegra.c \
	$(FREERTOS_COMMON_DIR)/code-common/cache-cortex-r5.c \
	$(FREERTOS_COMMON_DIR)/code-common/armv7-exception.c \
	$(FREERTOS_COMMON_DIR)/code-common/tke-tegra-udelay.c \
	$(FREERTOS_COMMON_DIR)/code-common/hsp-tegra.c \
	$(FREERTOS_COMMON_DIR)/code-common/debug.c \
	$(FREERTOS_COMMON_DIR)/code-common/tegra-ivc.c \
	$(FREERTOS_COMMON_DIR)/code-common/wake-tegra.c \
	$(FREERTOS_COMMON_DIR)/code-common/ast-tegra.c \
	$(FREERTOS_COMMON_CODE_SOC_DIR)-aon/tke-tegra-ids.c \
	$(FREERTOS_COMMON_CODE_SOC_DIR)-aon/hsp-tegra-ids.c \
	$(FREERTOS_COMMON_CODE_SOC_DIR)-aon/ast-tegra-ids.c \
	$(FREERTOS_COMMON_CODE_SOC_DIR)-aon/irqapi-vic-ids.c \
	$(FREERTOS_COMMON_CODE_SOC_DIR)/clk-tegra.c \
	$(FREERTOS_DIR)/portable/GCC/ARM_R5/portASM.S \
	$(FREERTOS_DIR)/list.c \
	$(FREERTOS_DIR)/portable/MemMang/heap_3.c \
	$(FREERTOS_DIR)/portable/GCC/ARM_R5/port.c \
	$(FREERTOS_DIR)/queue.c \
	$(FREERTOS_DIR)/tasks.c \
	$(FREERTOS_DIR)/timers.c \
	$(FREERTOS_DIR)/event_groups.c \
	app/ivc-echo-task.c \
	drivers/bpmp-ipc.c \
	drivers/interrupt.c \
	drivers/irq-handlers.S \
	drivers/ivc-channels.c \
	drivers/spe-vic.c \
	drivers/spe-pm.c \
	platform/debug_init.c \
	platform/ivc-channel-ids.c \
	boot.S \
	main.c

ifeq ($(ENABLE_UART_SRC), 1)
	SRCS += \
		$(FREERTOS_COMMON_DIR)/code-common/uart-tegra.c \
		$(FREERTOS_COMMON_CODE_SOC_DIR)/uart-tegra-ids.c
endif

ifeq ($(ENABLE_COMBINED_UART_SERVER), 1)
	SRCS += \
		$(SPE_TARGET_DIR)/src/combined-uart.c \
		$(SPE_TARGET_DIR)/src/combined-uart-ids.c
endif

ifeq ($(ENABLE_GPCDMA_FUNC), 1)
	SRCS += \
		$(FREERTOS_COMMON_DIR)/code-common/tegra-gpcdma.c \
		$(FREERTOS_COMMON_CODE_SOC_DIR)-aon/tegra-gpcdma-ids.c
endif

ifeq ($(ENABLE_GPIO_APP), 1)
	SRCS += \
		$(FREERTOS_COMMON_DIR)/code-common/gpio.c \
		$(FREERTOS_COMMON_DIR)/code-common/tegra186-gpio.c \
		$(RT_AUX_DIR)/app/gpio-app.c
endif

ifeq ($(ENABLE_TIMER_APP), 1)
	SRCS += \
		$(RT_AUX_DIR)/app/timer-app.c
endif

ifeq ($(ENABLE_I2C_APP), 1)
	SRCS += \
		$(FREERTOS_COMMON_CODE_SOC_DIR)/i2c-tegra-ids.c \
		$(FREERTOS_COMMON_DIR)/code-common/i2c-tegra.c \
		$(RT_AUX_DIR)/app/i2c-app.c \
		$(SPE_TARGET_DIR)/src/i2c-app-priv.c
endif

ifeq ($(ENABLE_UART_APP), 1)
	SRCS += \
		$(RT_AUX_DIR)/app/uart-app.c
endif

ifeq ($(ENABLE_SPI_APP), 1)
	SRCS += \
		$(FREERTOS_COMMON_CODE_SOC_DIR)/spi-tegra-ids.c \
		$(FREERTOS_COMMON_DIR)/code-common/spi-tegra.c \
		$(RT_AUX_DIR)/app/spi-app.c
endif

ifeq ($(ENABLE_GTE_APP), 1)
	SRCS += \
		$(FREERTOS_COMMON_CODE_SOC_DIR)-aon/gte-tegra-ids.c \
		$(FREERTOS_COMMON_DIR)/code-common/gte-tegra.c \
		$(RT_AUX_DIR)/app/gte-app.c
endif

ifeq ($(ENABLE_CAN_APP), 1)
	SRCS += \
		$(FREERTOS_COMMON_CODE_SOC_DIR)-aon/tegra-mttcan-ids.c \
		$(FREERTOS_COMMON_DIR)/code-common/tegra-mttcan.c \
		$(FREERTOS_COMMON_DIR)/code-common/tegra-can.c \
		$(RT_AUX_DIR)/app/can-app.c \
		$(RT_AUX_DIR)/app/can-app-common.c
   ifeq ($(ENABLE_CAN_RX), 1)
	SRCS += \
		$(RT_AUX_DIR)/app/can-app-rx-task.c
   endif
   ifeq ($(ENABLE_CAN_TX), 1)
	SRCS += \
		$(RT_AUX_DIR)/app/can-app-tx-task.c
   endif
endif

ifeq ($(ENABLE_AODMIC_APP), 1)
	SRCS += \
		$(FREERTOS_COMMON_CODE_SOC_DIR)-aon/tegra-aodmic-ids.c \
		$(FREERTOS_COMMON_DIR)/code-common/tegra-aodmic.c \
		$(RT_AUX_DIR)/app/aodmic-app.c
endif

objname = $(abspath $(addprefix $(OBJDIR)/,\
	$(addsuffix .o,$(basename $(notdir $1)))))

OBJS := $(call objname,$(SRCS))

.PHONY: $1
#
# Set all target-specific variables here
#
$1: CFLAGS := $(CFLAGS)
$1: OBJDIR := $(OBJDIR)

$1: $(OBJDIR) $(BIN) $(ELF) $(DIS) $(LDSCRIPT)

$(OBJDIR):
	mkdir -p $(OBJDIR)

$(foreach src,$(SRCS),$(call gen_rule,$(call objname,$(src)),compile,$(src)))

$(call gen_rule,$(LDSCRIPT),ldscript_cpp,spe.ld.in)

$(call gen_rule,$(ELF),link,$(OBJS) $(OBJDIR)/spe.ld,spe.map)

$(call gen_rule,$(BIN),elf2bin,$(OBJDIR)/spe.elf)

$(call gen_rule,$(DIS),disassemble,$(OBJDIR)/spe.elf)

endef

target_rules = $(eval $(value build_target))
$(foreach target,$(filter-out docs, $(ALL_BUILD_TARGETS)),$(call target_rules,$(target)))

$(DOC_BUILD_PATH):
	mkdir -p $(DOC_BUILD_PATH)

$(DOC_BUILD_PATH)/.stamp: $(DOC_PATH)/doxyfile | $(DOC_BUILD_PATH)
	${show} DOXYGEN $<
	${hide} export OUTDIR=$(OUTDIR) && doxygen $< 2>$(DOC_BUILD_PATH)/error_log.txt
	${hide} touch $@

.PHONY: docs
docs: $(DOC_BUILD_PATH)/.stamp

.PHONY: clean
clean: $(ALL_CLEAN_TARGETS)

define clean_target
BUILD_DIR := $(subst clean_,,$(target))

.PHONY: $(target)
$(target): BUILD_DIR := $(BUILD_DIR)
$(target):
	${show} RMDIR $(BUILD_DIR)
	${hide} rm -rf $(OUTDIR)/$(BUILD_DIR)
endef

$(foreach target,$(ALL_CLEAN_TARGETS),$(eval $(value clean_target)))

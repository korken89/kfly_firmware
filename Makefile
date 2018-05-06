#-------------------------------------------
# Makefile for KFly Dronecode
# Author: (C) Emil Fresk 2018
#-------------------------------------------

# Target same
TARGET ?= kfly_dronecode

# Verbose
V0 = @

# Where the build will be located and create the folder
OBJDIR = ./build/obj
ELFDIR = ./build

# Optimization
OPTIMIZATION = -O1 -g

# Create include paths and find sources in all the modules
MODULES = $(wildcard ./Modules/*)
$(foreach dir, $(MODULES), $(eval MODULES_INC += -I$(dir)/include))
$(foreach dir, $(MODULES), $(eval MODULES_CSRCS += $(wildcard $(dir)/source/*.c)))
$(foreach dir, $(MODULES), $(eval MODULES_ASRCS += $(wildcard $(dir)/source/*.s)))

# Sources
INCLUDE = $(MODULES_INC)
INCLUDE += -I./CMSIS -I./Libraries/STM32F4xx_StdPeriph_Driver/inc
CSRCS = $(wildcard ./CMSIS/*.c) $(MODULES_CSRCS)

# FreeRTOS includes and source files
RTOS_DIR = ./FreeRTOS/
RTOS_PORT = $(RTOS_DIR)portable/GCC/ARM_CM4F/
RTOS_MEM = $(RTOS_DIR)portable/MemMang/
INCLUDE += -I$(RTOS_PORT) -I$(RTOS_DIR)include/
CSRCS += $(RTOS_DIR)list.c $(RTOS_DIR)tasks.c $(RTOS_DIR)queue.c $(RTOS_PORT)port.c $(RTOS_MEM)heap_1.c

COMMON += -DDATE="\"$(DATE)\"" -DGIT_HASH="\"$(GIT_HASH_SUBSTR)\""  -DGIT_VERSION="\"$(GIT_VERSION)\"" -fno-builtin-_exit

ASRCS   = $(wildcard ./CMSIS/*.s) $(MODULES_ASRCS)
ALLSRC = $(ASRCS) $(CSRCS)
ALLSRCBASE = $(notdir $(basename $(ALLSRC)))
ALLOBJECTS = $(addprefix $(OBJDIR)/, $(addsuffix .o, $(ALLSRCBASE)))


# Include definitions
include make/defs.mk

all: build

build: elf bin hex lss sym size

# Link: Create elf output file from object files.
$(eval $(call LINK_TEMPLATE, $(ELFDIR)/$(TARGET).elf, $(ALLOBJECTS)))

# Assemble: Create object files from assembler source files.
$(foreach src, $(ASRCS), $(eval $(call ASSEMBLE_TEMPLATE, $(src))))

# Compile: Create object files from C source files.
$(foreach src, $(CSRCS), $(eval $(call COMPILE_C_TEMPLATE, $(src))))

# Compile: Create object files from C++ source files.
$(foreach src, $(CPPSRCS), $(eval $(call COMPILE_CPP_TEMPLATE, $(src))))

elf: $(ELFDIR)/$(TARGET).elf
lss: $(ELFDIR)/$(TARGET).lss
sym: $(ELFDIR)/$(TARGET).sym
hex: $(ELFDIR)/$(TARGET).hex
bin: $(ELFDIR)/$(TARGET).bin

size: $(ELFDIR)/$(TARGET).elf
	$(V0) echo $(MSG_SIZE) $(TARGET).elf
	$(V0) $(SIZE) -A $(ELFDIR)/$(TARGET).elf

dump: $(ELFDIR)/$(TARGET).elf
	$(V0) echo $(MSG_DUMP) $(TARGET).elf
	$(V0) $(OBJDUMP) -D $(ELFDIR)/$(TARGET).elf > $(ELFDIR)/$(TARGET)_dump.txt


clean: clean_list

clean_list:
	$(V0) echo $(MSG_CLEANING)
	$(V0) $(REMOVE) $(ALLOBJECTS)
	$(V0) $(REMOVE) $(ELFDIR)/$(TARGET).map
	$(V0) $(REMOVE) $(ELFDIR)/$(TARGET).elf
	$(V0) $(REMOVE) $(ELFDIR)/$(TARGET).hex
	$(V0) $(REMOVE) $(ELFDIR)/$(TARGET).bin
	$(V0) $(REMOVE) $(ELFDIR)/$(TARGET).sym
	$(V0) $(REMOVE) $(ELFDIR)/$(TARGET).lss


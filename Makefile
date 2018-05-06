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
DEPSDIR = ./build/deps
ELFDIR = ./build

# Optimization
OPTIMIZATION = -O1 -g

# Includes
INCLUDE  = -I./deps/esl/src/
INCLUDE += -I./deps/crect/deps/mpl/src
INCLUDE += -I./deps/crect/src
INCLUDE += -I./deps/CMSIS_5/CMSIS/Core/Include
INCLUDE += -I./system



# Sources
CSRCS   =
CPPSRCS = ./system/init.cpp
ASRCS   =

# Generate object files
ALLSRC  = $(ASRCS) $(CSRCS) $(CPPSRCS)
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


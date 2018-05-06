#-------------------------------------------
# Makefile for KFly Dronecode
# Author: (C) Emil Fresk 2018
#-------------------------------------------

# Target same
TARGET ?= kfly_dronecode

# Verbose
V0 =

# Where the build will be located and create the folder
BUILDDIR = ./build
ELFDIR = ./build
OBJDIR = $(BUILDDIR)/obj
DEPDIR = $(BUILDDIR)/deps

# Optimization
OPTIMIZATION = -O1 -g

# Includes
INCLUDE  = -I./deps/esl/src/
INCLUDE += -I./deps/crect/deps/mpl/src
INCLUDE += -I./deps/crect/src
INCLUDE += -I./system

# Sources
CSRCS   =
CPPSRCS = ./system/system_init.cpp
ASRCS   =

# Generate object files
ALLSRC  = $(ASRCS) $(CSRCS) $(CPPSRCS)
ALLSRCBASE = $(notdir $(basename $(ALLSRC)))
ALLOBJECTS = $(addprefix $(OBJDIR)/, $(addsuffix .o, $(ALLSRCBASE)))


# Include definitions
include make/defs.mk

all: build

build: dirs elf bin hex lss sym size

# Link: Create elf output file from object files.
$(eval $(call LINK_TEMPLATE, $(ELFDIR)/$(TARGET).elf, $(ALLOBJECTS)))

# Assemble: Create object files from assembler source files.
$(foreach src, $(ASRCS), $(eval $(call ASSEMBLE_TEMPLATE, $(src))))

# Compile: Create object files from C source files.
$(foreach src, $(CSRCS), $(eval $(call COMPILE_C_TEMPLATE, $(src))))

# Compile: Create object files from C++ source files.
$(foreach src, $(CPPSRCS), $(eval $(call COMPILE_CPP_TEMPLATE, $(src))))

dirs:
	$(V0) mkdir -p $(OBJDIR)
	$(V0) mkdir -p $(DEPDIR)
	$(V0) mkdir -p $(ELFDIR)

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


clean:
	$(V0) echo $(MSG_CLEANING) $(BUILDDIR)
	$(V0) rm -rf $(BUILDDIR)

#
# Include the dependency files
#
-include $(wildcard $(DEPDIR)/*)

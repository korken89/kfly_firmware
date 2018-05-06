EMPTY =

# Build tools
GCC     = arm-none-eabi-gcc
GPP     = arm-none-eabi-g++
SIZE    = arm-none-eabi-size
OBJDUMP = arm-none-eabi-objdump
OBJCOPY = arm-none-eabi-objcopy
GDB     = arm-none-eabi-gdb
NM      = arm-none-eabi-nm

# Flags
MCU     = -mcpu=cortex-m7 -mthumb -mfpu=fpv5-dp-d16 -mfloat-abi=hard
CFLAGS  = $(MCU) $(COMMON) -std=gnu99 -fno-builtin-exit $(OPTIMIZATION) $(INCLUDE) -ffast-math -fsingle-precision-constant
AFLAGS  = $(MCU) $(COMMON) $(INCLUDE)
LDFLAGS = $(MCU) $(COMMON) -T./system/stm32f765.ld -Wl,--build-id=none,-Map=$(ELFDIR)/$(TARGET).map
BINPLACE = -j.isr_vector -j.sw_version -j.text -j.ARM.extab -j.ARM
BINPLACE += -j.preinit_array -j.init_array -j.fini_array -j.data

MSG_BINARY_HEX       = ${EMPTY} BIN/HEX  ${EMPTY}
MSG_DUMP             = ${EMPTY} DUMP     ${EMPTY}
MSG_SIZE             = ${EMPTY} SIZE     ${EMPTY}
MSG_LINKING          = ${EMPTY} LD       ${EMPTY}
MSG_COMPILING        = ${EMPTY} CC       ${EMPTY}
MSG_COMPILING_CPP    = ${EMPTY} CPP      ${EMPTY}
MSG_ASSEMBLING       = ${EMPTY} AS       ${EMPTY}
MSG_CLEANING         = ${EMPTY} CLEAN    ${EMPTY}
MSG_EXTENDED_LISTING = ${EMPTY} LIS      ${EMPTY}
MSG_SYMBOL_TABLE     = ${EMPTY} NM       ${EMPTY}

%.hex: %.elf
	@echo $(MSG_BINARY_HEX) $@
	$(V0) $(OBJCOPY) -O ihex $< $@

%.bin: %.elf
	@echo $(MSG_BINARY_HEX) $@
	$(V0) $(OBJCOPY) $(BINPLACE) -O binary $< $@

%.lss: %.elf
	@echo $(MSG_EXTENDED_LISTING) $@
	$(V0) $(OBJDUMP) -h -S -C -r $< > $@

%.sym: %.elf
	@echo $(MSG_SYMBOL_TABLE) $@
	$(V0) $(NM) -n $< > $@

# Compile: Create object files from ASM source files.
define ASSEMBLE_TEMPLATE
$(OBJDIR)/$(notdir $(basename $(1))).o : $(1)
	@echo $(MSG_ASSEMBLING) $$<
	$(V0) $(GCC) -c $$(AFLAGS) $$< -o $$@
endef

# Compile: Create object files from C source files.
define COMPILE_C_TEMPLATE
$(OBJDIR)/$(notdir $(basename $(1))).o : $(1)
	@echo $(MSG_COMPILING) $$<
	$(V0) $(GCC) $$(CFLAGS) -c $$< -o $$@
endef

# Compile: Create object files from C++ source files.
define COMPILE_CPP_TEMPLATE
$(OBJDIR)/$(notdir $(basename $(1))).o : $(1)
	@echo $(MSG_COMPILING_CPP) $$<
	$(V0) $(GPP) $$(CFLAGS) -c $$< -o $$@
endef

# Link: create ELF output file from object files.
#   $1 = elf file to produce
#   $2 = list of object files that make up the elf file
define LINK_TEMPLATE
.SECONDARY : $(1)
.PRECIOUS : $(2)
$(1):  $(2)
	@echo $(MSG_LINKING) $$@
	$(V0) $(GCC) $$(CFLAGS) $(2) --output $$@ $$(LDFLAGS) -lm -lc
endef

result = ${shell echo "test"}
ifeq (${result}, test)
	quote = "
else
	quote =
endif

# Build tools
GCC     = arm-none-eabi-gcc
GXX     = arm-none-eabi-g++
SIZE    = arm-none-eabi-size
OBJDUMP = arm-none-eabi-objdump
OBJCOPY = arm-none-eabi-objcopy
GDB     = arm-none-eabi-gdb
NM      = arm-none-eabi-nm

# Flags
COMMON  = -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard
COMMON += -ffast-math -fsingle-precision-constant
COMMON += -fno-rtti -fno-non-call-exceptions -fno-exceptions
COMMON += -ffreestanding -fomit-frame-pointer -falign-functions=16 -fno-common
COMMON += -mno-thumb-interwork -fno-threadsafe-statics -nostartfiles
COMMON += -fsigned-char -fno-move-loop-invariants -ffunction-sections
COMMON += -fdata-sections -Xlinker --gc-sections

CFLAGS  = $(COMMON) $(OPTIMIZATION) $(INCLUDE) $(WARNINGS)
CFLAGS += -std=c99

CPPFLAGS  = $(COMMON) $(OPTIMIZATION) $(INCLUDE) $(WARNINGS)
CPPFLAGS += -std=c++17

AFLAGS    = $(COMMON) $(INCLUDE) $(WARNINGS)

LDFLAGS   = $(COMMON) -Tsystem/stm32f765.ld -Wl,--build-id=none,-Map=$(ELFDIR)/$(TARGET).map
LDFLAGS  += --specs=nano.specs -lm -lc

# Binary generation sections
BINPLACE = -j.isr_vector -j.sw_version -j.text -j.ARM.extab -j.ARM
BINPLACE += -j.preinit_array -j.init_array -j.fini_array -j.data

# Generate deps
CFLAGS   += -MD -MP -MF $(DEPDIR)/$(@F).d
CPPFLAGS += -MD -MP -MF $(DEPDIR)/$(@F).d
AFLAGS 	 += -MD -MP -MF $(DEPDIR)/$(@F).d

################

MSG_BINARY_HEX       = $(quote) BIN/HEX    $(quote)
MSG_DUMP             = $(quote) DUMP       $(quote)
MSG_SIZE             = $(quote) SIZE       $(quote)
MSG_LINKING          = $(quote) LD         $(quote)
MSG_COMPILING        = $(quote) CC         $(quote)
MSG_ASSEMBLING       = $(quote) AS         $(quote)
MSG_CLEANING         = $(quote) CLEAN      $(quote)
MSG_EXTENDED_LISTING = $(quote) LIS        $(quote)
MSG_SYMBOL_TABLE     = $(quote) NM         $(quote)

%.hex: %.elf Makefile make/defs.mk
	@echo $(MSG_BINARY_HEX) $@
	$(V0) $(OBJCOPY) -O ihex $< $@

%.bin: %.elf Makefile make/defs.mk
	@echo $(MSG_BINARY_HEX) $@
	$(V0) $(OBJCOPY) $(BINPLACE) -O binary $< $@

%.lss: %.elf Makefile make/defs.mk
	@echo $(MSG_EXTENDED_LISTING) $@
	$(V0) $(OBJDUMP) -h -S -C -r $< > $@

%.sym: %.elf Makefile make/defs.mk
	@echo $(MSG_SYMBOL_TABLE) $@
	$(V0) $(NM) -n $< > $@

# Compile: Create object files from ASM source files.
define ASSEMBLE_TEMPLATE
$(OBJDIR)/$(notdir $(basename $(1))).o : $(1) Makefile make/defs.mk
	@echo $(MSG_ASSEMBLING) $$<
	$(V0) $(GXX) -c $$(AFLAGS) $$< -o $$@
endef

# Compile: Create object files from C source files.
define COMPILE_C_TEMPLATE
$(OBJDIR)/$(notdir $(basename $(1))).o : $(1) Makefile make/defs.mk
	@echo $(MSG_COMPILING) $$<
	$(V0) $(GXX) $$(CFLAGS) -c $$< -o $$@
endef

# Compile: Create object files from C++ source files.
define COMPILE_CPP_TEMPLATE
$(OBJDIR)/$(notdir $(basename $(1))).o : $(1) Makefile make/defs.mk
	@echo $(MSG_COMPILING) $$<
	$(V0) $(GXX) $$(CPPFLAGS) -c $$< -o $$@
endef

# Link: create ELF output file from object files.
#   $1 = elf file to produce
#   $2 = list of object files that make up the elf file
define LINK_TEMPLATE
.SECONDARY : $(1)
.PRECIOUS : $(2)
$(1):  $(2) Makefile make/defs.mk
	@echo $(MSG_LINKING) $$@
	$(V0) $(GXX) $$(CFLAGS) $(2) --output $$@ $$(LDFLAGS)
endef

all: build

build: dirs elf bin hex lss sym size

# Generate object files
ALLSRC  = $(ASRCS) $(CSRCS) $(CPPSRCS)
ALLSRCBASE = $(notdir $(basename $(ALLSRC)))
ALLOBJECTS = $(addprefix $(OBJDIR)/, $(addsuffix .o, $(ALLSRCBASE)))

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
	$(V0) mkdir -p $(BUILDDIR)

elf: dirs $(ELFDIR)/$(TARGET).elf
lss: dirs $(ELFDIR)/$(TARGET).lss
sym: dirs $(ELFDIR)/$(TARGET).sym
hex: dirs $(ELFDIR)/$(TARGET).hex
bin: dirs $(ELFDIR)/$(TARGET).bin

size: $(ELFDIR)/$(TARGET).elf Makefile make/defs.mk
	@sleep 0.1
	@echo $(MSG_SIZE) $(TARGET).elf
	@echo
	$(V0) $(SIZE) -A -x $(ELFDIR)/$(TARGET).elf | sed '/.debug_*/Q'

dump: $(ELFDIR)/$(TARGET).elf Makefile make/defs.mk
	@echo $(MSG_DUMP) $(TARGET).elf
	$(V0) $(OBJDUMP) -D $(ELFDIR)/$(TARGET).elf | sed '/Disassembly of section .debug_info:/Q' > $(ELFDIR)/$(TARGET)_dump.txt

clean:
	@echo $(MSG_CLEANING) $(BUILDDIR)
	$(V0) rm -rf $(BUILDDIR)

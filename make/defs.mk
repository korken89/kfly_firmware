# Build tools
GCC     = arm-none-eabi-gcc
GXX     = arm-none-eabi-g++
SIZE    = arm-none-eabi-size
OBJDUMP = arm-none-eabi-objdump
OBJCOPY = arm-none-eabi-objcopy
GDB     = arm-none-eabi-gdb
NM      = arm-none-eabi-nm

# Flags
MCU       = -ffast-math -fsingle-precision-constant -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard

CFLAGS  = $(MCU) $(OPTIMIZATION) $(INCLUDE)
CFLAGS += -std=c99
CPPFLAGS  = $(MCU) $(OPTIMIZATION) $(INCLUDE)
CPPFLAGS += -std=c++17
AFLAGS    = $(MCU) $(INCLUDE)
LDFLAGS   = $(MCU) -T./system/stm32f765.ld -Wl,--build-id=none,-Map=$(ELFDIR)/$(TARGET).map

# Binary generation sections
BINPLACE = -j.isr_vector -j.sw_version -j.text -j.ARM.extab -j.ARM
BINPLACE += -j.preinit_array -j.init_array -j.fini_array -j.data

# Generate deps
CFLAGS   += -MD -MP -MF $(DEPDIR)/$(@F).d
CPPFLAGS += -MD -MP -MF $(DEPDIR)/$(@F).d
AFLAGS 	 += -MD -MP -MF $(DEPDIR)/$(@F).d

################

MSG_BINARY_HEX       = BIN/HEX
MSG_DUMP             = DUMP
MSG_SIZE             = SIZE
MSG_LINKING          = LD
MSG_COMPILING        = CC
MSG_ASSEMBLING       = AS
MSG_CLEANING         = CLEAN
MSG_EXTENDED_LISTING = LIS
MSG_SYMBOL_TABLE     = NM

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
	$(V0) $(GXX) -c $$(AFLAGS) $$< -o $$@
endef

# Compile: Create object files from C source files.
define COMPILE_C_TEMPLATE
$(OBJDIR)/$(notdir $(basename $(1))).o : $(1)
	@echo $(MSG_COMPILING) $$<
	$(V0) $(GXX) $$(CFLAGS) -c $$< -o $$@
endef

# Compile: Create object files from C++ source files.
define COMPILE_CPP_TEMPLATE
$(OBJDIR)/$(notdir $(basename $(1))).o : $(1)
	@echo $(MSG_COMPILING) $$<
	$(V0) $(GXX) $$(CPPFLAGS) -c $$< -o $$@
endef

# Link: create ELF output file from object files.
#   $1 = elf file to produce
#   $2 = list of object files that make up the elf file
define LINK_TEMPLATE
.SECONDARY : $(1)
.PRECIOUS : $(2)
$(1):  $(2)
	@echo $(MSG_LINKING) $$@
	$(V0) $(GXX) $$(CFLAGS) $(2) --output $$@ $$(LDFLAGS) -lm -lc
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

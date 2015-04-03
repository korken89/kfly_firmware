# Modules directory location from Makefile
MODULE_DIR = ./modules

# Imported source files and paths from modules
include $(MODULE_DIR)/communication/communication.mk
include $(MODULE_DIR)/control/control.mk
include $(MODULE_DIR)/crc/crc.mk
include $(MODULE_DIR)/estimation/estimation.mk
include $(MODULE_DIR)/external_flash/external_flash.mk
include $(MODULE_DIR)/math/math.mk
include $(MODULE_DIR)/rc_input/rc_input.mk
include $(MODULE_DIR)/rc_output/rc_output.mk
include $(MODULE_DIR)/sensors/sensors.mk
include $(MODULE_DIR)/usb/usb.mk
include $(MODULE_DIR)/version_information/version_information.mk
include $(MODULE_DIR)/vicon/vicon.mk

# List of all the module related files.
MODULES_SRC = $(COMMUNICATION_SRCS) \
              $(CONTROL_SRCS) \
              $(CRC_SRCS) \
              $(ESTIMATION_SRCS) \
              $(EXTERNALFLASH_SRCS) \
              $(MATH_SRCS) \
              $(RCINPUT_SRCS) \
              $(RCOUTPUT_SRCS) \
              $(SENSORS_SRCS) \
              $(USB_SRCS) \
              $(VERSIONINFO_SRCS) \
              $(VICON_SRCS)

# Required include directories
MODULES_INC = $(COMMUNICATION_INC) \
              $(CONTROL_INC) \
              $(CRC_INC) \
              $(ESTIMATION_INC) \
              $(EXTERNALFLASH_INC) \
              $(MATH_INC) \
              $(RCINPUT_INC) \
              $(RCOUTPUT_INC) \
              $(SENSORS_INC) \
              $(USB_INC) \
              $(VERSIONINFO_INC) \
              $(VICON_INC)

# List of all the module's related files.
CONTROL_SRCS = $(MODULE_DIR)/control/src/control.c \
               $(MODULE_DIR)/control/src/computer_control.c \
               $(MODULE_DIR)/control/src/rate_loop.c \
               $(MODULE_DIR)/control/src/attitude_loop.c \
               $(MODULE_DIR)/control/src/position_loop.c \
               $(MODULE_DIR)/control/src/arming.c \
               $(MODULE_DIR)/control/src/pid.c

# Required include directories
CONTROL_INC = $(MODULE_DIR)/control/inc

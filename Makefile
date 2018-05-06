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
OPTIMIZATION = -O1 -ggdb

# Warnings
WARNINGS = -Wall -Wextra -Wfatal-errors

# Includes
INCLUDE  = -I./deps/esl/src/
INCLUDE += -I./deps/crect/deps/mpl/src
INCLUDE += -I./deps/crect/src
INCLUDE += -I./system

# Sources
CSRCS   =
CPPSRCS = system/system_init.cpp
ASRCS   =

#
# Include make definitions
#
include make/defs.mk

#
# Include the dependency files
#
-include $(wildcard $(DEPDIR)/*)

####
#### NOTHING AFTER THE DEP INCLUDE!!!
#### EOF

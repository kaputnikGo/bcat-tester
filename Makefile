# The TARGET variable determines what target system the application is
# compiled for. It either refers to an XN file in the source directories
# or a valid argument for the --target option when compiling
#TARGET = XS1-U16A-128-FB217-C10
TARGET = BCAT-08-BRD

# The APP_NAME variable determines the name of the final .xe file. It should
# not include the .xe postfix. If left blank the name will default to
# the project name
APP_NAME = BCAT-08-Tester

# This header file is marked as optional since it is only included for
# a known XMOS development board - hacky into Bcat board
OPTIONAL_HEADERS = avb_app_board_config.h

# The USED_MODULES variable lists other module used by the application.
USED_MODULES = module_avb module_avb_1722 module_avb_1722_1 module_avb_1722_maap module_avb_audio module_avb_flash module_avb_media_clock module_avb_srp module_avb_util module_ethernet module_gptp module_locks module_logging module_otp_board_info module_random

# The flags passed to xcc when building the application
# You can also set the following to override flags for a particular language:
# XCC_XC_FLAGS, XCC_C_FLAGS, XCC_ASM_FLAGS, XCC_CPP_FLAGS
# If the variable XCC_MAP_FLAGS is set it overrides the flags passed to
# xcc for the final link (mapping) stage.
XCC_FLAGS = -O2 -g -report

# The XCORE_ARM_PROJECT variable, if set to 1, configures this
# project to create both xCORE and ARM binaries.
XCORE_ARM_PROJECT = 0

# The VERBOSE variable, if set to 1, enables verbose output from the make system.
VERBOSE = 0

XMOS_MAKE_PATH ?= ../..
-include $(XMOS_MAKE_PATH)/xcommon/module_xcommon/build/Makefile.common

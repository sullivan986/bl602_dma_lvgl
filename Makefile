
BL_SDK_BASE ?= /home/sullivan/sdk/bouffalo_sdk

export BL_SDK_BASE

CHIP ?= bl602
BOARD ?= bl602dk
CROSS_COMPILE ?= /home/sullivan/bin/compiler/toolchain_gcc_t-head_linux/bin/riscv64-unknown-elf-

include $(BL_SDK_BASE)/project.build
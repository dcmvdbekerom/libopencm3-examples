##
## This file is part of the libopencm3 project.
##
## Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
##
## This library is free software: you can redistribute it and/or modify
## it under the terms of the GNU Lesser General Public License as published by
## the Free Software Foundation, either version 3 of the License, or
## (at your option) any later version.
##
## This library is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU Lesser General Public License for more details.
##
## You should have received a copy of the GNU Lesser General Public License
## along with this library.  If not, see <http://www.gnu.org/licenses/>.
##

BINARY = cdcacm_tinyusb

OBJS = usb_descriptors.o \
    bsp/board.o \
    bsp/family.o \
    tinyusb/src/tusb.o \
    tinyusb/src/class/cdc/cdc_device.o \
    tinyusb/src/common/tusb_fifo.o \
    tinyusb/src/device/usbd.o \
    tinyusb/src/device/usbd_control.o \
    tinyusb/src/portable/st/stm32_fsdev/dcd_stm32_fsdev.o


LDSCRIPT = ../stm32g0B1.ld

include ../../Makefile.include

CFLAGS += -DCFG_TUSB_MCU=OPT_MCU_STM32G0 -DSTM32G0B1xx
CFLAGS += -I. \
          -Ibsp \
          -Itinyusb/src \
          -Itinyusb/src/class/cdc \
          -Itinyusb/src/class/cdc/serial \
          -Itinyusb/src/common \
          -Itinyusb/src/device \
          -Itinyusb/src/st/stm32_fsdev



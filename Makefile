#
# Copyright (C) 2016  Matt Borgerson
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License along
# with this program; if not, write to the Free Software Foundation, Inc.,
# 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
#

CC           := clang
CFLAGS       := -I../include -D_LIBCPP_HAS_NO_BUILTIN_OPERATOR_NEW_DELETE -fno-use-cxa-atexit \
-mno-sse2 -nostdlib -mno-red-zone -mno-mmx -mno-sse -mno-sse2 -fpie \
-ffreestanding -Werror -ansi -Os -g -ffunction-sections -fno-stack-protector
LD           := ld
LDFLAGS      := --script=gdbstub.ld -m elf_x86_64 --gc-sections
NASM         := nasm
NASM_FLAGS   := -felf64
OBJCOPY      := objcopy
OBJCOPYFLAGS := --output-target=binary
TARGET       := gdbstub.bin
BASE_ADDRESS := 0x500000
OBJECTS      := gdbstub_rsp.o \
                gdbstub_int.o \
                gdbstub_sys.o

all: $(OBJECTS)
.PRECIOUS: %.elf

%.bin: %.elf
	$(OBJCOPY) $(OBJCOPYFLAGS) $^ $@

%.elf: $(OBJECTS) gdbstub.ld
	$(LD) $(LDFLAGS) -o $@ $(OBJECTS)

gdbstub.ld: gdbstub.ld.in
	$(shell sed -e "s/\$$BASE_ADDRESS/$(BASE_ADDRESS)/" gdbstub.ld.in > gdbstub.ld)

%.o: %.c
	$(CC) $(CFLAGS) -o $@ -c $<

%.o: %.nasm
	$(NASM) -o $@ $(NASM_FLAGS) $<

.PHONY: clean
clean:
	rm -f \
		$(TARGET) \
		$(TARGET:.bin=.elf) \
		$(OBJECTS) \
		gdbstub.ld

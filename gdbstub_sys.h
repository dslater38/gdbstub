/*
 * Copyright (C) 2016  Matt Borgerson
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

/* Define the size_t type */
#define DBG_DEFINE_SIZET 0

/* Define required standard integer types (e.g. uint16_t) */
#define DBG_DEFINE_STDINT 0

#include "../include/isr.h"
#include <stddef.h>

/*****************************************************************************
 * Types
 ****************************************************************************/

#if DBG_DEFINE_STDINT
typedef unsigned char  uint8_t;
typedef unsigned short uint16_t;
typedef unsigned long  uint32_t;
typedef unsigned long long uint64_t;
#endif

#if DBG_DEFINE_SIZET
typedef uint64_t size_t;
#endif

typedef uintptr_t address;
typedef uint64_t reg;

typedef registers64_t dbg_interrupt_state;

#if 0
#pragma pack(1)
struct dbg_interrupt_state {
	uint64_t ss;
	uint64_t gs;
	uint64_t fs;
	uint64_t es;
	uint64_t ds;
	uint64_t rdi;
	uint64_t rsi;
	uint64_t rbp;
	uint64_t rsp;
	uint64_t rbx;
	uint64_t rdx;
	uint64_t rcx;
	uint64_t rax;
	uint64_t r8;
	uint64_t r9;
	uint64_t r10;
	uint64_t r11;
	uint64_t r12;
	uint64_t r13;
	uint64_t r14;
	uint64_t r15;
	uint64_t vector;
	uint64_t error_code;
	uint64_t rip;
	uint64_t cs;
	uint64_t rflags;
	uint64_t userrsp;
};
#pragma pack()

#endif // 0

#pragma pack(1)
struct dbg_idtr
{
	uint16_t len;
	uint64_t offset;
};
#pragma pack()

#pragma pack(1)
struct dbg_idt_gate
{
	uint16_t offset_low;
	uint16_t segment;
	uint16_t flags;
	uint16_t offset_mid;
	uint32_t offset_high;
	uint32_t reserved;
};
#pragma pack()

enum DBG_REGISTER {
	DBG_CPU_I386_REG_RAX       = 0,
	DBG_CPU_I386_REG_RBX       = 1,
	DBG_CPU_I386_REG_RCX       = 2,
	DBG_CPU_I386_REG_RDX       = 3,
	DBG_CPU_I386_REG_RSI       = 4,
	DBG_CPU_I386_REG_RDI       = 5,
	DBG_CPU_I386_REG_RBP       = 6,
	DBG_CPU_I386_REG_RSP       = 7,	
	DBG_CPU_I386_REG_R8        = 8,
	DBG_CPU_I386_REG_R9        = 9,
	DBG_CPU_I386_REG_R10        = 10,
	DBG_CPU_I386_REG_R11        = 11,
	DBG_CPU_I386_REG_R12        = 12,
	DBG_CPU_I386_REG_R13        = 13,
	DBG_CPU_I386_REG_R14		= 14,
	DBG_CPU_I386_REG_R15		= 15,
	DBG_CPU_I386_REG_RIP       	= 16,
	DBG_CPU_I386_REG_RFLAGS	= 17,
	DBG_CPU_I386_REG_CS       = 18,
	DBG_CPU_I386_REG_SS       = 19,
	DBG_CPU_I386_REG_DS       = 20,
	DBG_CPU_I386_REG_ES       = 21,
	DBG_CPU_I386_REG_FS       = 22,
	DBG_CPU_I386_REG_GS       = 23,
	DBG_CPU_I386_NUM_REGISTERS = 24
};

struct dbg_state {
	int signum;
	reg registers[DBG_CPU_I386_NUM_REGISTERS];
};

/*****************************************************************************
 * Const Data
 ****************************************************************************/

extern void const * const dbg_int_handlers[];

/*****************************************************************************
 * Prototypes
 ****************************************************************************/

int dbg_hook_idt(uint8_t vector, const void *function);
int dbg_init_gates(void);
int dbg_init_idt(void);
int dbg_load_idt(struct dbg_idtr *idtr);
int dbg_store_idt(struct dbg_idtr *idtr);
uint32_t dbg_get_cs(void);
void dbg_int_handler(dbg_interrupt_state *istate);
void dbg_interrupt(dbg_interrupt_state *istate);
void dbg_start(void);
void dbg_io_write_8(uint16_t port, uint8_t val);
uint8_t dbg_io_read_8(uint16_t port);
void *dbg_sys_memset(void *ptr, int data, size_t len);
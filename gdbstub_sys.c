#ifdef NOT_YET
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

#include "gdbstub.h"

#ifdef __STRICT_ANSI__
#define asm __asm__
#endif

#define SERIAL_COM1 0x3f8
#define SERIAL_COM2 0x2f8
#define SERIAL_PORT SERIAL_COM1

#define NUM_IDT_ENTRIES 32

/*****************************************************************************
 * BSS Data
 ****************************************************************************/

static struct dbg_idt_gate dbg_idt_gates[NUM_IDT_ENTRIES];
static struct dbg_state    dbg_state;

/*****************************************************************************
 * Misc. Functions
 ****************************************************************************/

void *dbg_sys_memset(void *ptr, int data, size_t len)
{
	char *p = ptr;

	while (len--) {
		*p++ = (char)data;
	}

	return ptr;
}

/*
 * Get current code segment (CS register).
 */
uint32_t dbg_get_cs(void)
{
	uint32_t cs;

	asm volatile (
		"mov %%cs, %%eax;"
		/* Outputs  */ : "=a" (cs)
		/* Inputs   */ : /* None */
		/* Clobbers */ : /* None */
		);

	return cs;
}

/*****************************************************************************
 * Interrupt Management Functions
 ****************************************************************************/

/*
 * Initialize idt_gates with the interrupt handlers.
 */
int dbg_init_gates(void)
{
	size_t   i;
	uint16_t cs;

	cs = dbg_get_cs();
	for (i = 0; i < NUM_IDT_ENTRIES; i++) {
		dbg_idt_gates[i].flags       = 0x8E00;
		dbg_idt_gates[i].segment     = cs;
		dbg_idt_gates[i].offset_low  =
			((uint32_t)dbg_int_handlers[i]      ) & 0xffff;
		dbg_idt_gates[i].offset_high =
			((uint32_t)dbg_int_handlers[i] >> 16) & 0xffff;
	}

	return 0;
}

/*
 * Load a new IDT.
 */
int dbg_load_idt(struct dbg_idtr *idtr)
{
	asm volatile (
		"lidt    %0"
		/* Outputs  */ : /* None */
		/* Inputs   */ : "m" (*idtr)
		/* Clobbers */ : /* None */
		);

	return 0;
}

/*
 * Get current IDT.
 */
int dbg_store_idt(struct dbg_idtr *idtr)
{
	asm volatile (
		"sidt    %0"
		/* Outputs  */ : "=m" (*idtr)
		/* Inputs   */ : /* None */
		/* Clobbers */ : /* None */
		);

	return 0;
}

/*
 * Hook a vector of the current IDT.
 */
int dbg_hook_idt(uint8_t vector, const void *function)
{
	struct dbg_idtr      idtr;
	struct dbg_idt_gate *gates;

	dbg_store_idt(&idtr);
	gates = (struct dbg_idt_gate *)idtr.offset;
	gates[vector].flags       = 0x8E00;
	gates[vector].segment     = dbg_get_cs();
	gates[vector].offset_low  = (((uint32_t)function)      ) & 0xffff;
	gates[vector].offset_mid = (((uint32_t)function) >> 16) & 0xffff;
	gates[vector].offset_high = (uint32_t)((((uint64_t)function) & 0xFFFFFFFF00000000) >> 32);
	gates[vector].reserved = 0;

	return 0;
}

/*
 * Initialize IDT gates and load the new IDT.
 */
int dbg_init_idt(void)
{
	struct dbg_idtr idtr;

	dbg_init_gates();
	idtr.len = sizeof(dbg_idt_gates)-1;
	idtr.offset = (uint32_t)dbg_idt_gates;
	dbg_load_idt(&idtr);

	return 0;
}

/*
 * Common interrupt handler routine.
 */
void dbg_int_handler(dbg_interrupt_state *istate)
{
	dbg_interrupt(istate);
}

/*
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
*/

/*
 * Debug interrupt handler.
 */
void dbg_interrupt(dbg_interrupt_state *istate)
{
	dbg_sys_memset(&dbg_state.registers, 0, sizeof(dbg_state.registers));

	dbg_state.signum = istate->vector;

	/* Load Registers */
	dbg_state.registers[DBG_CPU_I386_REG_RAX] = istate->rax;
	dbg_state.registers[DBG_CPU_I386_REG_RBX] = istate->rbx;
	dbg_state.registers[DBG_CPU_I386_REG_RCX] = istate->rcx;
	dbg_state.registers[DBG_CPU_I386_REG_RDX] = istate->rdx;
	dbg_state.registers[DBG_CPU_I386_REG_RSI] = istate->rsi;
	dbg_state.registers[DBG_CPU_I386_REG_RDI] = istate->rdi;
	dbg_state.registers[DBG_CPU_I386_REG_RBP] = istate->rbp;
	dbg_state.registers[DBG_CPU_I386_REG_RSP] = istate->rsp;
	dbg_state.registers[DBG_CPU_I386_REG_R8] = istate->r8;
	dbg_state.registers[DBG_CPU_I386_REG_R9] = istate->r9;
	dbg_state.registers[DBG_CPU_I386_REG_R10] = istate->r10;
	dbg_state.registers[DBG_CPU_I386_REG_R11] = istate->r11;
	dbg_state.registers[DBG_CPU_I386_REG_R12] = istate->r12;
	dbg_state.registers[DBG_CPU_I386_REG_R13] = istate->r13;
	dbg_state.registers[DBG_CPU_I386_REG_R14] = istate->r14;
	dbg_state.registers[DBG_CPU_I386_REG_R15] = istate->r15;	
	dbg_state.registers[DBG_CPU_I386_REG_RIP]  = istate->rip;
	dbg_state.registers[DBG_CPU_I386_REG_PS]  = istate->rflags;
	dbg_state.registers[DBG_CPU_I386_REG_CS]  = istate->cs;
	dbg_state.registers[DBG_CPU_I386_REG_SS]  = istate->ss;
	dbg_state.registers[DBG_CPU_I386_REG_DS]  = istate->ds;
	dbg_state.registers[DBG_CPU_I386_REG_ES]  = istate->es;
	dbg_state.registers[DBG_CPU_I386_REG_FS]  = istate->fs;
	dbg_state.registers[DBG_CPU_I386_REG_GS]  = istate->gs;	



	dbg_main(&dbg_state);

	/* Restore Registers */
	istate->rax    = dbg_state.registers[DBG_CPU_I386_REG_RAX];
	istate->rbx    = dbg_state.registers[DBG_CPU_I386_REG_RBX];
	istate->rcx    = dbg_state.registers[DBG_CPU_I386_REG_RCX];
	istate->rdx    = dbg_state.registers[DBG_CPU_I386_REG_RDX];
	istate->rsi    = dbg_state.registers[DBG_CPU_I386_REG_RSI];
	istate->rdi    = dbg_state.registers[DBG_CPU_I386_REG_RDI];
	istate->rbp    = dbg_state.registers[DBG_CPU_I386_REG_RBP];
	istate->rsp    = dbg_state.registers[DBG_CPU_I386_REG_RSP];
	istate->r8     = dbg_state.registers[DBG_CPU_I386_REG_R8];
	istate->r9     = dbg_state.registers[DBG_CPU_I386_REG_R9];
	istate->r10     = dbg_state.registers[DBG_CPU_I386_REG_R10];
	istate->r11     = dbg_state.registers[DBG_CPU_I386_REG_R11];
	istate->r12     = dbg_state.registers[DBG_CPU_I386_REG_R12];
	istate->r13      = dbg_state.registers[DBG_CPU_I386_REG_R13];
	istate->r14     = dbg_state.registers[DBG_CPU_I386_REG_R14];
	istate->r15     = dbg_state.registers[DBG_CPU_I386_REG_R15];
	istate->rip    = dbg_state.registers[DBG_CPU_I386_REG_RIP];
	istate->rflags = dbg_state.registers[DBG_CPU_I386_REG_PS];
	istate->cs     = dbg_state.registers[DBG_CPU_I386_REG_CS];
	istate->ss     = dbg_state.registers[DBG_CPU_I386_REG_SS];
	istate->ds     = dbg_state.registers[DBG_CPU_I386_REG_DS];
	istate->es     = dbg_state.registers[DBG_CPU_I386_REG_ES];
	istate->fs     = dbg_state.registers[DBG_CPU_I386_REG_FS];
	istate->gs     = dbg_state.registers[DBG_CPU_I386_REG_GS];	
}

/*****************************************************************************
 * I/O Functions
 ****************************************************************************/

/*
 * Write to I/O port.
 */
void dbg_io_write_8(uint16_t port, uint8_t val)
{
	asm volatile (
		"outb    %%al, %%dx;"
		/* Outputs  */ : /* None */
		/* Inputs   */ : "a" (val), "d" (port)
		/* Clobbers */ : /* None */
		);
}

/*
 * Read from I/O port.
 */
uint8_t dbg_io_read_8(uint16_t port)
{
	uint8_t val;

	asm volatile (
		"inb     %%dx, %%al;"
		/* Outputs  */ : "=a" (val)
		/* Inputs   */ : "d" (port)
		/* Clobbers */ : /* None */
		);

	return val;
}

/*****************************************************************************
 * NS16550 Serial Port (IO)
 ****************************************************************************/

#define SERIAL_THR 0
#define SERIAL_RBR 0
#define SERIAL_LSR 5

int dbg_serial_getc(void)
{
	/* Wait for data */
	while ((dbg_io_read_8(SERIAL_PORT + SERIAL_LSR) & 1) == 0);
	return dbg_io_read_8(SERIAL_PORT + SERIAL_RBR);
}

int dbg_serial_putchar(int ch)
{
	/* Wait for THRE (bit 5) to be high */
	while ((dbg_io_read_8(SERIAL_PORT + SERIAL_LSR) & (1<<5)) == 0);
	dbg_io_write_8(SERIAL_PORT + SERIAL_THR, ch);
	return ch;
}

/*****************************************************************************
 * Debugging System Functions
 ****************************************************************************/

/*
 * Write one character to the debugging stream.
 */
int dbg_sys_putchar(int ch)
{
	return dbg_serial_putchar(ch);
}

/*
 * Read one character from the debugging stream.
 */
int dbg_sys_getc(void)
{
	return dbg_serial_getc() & 0xff;
}

/*
 * Read one byte from memory.
 */
int dbg_sys_mem_readb(address addr, char *val)
{
	*val = *(volatile char *)addr;
	return 0;
}

/*
 * Write one byte to memory.
 */
int dbg_sys_mem_writeb(address addr, char val)
{
	*(volatile char *)addr = val;
	return 0;
}

/*
 * Continue program execution.
 */
int dbg_sys_continue(void)
{
	dbg_state.registers[DBG_CPU_I386_REG_PS] &= ~(1<<8);
	return 0;
}

/*
 * Single step the next instruction.
 */
int dbg_sys_step(void)
{
	dbg_state.registers[DBG_CPU_I386_REG_PS] |= 1<<8;
	return 0;
}

/*
 * Debugger init function.
 *
 * Hooks the IDT to enable debugging.
 */
void dbg_start(void)
{
	/* Hook current IDT. */
	dbg_hook_idt(1, dbg_int_handlers[1]);
	dbg_hook_idt(3, dbg_int_handlers[3]);

	/* Interrupt to start debugging. */
	asm volatile ("int3");
}

#endif // NOT_YET

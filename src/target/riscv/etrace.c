// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2023 by Nuclei wangyanwen                               *
 *   wangyanwen@nucleisys.com                                              *
 ***************************************************************************/
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "target/target.h"
#include "target/target_type.h"
#include "helper/log.h"
#include "helper/fileio.h"
#include "helper/time_support.h"
#include "riscv.h"
#include "rtos/rtos.h"
#include "debug_defines.h"

#define get_field(reg, mask) (((reg) & (mask)) / ((mask) & ~((mask) << 1)))
#define set_field(reg, mask, val) (((reg) & ~(mask)) | (((val) * ((mask) & ~((mask) << 1))) & (mask)))
#define field_value(mask, val) set_field((riscv_reg_t) 0, mask, val)

#define ETRACE_BASE_HI 		(0x00)
#define ETRACE_BASE_LO 		(0x04)
#define ETRACE_WLEN 		(0x08)
#define ETRACE_ENA 			(0x0c)
#define ETRACE_INTERRUPT 	(0x10)
#define ETRACE_MAXTIME 		(0x14)
#define ETRACE_EARLY 		(0x18)
#define ETRACE_ATOVF 		(0x1c)
#define ETRACE_ENDOFFSET 	(0x20)
#define ETRACE_FLG 			(0x24)
#define ETRACE_ONGOING 		(0x28)
#define ETRACE_TIMEOUT 		(0x2c)
#define ETRACE_IDLE 		(0x30)
#define ETRACE_FIFO 		(0x34)
#define ETRACE_ATBDW 		(0x38)
#define ETRACE_WRAP			(0x3c)
#define ETRACE_COMPACT 		(0x40)

static uint32_t etrace_addr = 0;
static target_addr_t buffer_addr = 0;
static uint32_t buffer_size = 0;
static uint8_t etrace_start = 0;

static int etrace_read_reg(struct target *target, uint32_t offset, uint32_t *value)
{
	int result = target_read_u32(target, etrace_addr + offset, value);
	if (result != ERROR_OK) {
		LOG_ERROR("etrace_read_reg() error at %#x", etrace_addr + offset);
		return result;
	}
	return ERROR_OK;
}

static int etrace_write_reg(struct target *target, uint32_t offset, uint32_t value)
{
	int result = target_write_u32(target, etrace_addr + offset, value);
	if (result != ERROR_OK) {
		LOG_ERROR("etrace_write_reg() error writing %#x to %#x", value, etrace_addr + offset);
		return result;
	}
	return ERROR_OK;
}

void etrace_stop(struct target *target)
{
	uint32_t tmp;
	uint32_t wait_idle = 0x100;

	etrace_write_reg(target, ETRACE_ENA, 0);
	do {
		etrace_read_reg(target, ETRACE_IDLE, &tmp);
		wait_idle -= 1;
		if (0 == wait_idle)
			break;
	} while(tmp != 1);
}

COMMAND_HANDLER(handle_etrace_config_command)
{
	uint32_t wrap;

	if (CMD_ARGC != 4) {
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	if (etrace_start) {
		LOG_ERROR("config must come before start");
		return 0;
	}

	struct target *target = get_current_target(CMD_CTX);

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], etrace_addr);
	COMMAND_PARSE_NUMBER(target_addr, CMD_ARGV[1], buffer_addr);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[2], buffer_size);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[3], wrap);

	etrace_write_reg(target, ETRACE_BASE_HI, (uint32_t)(buffer_addr >> 32));
	etrace_write_reg(target, ETRACE_BASE_LO, (uint32_t)buffer_addr);
	etrace_write_reg(target, ETRACE_WLEN, buffer_size);
	if (wrap) {
		etrace_write_reg(target, ETRACE_WRAP, 1);
		etrace_write_reg(target, ETRACE_COMPACT, 0);
	} else {
		etrace_write_reg(target, ETRACE_WRAP, 0);
		etrace_write_reg(target, ETRACE_COMPACT, 1);
	}

	return ERROR_OK;
}

COMMAND_HANDLER(handle_etrace_enable_command)
{
	if (CMD_ARGC > 0) {
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	struct target *target = get_current_target(CMD_CTX);
	struct target *target_real;
	if (target->smp) {
		target_real = get_target_by_num(target->current_targetid);
	} else {
		target_real = target;
	}

	riscv_reg_t dpc_rb;
	riscv_reg_t tdata1 = 	field_value(CSR_MCONTROL_TYPE(riscv_xlen(target_real)), CSR_TDATA1_TYPE_MCONTROL) |
							field_value(CSR_MCONTROL_ACTION, CSR_MCONTROL_ACTION_TRACE_ON) |
							field_value(CSR_MCONTROL_M, 1) |
							field_value(CSR_MCONTROL_S, 1) |
							field_value(CSR_MCONTROL_U, 1) |
							field_value(CSR_MCONTROL_EXECUTE, 1);
	if (riscv_set_register(target_real, GDB_REGNO_TSELECT, 0) != ERROR_OK)
		return ERROR_FAIL;
	if (riscv_set_register(target_real, GDB_REGNO_TDATA1, tdata1) != ERROR_OK)
		return ERROR_FAIL;
	if (riscv_get_register(target_real, &dpc_rb, GDB_REGNO_DPC) != ERROR_OK)
		return ERROR_FAIL;
	if (riscv_set_register(target_real, GDB_REGNO_TDATA2, dpc_rb) != ERROR_OK)
		return ERROR_FAIL;

	return ERROR_OK;
}

COMMAND_HANDLER(handle_etrace_disable_command)
{
	if (CMD_ARGC > 0) {
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	struct target *target = get_current_target(CMD_CTX);
	struct target *target_real;
	if (target->smp) {
		target_real = get_target_by_num(target->current_targetid);
	} else {
		target_real = target;
	}

	riscv_reg_t dpc_rb;
	riscv_reg_t tdata1 = 	field_value(CSR_MCONTROL_TYPE(riscv_xlen(target_real)), CSR_TDATA1_TYPE_MCONTROL) |
							field_value(CSR_MCONTROL_ACTION, CSR_MCONTROL_ACTION_TRACE_OFF) |
							field_value(CSR_MCONTROL_M, 1) |
							field_value(CSR_MCONTROL_S, 1) |
							field_value(CSR_MCONTROL_U, 1) |
							field_value(CSR_MCONTROL_EXECUTE, 1);
	if (riscv_set_register(target_real, GDB_REGNO_TSELECT, 1) != ERROR_OK)
		return ERROR_FAIL;
	if (riscv_set_register(target_real, GDB_REGNO_TDATA1, tdata1) != ERROR_OK)
		return ERROR_FAIL;
	if (riscv_get_register(target_real, &dpc_rb, GDB_REGNO_DPC) != ERROR_OK)
		return ERROR_FAIL;
	if (riscv_set_register(target_real, GDB_REGNO_TDATA2, dpc_rb) != ERROR_OK)
		return ERROR_FAIL;

	return ERROR_OK;
}

COMMAND_HANDLER(handle_etrace_start_command)
{
	if (CMD_ARGC > 0) {
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	struct target *target = get_current_target(CMD_CTX);

	etrace_write_reg(target, ETRACE_ENA, 1);
	etrace_start = 1;

	return ERROR_OK;
}

COMMAND_HANDLER(handle_etrace_stop_command)
{
	if (CMD_ARGC > 0) {
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	struct target *target = get_current_target(CMD_CTX);

	etrace_stop(target);
	etrace_start = 0;

	return ERROR_OK;
}

COMMAND_HANDLER(handle_etrace_dump_command)
{
	uint32_t end_offset;
	uint32_t full_flag;
	target_addr_t address;
	uint32_t size;
	uint8_t *temp;
	struct fileio *fileio;
	int retval, retvaltemp;
	struct duration bench;

	if (CMD_ARGC != 1) {
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	if (etrace_start) {
		LOG_ERROR("you must stop before dumping");
		return 0;
	}

	struct target *target = get_current_target(CMD_CTX);

	etrace_read_reg(target, ETRACE_ENDOFFSET, &end_offset);
	etrace_read_reg(target, ETRACE_FLG, &full_flag);
	if (full_flag) {
		address = buffer_addr + end_offset;
		size = buffer_size;
	} else {
		address = buffer_addr;
		size = end_offset;
	}

	uint32_t temp_size = (size > 4096) ? 4096 : size;
	temp = malloc(temp_size);
	if (!temp)
		return ERROR_FAIL;

	if (fileio_open(&fileio, CMD_ARGV[0], FILEIO_WRITE, FILEIO_BINARY) != ERROR_OK)
		return ERROR_FAIL;

	duration_start(&bench);

	while (size > 0) {
		size_t size_written;
		uint32_t this_run_size = (size > temp_size) ? temp_size : size;
		if ((this_run_size + address) > (buffer_addr + buffer_size)) {
			this_run_size = (buffer_addr + buffer_size) - address;
		}
		retval = target_read_buffer(target, address, this_run_size, temp);
		if (retval != ERROR_OK)
			break;

		retval = fileio_write(fileio, this_run_size, temp, &size_written);
		if (retval != ERROR_OK)
			break;

		size -= this_run_size;
		if ((this_run_size + address) == (buffer_addr + buffer_size)) {
			address = buffer_addr;
		} else {
			address += this_run_size;
		}
	}

	free(temp);

	if ((retval == ERROR_OK) && (duration_measure(&bench) == ERROR_OK)) {
		size_t filesize;
		retval = fileio_size(fileio, &filesize);
		if (retval != ERROR_OK)
			return retval;
		command_print(CMD,
				"dumped %zu bytes in %fs (%0.3f KiB/s)", filesize,
				duration_elapsed(&bench), duration_kbps(&bench, filesize));
	}

	retvaltemp = fileio_close(fileio);
	if (retvaltemp != ERROR_OK)
		return retvaltemp;

	return retval;
}

COMMAND_HANDLER(handle_etrace_clear_command)
{
	if (CMD_ARGC > 0) {
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	struct target *target = get_current_target(CMD_CTX);

	etrace_stop(target);

	etrace_write_reg(target, ETRACE_ENDOFFSET, 0);
	etrace_write_reg(target, ETRACE_FLG, 0);

	return ERROR_OK;
}

COMMAND_HANDLER(handle_etrace_info_command)
{
	uint32_t end_offset;
	uint32_t full_flag;

	if (CMD_ARGC > 0) {
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	struct target *target = get_current_target(CMD_CTX);

	etrace_read_reg(target, ETRACE_ENDOFFSET, &end_offset);
	etrace_read_reg(target, ETRACE_FLG, &full_flag);

	command_print(CMD, "Etrace Base: %#x", etrace_addr);
	command_print(CMD, "Buffer Addr: %#lx", buffer_addr);
	command_print(CMD, "Buffer Size: %#x", buffer_size);
	command_print_sameline(CMD, "Buffer Status: ");
	if (full_flag) {
		command_print(CMD, "used 100%% from %#lx [wraped]", buffer_addr + end_offset);
	} else {
		command_print(CMD, "used %d%% from %#lx to %#lx", (end_offset * 100) / buffer_size, buffer_addr, buffer_addr + end_offset);
	}

	return ERROR_OK;
}

static const struct command_registration etrace_command_handlers[] = {
	{
		.name = "config",
		.handler = handle_etrace_config_command,
		.mode = COMMAND_EXEC,
		.help = "Configuration etrace.",
		.usage = "etrace-addr buffer-addr buffer-size wrap",
	},
	{
		.name = "enable",
		.handler = handle_etrace_enable_command,
		.mode = COMMAND_EXEC,
		.help = "enable etrace",
		.usage = "",
	},
	{
		.name = "disable",
		.handler = handle_etrace_disable_command,
		.mode = COMMAND_EXEC,
		.help = "disable etrace",
		.usage = "",
	},
	{
		.name = "start",
		.handler = handle_etrace_start_command,
		.mode = COMMAND_EXEC,
		.help = "start etrace collection",
		.usage = "",
	},
	{
		.name = "stop",
		.handler = handle_etrace_stop_command,
		.mode = COMMAND_EXEC,
		.help = "stop etrace collection",
		.usage = "",
	},
	{
		.name = "dump",
		.handler = handle_etrace_dump_command,
		.mode = COMMAND_EXEC,
		.help = "dump etrace data",
		.usage = "filename",
	},
	{
		.name = "clear",
		.handler = handle_etrace_clear_command,
		.mode = COMMAND_EXEC,
		.help = "clear etrace data",
		.usage = "",
	},
	{
		.name = "info",
		.handler = handle_etrace_info_command,
		.mode = COMMAND_EXEC,
		.help = "display etrace info",
		.usage = "",
	},
	COMMAND_REGISTRATION_DONE
};

const struct command_registration etrace_command_group_handlers[] = {
	{
		.name = "etrace",
		.mode = COMMAND_ANY,
		.help = "Embedded Trace command group",
		.usage = "",
		.chain = etrace_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

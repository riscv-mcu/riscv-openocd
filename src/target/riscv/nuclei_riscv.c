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
#include "riscv_reg.h"
#include "rtos/rtos.h"
#include "debug_defines.h"
#include <helper/bits.h>
#include <math.h>

#define get_field(reg, mask) (((reg) & (mask)) / ((mask) & ~((mask) << 1)))
#define set_field(reg, mask, val) (((reg) & ~(mask)) | (((val) * ((mask) & ~((mask) << 1))) & (mask)))
#define field_value(mask, val) set_field((riscv_reg_t)0, mask, val)

volatile struct command_invocation *PCMD = NULL;

void cif_printf(const char *format, ...)
{
	char *string;

	va_list ap;
	va_start(ap, format);

	string = alloc_vprintf(format, ap);
	if (string && PCMD) {
		/* we want this collected in the log + we also want to pick it up as a tcl return
		 * value.
		 *
		 * The latter bit isn't precisely neat, but will do for now.
		 */
		Jim_AppendString(PCMD->ctx->interp, PCMD->output, string, -1);
		/* We already printed it above
		 * command_output_text(context, string); */
		free(string);
	}

	va_end(ap);
}

void set_print_interface(struct command_invocation *cmd)
{
	PCMD = cmd;
}

#include "cpuinfo.h"

int nuclei_get_cpuinfo(struct target *target, NUCLEI_CPUINFO *cpu)
{
    riscv_reg_t regval;
    CPU_INFO_Group *cpuinfo;
    uint32_t retry, csr_ready;

    if (cpu == NULL || target == NULL) {
        return -1;
    }
    memset(cpu, 0, sizeof(NUCLEI_CPUINFO)); // clear the struct
    cpuinfo = &cpu->cpuinfo;
	cpuinfo->iinfo = &cpu->iinfo;
    cpuinfo->eclic = &cpu->eclic;
	cpuinfo->xlen = riscv_xlen(target);

    // Sometimes read csr for following csrs are not stable, max retry five times
    // to get it
    csr_ready = 0;
    for (retry = 0; retry < 5; retry ++) {
        if (riscv_reg_get(target, &regval, GDB_REGNO_CSR0 + CSR_MARCHID) == ERROR_OK) {
            cpuinfo->marchid.d = (uint32_t)regval;
            csr_ready |= 0x1;
        }
        if (riscv_reg_get(target, &regval, GDB_REGNO_CSR0 + CSR_MHARTID) == ERROR_OK) {
            cpuinfo->mhartid = (uint32_t)regval;
            csr_ready |= 0x2;
        }
        if (riscv_reg_get(target, &regval, GDB_REGNO_CSR0 + CSR_MIMPID) == ERROR_OK) {
            cpuinfo->mimpid.d = (uint32_t)regval;
            csr_ready |= 0x4;
        }
        if (riscv_reg_get(target, &regval, GDB_REGNO_CSR0 + CSR_MISA) == ERROR_OK) {
            cpuinfo->misa.d = (uint32_t)regval;
            csr_ready |= 0x8;
        }
        if (csr_ready == 0xF) {
            break;
        }
    }
    // If theses CSRs cannot be ready, just mark cpuinfo read fail
    if (csr_ready != 0xF) {
        return -1;
    }
    if (riscv_reg_get(target, &regval, GDB_REGNO_CSR0 + CSR_MCFG_INFO) == ERROR_OK) {
        cpuinfo->mcfginfo.d = (uint32_t)regval;
		cpuinfo->mcfg_exist = 1;
    }
    // workaround for gd32vf103, it have mcfg_info csr but the bitfields are different from latest version
    // just mark mcfg_exist = 0
    if (cpuinfo->marchid.d == 0x80000022U && cpuinfo->mimpid.d == 0x100U) {
		cpuinfo->mcfg_exist = 0;
        cpuinfo->mcfginfo.d = 0;
    }
	/**
	 * mtlbcfginfo has a `mapping` field at the highest bit.
	 * For RV64, move the bit 63 to bit 31 to use the common
	 * struct as RV32.
	 */
	if (riscv_xlen(target) == 32) {
		if (cpuinfo->mcfginfo.b.plic)
            if (riscv_reg_get(target, &regval, GDB_REGNO_CSR0 + CSR_MTLBCFG_INFO) == ERROR_OK) {
                cpuinfo->mtlbcfginfo.d = (uint32_t)regval;
            }
	} else {
		if (cpuinfo->mcfginfo.b.plic) {
            if (riscv_reg_get(target, &regval, GDB_REGNO_CSR0 + CSR_MTLBCFG_INFO) == ERROR_OK) {
			    cpuinfo->mtlbcfginfo.d = (uint32_t)regval | (uint32_t)((regval >> 63) << 31);
            }
		}
	}

    if (cpuinfo->mcfginfo.b.icache || cpuinfo->mcfginfo.b.ilm) {
        if (riscv_reg_get(target, &regval, GDB_REGNO_CSR0 + CSR_MICFG_INFO) == ERROR_OK) {
            cpuinfo->micfginfo.d = (uint32_t)regval;
        }
    }
    if (cpuinfo->mcfginfo.b.dcache || cpuinfo->mcfginfo.b.dlm) {
        if (riscv_reg_get(target, &regval, GDB_REGNO_CSR0 + CSR_MDCFG_INFO) == ERROR_OK) {
            cpuinfo->mdcfginfo.d = (uint32_t)regval;
        }
    }
	if (cpuinfo->mcfginfo.b.iregion) {
        if (riscv_reg_get(target, &regval, GDB_REGNO_CSR0 + CSR_MIRGB_INFO) == ERROR_OK) {
            cpuinfo->mirgbinfo.d = (uint64_t)regval;
        }
		uint64_t iregion_base = (uint64_t)(cpuinfo->mirgbinfo.d) & (~0x3FFULL);
        memset(&cpu->iinfo, 0 ,sizeof(CIF_IINFO_Type));
		target_read_memory(target, iregion_base, 4, sizeof(CIF_IINFO_Type) / 4, (uint8_t *)&cpu->iinfo);
        cpuinfo->iregion_base = iregion_base;
        if (cpuinfo->mcfginfo.b.smp) {
			target_read_memory(target, (iregion_base + CPUINFO_IRG_SMP_OFS + 0x4), 4, 1, (uint8_t *)&cpuinfo->smpcfg.d);
        }
        if (cpuinfo->smpcfg.b.cc) {
			target_read_memory(target, (iregion_base + CPUINFO_IRG_SMP_OFS + 0x8), 4, 1, (uint8_t *)&cpuinfo->cccfg.d);
        }
        if (cpuinfo->mcfginfo.b.eclic) {
            memset(&cpu->eclic, 0, sizeof(CIF_ECLIC_Type));
			target_read_memory(target, (iregion_base + CPUINFO_IRG_ECLIC_OFS), 4, sizeof(CIF_ECLIC_Type) / 4, (uint8_t *)&cpu->eclic);
	    }
    }
    if (cpuinfo->mcfginfo.b.ppi) {
        if ( riscv_reg_get(target, &regval, GDB_REGNO_CSR0 + CSR_MPPICFG_INFO) == ERROR_OK) {
            cpuinfo->mppicfginfo.d = (uint64_t)regval;
        }
    }
    if (cpuinfo->mcfginfo.b.fio) {
        if (riscv_reg_get(target, &regval, GDB_REGNO_CSR0 + CSR_MFIOCFG_INFO) == ERROR_OK) {
            cpuinfo->mfiocfginfo.d = (uint64_t)regval;
        }
    }
    return 0;
}

#define EXTRACT_MFG(X)  (((X) & 0xffe) >> 1)
void nuclei_display_cpuinfo(struct target *target, void *cmd)
{
    if (target == NULL)
        return;
    // Nuclei Manufacture ID is 0x536
    if (EXTRACT_MFG(target->tap->idcode) != 0x536) {
        if (cmd == NULL) { // Just basic info
            return;
        }
    }
	NUCLEI_CPUINFO xlcpu;
    if (nuclei_get_cpuinfo(target, &xlcpu) != 0) {
        if (cmd != NULL) {
            cif_printf("Unable to get any Nuclei CPU INFO, please retry it and make sure JTAG connection is stable!");
        }
        return;
    }

	char strbuf[8192];
    if (cmd != NULL) {
	    set_print_interface((struct command_invocation *)cmd);
    }
    if (get_basic_cpuinfo(&xlcpu.cpuinfo, strbuf, 8192) > 0) {
        if (cmd == NULL) {
	        LOG_TARGET_INFO(target, "%s", strbuf);
        } else {
            cif_printf("%s: %s", target->cmd_name, strbuf);
        }
    }
    if (CMD != NULL) {
	    show_cpuinfo(&xlcpu.cpuinfo);
    }
}

COMMAND_HANDLER(handle_nuclei_cpuinfo)
{
	struct target *target = NULL;

    target = get_current_target(CMD_CTX);
    if (CMD_CTX->current_target_override == NULL)
		target = get_target(target->current_target_name);
    if (target == NULL)
        target = get_current_target(CMD_CTX);

    nuclei_display_cpuinfo(target, (void *)CMD);

	return ERROR_OK;
}

uint64_t nuclei_get_dmcustom(struct target *target, uint32_t type, uint32_t hart_id, uint32_t index)
{
	uint64_t out_data;
	uint32_t address, value;
	RISCV_INFO(r);
	if (!r) {
		LOG_ERROR("riscv_info is NULL!");
		return ERROR_FAIL;
	}

	if (r->dmi_write) {
		address = 0x70;
		value = (type << 28) | (hart_id << 8) | index;
		if (r->dmi_write(target, address, value) != ERROR_OK)
			return ERROR_FAIL;
	} else {
		LOG_ERROR("dmi_write is not implemented for this target.");
		return ERROR_FAIL;
	}

	if (r->dmi_read) {
		address = 0x71;
		if (r->dmi_read(target, &value, address) != ERROR_OK)
			return ERROR_FAIL;
		out_data = value;
		address = 0x72;
		if (r->dmi_read(target, &value, address) != ERROR_OK)
			return ERROR_FAIL;
		out_data |= (uint64_t)value << 32;
	} else {
		LOG_ERROR("dmi_read is not implemented for this target.");
		return ERROR_FAIL;
	}
	return out_data;
}

COMMAND_HANDLER(nuclei_set_expose_cpu_core)
{
	if (CMD_ARGC == 0) {
		LOG_ERROR("This command expects at least 1 parameters");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	struct target *target = get_current_target(CMD_CTX);
	RISCV_INFO(info);
	int ret = ERROR_OK;

	for (unsigned int i = 0; i < CMD_ARGC; i++) {
		ret = parse_reg_ranges(&info->expose_nuclei_cpu_core, CMD_ARGV[i], "nuclei_cpu_core", 0xFF);
		if (ret != ERROR_OK)
			break;
	}

	return ret;
}

COMMAND_HANDLER(nuclei_set_examine_cpu_core)
{
	if (CMD_ARGC == 0) {
		LOG_ERROR("This command expects at least 1 parameters");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	uint64_t value;
	uint32_t halt_id = 0;
	uint32_t index = 0;
	struct target *target = get_current_target(CMD_CTX);
	RISCV_INFO(r);
	if (!r) {
		LOG_ERROR("riscv_info is NULL!");
		return ERROR_FAIL;
	}

	if (CMD_ARGC == 1) {
		COMMAND_PARSE_NUMBER(uint, CMD_ARGV[0], halt_id);
		range_list_t *entry;
		list_for_each_entry(entry, &r->expose_nuclei_cpu_core, list) {
			value = nuclei_get_dmcustom(target, 0, halt_id, entry->high);
			command_print_sameline(CMD, "%u: ", entry->high);
			command_print(CMD, "0x%016" PRIx64, value);
		}
	} else if (CMD_ARGC == 2) {
		COMMAND_PARSE_NUMBER(uint, CMD_ARGV[0], halt_id);
		COMMAND_PARSE_NUMBER(uint, CMD_ARGV[1], index);
		value = nuclei_get_dmcustom(target, 0, halt_id, index);
		command_print_sameline(CMD, "%u: ", index);
		command_print(CMD, "0x%016" PRIx64, value);
	}

	return ERROR_OK;
}

COMMAND_HANDLER(handle_etrace_enable_command)
{
	if (CMD_ARGC > 0)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct target *target = get_current_target(CMD_CTX);
	struct target *target_real;
	if (target->smp)
		target_real = get_target(target->current_target_name);
	else
		target_real = target;

	riscv_reg_t dpc_rb;
	riscv_reg_t tselect;
	riscv_reg_t tdata1 = field_value(CSR_MCONTROL_TYPE(riscv_xlen(target_real)), CSR_TDATA1_TYPE_MCONTROL) |
							field_value(CSR_MCONTROL_ACTION, CSR_MCONTROL_ACTION_TRACE_ON) |
							field_value(CSR_MCONTROL_M, 1) |
							field_value(CSR_MCONTROL_S, 1) |
							field_value(CSR_MCONTROL_U, 1) |
							field_value(CSR_MCONTROL_EXECUTE, 1);
	RISCV_INFO(r);
	if (riscv_enumerate_triggers(target_real) != ERROR_OK) {
		LOG_ERROR("Unable to enumerate trigger for target %s.\n", target->current_target_name);
		return ERROR_OK;
	}
	if (target_real->trace_trigger_index == 0xFFFFFFFF) {
		for (unsigned int i = 0; i < r->trigger_count; i++) {
			if (r->trigger_unique_id[i] != -1) {
				LOG_DEBUG("Trigger %d is in use, could't be reserved for trace usage.\n", i);
				continue;
			}
			target_real->trace_trigger_index = i;
			r->trigger_unique_id[i] = 0xFE;
			r->reserved_triggers[i] = true;
			LOG_DEBUG("Trigger %d is reserved for trace usage.\n", i);
			break;
		}
	}
	if (target_real->trace_trigger_index == 0xFFFFFFFF) {
		LOG_ERROR("No available trigger could be reserved for trace usage.\n");
		return ERROR_OK;
	}
	tselect = target_real->trace_trigger_index;
	if (riscv_reg_set(target_real, GDB_REGNO_TSELECT, tselect) != ERROR_OK)
		goto error;
	if (riscv_reg_set(target_real, GDB_REGNO_TDATA1, tdata1) != ERROR_OK)
		goto error;
	if (riscv_reg_get(target_real, &dpc_rb, GDB_REGNO_DPC) != ERROR_OK)
		goto error;
	if (riscv_reg_set(target_real, GDB_REGNO_TDATA2, dpc_rb) != ERROR_OK)
		goto error;

	return ERROR_OK;
error:
	LOG_ERROR("Unable to configure selected trace trigger %u.\n", (uint32_t)tselect);
	return ERROR_OK;
}

COMMAND_HANDLER(handle_etrace_disable_command)
{
	if (CMD_ARGC > 0)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct target *target = get_current_target(CMD_CTX);
	struct target *target_real;
	if (target->smp)
		target_real = get_target(target->current_target_name);
	else
		target_real = target;

	riscv_reg_t dpc_rb;
	riscv_reg_t tselect;
	riscv_reg_t tdata1 = field_value(CSR_MCONTROL_TYPE(riscv_xlen(target_real)), CSR_TDATA1_TYPE_MCONTROL) |
							field_value(CSR_MCONTROL_ACTION, CSR_MCONTROL_ACTION_TRACE_OFF) |
							field_value(CSR_MCONTROL_M, 1) |
							field_value(CSR_MCONTROL_S, 1) |
							field_value(CSR_MCONTROL_U, 1) |
							field_value(CSR_MCONTROL_EXECUTE, 1);
	RISCV_INFO(r);
	if (target_real->trace_trigger_index == 0xFFFFFFFF)
		return ERROR_OK;
	if (riscv_enumerate_triggers(target_real) != ERROR_OK) {
		LOG_ERROR("Unable to enumerate trigger for target %s.\n", target->current_target_name);
		return ERROR_OK;
	}
	tselect = target_real->trace_trigger_index;
	if (riscv_reg_set(target_real, GDB_REGNO_TSELECT, tselect) != ERROR_OK)
		goto error;
	if (riscv_reg_set(target_real, GDB_REGNO_TDATA1, tdata1) != ERROR_OK)
		goto error;
	if (riscv_reg_get(target_real, &dpc_rb, GDB_REGNO_DPC) != ERROR_OK)
		goto error;
	if (riscv_reg_set(target_real, GDB_REGNO_TDATA2, dpc_rb) != ERROR_OK)
		goto error;

	LOG_DEBUG("Trace free up trigger %d.\n", target_real->trace_trigger_index);
	r->reserved_triggers[target_real->trace_trigger_index] = false;
	r->trigger_unique_id[target_real->trace_trigger_index] = -1;
	target_real->trace_trigger_index = 0xFFFFFFFF;

	return ERROR_OK;
error:
	LOG_ERROR("Unable to configure selected trace trigger %u.\n", (uint32_t)tselect);
	return ERROR_OK;
}

#define ETRACE_BASE_HI		(0x00)
#define ETRACE_BASE_LO		(0x04)
#define ETRACE_WLEN		(0x08)
#define ETRACE_ENA		(0x0c)
#define ETRACE_INTERRUPT	(0x10)
#define ETRACE_MAXTIME		(0x14)
#define ETRACE_EARLY		(0x18)
#define ETRACE_ATOVF		(0x1c)
#define ETRACE_ENDOFFSET	(0x20)
#define ETRACE_FLG		(0x24)
#define ETRACE_ONGOING		(0x28)
#define ETRACE_TIMEOUT		(0x2c)
#define ETRACE_IDLE		(0x30)
#define ETRACE_FIFO		(0x34)
#define ETRACE_ATBDW		(0x38)
#define ETRACE_WRAP		(0x3c)
#define ETRACE_COMPACT		(0x40)

static target_addr_t atb2axi_config_addr = 0xa5a5a5a5;
static target_addr_t buffer_addr = 0xa5a5a5a5;
static uint32_t buffer_size;

#define CHECK_ETRACE_CONFIGURED()	{ \
		if (atb2axi_config_addr == 0xa5a5a5a5) { \
			LOG_ERROR("ETrace is not configured, please configure it first!"); \
			LOG_INFO("eg. nuclei etrace config <atb2axi-addr> <buffer-addr> <buffer-size> <wrap>"); \
			return ERROR_OK; \
		} \
	}

static int etrace_read_reg(struct target *target, uint32_t offset, uint32_t *value)
{
	CHECK_ETRACE_CONFIGURED();
	int result = target_read_u32(target, atb2axi_config_addr + offset, value);
	if (result != ERROR_OK) {
		LOG_ERROR("Failed to read etrace atb2axi register at %#lx", (uint64_t)(atb2axi_config_addr + offset));
		return result;
	}
	return ERROR_OK;
}

static int etrace_write_reg(struct target *target, uint32_t offset, uint32_t value)
{
	CHECK_ETRACE_CONFIGURED();
	int result = target_write_u32(target, atb2axi_config_addr + offset, value);
	if (result != ERROR_OK) {
		LOG_ERROR("Failed to write etrace atb2axi register %#lx with %#x", (uint64_t)(atb2axi_config_addr + offset), value);
		return result;
	}
	return ERROR_OK;
}

static int etrace_stop(struct target *target)
{
	uint32_t tmp;
	uint32_t wait_idle = 0x100;

	CHECK_ETRACE_CONFIGURED();
	etrace_write_reg(target, ETRACE_ENA, 0);
	do {
		etrace_read_reg(target, ETRACE_IDLE, &tmp);
		wait_idle -= 1;
		if (wait_idle == 0)
			break;
	} while (tmp != 1);
    return ERROR_OK;
}

COMMAND_HANDLER(handle_etrace_config_command)
{
	uint32_t wrap;

	if (CMD_ARGC != 4)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct target *target = get_current_target(CMD_CTX);

	uint32_t tmp = 0;
	if (atb2axi_config_addr != 0xa5a5a5a5)
		etrace_read_reg(target, ETRACE_ENA, &tmp);
	if (tmp) {
		LOG_ERROR("ETrace is still running, please run nuclei etrace stop to stop it first!");
		return ERROR_OK;
	}

	COMMAND_PARSE_NUMBER(target_addr, CMD_ARGV[0], atb2axi_config_addr);
	COMMAND_PARSE_NUMBER(target_addr, CMD_ARGV[1], buffer_addr);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[2], buffer_size);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[3], wrap);

	if (etrace_write_reg(target, ETRACE_ENDOFFSET, 0) != ERROR_OK) {
		/* Only check first register write, if ok, assume configured correct */
		LOG_ERROR("Unable to write etrace atb2axi register, please check!");
		return ERROR_OK;
	}
	etrace_write_reg(target, ETRACE_FLG, 0);
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

COMMAND_HANDLER(handle_etrace_start_command)
{
	if (CMD_ARGC > 0)
		return ERROR_COMMAND_SYNTAX_ERROR;

	CHECK_ETRACE_CONFIGURED();
	struct target *target = get_current_target(CMD_CTX);

	etrace_write_reg(target, ETRACE_ENA, 1);

	return ERROR_OK;
}

COMMAND_HANDLER(handle_etrace_stop_command)
{
	if (CMD_ARGC > 0)
		return ERROR_COMMAND_SYNTAX_ERROR;

	CHECK_ETRACE_CONFIGURED();
	struct target *target = get_current_target(CMD_CTX);

	etrace_stop(target);

	return ERROR_OK;
}

COMMAND_HANDLER(handle_etrace_dump_command)
{
	uint32_t end_offset;
	uint32_t full_flag;
	target_addr_t address;
	uint32_t size;
	uint8_t *temp = NULL;
	struct fileio *fileio = NULL;
	int retval = ERROR_OK;
	struct duration bench;

	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	CHECK_ETRACE_CONFIGURED();
	struct target *target = get_current_target(CMD_CTX);

	uint32_t tmp = 0;

	etrace_read_reg(target, ETRACE_ENA, &tmp);
	if (tmp) {
		LOG_ERROR("ETrace is still running, please use nuclei etrace stop to stop it first!");
		return ERROR_OK;
	}

	etrace_read_reg(target, ETRACE_ENDOFFSET, &end_offset);
	etrace_read_reg(target, ETRACE_FLG, &full_flag);
	if (full_flag) {
		address = buffer_addr + end_offset;
		size = buffer_size;
	} else {
		address = buffer_addr;
		size = end_offset;
	}

	if (size == 0) {
		command_print(CMD, "No etrace could be dumped!");
		goto ok;
	}
	uint32_t temp_size = (size > 4096) ? 4096 : size;
	temp = malloc(temp_size);
	if (!temp)
		goto fail;

	if (fileio_open(&fileio, CMD_ARGV[0], FILEIO_WRITE, FILEIO_BINARY) != ERROR_OK)
		goto fail;

	duration_start(&bench);

	while (size > 0) {
		size_t size_written;
		uint32_t this_run_size = (size > temp_size) ? temp_size : size;
		if ((this_run_size + address) > (buffer_addr + buffer_size))
			this_run_size = (buffer_addr + buffer_size) - address;

		retval = target_read_buffer(target, address, this_run_size, temp);
		if (retval != ERROR_OK)
			break;

		retval = fileio_write(fileio, this_run_size, temp, &size_written);
		if (retval != ERROR_OK)
			break;

		size -= this_run_size;
		if ((this_run_size + address) == (buffer_addr + buffer_size))
			address = buffer_addr;
		else
			address += this_run_size;
	}

	if (retval == ERROR_OK && (duration_measure(&bench) == ERROR_OK)) {
		size_t filesize;
		fileio_size(fileio, &filesize);
		command_print(CMD,
				"Dumped etrace %zu bytes in %fs (%0.3f KiB/s)", filesize,
				duration_elapsed(&bench), duration_kbps(&bench, filesize));
	} else {
		goto fail;
	}

	goto ok;
fail:
	LOG_ERROR("Unexpected error happened during dumping etrace!");
	retval = ERROR_OK;
ok:
	if (fileio)
		fileio_close(fileio);
	if (temp)
		free(temp);

	return retval;
}

COMMAND_HANDLER(handle_etrace_clear_command)
{
	if (CMD_ARGC > 0)
		return ERROR_COMMAND_SYNTAX_ERROR;

	CHECK_ETRACE_CONFIGURED();
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

	CHECK_ETRACE_CONFIGURED();
	if (CMD_ARGC > 0)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct target *target = get_current_target(CMD_CTX);

	etrace_read_reg(target, ETRACE_ENDOFFSET, &end_offset);
	etrace_read_reg(target, ETRACE_FLG, &full_flag);

	command_print(CMD, "ATB2AXI Addr: 0x%" PRIx64, atb2axi_config_addr);
	command_print(CMD, "Buffer  Addr: 0x%" PRIx64, buffer_addr);
	command_print(CMD, "Buffer  Size: %#x", buffer_size);
	command_print_sameline(CMD, "Buffer Status: ");
	if (full_flag)
		command_print(CMD, "used 100%% from 0x%" PRIx64 " [wraped]", buffer_addr + end_offset);
	else
		command_print(CMD, "used %d%% from 0x%" PRIx64 " to 0x%" PRIx64, (end_offset * 100) / buffer_size, \
			buffer_addr, buffer_addr + end_offset);

	return ERROR_OK;
}

static const struct command_registration etrace_command_handlers[] = {
	{
		.name = "config",
		.handler = handle_etrace_config_command,
		.mode = COMMAND_EXEC,
		.help = "Configuration etrace.",
		.usage = "atb2axi-addr buffer-addr buffer-size wrap",
	},
	{
		.name = "enable",
		.handler = handle_etrace_enable_command,
		.mode = COMMAND_EXEC,
		.help = "enable etrace within a core",
		.usage = "",
	},
	{
		.name = "disable",
		.handler = handle_etrace_disable_command,
		.mode = COMMAND_EXEC,
		.help = "disable etrace within a core",
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

//TRACE FUNNEL Registers
#define TRACE_FUNNEL		(0x0000)
#define TRFUNNELCONTROL		(0x0)
#define TRFUNNELDISINPUT	(0x8)
#define TRTSCONTROL		(0x40)
//TRACE RAM Registers
#define TRACE_RAM		(0x1000)
#define TRRAMCONTROL		(0x0)
#define TRRAMSTARTLOW		(0x10)
#define TRRAMSTARTHIGH		(0x14)
#define TRRAMLIMITLOW		(0x18)
#define TRRAMLIMITHIGH		(0x1C)
#define TRRAMWPLOW		(0x20)
#define TRRAMWPHIGH		(0x24)
//TRACE TE Registers
#define TRACE_TE		(0x4000)
#define TRTECONTROL		(0x0)
#define TRTEINSTFEATURES	(0x8)
#define TRTEDATACONTROL		(0x10)
#define TRTEDATAFILTERS		(0x1c)
#define TRTSCONTROL		(0x40)
#define TRTEFILTER0CONTROL	(0x400)
#define TRTEFILTER0MATCHDATA	(0x418)

static target_addr_t tracebundle_addr = 0xa5a5a5a5;
static target_addr_t trace_ram_addr = 0xa5a5a5a5;
static target_addr_t trace_ram_size;
static target_addr_t end_ram_addr;
static uint32_t trace_core_num = 0;
static uint32_t trace_core_mask = 0;

#define CHECK_TRACE_CONFIGURED()	{ \
		if (tracebundle_addr == 0xa5a5a5a5) { \
			LOG_ERROR("Trace is not configured, please configure it first!"); \
			LOG_INFO("eg. nuclei trace config tracebundle-addr buffer-addr buffer-size wrap(0|1) core-mask timestamp(0|1|2|3) timestamp-div(0|1|2|3) datatrace"); \
			return ERROR_OK; \
		} \
	}

static int trace_read_reg(struct target *target, uint32_t offset, uint32_t *value)
{
	CHECK_TRACE_CONFIGURED();
	int result = target_read_u32(target, tracebundle_addr + offset, value);
	if (result != ERROR_OK) {
		LOG_ERROR("Failed to read trace register at %#lx", (uint64_t)(tracebundle_addr + offset));
		return result;
	}
	return ERROR_OK;
}

static int trace_write_reg(struct target *target, uint32_t offset, uint32_t value)
{
	CHECK_TRACE_CONFIGURED();
	int result = target_write_u32(target, tracebundle_addr + offset, value);
	if (result != ERROR_OK) {
		LOG_ERROR("Failed to write trace register %#lx with %#x", (uint64_t)(tracebundle_addr + offset), value);
		return result;
	}
	return ERROR_OK;
}

COMMAND_HANDLER(handle_trace_config_command)
{
	uint32_t wrap,timestamp,timestamp_div,datatrace,noaddr,nodata,load,store,tmp;

	if (CMD_ARGC != 0x7)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct target *target = get_current_target(CMD_CTX);

	COMMAND_PARSE_NUMBER(target_addr, CMD_ARGV[0], tracebundle_addr);
	COMMAND_PARSE_NUMBER(target_addr, CMD_ARGV[1], trace_ram_addr);
	COMMAND_PARSE_NUMBER(target_addr, CMD_ARGV[2], trace_ram_size);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[3], wrap); wrap = wrap ? 1 : 0;
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[4], trace_core_mask);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[5], timestamp);
	timestamp_div = (timestamp >> 4) & 0xF;
	timestamp = timestamp & 0xF;
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[6], datatrace);
	noaddr = (datatrace >> 1) & 0x1;
	nodata = (datatrace >> 2) & 0x1;
	load = ((datatrace >> 3) & 0x1) ? 0 : 1;
	store = ((datatrace >> 4) & 0x1) ? 0 : 1;

	// set active
	trace_read_reg(target, TRACE_RAM + TRRAMCONTROL, &tmp);
	trace_write_reg(target, TRACE_RAM + TRRAMCONTROL, tmp | 0x1);
	trace_read_reg(target, TRACE_FUNNEL + TRFUNNELCONTROL, &tmp);
	trace_write_reg(target, TRACE_FUNNEL + TRFUNNELCONTROL, tmp | 0x1);
	trace_read_reg(target, TRACE_FUNNEL + TRTSCONTROL, &tmp);
	trace_write_reg(target, TRACE_FUNNEL + TRTSCONTROL, tmp | 0x1);
	// Read Core Number
	if (trace_write_reg(target,TRACE_FUNNEL + TRFUNNELDISINPUT, 0xFFFFFFFF) != ERROR_OK) {
		/* Only check first register read, if ok, assume configured correct */
		LOG_ERROR("Unable to read tracebundle register, please check!");
		return ERROR_OK;
	}
	trace_read_reg(target, TRACE_FUNNEL + TRFUNNELDISINPUT, &trace_core_num);
	trace_core_num = log2(trace_core_num + 1);
	trace_write_reg(target,TRACE_FUNNEL + TRFUNNELDISINPUT, 0x0);
	for (int i = 0;i < trace_core_num;i++) {
		if ((trace_core_mask & (0x1 << i)) == 0) continue;
		trace_read_reg(target, TRACE_TE + TRTECONTROL + i * 0x1000, &tmp);
		trace_write_reg(target, TRACE_TE + TRTECONTROL + i * 0x1000, tmp | 0x1);
		trace_read_reg(target, TRACE_TE + TRTSCONTROL + i * 0x1000, &tmp);
		trace_write_reg(target, TRACE_TE + TRTSCONTROL + i * 0x1000, tmp | 0x1);
	}
	// config
	if (wrap == 0) { // need to set trRamStopOnWrap
		trace_read_reg(target, TRACE_RAM + TRRAMCONTROL, &tmp);
		trace_write_reg(target, TRACE_RAM + TRRAMCONTROL, tmp | (0x1 << 8));
	}
	trace_write_reg(target, TRACE_RAM + TRRAMSTARTLOW, (uint32_t)trace_ram_addr);
	trace_write_reg(target, TRACE_RAM + TRRAMSTARTHIGH, (uint32_t)(trace_ram_addr >> 32));
	trace_write_reg(target, TRACE_RAM + TRRAMLIMITLOW, (uint32_t)(trace_ram_addr + trace_ram_size - 4));
	trace_write_reg(target, TRACE_RAM + TRRAMLIMITHIGH, (uint32_t)((trace_ram_addr + trace_ram_size) >> 32));
	trace_write_reg(target, TRACE_RAM + TRRAMWPLOW, (uint32_t)trace_ram_addr);
	trace_write_reg(target, TRACE_RAM + TRRAMWPHIGH, (uint32_t)(trace_ram_addr >> 32));
	switch (timestamp) {
	case 0:// none-timestamp
		trace_read_reg(target, TRACE_FUNNEL + TRTSCONTROL, &tmp);
		trace_write_reg(target, TRACE_FUNNEL + TRTSCONTROL, tmp & ~(0x7 << 4));
		for (int i = 0;i < trace_core_num;i++) {
			if ((trace_core_mask & (0x1 << i)) == 0) continue;
			trace_read_reg(target, TRACE_TE + TRTSCONTROL + i * 0x1000, &tmp);
			trace_write_reg(target, TRACE_TE + TRTSCONTROL + i * 0x1000, tmp & ~(0x7 << 4));
		}
		break;
	case 1:// cpu-timestamp
		trace_read_reg(target, TRACE_FUNNEL + TRTSCONTROL, &tmp);
		trace_write_reg(target, TRACE_FUNNEL + TRTSCONTROL, tmp & ~(0x7 << 4));
		for (int i = 0;i < trace_core_num;i++) {
			if ((trace_core_mask & (0x1 << i)) == 0) continue;
			trace_read_reg(target, TRACE_TE + TRTSCONTROL + i * 0x1000, &tmp);
			trace_write_reg(target, TRACE_TE + TRTSCONTROL + i * 0x1000, (tmp & ~(0x7 << 4)) | (0x3 << 4) | (0x1 << 1));
			// config div in encoder
			trace_read_reg(target, TRACE_TE + TRTSCONTROL + i * 0x1000, &tmp);
			trace_write_reg(target, TRACE_TE + TRTSCONTROL + i * 0x1000, (tmp & ~(0x3 << 8)) | (timestamp_div << 8) | (0x1 << 15));
		}
		break;
	case 2:// funnel-timestamp
		trace_read_reg(target, TRACE_FUNNEL + TRTSCONTROL, &tmp);
		trace_write_reg(target, TRACE_FUNNEL + TRTSCONTROL, (tmp & ~(0x7 << 4)) | (0x2 << 4) | (0x1 << 1));
		for (int i = 0;i < trace_core_num;i++) {
			if ((trace_core_mask & (0x1 << i)) == 0) continue;
			trace_read_reg(target, TRACE_TE + TRTSCONTROL + i * 0x1000, &tmp);
			trace_write_reg(target, TRACE_TE + TRTSCONTROL + i * 0x1000, (tmp & ~(0x7 << 4)) | (0x4 << 4) | (0x1 << 15));
		}
		// config div in funnel
		trace_read_reg(target, TRACE_FUNNEL + TRTSCONTROL, &tmp);
		trace_write_reg(target, TRACE_FUNNEL + TRTSCONTROL, (tmp & ~(0x3 << 8)) | (timestamp_div << 8));
		break;
	case 3:// external-timestamp
		trace_read_reg(target, TRACE_FUNNEL + TRTSCONTROL, &tmp);
		trace_write_reg(target, TRACE_FUNNEL + TRTSCONTROL, tmp & ~(0x7 << 4));
		for (int i = 0;i < trace_core_num;i++) {
			if ((trace_core_mask & (0x1 << i)) == 0) continue;
			trace_read_reg(target, TRACE_TE + TRTSCONTROL + i * 0x1000, &tmp);
			trace_write_reg(target, TRACE_TE + TRTSCONTROL + i * 0x1000, (tmp & ~(0x7 << 4)) | (0x1 << 4) | (0x1 << 15));
		}
		break;
	default:
		break;
	}
	for (int i = 0;i < trace_core_num;i++) {
		if ((trace_core_mask & (0x1 << i)) == 0) continue;
		trace_read_reg(target, TRACE_TE + TRTECONTROL + i * 0x1000, &tmp);
		trace_write_reg(target, TRACE_TE + TRTECONTROL + i * 0x1000, tmp | (0x1 << 11) | (0x1 << 2));
		// set srcid
		trace_read_reg(target, TRACE_TE + TRTEINSTFEATURES + i * 0x1000, &tmp);
		trace_write_reg(target, TRACE_TE + TRTEINSTFEATURES + i * 0x1000, (tmp & ~(0xFFF << 16)) | (i << 16));
		// datatrace
		trace_read_reg(target, TRACE_TE + TRTEDATACONTROL + i * 0x1000, &tmp);
		if (0 == (tmp & 0x1))
			continue;// not support datatrace
		if (datatrace & 0x1) {
			trace_write_reg(target, TRACE_TE + TRTEDATACONTROL + i * 0x1000, (tmp & ~(0x3 << 16)) | (0x1 << 1) | (nodata << 16) | (noaddr << 17));
			trace_read_reg(target, TRACE_TE + TRTEDATAFILTERS + i * 0x1000, &tmp);
			trace_write_reg(target, TRACE_TE + TRTEDATAFILTERS + i * 0x1000, tmp | 0x1);
			trace_read_reg(target, TRACE_TE + TRTEFILTER0CONTROL + i * 0x1000, &tmp);
			trace_write_reg(target, TRACE_TE + TRTEFILTER0CONTROL + i * 0x1000, tmp | 0x1 | (0x1 << 24));
			trace_read_reg(target, TRACE_TE + TRTEFILTER0MATCHDATA + i * 0x1000, &tmp);
			trace_write_reg(target, TRACE_TE + TRTEFILTER0MATCHDATA + i * 0x1000, (tmp & ~(0x3)) | load | (store << 1));
		}
	}

	return ERROR_OK;
}

COMMAND_HANDLER(handle_trace_start_command)
{
	if (CMD_ARGC > 0)
		return ERROR_COMMAND_SYNTAX_ERROR;

	CHECK_TRACE_CONFIGURED();
	struct target *target = get_current_target(CMD_CTX);

	uint32_t tmp;
	trace_read_reg(target, TRACE_RAM + TRRAMCONTROL, &tmp);
	trace_write_reg(target, TRACE_RAM + TRRAMCONTROL, tmp | (0x1 << 1));
	trace_read_reg(target, TRACE_FUNNEL + TRFUNNELCONTROL, &tmp);
	trace_write_reg(target, TRACE_FUNNEL + TRFUNNELCONTROL, tmp | (0x1 << 1));
	for (int i = 0;i < trace_core_num;i++) {
		trace_read_reg(target, TRACE_TE + TRTECONTROL + i * 0x1000, &tmp);
		trace_write_reg(target, TRACE_TE + TRTECONTROL + i * 0x1000, tmp | (0x1 << 1));
	}

	return ERROR_OK;
}

COMMAND_HANDLER(handle_trace_stop_command)
{
	if (CMD_ARGC > 0)
		return ERROR_COMMAND_SYNTAX_ERROR;

	CHECK_TRACE_CONFIGURED();
	struct target *target = get_current_target(CMD_CTX);

	uint32_t tmp, wait_idle;
	for (int i = 0;i < trace_core_num;i++) {
		trace_read_reg(target, TRACE_TE + TRTECONTROL + i * 0x1000, &tmp);
		trace_write_reg(target, TRACE_TE + TRTECONTROL + i * 0x1000, tmp & ~(0x1 << 1));
		wait_idle = 0x100;
		do {
			trace_read_reg(target, TRACE_TE + TRTECONTROL + i * 0x1000, &tmp);
			if (wait_idle == 0) {
				LOG_ERROR("Timeout to wait core%d encoder empty!", i);
				break;
			}
			wait_idle -= 1;
		} while ((tmp & 0x8) != 0x8);
		trace_write_reg(target, TRACE_TE + TRTECONTROL + i * 0x1000, 0x0);
	}
	trace_read_reg(target, TRACE_FUNNEL + TRFUNNELCONTROL, &tmp);
	trace_write_reg(target, TRACE_FUNNEL + TRFUNNELCONTROL, tmp & ~(0x1 << 1));
	wait_idle = 0x100;
	do {
		trace_read_reg(target, TRACE_FUNNEL + TRFUNNELCONTROL, &tmp);
		if (wait_idle == 0) {
			LOG_ERROR("Timeout to wait funnel empty!");
			break;
		}
		wait_idle -= 1;
	} while ((tmp & 0x8) != 0x8);
	trace_write_reg(target, TRACE_FUNNEL + TRFUNNELCONTROL, 0x0);
	trace_read_reg(target, TRACE_RAM + TRRAMCONTROL, &tmp);
	trace_write_reg(target, TRACE_RAM + TRRAMCONTROL, tmp & ~(0x1 << 1));
	wait_idle = 0x100;
	do {
		trace_read_reg(target, TRACE_RAM + TRRAMCONTROL, &tmp);
		if (wait_idle == 0) {
			LOG_ERROR("Timeout to wait ram empty!");
			break;
		}
		wait_idle -= 1;
	} while ((tmp & 0x8) != 0x8);
	trace_write_reg(target, TRACE_RAM + TRRAMCONTROL, 0x0);

	return ERROR_OK;
}

COMMAND_HANDLER(handle_trace_dump_command)
{
	uint32_t wrap_flag;
	target_addr_t address;
	uint32_t size;
	uint8_t *temp = NULL;
	struct fileio *fileio = NULL;
	int retval = ERROR_OK;
	struct duration bench;

	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	CHECK_TRACE_CONFIGURED();
	struct target *target = get_current_target(CMD_CTX);

	uint32_t tmp = 0;
	trace_read_reg(target, TRACE_RAM + TRRAMCONTROL, &tmp);
	if (tmp & (0x1 << 1)) {
		LOG_ERROR("trace is still running, please use nuclei trace stop to stop it first!");
		return ERROR_OK;
	}

	trace_read_reg(target, TRACE_RAM + TRRAMWPHIGH, &tmp);
	end_ram_addr = tmp;
	trace_read_reg(target, TRACE_RAM + TRRAMWPLOW, &tmp);
	wrap_flag = tmp & 0x1;
	end_ram_addr = (end_ram_addr << 32) + (tmp & (~0x1));
	if (wrap_flag) {
		address = end_ram_addr;
		size = trace_ram_size;
	} else {
		address = trace_ram_addr;
		size = end_ram_addr - trace_ram_addr;
	}

	if (size == 0) {
		command_print(CMD, "No trace could be dumped!");
		goto ok;
	}
	command_print(CMD, "Dumped %#x bytes data from 0x%" PRIx64, size, address);
	uint32_t temp_size = (size > 4096) ? 4096 : size;
	temp = malloc(temp_size);
	if (!temp)
		goto fail;

	if (fileio_open(&fileio, CMD_ARGV[0], FILEIO_WRITE, FILEIO_BINARY) != ERROR_OK)
		goto fail;

	duration_start(&bench);

	while (size > 0) {
		size_t size_written;
		uint32_t this_run_size = (size > temp_size) ? temp_size : size;
		if ((this_run_size + address) > (trace_ram_addr + trace_ram_size))
			this_run_size = (trace_ram_addr + trace_ram_size) - address;

		retval = target_read_buffer(target, address, this_run_size, temp);
		if (retval != ERROR_OK)
			break;

		retval = fileio_write(fileio, this_run_size, temp, &size_written);
		if (retval != ERROR_OK)
			break;

		size -= this_run_size;
		if ((this_run_size + address) == (trace_ram_addr + trace_ram_size))
			address = trace_ram_addr;
		else
			address += this_run_size;
	}

	if (retval == ERROR_OK && (duration_measure(&bench) == ERROR_OK)) {
		size_t filesize;
		fileio_size(fileio, &filesize);
		command_print(CMD,
				"Dumped trace %zu bytes in %fs (%0.3f KiB/s)", filesize,
				duration_elapsed(&bench), duration_kbps(&bench, filesize));
	} else {
		goto fail;
	}

	goto ok;
fail:
	LOG_ERROR("Unexpected error happened during dumping trace!");
	retval = ERROR_OK;
ok:
	if (fileio)
		fileio_close(fileio);
	if (temp)
		free(temp);

	return retval;
}

COMMAND_HANDLER(handle_trace_info_command)
{
	uint32_t wrap_flag, tmp;

	CHECK_TRACE_CONFIGURED();
	if (CMD_ARGC > 0)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct target *target = get_current_target(CMD_CTX);

	trace_read_reg(target, TRACE_RAM + TRRAMWPHIGH, &tmp);
	end_ram_addr = tmp;
	trace_read_reg(target, TRACE_RAM + TRRAMWPLOW, &tmp);
	wrap_flag = tmp & 0x1;
	end_ram_addr = (end_ram_addr << 32) + (tmp & (~0x1));

	command_print(CMD, "TraceBundle Addr: 0x%" PRIx64, tracebundle_addr);
	command_print(CMD, "Ram Addr        : 0x%" PRIx64, trace_ram_addr);
	command_print(CMD, "Ram Size        : 0x%" PRIx64, trace_ram_size);
	command_print_sameline(CMD, "Ram Status: ");
	trace_read_reg(target, TRACE_RAM + TRRAMCONTROL, &tmp);
	tmp = (tmp & (0x1 << 8)) ? 0 : 1;
	if (wrap_flag & tmp) {
		command_print(CMD, "used 100%% from 0x%" PRIx64 " [wraped]", end_ram_addr);
	} else {
		if (wrap_flag)
			tmp = 100;
		else
			tmp = ((end_ram_addr - trace_ram_addr) * 100) / trace_ram_size;
		command_print(CMD, "used %d%% from 0x%" PRIx64 " to 0x%" PRIx64, tmp, trace_ram_addr, end_ram_addr);
	}

	return ERROR_OK;
}

static const struct command_registration trace_command_handlers[] = {
	{
		.name = "config",
		.handler = handle_trace_config_command,
		.mode = COMMAND_EXEC,
		.help = "Configuration trace.",
		.usage = "tracebundle-addr buffer-addr buffer-size wrap core-mask timestamp datatrace"
			 "timestamp: [7:4]:timestamp-div, [3:0]:timestamp-src"
			 "datatrace: [4]:no-store, [3]:no-load,[2]:no-value,[1]:no-addr, [0]:datatrace-en",
	},
	{
		.name = "start",
		.handler = handle_trace_start_command,
		.mode = COMMAND_EXEC,
		.help = "start trace collection",
		.usage = "",
	},
	{
		.name = "stop",
		.handler = handle_trace_stop_command,
		.mode = COMMAND_EXEC,
		.help = "stop trace collection",
		.usage = "",
	},
	{
		.name = "dump",
		.handler = handle_trace_dump_command,
		.mode = COMMAND_EXEC,
		.help = "dump trace data",
		.usage = "filename",
	},
	{
		.name = "info",
		.handler = handle_trace_info_command,
		.mode = COMMAND_EXEC,
		.help = "display trace info",
		.usage = "",
	},
	COMMAND_REGISTRATION_DONE
};

COMMAND_HANDLER(handle_halt_group_command)
{
	if (CMD_ARGC < 2) {
		LOG_ERROR("This command expects at least 2 parameters");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	bool on_off;
	COMMAND_PARSE_BOOL(CMD_ARGV[0], on_off, "on", "off");

	struct target *target = get_current_target(CMD_CTX);
	int retval = ERROR_OK;
	for (int i = 1; i < CMD_ARGC; i++) {
		target = get_target(CMD_ARGV[i]);
		RISCV_INFO(r);
		if (r->dmi_write) {
			if (on_off)
				retval = r->dmi_write(target, DM_DMCS2, 0x7);
			else
				retval = r->dmi_write(target, DM_DMCS2, 0x0);
		}
	}

	return retval;
}

COMMAND_HANDLER(handle_resume_group_command)
{
	if (CMD_ARGC < 2) {
		LOG_ERROR("This command expects at least 2 parameters");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	bool on_off;
	COMMAND_PARSE_BOOL(CMD_ARGV[0], on_off, "on", "off");

	struct target *target = get_current_target(CMD_CTX);
	int retval = ERROR_OK;
	for (int i = 1; i < CMD_ARGC; i++) {
		target = get_target(CMD_ARGV[i]);
		RISCV_INFO(r);
		if (r->dmi_write) {
			if (on_off)
				retval = r->dmi_write(target, DM_DMCS2, 0x887);
			else
				retval = r->dmi_write(target, DM_DMCS2, 0x0);
		}
	}

	return retval;
}

static const struct command_registration cti_command_handlers[] = {
	{
		.name = "halt_group",
		.handler = handle_halt_group_command,
		.mode = COMMAND_EXEC,
		.help = "Configure and clear corss-trigger halt group",
		.usage = "halt_group on/off target_name0 target_name1",
	},
	{
		.name = "resume_group",
		.handler = handle_resume_group_command,
		.mode = COMMAND_EXEC,
		.help = "Configure and clear corss-trigger resume group",
		.usage = "resume_group on/off target_name0 target_name1",
	},
	COMMAND_REGISTRATION_DONE
};

const struct command_registration nuclei_command_group_handlers[] = {
	{
		.name = "cpuinfo",
		.handler = handle_nuclei_cpuinfo,
		.mode = COMMAND_ANY,
		.usage = "",
		.help = "Displays some information about the target"
	},
	{
		.name = "expose_cpu_core",
		.handler = nuclei_set_expose_cpu_core,
		.mode = COMMAND_CONFIG,
		.usage = "<index0> ... <index255>",
		.help = "Configure a list of index for `nuclei_examine_cpu_core` to expose in "
				"This must be executed before `init`."
	},
	{
		.name = "examine_cpu_core",
		.handler = nuclei_set_examine_cpu_core,
		.mode = COMMAND_ANY,
		.usage = "<hartid> [index]",
		.help = "Return the 64-bit value read from dm-custom1 and dm-custom2 "
				"value = dm-custom2 << 32 + dm-custom1 "
	},
	{
		.name = "etrace",
		.mode = COMMAND_ANY,
		.help = "Etrace command group",
		.usage = "",
		.chain = etrace_command_handlers,
	},
	{
		.name = "trace",
		.mode = COMMAND_ANY,
		.help = "Trace command group",
		.usage = "",
		.chain = trace_command_handlers,
	},
	{
		.name = "cti",
		.mode = COMMAND_ANY,
		.help = "Cross trigger command group",
		.usage = "",
		.chain = cti_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

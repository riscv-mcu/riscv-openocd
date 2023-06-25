/***************************************************************************
 *   Copyright (C) 2010 by Antonio Borneo <borneo.antonio@gmail.com>       *
 *   Modified by Yanwen Wang <wangyanwen@nucleisys.com> based on fespi.c   *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdio.h>
#include "imp.h"
#include "spi.h"
#include <jtag/jtag.h>
#include <helper/time_support.h>
#include <target/algorithm.h>
#include "target/riscv/riscv.h"
#include <helper/configuration.h>

#define ERASE_CMD			(1)
#define WRITE_CMD			(2)
#define READ_CMD			(3)
#define PROBE_CMD			(4)

struct flash_bank_msg {
	bool probed;
	const struct flash_device *dev;
	target_addr_t ctrl_base;
	uint8_t *loader_path;
	uint8_t cs;
	uint8_t *buffer;
	uint32_t param_0;
	uint32_t param_1;
	bool simulation;
	uint32_t sectorsize;
};

static int custom_run_algorithm(struct flash_bank *bank)
{
	struct flash_bank_msg *bank_msg = bank->driver_priv;
	struct target *target = bank->target;
	int retval = ERROR_OK;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	int xlen = riscv_xlen(target);
	struct working_area *algorithm_wa = NULL;
	struct working_area *data_wa = NULL;
	uint8_t* bin = (uint8_t*)malloc(target->working_area_size);
	size_t bin_size;

	FILE* fd = fopen((char*)bank_msg->loader_path, "rb");
	if (NULL == fd) {
		fd = fopen(find_file(strrchr((char*)bank_msg->loader_path, '/')), "rb");
	}
	if (fd) {
		fseek(fd, 0, SEEK_END);
		bin_size = ftell(fd);
		rewind(fd);
		if (target->working_area_size < bin_size) {
			LOG_ERROR("working_area_size less than loader_bin_size");
			goto err;
		}
		if (1 != fread(bin, bin_size, 1, fd)) {
			LOG_ERROR("read loader error");
			goto err;
		}
		fclose(fd);
	} else {
		LOG_ERROR("Failed to open loader:%s ", bank_msg->loader_path);
		goto err;
	}

	unsigned data_wa_size = 0;
	if (target_alloc_working_area(target, bin_size, &algorithm_wa) == ERROR_OK) {
		retval = target_write_buffer(target, algorithm_wa->address, bin_size, bin);
		if (retval != ERROR_OK) {
			LOG_ERROR("Failed to write code to " TARGET_ADDR_FMT ": %d",
					algorithm_wa->address, retval);
			target_free_working_area(target, algorithm_wa);
			algorithm_wa = NULL;
		} else {
			data_wa_size = MIN(target->working_area_size - algorithm_wa->size, bank_msg->param_0);
			while (1) {
				if (target_alloc_working_area_try(target, data_wa_size, &data_wa) == ERROR_OK) {
					break;
				}
				data_wa_size = data_wa_size * 3 / 4;
			}
		}
	} else {
		LOG_WARNING("Couldn't allocate %zd-byte working area.", bin_size);
		algorithm_wa = NULL;
	}

	if (algorithm_wa) {
		uint32_t count = 0;
		uint32_t offset = 0;
		uint32_t first_addr = 0;
		uint32_t end_addr = 0;
		uint32_t cur_count = 0;
		int algorithm_result = 0;
		struct reg_param reg_params[5];
		init_reg_param(&reg_params[0], "a0", xlen, PARAM_IN_OUT);
		init_reg_param(&reg_params[1], "a1", xlen, PARAM_OUT);
		init_reg_param(&reg_params[2], "a2", xlen, PARAM_OUT);
		init_reg_param(&reg_params[3], "a3", xlen, PARAM_OUT);
		init_reg_param(&reg_params[4], "a4", xlen, PARAM_OUT);
		switch (bank_msg->cs)
		{
		case ERASE_CMD:
			first_addr = bank_msg->param_0;
			end_addr = bank_msg->param_1;
			buf_set_u64(reg_params[0].value, 0, xlen, bank_msg->cs);
			buf_set_u64(reg_params[1].value, 0, xlen, bank_msg->ctrl_base);
			buf_set_u64(reg_params[2].value, 0, xlen, first_addr);
			buf_set_u64(reg_params[3].value, 0, xlen, end_addr);
			buf_set_u64(reg_params[4].value, 0, xlen, 0);
			if (bank_msg->simulation) {
				retval = target_run_algorithm(target, 0, NULL,
						ARRAY_SIZE(reg_params), reg_params,
						algorithm_wa->address, 0, 0x7FFFFFFF, NULL);
			} else {
				retval = target_run_algorithm(target, 0, NULL,
						ARRAY_SIZE(reg_params), reg_params,
						algorithm_wa->address, 0, (end_addr - first_addr) * 2, NULL);
			}
			if (retval != ERROR_OK) {
				LOG_ERROR("Failed to execute algorithm at " TARGET_ADDR_FMT ": %d",
						algorithm_wa->address, retval);
				goto err;
			}
			algorithm_result = buf_get_u64(reg_params[0].value, 0, xlen);
			if (algorithm_result != 0) {
				LOG_ERROR("Algorithm returned error %d", algorithm_result);
				LOG_ERROR("erase command error");
				retval = ERROR_FAIL;
				goto err;
			}
			break;
		case WRITE_CMD:
			count = bank_msg->param_0;
			offset = bank_msg->param_1;
			cur_count = 0;
			while (count > 0) {
				cur_count = MIN(count, data_wa_size);
				buf_set_u64(reg_params[0].value, 0, xlen, bank_msg->cs);
				buf_set_u64(reg_params[1].value, 0, xlen, bank_msg->ctrl_base);
				buf_set_u64(reg_params[2].value, 0, xlen, data_wa->address);
				buf_set_u64(reg_params[3].value, 0, xlen, offset);
				buf_set_u64(reg_params[4].value, 0, xlen, cur_count);
				retval = target_write_buffer(target, data_wa->address, cur_count, bank_msg->buffer);
				if (retval != ERROR_OK) {
					LOG_DEBUG("Failed to write %d bytes to " TARGET_ADDR_FMT ": %d",
							cur_count, data_wa->address, retval);
					goto err;
				}
				if (bank_msg->simulation) {
					retval = target_run_algorithm(target, 0, NULL,
						ARRAY_SIZE(reg_params), reg_params,
						algorithm_wa->address, 0, 0x7FFFFFFF, NULL);
				} else {
					retval = target_run_algorithm(target, 0, NULL,
							ARRAY_SIZE(reg_params), reg_params,
							algorithm_wa->address, 0, cur_count * 2, NULL);
				}
				if (retval != ERROR_OK) {
					LOG_ERROR("Failed to execute algorithm at " TARGET_ADDR_FMT ": %d",
							algorithm_wa->address, retval);
					goto err;
				}
				algorithm_result = buf_get_u64(reg_params[0].value, 0, xlen);
				if (algorithm_result != 0) {
					LOG_ERROR("Algorithm returned error %d", algorithm_result);
					LOG_ERROR("write command error");
					retval = ERROR_FAIL;
					goto err;
				}
				bank_msg->buffer += cur_count;
				offset += cur_count;
				count -= cur_count;
			}
			break;
		case READ_CMD:
			count = bank_msg->param_0;
			offset = bank_msg->param_1;
			cur_count = 0;
			while (count > 0) {
				cur_count = MIN(count, data_wa_size);
				buf_set_u64(reg_params[0].value, 0, xlen, bank_msg->cs);
				buf_set_u64(reg_params[1].value, 0, xlen, bank_msg->ctrl_base);
				buf_set_u64(reg_params[2].value, 0, xlen, data_wa->address);
				buf_set_u64(reg_params[3].value, 0, xlen, offset);
				buf_set_u64(reg_params[4].value, 0, xlen, cur_count);
				if (bank_msg->simulation) {
					retval = target_run_algorithm(target, 0, NULL,
						ARRAY_SIZE(reg_params), reg_params,
						algorithm_wa->address, 0, 0x7FFFFFFF, NULL);
				} else {
					retval = target_run_algorithm(target, 0, NULL,
							ARRAY_SIZE(reg_params), reg_params,
							algorithm_wa->address, 0, cur_count * 2, NULL);
				}
				if (retval != ERROR_OK) {
					LOG_ERROR("Failed to execute algorithm at " TARGET_ADDR_FMT ": %d",
							algorithm_wa->address, retval);
					goto err;
				}
				algorithm_result = buf_get_u64(reg_params[0].value, 0, xlen);
				if (algorithm_result != 0) {
					LOG_ERROR("Algorithm returned error %d", algorithm_result);
					LOG_ERROR("read command error");
					retval = ERROR_FAIL;
					goto err;
				}
				retval = target_read_buffer(target, data_wa->address, cur_count, bank_msg->buffer);
				if (retval != ERROR_OK) {
					LOG_DEBUG("Failed to read %d bytes from " TARGET_ADDR_FMT ": %d",
							cur_count, data_wa->address, retval);
					goto err;
				}
				bank_msg->buffer += cur_count;
				offset += cur_count;
				count -= cur_count;
			}
			break;
		case PROBE_CMD:
			buf_set_u64(reg_params[0].value, 0, xlen, bank_msg->cs);
			buf_set_u64(reg_params[1].value, 0, xlen, bank_msg->ctrl_base);
			buf_set_u64(reg_params[2].value, 0, xlen, 0);
			buf_set_u64(reg_params[3].value, 0, xlen, 0);
			buf_set_u64(reg_params[4].value, 0, xlen, 0);
			if (bank_msg->simulation) {
				retval = target_run_algorithm(target, 0, NULL,
					ARRAY_SIZE(reg_params), reg_params,
					algorithm_wa->address, 0, 0x7FFFFFFF, NULL);
			} else {
				retval = target_run_algorithm(target, 0, NULL,
						ARRAY_SIZE(reg_params), reg_params,
						algorithm_wa->address, 0, 10000, NULL);
			}
			if (retval != ERROR_OK) {
				LOG_ERROR("Failed to execute algorithm at " TARGET_ADDR_FMT ": %d",
						algorithm_wa->address, retval);
				goto err;
			}
			algorithm_result = buf_get_u64(reg_params[0].value, 0, xlen);
			retval = algorithm_result;
			break;
		default:
			break;
		}
		target_free_working_area(target, data_wa);
		target_free_working_area(target, algorithm_wa);
	}

err:
	if (bin) {
		free(bin);
	}
	if (algorithm_wa) {
		target_free_working_area(target, data_wa);
		target_free_working_area(target, algorithm_wa);
	}
	return retval;
}

FLASH_BANK_COMMAND_HANDLER(custom_flash_bank_command)
{
	struct flash_bank_msg *bank_msg;

	LOG_DEBUG("%s", __func__);

	if (CMD_ARGC < 8) {
		LOG_ERROR("Parameter error:");
		LOG_ERROR("flash bank $FLASHNAME custom 0x20000000 0 0 0 $TARGETNAME 0x10014000 ~/work/riscv.bin [simulation] [sectorsize=]");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	bank_msg = malloc(sizeof(struct flash_bank_msg));
	if (bank_msg == NULL) {
		LOG_ERROR("not enough memory");
		return ERROR_FAIL;
	}

	bank->driver_priv = bank_msg;
	bank_msg->probed = false;
	bank_msg->ctrl_base = 0;
	bank_msg->loader_path = NULL;
	bank_msg->cs = 0;
	bank_msg->buffer = NULL;
	bank_msg->param_0 = 0;
	bank_msg->param_1 = 0;

	COMMAND_PARSE_ADDRESS(CMD_ARGV[6], bank_msg->ctrl_base);
	LOG_DEBUG("ASSUMING CUSTOM device at ctrl_base = " TARGET_ADDR_FMT,
			bank_msg->ctrl_base);
	bank_msg->loader_path = malloc(strlen(CMD_ARGV[7]));
	strcpy((char*)bank_msg->loader_path, CMD_ARGV[7]);
	for (char *p = bank_msg->loader_path; *p; p++) {
		if (*p == '\\')
			*p = '/';
	}
	bank_msg->simulation = false;
	bank_msg->sectorsize = 0;
	for (int i = 8; i < CMD_ARGC; i++) {
		if(strcmp(CMD_ARGV[i], "simulation") == 0) {
			bank_msg->simulation = true;
			LOG_DEBUG("Custom Simulation Mode");
		}
		if(strncmp(CMD_ARGV[i], "sectorsize=", strlen("sectorsize=")) == 0) {
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[i]+strlen("sectorsize="), bank_msg->sectorsize);
			LOG_DEBUG("Custom flash sectorsize is %x", bank_msg->sectorsize);
		}
	}

	return ERROR_OK;
}

static int custom_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	struct flash_bank_msg *bank_msg = bank->driver_priv;

	bank_msg->cs = ERASE_CMD;
	bank_msg->buffer = NULL;
	bank_msg->param_0 = bank->sectors[first].offset;
	bank_msg->param_1 = bank->sectors[last].offset + bank_msg->sectorsize;

	return custom_run_algorithm(bank);
}

static int custom_write(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct flash_bank_msg *bank_msg = bank->driver_priv;

	bank_msg->cs = WRITE_CMD;
	bank_msg->buffer = (uint8_t*)buffer;
	bank_msg->param_0 = count;
	bank_msg->param_1 = offset;

	return custom_run_algorithm(bank);
}

static int custom_read(struct flash_bank *bank, uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct flash_bank_msg *bank_msg = bank->driver_priv;

	bank_msg->cs = READ_CMD;
	bank_msg->buffer = buffer;
	bank_msg->param_0 = count;
	bank_msg->param_1 = offset;

	return custom_run_algorithm(bank);
}

static int custom_probe(struct flash_bank *bank)
{
	struct flash_bank_msg *bank_msg = bank->driver_priv;

	uint32_t id = 0x12345678;
	struct flash_sector *sectors;

	if (bank_msg->probed)
		free(bank->sectors);
	bank_msg->probed = false;

	bank_msg->dev = NULL;
	for (const struct flash_device *p = flash_devices; p->name ; p++) {
		if (p->device_id == id) {
			bank_msg->dev = p;
			break;
		}
	}
	/* Set correct size value */
	bank->size = bank_msg->dev->size_in_bytes;
	/* if no sectors, treat whole bank as single sector */
	if (0 == bank_msg->sectorsize) {
		bank_msg->sectorsize = bank_msg->dev->sectorsize ?
		bank_msg->dev->sectorsize : bank_msg->dev->size_in_bytes;
	}
	/* create and fill sectors array */
	bank->num_sectors = bank_msg->dev->size_in_bytes / bank_msg->sectorsize;
	sectors = malloc(sizeof(struct flash_sector) * bank->num_sectors);
	if (sectors == NULL) {
		LOG_ERROR("not enough memory");
		return ERROR_FAIL;
	}
	for (unsigned int sector = 0; sector < bank->num_sectors; sector++) {
		sectors[sector].offset = sector * bank_msg->sectorsize;
		sectors[sector].size = bank_msg->sectorsize;
		sectors[sector].is_erased = -1;
		sectors[sector].is_protected = 0;
	}
	bank->sectors = sectors;

	bank_msg->cs = PROBE_CMD;
	bank_msg->buffer = NULL;
	bank_msg->param_0 = 0;
	bank_msg->param_1 = 0;

	id = custom_run_algorithm(bank);

	LOG_INFO("Found custom flash device (ID 0x%08" PRIx32 ")", id);
	bank_msg->probed = true;
	return ERROR_OK;
}

static int custom_info(struct flash_bank *bank, struct command_invocation *command)
{
	struct flash_bank_msg *bank_msg = bank->driver_priv;

	if (!(bank_msg->probed)) {
		return ERROR_OK;
	}

	return ERROR_OK;
}

static int custom_auto_probe(struct flash_bank *bank)
{
	struct flash_bank_msg *bank_msg = bank->driver_priv;
	if (bank_msg->probed)
		return ERROR_OK;
	return custom_probe(bank);
}

static int custom_protect(struct flash_bank *bank, int set,
		unsigned int first, unsigned int last)
{
	for (unsigned int sector = first; sector <= last; sector++)
		bank->sectors[sector].is_protected = set;
	return ERROR_OK;
}

static int custom_protect_check(struct flash_bank *bank)
{
	/* Nothing to do. Protection is only handled in SW. */
	return ERROR_OK;
}

const struct flash_driver custom_flash = {
	.name = "custom",
	.flash_bank_command = custom_flash_bank_command,
	.erase = custom_erase,
	.protect = custom_protect,
	.write = custom_write,
	.read = custom_read,
	.probe = custom_probe,
	.auto_probe = custom_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = custom_protect_check,
	.info = custom_info,
	.free_driver_priv = default_flash_free_driver_priv
};

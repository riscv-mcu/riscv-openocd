/***************************************************************************
 *   Copyright (C) 2010 by Yanwen Wang <wangyanwen@nucleisys.com>          *
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

/* This is a dummy flash controller
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include "spi.h"

struct dummy_bank_msg {
	bool probed;
};

FLASH_BANK_COMMAND_HANDLER(dummy_flash_bank_command)
{
	struct dummy_bank_msg *bank_msg;

	LOG_DEBUG("%s", __func__);

	if (CMD_ARGC < 6) {
		LOG_ERROR("Parameter error:");
		LOG_ERROR("flash bank $FLASHNAME dummy 0x20000000 0 0 0 $TARGETNAME");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	bank_msg = malloc(sizeof(struct dummy_bank_msg));
	if (bank_msg == NULL) {
		LOG_ERROR("not enough memory");
		return ERROR_FAIL;
	}

	bank->driver_priv = bank_msg;
	bank_msg->probed = false;

	return ERROR_OK;
}

static int dummy_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	return ERROR_OK;
}

static int dummy_protect(struct flash_bank *bank, int set,
		unsigned int first, unsigned int last)
{
	return ERROR_OK;
}

static int dummy_write(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	return ERROR_OK;
}

static int dummy_probe(struct flash_bank *bank)
{
	struct dummy_bank_msg *bank_msg = bank->driver_priv;
	struct flash_sector *sectors;

	free(bank->sectors);

	if (bank->size == 0)
		bank->size = 0x10000000;

	/* create and fill sectors array */
	uint32_t sectorsize = bank->size;
	bank->num_sectors = bank->size / sectorsize;
	sectors = malloc(sizeof(struct flash_sector) * bank->num_sectors);
	if (sectors == NULL) {
		LOG_ERROR("not enough memory");
		return ERROR_FAIL;
	}
	for (unsigned int sector = 0; sector < bank->num_sectors; sector++) {
		sectors[sector].offset = sector * sectorsize;
		sectors[sector].size = sectorsize;
		sectors[sector].is_erased = -1;
		sectors[sector].is_protected = 0;
	}
	bank->sectors = sectors;

	LOG_INFO("Found dummy flash device (size %d bytes)", bank->size);
	bank_msg->probed = true;
	return ERROR_OK;
}

static int dummy_auto_probe(struct flash_bank *bank)
{
	struct dummy_bank_msg *bank_msg = bank->driver_priv;

	if (bank_msg->probed) {
		return ERROR_OK;
	}

	return dummy_probe(bank);
}

static int dummy_protect_check(struct flash_bank *bank)
{
	return ERROR_OK;
}

const struct flash_driver dummy_flash = {
	.name = "dummy",
	.flash_bank_command = dummy_flash_bank_command,
	.erase = dummy_erase,
	.protect = dummy_protect,
	.write = dummy_write,
	.read = default_flash_read,
	.probe = dummy_probe,
	.auto_probe = dummy_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = dummy_protect_check,
	.free_driver_priv = default_flash_free_driver_priv
};


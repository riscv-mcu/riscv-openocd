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

/* The Nuclei SPI controller is a SPI bus controller
 * specifically designed for SPI Flash Memories on Nuclei RISC-V platforms.
 *
 * Two working modes are available:
 * - SW mode: the SPI is controlled by SW. Any custom commands can be sent
 *   on the bus. Writes are only possible in this mode.
 * - HW mode: Memory content is directly
 *   accessible in CPU memory space. CPU can read and execute memory content.
 */

/* ATTENTION:
 * To have flash memory mapped in CPU memory space, the controller
 * must have "HW mode" enabled.
 * 1) The command "reset init" has to initialize the controller and put
 *    it in HW mode (this is actually the default out of reset for Freedom E systems).
 * 2) every command in this file have to return to prompt in HW mode. */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include "spi.h"
#include <jtag/jtag.h>
#include <helper/time_support.h>
#include <target/algorithm.h>
#include "target/riscv/riscv.h"

/* Register offsets */

#define NUSPI_REG_SCKDIV			(0x00)
#define NUSPI_REG_SCKMODE			(0x04)
#define NUSPI_REG_SCKSAMPLE			(0x08)
#define NUSPI_REG_FORCE				(0x0C)
#define NUSPI_REG_CSID				(0x10)
#define NUSPI_REG_CSDEF				(0x14)
#define NUSPI_REG_CSMODE			(0x18)
#define NUSPI_REG_VERSION			(0x1C)

#define NUSPI_REG_DCSSCK			(0x28)
#define NUSPI_REG_DSCKCS			(0x2a)
#define NUSPI_REG_DINTERCS			(0x2c)
#define NUSPI_REG_DINTERXFR			(0x2e)

#define NUSPI_REG_FMT				(0x40)

#define NUSPI_REG_TXDATA			(0x48)
#define NUSPI_REG_RXDATA			(0x4C)
#define NUSPI_REG_TXMARK			(0x50)
#define NUSPI_REG_RXMARK			(0x54)

#define NUSPI_REG_FCTRL				(0x60)
#define NUSPI_REG_FFMT				(0x64)

#define NUSPI_REG_IE				(0x70)
#define NUSPI_REG_IP				(0x74)
#define NUSPI_REG_FFMT1				(0x78)
#define NUSPI_REG_STATUS			(0x7C)
#define NUSPI_REG_RXEDGE			(0x80)
#define NUSPI_REG_CR				(0x84)

/* Fields */

#define NUSPI_SCK_POL				(0x1)
#define NUSPI_SCK_PHA				(0x2)

#define NUSPI_FMT_PROTO(x)			((x) & 0x3)
#define NUSPI_FMT_ENDIAN(x)			(((x) & 0x1) << 2)
#define NUSPI_FMT_DIR(x)			(((x) & 0x1) << 3)
#define NUSPI_FMT_LEN(x)			(((x) & 0xf) << 16)//TODO:

/* TXMARK register */
#define NUSPI_TXWM(x)				((x) & 0xFFFF)//TODO:
/* RXMARK register */
#define NUSPI_RXWM(x)				((x) & 0xFFFF)//TODO:

#define NUSPI_IP_TXWM				(0x1)
#define NUSPI_IP_RXWM				(0x2)

#define NUSPI_FCTRL_EN				(0x1)

#define NUSPI_INSN_CMD_EN			(0x1)
#define NUSPI_INSN_ADDR_LEN(x)		(((x) & 0x7) << 1)
#define NUSPI_INSN_PAD_CNT(x)		(((x) & 0xf) << 4)
#define NUSPI_INSN_CMD_PROTO(x)		(((x) & 0x3) << 8)
#define NUSPI_INSN_ADDR_PROTO(x)	(((x) & 0x3) << 10)
#define NUSPI_INSN_DATA_PROTO(x)	(((x) & 0x3) << 12)
#define NUSPI_INSN_CMD_CODE(x)		(((x) & 0xff) << 16)
#define NUSPI_INSN_PAD_CODE(x)		(((x) & 0xff) << 24)

#define NUSPI_STAT_BUSY				(0x1 << 0)
#define NUSPI_STAT_TXFULL			(0x1 << 4)
#define NUSPI_STAT_RXEMPTY			(0x1 << 5)

/* Values */
#define SPIFLASH_3BYTE_FAST_READ	(0x03)
#define SPIFLASH_4BYTE_FAST_READ	(0x13)
#define SPIFLASH_ENTER_4BYTE		(0xB7)
#define SPIFLASH_EXIT_4BYTE			(0xE9)
#define SPIFLASH_ENABLE_RESET		(0x66)
#define SPIFLASH_RESET_DEVICE		(0x99)
#define SPIFLASH_WRITE_STATUS1		(0x01)

#define NUSPI_CSMODE_AUTO			(0)
#define NUSPI_CSMODE_HOLD			(2)
#define NUSPI_CSMODE_OFF			(3)

#define NUSPI_DIR_RX				(0)
#define NUSPI_DIR_TX				(1)

#define NUSPI_PROTO_S				(0)
#define NUSPI_PROTO_D				(1)
#define NUSPI_PROTO_Q				(2)

#define NUSPI_ENDIAN_MSB			(0)
#define NUSPI_ENDIAN_LSB			(1)

/* Timeout in ms */
#define NUSPI_CMD_TIMEOUT			(100)
#define NUSPI_PROBE_TIMEOUT			(100)
#define NUSPI_MAX_TIMEOUT			(3000)
#define NUSPI_SIM_TIMEOUT			(0x7FFFFFFF)

#define NUSPI_FLAGS_32B_DAT 		(1 << 0)

struct nuspi_flash_bank {
	uint32_t version;
	int nuspi_flags;
	bool probed;
	target_addr_t ctrl_base;
	const struct flash_device *dev;
	bool simulation;
};

struct nuspi_target {
	char *name;
	uint32_t tap_idcode;
	uint32_t tap_idmask;
	uint32_t ctrl_base;
};

/* TODO !!! What is the right naming convention here? */
static const struct nuspi_target target_devices[] = {
	/* name,   tap_idcode, ctrl_base */
	{ "Freedom E310-G000 SPI Flash", 0x10e31913, 0xFFFFFFFF, 0x10014000 },
	{ "Nuclei SoC SPI Flash", 0x00000a6d, 0xFFF, 0x10014000},
	{ NULL, 0, 0, 0}
};

FLASH_BANK_COMMAND_HANDLER(nuspi_flash_bank_command)
{
	struct nuspi_flash_bank *nuspi_info;

	LOG_DEBUG("%s", __func__);

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	nuspi_info = malloc(sizeof(struct nuspi_flash_bank));
	if (nuspi_info == NULL) {
		LOG_ERROR("not enough memory");
		return ERROR_FAIL;
	}

	bank->driver_priv = nuspi_info;
	nuspi_info->probed = false;
	nuspi_info->ctrl_base = 0;
	if (CMD_ARGC >= 7) {
		COMMAND_PARSE_ADDRESS(CMD_ARGV[6], nuspi_info->ctrl_base);
		LOG_DEBUG("ASSUMING NUSPI device at ctrl_base = " TARGET_ADDR_FMT,
				nuspi_info->ctrl_base);
	}
	nuspi_info->simulation = false;
	if (CMD_ARGC >= 8) {
		if(strcmp(CMD_ARGV[7], "simulation") == 0) {
			nuspi_info->simulation = true;
			LOG_DEBUG("Nuspi Simulation Mode");
		}
	}

	return ERROR_OK;
}

static int nuspi_read_reg(struct flash_bank *bank, uint32_t *value, target_addr_t address)
{
	struct target *target = bank->target;
	struct nuspi_flash_bank *nuspi_info = bank->driver_priv;

	int result = target_read_u32(target, nuspi_info->ctrl_base + address, value);
	if (result != ERROR_OK) {
		LOG_ERROR("nuspi_read_reg() error at " TARGET_ADDR_FMT,
				nuspi_info->ctrl_base + address);
		return result;
	}
	return ERROR_OK;
}

static int nuspi_write_reg(struct flash_bank *bank, target_addr_t address, uint32_t value)
{
	struct target *target = bank->target;
	struct nuspi_flash_bank *nuspi_info = bank->driver_priv;

	int result = target_write_u32(target, nuspi_info->ctrl_base + address, value);
	if (result != ERROR_OK) {
		LOG_ERROR("nuspi_write_reg() error writing 0x%" PRIx32 " to " TARGET_ADDR_FMT,
				value, nuspi_info->ctrl_base + address);
		return result;
	}
	return ERROR_OK;
}

static int nuspi_disable_hw_mode(struct flash_bank *bank)
{
	uint32_t fctrl;
	if (nuspi_read_reg(bank, &fctrl, NUSPI_REG_FCTRL) != ERROR_OK)
		return ERROR_FAIL;
	return nuspi_write_reg(bank, NUSPI_REG_FCTRL, fctrl & ~NUSPI_FCTRL_EN);
}

static int nuspi_enable_hw_mode(struct flash_bank *bank)
{
	uint32_t fctrl;
	if (nuspi_read_reg(bank, &fctrl, NUSPI_REG_FCTRL) != ERROR_OK)
		return ERROR_FAIL;
	return nuspi_write_reg(bank, NUSPI_REG_FCTRL, fctrl | NUSPI_FCTRL_EN);
}

static int nuspi_set_dir(struct flash_bank *bank, bool dir)
{
	uint32_t fmt;
	if (nuspi_read_reg(bank, &fmt, NUSPI_REG_FMT) != ERROR_OK)
		return ERROR_FAIL;

	return nuspi_write_reg(bank, NUSPI_REG_FMT,
			(fmt & ~(NUSPI_FMT_DIR(0xFFFFFFFF))) | NUSPI_FMT_DIR(dir));
}

static int nuspi_txwm_wait(struct flash_bank *bank)
{
	int64_t start = timeval_ms();
	struct nuspi_flash_bank *nuspi_info = bank->driver_priv;
	if(nuspi_info->nuspi_flags & NUSPI_FLAGS_32B_DAT) {
		while (1) {
			uint32_t status;
			if (nuspi_read_reg(bank, &status, NUSPI_REG_STATUS) != ERROR_OK)
				return ERROR_FAIL;
			if (0 == (status & NUSPI_STAT_BUSY))
				break;
			int64_t now = timeval_ms();
			if (now - start > 1000) {
				LOG_ERROR("ip.txwm didn't get set.");
				return ERROR_TARGET_TIMEOUT;
			}
		}
	} else {
		while (1) {
			uint32_t ip;
			if (nuspi_read_reg(bank, &ip, NUSPI_REG_IP) != ERROR_OK)
				return ERROR_FAIL;
			if (ip & NUSPI_IP_TXWM)
				break;
			int64_t now = timeval_ms();
			if (now - start > 1000) {
				LOG_ERROR("ip.txwm didn't get set.");
				return ERROR_TARGET_TIMEOUT;
			}
		}
	}

	return ERROR_OK;
}

static int nuspi_tx(struct flash_bank *bank, uint8_t in)
{
	int64_t start = timeval_ms();
	struct nuspi_flash_bank *nuspi_info = bank->driver_priv;
	if(nuspi_info->nuspi_flags & NUSPI_FLAGS_32B_DAT) {
		while (1) {
			uint32_t status;
			if (nuspi_read_reg(bank, &status, NUSPI_REG_STATUS) != ERROR_OK)
				return ERROR_FAIL;
			if (!(status & NUSPI_STAT_TXFULL))
				break;
			int64_t now = timeval_ms();
			if (now - start > 1000) {
				LOG_ERROR("txfifo stayed negative.");
				return ERROR_TARGET_TIMEOUT;
			}
		}
	} else {
		while (1) {
			uint32_t txfifo;
			if (nuspi_read_reg(bank, &txfifo, NUSPI_REG_TXDATA) != ERROR_OK)
				return ERROR_FAIL;
			if (!(txfifo >> 31))
				break;
			int64_t now = timeval_ms();
			if (now - start > 1000) {
				LOG_ERROR("txfifo stayed negative.");
				return ERROR_TARGET_TIMEOUT;
			}
		}
	}
	return nuspi_write_reg(bank, NUSPI_REG_TXDATA, in);
}

static int nuspi_rx(struct flash_bank *bank, uint8_t *out)
{
	int64_t start = timeval_ms();
	uint32_t value;
	struct nuspi_flash_bank *nuspi_info = bank->driver_priv;
	if(nuspi_info->nuspi_flags & NUSPI_FLAGS_32B_DAT) {
		while (1) {
			if (nuspi_read_reg(bank, &value, NUSPI_REG_STATUS) != ERROR_OK)
				return ERROR_FAIL;
			if (!(value & NUSPI_STAT_RXEMPTY))
				break;
			int64_t now = timeval_ms();
			if (now - start > 1000) {
				LOG_ERROR("rxfifo didn't go positive (value=0x%x).", value);
				return ERROR_TARGET_TIMEOUT;
			}
		}
		if (nuspi_read_reg(bank, &value, NUSPI_REG_RXDATA) != ERROR_OK)
			return ERROR_FAIL;
	} else {
		while (1) {
			if (nuspi_read_reg(bank, &value, NUSPI_REG_RXDATA) != ERROR_OK)
				return ERROR_FAIL;
			if (!(value >> 31))
				break;
			int64_t now = timeval_ms();
			if (now - start > 1000) {
				LOG_ERROR("rxfifo didn't go positive (value=0x%" PRIx32 ").", value);
				return ERROR_TARGET_TIMEOUT;
			}
		}
	}
	if (out)
		*out = value & 0xff;

	return ERROR_OK;
}

/* TODO!!! Why don't we need to call this after writing? */
static int nuspi_wip(struct flash_bank *bank, int timeout)
{
	int64_t endtime;

	nuspi_set_dir(bank, NUSPI_DIR_RX);

	if (nuspi_write_reg(bank, NUSPI_REG_CSMODE, NUSPI_CSMODE_HOLD) != ERROR_OK)
		return ERROR_FAIL;
	endtime = timeval_ms() + timeout;

	nuspi_tx(bank, SPIFLASH_READ_STATUS);
	if (nuspi_rx(bank, NULL) != ERROR_OK)
		return ERROR_FAIL;
	do {
		alive_sleep(1);
		nuspi_tx(bank, 0);
		uint8_t rx;
		if (nuspi_rx(bank, &rx) != ERROR_OK)
			return ERROR_FAIL;
		if ((rx & SPIFLASH_BSY_BIT) == 0) {
			if (nuspi_write_reg(bank, NUSPI_REG_CSMODE, NUSPI_CSMODE_AUTO) != ERROR_OK)
				return ERROR_FAIL;
			nuspi_set_dir(bank, NUSPI_DIR_TX);
			return ERROR_OK;
		}
	} while (timeval_ms() < endtime);

	LOG_ERROR("timeout");
	return ERROR_FAIL;
}

static int nuspi_reset(struct flash_bank *bank)
{
	/* Clear Rx FIFO */
	uint32_t value;
	struct nuspi_flash_bank *nuspi_info = bank->driver_priv;
	if(nuspi_info->nuspi_flags & NUSPI_FLAGS_32B_DAT) {
		while (1) {
			if (nuspi_read_reg(bank, &value, NUSPI_REG_STATUS) != ERROR_OK)
				return ERROR_FAIL;
			if (value & NUSPI_STAT_RXEMPTY)
				break;
			if (nuspi_read_reg(bank, &value, NUSPI_REG_RXDATA) != ERROR_OK)
				return ERROR_FAIL;
		}
	} else {
		while (1) {
			if (nuspi_read_reg(bank, &value, NUSPI_REG_RXDATA) != ERROR_OK)
				return ERROR_FAIL;
			if (value >> 31)
				break;
		}
	}
	if (nuspi_write_reg(bank, NUSPI_REG_SCKMODE, 0x0) != ERROR_OK)
		return ERROR_FAIL;
	if (nuspi_write_reg(bank, NUSPI_REG_FORCE, 0x3) != ERROR_OK)
		return ERROR_FAIL;
	if (nuspi_write_reg(bank, NUSPI_REG_FCTRL, 0x0) != ERROR_OK)
		return ERROR_FAIL;
	if (nuspi_write_reg(bank, NUSPI_REG_FMT, 0x80008) != ERROR_OK)
		return ERROR_FAIL;
	if (nuspi_write_reg(bank, NUSPI_REG_FFMT, 0x30007) != ERROR_OK)
		return ERROR_FAIL;
	if (nuspi_write_reg(bank, NUSPI_REG_RXEDGE, 0x0) != ERROR_OK)
		return ERROR_FAIL;
	return ERROR_OK;
}

static int nuspi_set_xip_read_cmd(struct flash_bank *bank)
{
	uint32_t temp;
	struct nuspi_flash_bank *nuspi_info = bank->driver_priv;
	if (nuspi_read_reg(bank, &temp, NUSPI_REG_FFMT) != ERROR_OK)
		return ERROR_FAIL;
	temp &= ~(0x7 << 1);
	temp &= ~(0xF << 4);
	temp &= ~(0xFF << 16);
	temp |= NUSPI_INSN_PAD_CNT(0);
	if (bank->size  > 0x1000000) {
		temp |= NUSPI_INSN_ADDR_LEN(4);
	} else {
		temp |= NUSPI_INSN_ADDR_LEN(3);
	}
	if ((bank->size  > 0x1000000) & (nuspi_info->dev->erase_cmd == 0xd8)) {
		temp |= NUSPI_INSN_CMD_CODE(SPIFLASH_4BYTE_FAST_READ);
	} else {
		temp |= NUSPI_INSN_CMD_CODE(nuspi_info->dev->read_cmd);
	}
	return nuspi_write_reg(bank, NUSPI_REG_FFMT, temp);
}

static int flash_reset(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct nuspi_flash_bank *nuspi_info = bank->driver_priv;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}
	/* Send SPI command "66h" */
	if (nuspi_write_reg(bank, NUSPI_REG_CSMODE, NUSPI_CSMODE_HOLD) != ERROR_OK)
		return ERROR_FAIL;

	nuspi_tx(bank, SPIFLASH_ENABLE_RESET);
	if (nuspi_txwm_wait(bank) != ERROR_OK)
		return ERROR_FAIL;

	if (nuspi_write_reg(bank, NUSPI_REG_CSMODE, NUSPI_CSMODE_AUTO) != ERROR_OK)
		return ERROR_FAIL;

	/* Send SPI command "99h" */
	if (nuspi_write_reg(bank, NUSPI_REG_CSMODE, NUSPI_CSMODE_HOLD) != ERROR_OK)
		return ERROR_FAIL;

	nuspi_tx(bank, SPIFLASH_RESET_DEVICE);
	if (nuspi_txwm_wait(bank) != ERROR_OK)
		return ERROR_FAIL;

	if (nuspi_write_reg(bank, NUSPI_REG_CSMODE, NUSPI_CSMODE_AUTO) != ERROR_OK)
		return ERROR_FAIL;

	if (nuspi_info->simulation) {
		if (nuspi_wip(bank, NUSPI_SIM_TIMEOUT) != ERROR_OK)
			return ERROR_FAIL;
	} else {
		if (nuspi_wip(bank, NUSPI_MAX_TIMEOUT) != ERROR_OK)
			return ERROR_FAIL;
	}

	//unlock protect
	/* Send SPI command "01h" */
	if (nuspi_write_reg(bank, NUSPI_REG_CSMODE, NUSPI_CSMODE_HOLD) != ERROR_OK)
		return ERROR_FAIL;

	nuspi_tx(bank, SPIFLASH_WRITE_STATUS1);
	nuspi_tx(bank, 0x00);
	if (nuspi_txwm_wait(bank) != ERROR_OK)
		return ERROR_FAIL;

	if (nuspi_write_reg(bank, NUSPI_REG_CSMODE, NUSPI_CSMODE_AUTO) != ERROR_OK)
		return ERROR_FAIL;

	return ERROR_OK;
}

static int nuspi_enter_4byte_address_mode(struct flash_bank *bank)
{
	int retval = ERROR_OK;
	struct target *target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* Send SPI command "enter 4byte" */
	if (nuspi_write_reg(bank, NUSPI_REG_CSMODE, NUSPI_CSMODE_HOLD) != ERROR_OK)
		return ERROR_FAIL;

	retval = nuspi_tx(bank, SPIFLASH_ENTER_4BYTE);
	if (retval != ERROR_OK)
		return retval;

	if (nuspi_write_reg(bank, NUSPI_REG_CSMODE, NUSPI_CSMODE_AUTO) != ERROR_OK)
		return ERROR_FAIL;

	return ERROR_OK;
}

static int nuspi_exit_4byte_address_mode(struct flash_bank *bank)
{
	int retval = ERROR_OK;
	struct target *target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* Send SPI command "enter 4byte" */
	if (nuspi_write_reg(bank, NUSPI_REG_CSMODE, NUSPI_CSMODE_HOLD) != ERROR_OK)
		return ERROR_FAIL;

	retval = nuspi_tx(bank, SPIFLASH_EXIT_4BYTE);
	if (retval != ERROR_OK)
		return retval;

	if (nuspi_write_reg(bank, NUSPI_REG_CSMODE, NUSPI_CSMODE_AUTO) != ERROR_OK)
		return ERROR_FAIL;

	return ERROR_OK;
}

static int nuspi_erase_sector(struct flash_bank *bank, int sector)
{
	struct nuspi_flash_bank *nuspi_info = bank->driver_priv;
	int retval = ERROR_OK;

	retval = nuspi_tx(bank, SPIFLASH_WRITE_ENABLE);
	if (retval != ERROR_OK)
		return retval;
	retval = nuspi_txwm_wait(bank);
	if (retval != ERROR_OK)
		return retval;

	if (nuspi_write_reg(bank, NUSPI_REG_CSMODE, NUSPI_CSMODE_HOLD) != ERROR_OK)
		return ERROR_FAIL;
	retval = nuspi_tx(bank, nuspi_info->dev->erase_cmd);
	if (retval != ERROR_OK)
		return retval;
	sector = bank->sectors[sector].offset;
	if (bank->size > 0x1000000) {
		retval = nuspi_tx(bank, sector >> 24);
		if (retval != ERROR_OK)
			return retval;
	}
	retval = nuspi_tx(bank, sector >> 16);
	if (retval != ERROR_OK)
		return retval;
	retval = nuspi_tx(bank, sector >> 8);
	if (retval != ERROR_OK)
		return retval;
	retval = nuspi_tx(bank, sector);
	if (retval != ERROR_OK)
		return retval;
	retval = nuspi_txwm_wait(bank);
	if (retval != ERROR_OK)
		return retval;
	if (nuspi_write_reg(bank, NUSPI_REG_CSMODE, NUSPI_CSMODE_AUTO) != ERROR_OK)
		return ERROR_FAIL;

	if (nuspi_info->simulation) {
		retval = nuspi_wip(bank, NUSPI_SIM_TIMEOUT);
	} else {
		retval = nuspi_wip(bank, NUSPI_MAX_TIMEOUT);
	}
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int nuspi_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	struct target *target = bank->target;
	struct nuspi_flash_bank *nuspi_info = bank->driver_priv;
	int retval = ERROR_OK;

	LOG_DEBUG("%s: from sector %u to sector %u", __func__, first, last);

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if ((last < first) || (last >= bank->num_sectors)) {
		LOG_ERROR("Flash sector invalid");
		return ERROR_FLASH_SECTOR_INVALID;
	}

	if (!(nuspi_info->probed)) {
		LOG_ERROR("Flash bank not probed");
		return ERROR_FLASH_BANK_NOT_PROBED;
	}

	for (unsigned int sector = first; sector <= last; sector++) {
		if (bank->sectors[sector].is_protected) {
			LOG_ERROR("Flash sector %u protected", sector);
			return ERROR_FAIL;
		}
	}

	if (nuspi_info->dev->erase_cmd == 0x00)
		return ERROR_FLASH_OPER_UNSUPPORTED;

	if (nuspi_reset(bank) != ERROR_OK)
		return ERROR_FAIL;
	if (nuspi_write_reg(bank, NUSPI_REG_TXMARK, NUSPI_TXWM(1)) != ERROR_OK)
		return ERROR_FAIL;
	retval = nuspi_txwm_wait(bank);
	if (retval != ERROR_OK) {
		LOG_ERROR("WM Didn't go high before attempting.");
		return retval;
	}

	/* Disable Hardware accesses*/
	if (nuspi_disable_hw_mode(bank) != ERROR_OK)
		return ERROR_FAIL;

	/* poll WIP */
	if (nuspi_info->simulation) {
		retval = nuspi_wip(bank, NUSPI_SIM_TIMEOUT);
	} else {
		retval = nuspi_wip(bank, NUSPI_MAX_TIMEOUT);
	}
	if (retval != ERROR_OK)
		goto done;

	if ((bank->size  > 0x1000000) & (nuspi_info->dev->erase_cmd == 0xd8)) {
		retval = nuspi_enter_4byte_address_mode(bank);
		if (retval != ERROR_OK)
			return retval;
	}

	for (unsigned int sector = first; sector <= last; sector++) {
		retval = nuspi_erase_sector(bank, sector);
		if (retval != ERROR_OK)
			goto done;
		keep_alive();
	}

	/* Switch to HW mode before return to prompt */
done:
	if (nuspi_enable_hw_mode(bank) != ERROR_OK)
		return ERROR_FAIL;

	if (nuspi_set_xip_read_cmd(bank) != ERROR_OK)
		return ERROR_FAIL;

	return retval;
}

static int nuspi_protect(struct flash_bank *bank, int set,
		unsigned int first, unsigned int last)
{
	for (unsigned int sector = first; sector <= last; sector++)
		bank->sectors[sector].is_protected = set;
	return ERROR_OK;
}

static int slow_nuspi_write_buffer(struct flash_bank *bank,
		const uint8_t *buffer, uint32_t offset, uint32_t len)
{
	struct nuspi_flash_bank *nuspi_info = bank->driver_priv;
	uint32_t ii;

	/* TODO!!! assert that len < page size */

	if (nuspi_tx(bank, SPIFLASH_WRITE_ENABLE) != ERROR_OK)
		return ERROR_FAIL;
	if (nuspi_txwm_wait(bank) != ERROR_OK)
		return ERROR_FAIL;

	if (nuspi_write_reg(bank, NUSPI_REG_CSMODE, NUSPI_CSMODE_HOLD) != ERROR_OK)
		return ERROR_FAIL;

	if (nuspi_tx(bank, nuspi_info->dev->pprog_cmd) != ERROR_OK)
		return ERROR_FAIL;

	if (bank->size > 0x1000000 && nuspi_tx(bank, offset >> 24) != ERROR_OK)
		return ERROR_FAIL;
	if (nuspi_tx(bank, offset >> 16) != ERROR_OK)
		return ERROR_FAIL;
	if (nuspi_tx(bank, offset >> 8) != ERROR_OK)
		return ERROR_FAIL;
	if (nuspi_tx(bank, offset) != ERROR_OK)
		return ERROR_FAIL;

	for (ii = 0; ii < len; ii++) {
		if (nuspi_tx(bank, buffer[ii]) != ERROR_OK)
			return ERROR_FAIL;
	}

	if (nuspi_txwm_wait(bank) != ERROR_OK)
		return ERROR_FAIL;

	if (nuspi_write_reg(bank, NUSPI_REG_CSMODE, NUSPI_CSMODE_AUTO) != ERROR_OK)
		return ERROR_FAIL;

	keep_alive();

	return ERROR_OK;
}

static const uint8_t riscv32_bin[] = {
#include "../../../contrib/loaders/flash/nuspi/riscv32_nuspi.inc"
};

static const uint8_t riscv64_bin[] = {
#include "../../../contrib/loaders/flash/nuspi/riscv64_nuspi.inc"
};

static int nuspi_write(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	struct nuspi_flash_bank *nuspi_info = bank->driver_priv;
	uint32_t cur_count, page_size;
	int retval = ERROR_OK;

	LOG_DEBUG("bank->size=0x%x offset=0x%08" PRIx32 " count=0x%08" PRIx32,
			bank->size, offset, count);

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (offset + count > nuspi_info->dev->size_in_bytes) {
		LOG_WARNING("Write past end of flash. Extra data discarded.");
		count = nuspi_info->dev->size_in_bytes - offset;
	}

	/* Check sector protection */
	for (unsigned int sector = 0; sector < bank->num_sectors; sector++) {
		/* Start offset in or before this sector? */
		/* End offset in or behind this sector? */
		if ((offset <
					(bank->sectors[sector].offset + bank->sectors[sector].size))
				&& ((offset + count - 1) >= bank->sectors[sector].offset)
				&& bank->sectors[sector].is_protected) {
			LOG_ERROR("Flash sector %u protected", sector);
			return ERROR_FAIL;
		}
	}

	int xlen = riscv_xlen(target);
	struct working_area *algorithm_wa = NULL;
	struct working_area *data_wa = NULL;
	const uint8_t *bin;
	size_t bin_size;
	if (xlen == 32) {
		bin = riscv32_bin;
		bin_size = sizeof(riscv32_bin);
	} else {
		bin = riscv64_bin;
		bin_size = sizeof(riscv64_bin);
	}

	unsigned data_wa_size = 0;
	if (target_alloc_working_area(target, bin_size, &algorithm_wa) == ERROR_OK) {
		retval = target_write_buffer(target, algorithm_wa->address,
				bin_size, bin);
		if (retval != ERROR_OK) {
			LOG_ERROR("Failed to write code to " TARGET_ADDR_FMT ": %d",
					algorithm_wa->address, retval);
			target_free_working_area(target, algorithm_wa);
			algorithm_wa = NULL;

		} else {
			data_wa_size = MIN(target->working_area_size - algorithm_wa->size, count);
			while (1) {
				if (data_wa_size < 128) {
					LOG_WARNING("Couldn't allocate data working area.");
					target_free_working_area(target, algorithm_wa);
					algorithm_wa = NULL;
				}
				if (target_alloc_working_area_try(target, data_wa_size, &data_wa) ==
						ERROR_OK) {
					break;
				}

				data_wa_size = data_wa_size * 3 / 4;
			}
		}
	} else {
		LOG_WARNING("Couldn't allocate %zd-byte working area.", bin_size);
		algorithm_wa = NULL;
	}

	/* If no valid page_size, use reasonable default. */
	page_size = nuspi_info->dev->pagesize ?
		nuspi_info->dev->pagesize : SPIFLASH_DEF_PAGESIZE;

	/* Disable Hardware accesses*/
	if (nuspi_disable_hw_mode(bank) != ERROR_OK)
		return ERROR_FAIL;

	if (algorithm_wa) {
		struct reg_param reg_params[6];
		init_reg_param(&reg_params[0], "a0", xlen, PARAM_IN_OUT);
		init_reg_param(&reg_params[1], "a1", xlen, PARAM_OUT);
		init_reg_param(&reg_params[2], "a2", xlen, PARAM_OUT);
		init_reg_param(&reg_params[3], "a3", xlen, PARAM_OUT);
		init_reg_param(&reg_params[4], "a4", xlen, PARAM_OUT);
		init_reg_param(&reg_params[5], "a5", xlen, PARAM_OUT);

		while (count > 0) {
			cur_count = MIN(count, data_wa_size);
			buf_set_u64(reg_params[0].value, 0, xlen, nuspi_info->ctrl_base);
			buf_set_u64(reg_params[1].value, 0, xlen, page_size);
			buf_set_u64(reg_params[2].value, 0, xlen, data_wa->address);
			buf_set_u64(reg_params[3].value, 0, xlen, offset);
			buf_set_u64(reg_params[4].value, 0, xlen, cur_count);
			buf_set_u64(reg_params[5].value, 0, xlen,
					nuspi_info->dev->pprog_cmd | (bank->size > 0x1000000 ? 0x100 : 0));

			retval = target_write_buffer(target, data_wa->address, cur_count,
					buffer);
			if (retval != ERROR_OK) {
				LOG_DEBUG("Failed to write %d bytes to " TARGET_ADDR_FMT ": %d",
						cur_count, data_wa->address, retval);
				goto err;
			}

			LOG_DEBUG("write(ctrl_base=0x%" TARGET_PRIxADDR ", page_size=0x%x, "
					"address=0x%" TARGET_PRIxADDR ", offset=0x%" PRIx32
					", count=0x%" PRIx32 "), buffer=%02x %02x %02x %02x %02x %02x ..." PRIx32,
					nuspi_info->ctrl_base, page_size, data_wa->address, offset, cur_count,
					buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5]);
			if (nuspi_info->simulation) {
				retval = target_run_algorithm(target, 0, NULL,
						ARRAY_SIZE(reg_params), reg_params,
						algorithm_wa->address, 0, NUSPI_SIM_TIMEOUT, NULL);
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

			int algorithm_result = buf_get_u64(reg_params[0].value, 0, xlen);
			if (algorithm_result != 0) {
				LOG_ERROR("Algorithm returned error %d", algorithm_result);
				retval = ERROR_FAIL;
				goto err;
			}

			buffer += cur_count;
			offset += cur_count;
			count -= cur_count;
		}

		target_free_working_area(target, data_wa);
		target_free_working_area(target, algorithm_wa);

	} else {
		nuspi_txwm_wait(bank);

		/* poll WIP */
		if (nuspi_info->simulation) {
			retval = nuspi_wip(bank, NUSPI_SIM_TIMEOUT);
		} else {
			retval = nuspi_wip(bank, NUSPI_PROBE_TIMEOUT);
		}
		if (retval != ERROR_OK)
			goto err;

		uint32_t page_offset = offset % page_size;
		/* central part, aligned words */
		while (count > 0) {
			/* clip block at page boundary */
			if (page_offset + count > page_size)
				cur_count = page_size - page_offset;
			else
				cur_count = count;

			retval = slow_nuspi_write_buffer(bank, buffer, offset, cur_count);
			if (retval != ERROR_OK)
				goto err;

			page_offset = 0;
			buffer += cur_count;
			offset += cur_count;
			count -= cur_count;
		}
	}

err:
	if (algorithm_wa) {
		target_free_working_area(target, data_wa);
		target_free_working_area(target, algorithm_wa);
	}

	if ((bank->size  > 0x1000000) & (nuspi_info->dev->erase_cmd == 0xd8)) {
		if (nuspi_exit_4byte_address_mode(bank) != ERROR_OK)
			return ERROR_FAIL;
	}
	/* Switch to HW mode before return to prompt */
	if (nuspi_enable_hw_mode(bank) != ERROR_OK)
		return ERROR_FAIL;

	if (nuspi_set_xip_read_cmd(bank) != ERROR_OK)
		return ERROR_FAIL;

	return retval;
}

static int nuspi_read(struct flash_bank *bank, uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	if (nuspi_set_xip_read_cmd(bank) != ERROR_OK)
		return ERROR_FAIL;
	return default_flash_read(bank, buffer, offset, count);
}

/* Return ID of flash device */
/* On exit, SW mode is kept */
static int nuspi_read_flash_id(struct flash_bank *bank, uint32_t *id)
{
	struct target *target = bank->target;
	int retval = ERROR_OK;
	struct nuspi_flash_bank *nuspi_info = bank->driver_priv;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	nuspi_txwm_wait(bank);

	/* poll WIP */
	if (nuspi_info->simulation) {
		retval = nuspi_wip(bank, NUSPI_SIM_TIMEOUT);
	} else {
		retval = nuspi_wip(bank, NUSPI_PROBE_TIMEOUT);
	}
	if (retval != ERROR_OK)
		return retval;

	nuspi_set_dir(bank, NUSPI_DIR_RX);

	/* Send SPI command "read ID" */
	if (nuspi_write_reg(bank, NUSPI_REG_CSMODE, NUSPI_CSMODE_HOLD) != ERROR_OK)
		return ERROR_FAIL;

	nuspi_tx(bank, SPIFLASH_READ_ID);
	/* Send dummy bytes to actually read the ID.*/
	nuspi_tx(bank, 0);
	nuspi_tx(bank, 0);
	nuspi_tx(bank, 0);

	/* read ID from Receive Register */
	*id = 0;
	if (nuspi_rx(bank, NULL) != ERROR_OK)
		return ERROR_FAIL;
	uint8_t rx;
	if (nuspi_rx(bank, &rx) != ERROR_OK)
		return ERROR_FAIL;
	*id = rx;
	if (nuspi_rx(bank, &rx) != ERROR_OK)
		return ERROR_FAIL;
	*id |= (rx << 8);
	if (nuspi_rx(bank, &rx) != ERROR_OK)
		return ERROR_FAIL;
	*id |= (rx << 16);

	if (nuspi_write_reg(bank, NUSPI_REG_CSMODE, NUSPI_CSMODE_AUTO) != ERROR_OK)
		return ERROR_FAIL;

	nuspi_set_dir(bank, NUSPI_DIR_TX);

	return ERROR_OK;
}

static int gdb_flash_write_end_callback(struct target *target,
										enum target_event event, void *priv)
{
	if (TARGET_EVENT_GDB_FLASH_WRITE_END == event)
	{
		if (nuspi_set_xip_read_cmd(priv) != ERROR_OK)
			return ERROR_FAIL;
	}
	return ERROR_OK;
}

static int nuspi_probe(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct nuspi_flash_bank *nuspi_info = bank->driver_priv;
	struct flash_sector *sectors;
	uint32_t id = 0; /* silence uninitialized warning */
	const struct nuspi_target *target_device;
	int retval = ERROR_OK;
	uint32_t sectorsize;

	if (nuspi_info->probed)
		free(bank->sectors);
	nuspi_info->probed = false;

	for (target_device = target_devices ; target_device->name ; ++target_device)
		/* NUCLEI: Special handling for nuclei risc-v target id code using mask */
		if (target_device->tap_idcode == (target->tap->idcode & target_device->tap_idmask))
			break;

	if (nuspi_info->ctrl_base == 0) {
		nuspi_info->ctrl_base = target_device->ctrl_base;
	}

	if (target_device->name) {
		LOG_INFO("Valid NUSPI on device %s at address " TARGET_ADDR_FMT
			" with spictrl regbase at " TARGET_ADDR_FMT, target_device->name, bank->base, nuspi_info->ctrl_base);
	} else {
		LOG_INFO("Valid NUSPI on unknown device at address " TARGET_ADDR_FMT
			" with spictrl regbase at " TARGET_ADDR_FMT, bank->base, nuspi_info->ctrl_base);
	}

	if (nuspi_read_reg(bank, &nuspi_info->version, NUSPI_REG_VERSION) != ERROR_OK)
		return ERROR_FAIL;
	nuspi_info->nuspi_flags = 0;
	LOG_INFO("Nuclei SPI controller version 0x%08x", nuspi_info->version);

	if(nuspi_info->version >= 0x00010100)
		nuspi_info->nuspi_flags |= NUSPI_FLAGS_32B_DAT;

	/* read and decode flash ID; returns in SW mode */
	if (nuspi_write_reg(bank, NUSPI_REG_TXMARK, NUSPI_TXWM(1)) != ERROR_OK)
		return ERROR_FAIL;
	nuspi_set_dir(bank, NUSPI_DIR_TX);

	/* Disable Hardware accesses*/
	if (nuspi_disable_hw_mode(bank) != ERROR_OK)
		return ERROR_FAIL;

	if (nuspi_reset(bank) != ERROR_OK)
		return ERROR_FAIL;

	if (flash_reset(bank) != ERROR_OK)
		return ERROR_FAIL;

	target_register_event_callback(gdb_flash_write_end_callback, bank);

	for (int i = 0; i < 4; i++) {
		if (nuspi_write_reg(bank, NUSPI_REG_SCKDIV, i) != ERROR_OK)
			continue;
		retval = nuspi_read_flash_id(bank, &id);
		if (retval != ERROR_OK)
			continue;
		nuspi_info->dev = NULL;
		for (const struct flash_device *p = flash_devices; p->name ; p++) {
			if (p->device_id == id) {
				nuspi_info->dev = p;
				break;
			}
		}
		if (NULL != nuspi_info->dev)
			break;
	}

	if (nuspi_enable_hw_mode(bank) != ERROR_OK)
		return ERROR_FAIL;

	if (!nuspi_info->dev) {
		LOG_ERROR("Unknown flash device (ID 0x%08" PRIx32 ")", id);
		return ERROR_FAIL;
	}

	LOG_INFO("Found flash device \'%s\' (ID 0x%08" PRIx32 ")",
			nuspi_info->dev->name, nuspi_info->dev->device_id);

	/* Set correct size value */
	bank->size = nuspi_info->dev->size_in_bytes;

	if (bank->size <= (1UL << 16))
		LOG_WARNING("device needs 2-byte addresses - not implemented");

	/* if no sectors, treat whole bank as single sector */
	sectorsize = nuspi_info->dev->sectorsize ?
		nuspi_info->dev->sectorsize : nuspi_info->dev->size_in_bytes;

	/* create and fill sectors array */
	bank->num_sectors = nuspi_info->dev->size_in_bytes / sectorsize;
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
	if (nuspi_set_xip_read_cmd(bank) != ERROR_OK)
		return ERROR_FAIL;
	nuspi_info->probed = true;
	return ERROR_OK;
}

static int nuspi_auto_probe(struct flash_bank *bank)
{
	struct nuspi_flash_bank *nuspi_info = bank->driver_priv;
	if (nuspi_info->probed)
		return ERROR_OK;
	return nuspi_probe(bank);
}

static int nuspi_protect_check(struct flash_bank *bank)
{
	/* Nothing to do. Protection is only handled in SW. */
	return ERROR_OK;
}

const struct flash_driver nuspi_flash = {
	.name = "nuspi",
	.flash_bank_command = nuspi_flash_bank_command,
	.erase = nuspi_erase,
	.protect = nuspi_protect,
	.write = nuspi_write,
	.read = nuspi_read,
	.probe = nuspi_probe,
	.auto_probe = nuspi_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = nuspi_protect_check,
	.free_driver_priv = default_flash_free_driver_priv
};

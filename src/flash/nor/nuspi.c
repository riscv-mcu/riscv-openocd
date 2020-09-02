/***************************************************************************
 *   Copyright (C) 2010 by Antonio Borneo <borneo.antonio@gmail.com>       *
 *   Modified by Megan Wachs <megan@sifive.com> from the original stmsmi.c *
 *   Modified by Zhiyuan Wan <hikari@nucleisys.com> from fespi driver      *
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

/* The Freedom E SPI controller is a SPI bus controller
 * specifically designed for SPI Flash Memories on Freedom E platforms.
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

#define NUSPI_REG_SCKDIV          0x00
#define NUSPI_REG_SCKMODE         0x04
#define NUSPI_REG_CSID            0x10

#define NUSPI_REG_DELAY0          0x28
#define NUSPI_REG_DELAY1          0x2c

#define NUSPI_REG_FMT             0x40
#define NUSPI_REG_TXDATA          0x48
#define NUSPI_REG_RXDATA          0x4c

#define NUSPI_REG_FCTRL           0x60
#define NUSPI_REG_FFMT            0x64

#define NUSPI_REG_MFMT            0x78
#define NUSPI_REG_STATUS          0x7C

/* Fields */

#define NUSPI_SCK_POL             0x1
#define NUSPI_SCK_PHA             0x2

#define NUSPI_FMT_PROTO(x)        ((x) & 0x3)
#define NUSPI_FMT_ENDIAN(x)       (((x) & 0x1) << 2)
#define NUSPI_FMT_DIR(x)          (((x) & 0x1) << 3)
#define NUSPI_FMT_LEN(x)          (((x) & 0xf) << 16)

/* TXCTRL register */
#define NUSPI_TXWM(x)             ((x) & 0xffff)
/* RXCTRL register */
#define NUSPI_RXWM(x)             ((x) & 0xffff)

#define NUSPI_IP_TXWM             0x1
#define NUSPI_IP_RXWM             0x2

#define NUSPI_FCTRL_EN            0x1

#define NUSPI_INSN_CMD_EN         0x1
#define NUSPI_INSN_ADDR_LEN(x)    (((x) & 0x7) << 1)
#define NUSPI_INSN_PAD_CNT(x)     (((x) & 0xf) << 4)
#define NUSPI_INSN_CMD_PROTO(x)   (((x) & 0x3) << 8)
#define NUSPI_INSN_ADDR_PROTO(x)  (((x) & 0x3) << 10)
#define NUSPI_INSN_DATA_PROTO(x)  (((x) & 0x3) << 12)
#define NUSPI_INSN_CMD_CODE(x)    (((x) & 0xff) << 16)
#define NUSPI_INSN_PAD_CODE(x)    (((x) & 0xff) << 24)

/* Values */

#define NUSPI_CSMODE_AUTO         0
#define NUSPI_CSMODE_HOLD         2
#define NUSPI_CSMODE_OFF          3

#define NUSPI_DIR_RX              0
#define NUSPI_DIR_TX              1

#define NUSPI_PROTO_S             0
#define NUSPI_PROTO_D             1
#define NUSPI_PROTO_Q             2

#define NUSPI_ENDIAN_MSB          0
#define NUSPI_ENDIAN_LSB          1


/* Timeout in ms */
#define NUSPI_CMD_TIMEOUT   (100)
#define NUSPI_PROBE_TIMEOUT (100)
#define NUSPI_MAX_TIMEOUT  (3000)


struct nuspi_flash_bank {
	int probed;
	target_addr_t ctrl_base;
	int default_csid;
	const struct flash_device *dev;
};

struct nuspi_target {
	char *name;
	uint32_t tap_idcode;
	uint32_t ctrl_base;
};

/* TODO !!! What is the right naming convention here? */
static const struct nuspi_target target_devices[] = {
	/* name,   tap_idcode, ctrl_base */
#if 0
	{ "Nuclei E203 SPI Flash",	0x1e200a6d,	0x10014000},
	{ "Nuclei N201 SPI Flash",	0x12010a6d,	0x10014000},
	{ "Nuclei N203 SPI Flash",	0x12030a6d,	0x10014000},
	{ "Nuclei N205 SPI Flash",	0x12050a6d,	0x10014000},
	{ "Nuclei N205f SPI Flash",	0x1205fa6d,	0x10014000},
	{ "Nuclei N207 SPI Flash",	0x12070a6d,	0x10014000},
	{ "Nuclei N207f SPI Flash",	0x1207fa6d,	0x10014000},
#else
	{ "Nuclei SoC SPI Flash", 0x00000a6d, 0x10014000},
#endif
	{ NULL,    0,           0          }
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
	nuspi_info->probed = 0;
	nuspi_info->ctrl_base = 0;
	if (CMD_ARGC >= 7) {
		COMMAND_PARSE_ADDRESS(CMD_ARGV[6], nuspi_info->ctrl_base);
		LOG_DEBUG("ASSUMING NUSPI device at ctrl_base = " TARGET_ADDR_FMT,
				nuspi_info->ctrl_base);
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
{								\
	struct target *target = bank->target;
	struct nuspi_flash_bank *nuspi_info = bank->driver_priv;

	//LOG_INFO("Write %x to reg %x", value, (uint32_t)address);

	int result = target_write_u32(target, nuspi_info->ctrl_base + address, value);
	if (result != ERROR_OK) {
		LOG_ERROR("nuspi_write_reg() error writing 0x%x to " TARGET_ADDR_FMT,
				value, nuspi_info->ctrl_base + address);
		return result;
	}
	return ERROR_OK;
}

static int nuspi_disable_hw_mode(struct flash_bank *bank)
{
/*
	uint32_t fctrl;

	if (nuspi_read_reg(bank, &fctrl, NUSPI_REG_FCTRL) != ERROR_OK)
		return ERROR_FAIL;
*/
	if (nuspi_write_reg(bank, NUSPI_REG_FCTRL, 0x0) != ERROR_OK)
		return ERROR_FAIL;
	return nuspi_write_reg(bank, NUSPI_REG_CSID, 0x0);
}

static int nuspi_enable_hw_mode(struct flash_bank *bank)
{
/*
	uint32_t fctrl;

	if (nuspi_read_reg(bank, &fctrl, NUSPI_REG_FCTRL) != ERROR_OK)
		return ERROR_FAIL;
*/
	if (nuspi_write_reg(bank, NUSPI_REG_FCTRL, 0x5) != ERROR_OK)
		return ERROR_FAIL;
	return nuspi_write_reg(bank, NUSPI_REG_CSID, 0x1);
}

static int nuspi_set_dir(struct flash_bank *bank, bool dir)
{
	uint32_t fmt;
	if (nuspi_read_reg(bank, &fmt, NUSPI_REG_FMT) != ERROR_OK)
		return ERROR_FAIL;

	return nuspi_write_reg(bank, NUSPI_REG_FMT,
			(fmt & ~(NUSPI_FMT_DIR(0xFFFFFFFF))) | NUSPI_FMT_DIR(dir));
}


static int nuspi_wait_op(struct flash_bank *bank)
{
	int64_t start = timeval_ms();
	while (1) {
		uint32_t txfifo;
		if (nuspi_read_reg(bank, &txfifo, NUSPI_REG_TXDATA) != ERROR_OK)
			return ERROR_FAIL;
		if (!(txfifo >> 31))
			break;
		int64_t now = timeval_ms();
		if (now - start > 1000) {
			LOG_ERROR("SPI transmit progress timed out.");
			return ERROR_TARGET_TIMEOUT;
		}
	}

	return ERROR_OK;
}

static int nuspi_xfer(struct flash_bank *bank, uint8_t in, uint8_t *out)
{
	uint32_t value;
	int ret = nuspi_write_reg(bank, NUSPI_REG_TXDATA, in);
	if (ret != ERROR_OK) return ret;

	ret = nuspi_wait_op(bank);
	if (ret != ERROR_OK) return ret;

	if(out != NULL)
	{
		ret = nuspi_read_reg(bank, &value, NUSPI_REG_RXDATA);
		if (ret != ERROR_OK) return ret;

		*out = value & 0xff;
	}
	return ERROR_OK;
}

#if 0
static int nuspi_rx(struct flash_bank *bank, uint8_t *out)
{
	int64_t start = timeval_ms();
	uint32_t value;

	while (1) {
		if (nuspi_read_reg(bank, &value, NUSPI_REG_RXDATA) != ERROR_OK)
			return ERROR_FAIL;
		if (!(value >> 31))
			break;
		int64_t now = timeval_ms();
		if (now - start > 1000) {
			LOG_ERROR("rxfifo didn't go positive (value=0x%x).", value);
			return ERROR_TARGET_TIMEOUT;
		}
	}

	if (out)
		*out = value & 0xff;

	return ERROR_OK;
}

#endif

/* TODO!!! Why don't we need to call this after writing? */
static int nuspi_wip(struct flash_bank *bank, int timeout)
{
	int64_t endtime;

	if (nuspi_write_reg(bank, NUSPI_REG_CSID, 0x1) != ERROR_OK)
		return ERROR_FAIL;
	endtime = timeval_ms() + timeout;

	nuspi_set_dir(bank, NUSPI_DIR_TX);
	if (nuspi_xfer(bank, SPIFLASH_READ_STATUS, NULL) != ERROR_OK)
		return ERROR_FAIL;
	nuspi_set_dir(bank, NUSPI_DIR_RX);
	do {
		alive_sleep(1);

		uint8_t rx;
		if (nuspi_xfer(bank, 0, &rx) != ERROR_OK)
			return ERROR_FAIL;
		if ((rx & SPIFLASH_BSY_BIT) == 0) {
			if (nuspi_write_reg(bank, NUSPI_REG_CSID, 0x0) != ERROR_OK)
				return ERROR_FAIL;
			return ERROR_OK;
		}
	} while (timeval_ms() < endtime);
	nuspi_write_reg(bank, NUSPI_REG_CSID, 0x0);

	LOG_ERROR("timeout");
	return ERROR_FAIL;
}

static int nuspi_erase_sector(struct flash_bank *bank, int sector)
{
	struct nuspi_flash_bank *nuspi_info = bank->driver_priv;
	int retval;
	nuspi_set_dir(bank, NUSPI_DIR_TX);
	if (nuspi_write_reg(bank, NUSPI_REG_CSID, 0x1) != ERROR_OK)
		return ERROR_FAIL;

	retval = nuspi_xfer(bank, SPIFLASH_WRITE_ENABLE, NULL);
	if (retval != ERROR_OK)
		return retval;

	if (nuspi_write_reg(bank, NUSPI_REG_CSID, 0x0) != ERROR_OK)
		return ERROR_FAIL;

	if (nuspi_write_reg(bank, NUSPI_REG_CSID, 0x1) != ERROR_OK)
		return ERROR_FAIL;
	retval = nuspi_xfer(bank, nuspi_info->dev->erase_cmd, NULL);
	if (retval != ERROR_OK)
		return retval;
	sector = bank->sectors[sector].offset;
	retval = nuspi_xfer(bank, sector >> 16, NULL);
	if (retval != ERROR_OK)
		return retval;
	retval = nuspi_xfer(bank, sector >> 8, NULL);
	if (retval != ERROR_OK)
		return retval;
	retval = nuspi_xfer(bank, sector, NULL);
	if (retval != ERROR_OK)
		return retval;
	if (nuspi_write_reg(bank, NUSPI_REG_CSID, 0x0) != ERROR_OK)
		return ERROR_FAIL;

	retval = nuspi_wip(bank, NUSPI_MAX_TIMEOUT);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int nuspi_erase(struct flash_bank *bank, int first, int last)
{
	struct target *target = bank->target;
	struct nuspi_flash_bank *nuspi_info = bank->driver_priv;
	int retval = ERROR_OK;
	int sector;

	LOG_DEBUG("%s: from sector %d to sector %d", __func__, first, last);

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if ((first < 0) || (last < first) || (last >= bank->num_sectors)) {
		LOG_ERROR("Flash sector invalid");
		return ERROR_FLASH_SECTOR_INVALID;
	}

	if (!(nuspi_info->probed)) {
		LOG_ERROR("Flash bank not probed");
		return ERROR_FLASH_BANK_NOT_PROBED;
	}

	for (sector = first; sector <= last; sector++) {
		if (bank->sectors[sector].is_protected) {
			LOG_ERROR("Flash sector %d protected", sector);
			return ERROR_FAIL;
		}
	}

	if (nuspi_info->dev->erase_cmd == 0x00)
		return ERROR_FLASH_OPER_UNSUPPORTED;

#if 0
	if (nuspi_write_reg(bank, NUSPI_REG_TXCTRL, NUSPI_TXWM(1)) != ERROR_OK)
		return ERROR_FAIL;
	//retval = nuspi_txwm_wait(bank);
	if (retval != ERROR_OK) {
		LOG_ERROR("WM Didn't go high before attempting.");
		return retval;
	}
#endif

	/* Disable Hardware accesses*/
	if (nuspi_disable_hw_mode(bank) != ERROR_OK)
		return ERROR_FAIL;
	nuspi_set_dir(bank, NUSPI_DIR_TX);
	/* poll WIP */
	retval = nuspi_wip(bank, NUSPI_PROBE_TIMEOUT);
	if (retval != ERROR_OK)
		goto done;

	for (sector = first; sector <= last; sector++) {
		retval = nuspi_erase_sector(bank, sector);
		if (retval != ERROR_OK)
			goto done;
		keep_alive();
	}

	/* Switch to HW mode before return to prompt */
done:
	if (nuspi_enable_hw_mode(bank) != ERROR_OK)
		return ERROR_FAIL;
	return retval;
}

static int nuspi_protect(struct flash_bank *bank, int set,
		int first, int last)
{
	int sector;

	for (sector = first; sector <= last; sector++)
		bank->sectors[sector].is_protected = set;
	return ERROR_OK;
}

static int slow_nuspi_write_buffer(struct flash_bank *bank,
		const uint8_t *buffer, uint32_t offset, uint32_t len)
{
	uint32_t ii;

	if (offset & 0xFF000000) {
		LOG_ERROR("Currently OpenOCD NuSPI interface doesn't support address bits greater than 24-bits, can't do any operation to address 0x%08x",
				offset);
		return ERROR_FAIL;
	}

	/* TODO!!! assert that len < page size */
	nuspi_set_dir(bank, NUSPI_DIR_TX);
	if (nuspi_write_reg(bank, NUSPI_REG_CSID, 0x1) != ERROR_OK)
		return ERROR_FAIL;
	nuspi_xfer(bank, SPIFLASH_WRITE_ENABLE, NULL);
	if (nuspi_write_reg(bank, NUSPI_REG_CSID, 0x0) != ERROR_OK)
		return ERROR_FAIL;

	if (nuspi_write_reg(bank, NUSPI_REG_CSID, 0x1) != ERROR_OK)
		return ERROR_FAIL;

	nuspi_xfer(bank, SPIFLASH_PAGE_PROGRAM, NULL);

	nuspi_xfer(bank, offset >> 16, NULL);
	nuspi_xfer(bank, offset >> 8, NULL);
	nuspi_xfer(bank, offset, NULL);

	for (ii = 0; ii < len; ii++)
		nuspi_xfer(bank, buffer[ii], NULL);

	if (nuspi_write_reg(bank, NUSPI_REG_CSID, 0x0) != ERROR_OK)
		return ERROR_FAIL;


	keep_alive();

	return nuspi_wip(bank, NUSPI_MAX_TIMEOUT);
}

static const uint8_t algorithm_bin[] = {
#include "../../../contrib/loaders/flash/nuspi/nuspi.inc"
};
#define STEP_EXIT			4
#define STEP_TX				8
#define STEP_TXWM_WAIT		12
#define STEP_WRITE_REG		16
#define STEP_WIP_WAIT		20
#define STEP_SET_DIR		24
#define STEP_NOP			0xff

struct algorithm_steps {
	unsigned size;
	unsigned used;
	uint8_t **steps;
};

static struct algorithm_steps *as_new(void)
{
	struct algorithm_steps *as = calloc(1, sizeof(struct algorithm_steps));
	as->size = 8;
	as->steps = malloc(as->size * sizeof(as->steps[0]));
	return as;
}

static struct algorithm_steps *as_delete(struct algorithm_steps *as)
{
	for (unsigned step = 0; step < as->used; step++) {
		free(as->steps[step]);
		as->steps[step] = NULL;
	}
	free(as->steps);
	free(as);
	return NULL;
}

static int as_empty(struct algorithm_steps *as)
{
	for (unsigned s = 0; s < as->used; s++) {
		if (as->steps[s][0] != STEP_NOP)
			return 0;
	}
	return 1;
}

/* Return size of compiled program. */
static unsigned as_compile(struct algorithm_steps *as, uint8_t *target,
		unsigned target_size)
{
	unsigned offset = 0;
	bool finish_early = false;
	for (unsigned s = 0; s < as->used && !finish_early; s++) {
		unsigned bytes_left = target_size - offset;
		switch (as->steps[s][0]) {
			case STEP_NOP:
				break;
			case STEP_TX:
				{
					unsigned size = as->steps[s][1];
					if (size + 3 > bytes_left) {
						finish_early = true;
						break;
					}
					memcpy(target + offset, as->steps[s], size + 2);
					offset += size + 2;
					break;
				}
			case STEP_WRITE_REG:
				if (4 > bytes_left) {
					finish_early = true;
					break;
				}
				memcpy(target + offset, as->steps[s], 3);
				offset += 3;
				break;
			case STEP_SET_DIR:
				if (3 > bytes_left) {
					finish_early = true;
					break;
				}
				memcpy(target + offset, as->steps[s], 2);
				offset += 2;
				break;
			case STEP_TXWM_WAIT:
			case STEP_WIP_WAIT:
				if (2 > bytes_left) {
					finish_early = true;
					break;
				}
				memcpy(target + offset, as->steps[s], 1);
				offset += 1;
				break;
			default:
				assert(0);
		}
		if (!finish_early)
			as->steps[s][0] = STEP_NOP;
	}
	assert(offset + 1 <= target_size);
	target[offset++] = STEP_EXIT;

	LOG_DEBUG("%d-byte program:", offset);
	for (unsigned i = 0; i < offset;) {
		char buf[80];
		for (unsigned x = 0; i < offset && x < 16; x++, i++)
			sprintf(buf + x*3, "%02x ", target[i]);
		LOG_DEBUG("%s", buf);
	}

	return offset;
}

static void as_add_step(struct algorithm_steps *as, uint8_t *step)
{
	if (as->used == as->size) {
		as->size *= 2;
		as->steps = realloc(as->steps, sizeof(as->steps[0]) * as->size);
		LOG_DEBUG("Increased size to 0x%x", as->size);
	}
	as->steps[as->used] = step;
	as->used++;
}

static void as_add_tx(struct algorithm_steps *as, unsigned count, const uint8_t *data)
{
	LOG_DEBUG("count=%d", count);
	while (count > 0) {
		unsigned step_count = MIN(count, 255);
		uint8_t *step = malloc(step_count + 2);
		step[0] = STEP_TX;
		step[1] = step_count;
		memcpy(step + 2, data, step_count);
		as_add_step(as, step);
		data += step_count;
		count -= step_count;
	}
}

static void as_add_tx1(struct algorithm_steps *as, uint8_t byte)
{
	uint8_t data[1];
	data[0] = byte;
	as_add_tx(as, 1, data);
}

static void as_add_write_reg(struct algorithm_steps *as, uint8_t offset, uint8_t data)
{
	uint8_t *step = malloc(3);
	step[0] = STEP_WRITE_REG;
	step[1] = offset;
	step[2] = data;
	as_add_step(as, step);
}

static void as_add_txwm_wait(struct algorithm_steps *as)
{
	uint8_t *step = malloc(1);
	step[0] = STEP_TXWM_WAIT;
	as_add_step(as, step);
}

static void as_add_wip_wait(struct algorithm_steps *as)
{
	uint8_t *step = malloc(1);
	step[0] = STEP_WIP_WAIT;
	as_add_step(as, step);
}

static void as_add_set_dir(struct algorithm_steps *as, bool dir)
{
	uint8_t *step = malloc(2);
	step[0] = STEP_SET_DIR;
	step[1] = NUSPI_FMT_DIR(dir);
	as_add_step(as, step);
}

/* This should write something less than or equal to a page.*/
static int steps_add_buffer_write(struct algorithm_steps *as,
		const uint8_t *buffer, uint32_t chip_offset, uint32_t len)
{
	if (chip_offset & 0xFF000000) {
		LOG_ERROR("Currently OpenOCD NuSPI interface doesn't support address bits greater than 24-bits, can't do any operation to address 0x%08x",
				chip_offset);
		return ERROR_FAIL;
	}

	as_add_write_reg(as, NUSPI_REG_CSID, 0x1);
	as_add_tx1(as, SPIFLASH_WRITE_ENABLE);
	as_add_write_reg(as, NUSPI_REG_CSID, 0x0);
	//as_add_txwm_wait(as);
	as_add_write_reg(as, NUSPI_REG_CSID, 0x1);

	uint8_t setup[] = {
		SPIFLASH_PAGE_PROGRAM,
		chip_offset >> 16,
		chip_offset >> 8,
		chip_offset,
	};
	as_add_tx(as, sizeof(setup), setup);

	as_add_tx(as, len, buffer);
	//as_add_txwm_wait(as);
	as_add_write_reg(as, NUSPI_REG_CSID, 0x0);

	/* nuspi_wip() */
	as_add_set_dir(as, NUSPI_DIR_RX);
	as_add_write_reg(as, NUSPI_REG_CSID, 0x1);
	as_add_wip_wait(as);
	as_add_write_reg(as, NUSPI_REG_CSID, 0x0);
	as_add_set_dir(as, NUSPI_DIR_TX);

	return ERROR_OK;
}

static int steps_execute(struct algorithm_steps *as,
		struct flash_bank *bank, struct working_area *algorithm_wa,
		struct working_area *data_wa)
{
	struct target *target = bank->target;
	struct nuspi_flash_bank *nuspi_info = bank->driver_priv;
	uint32_t ctrl_base = nuspi_info->ctrl_base;
	int xlen = riscv_xlen(target);

	struct reg_param reg_params[2];
	init_reg_param(&reg_params[0], "a0", xlen, PARAM_OUT);
	init_reg_param(&reg_params[1], "a1", xlen, PARAM_OUT);
	buf_set_u64(reg_params[0].value, 0, xlen, ctrl_base);
	buf_set_u64(reg_params[1].value, 0, xlen, data_wa->address);

	int retval = ERROR_OK;
	while (!as_empty(as)) {
		keep_alive();
		uint8_t *data_buf = malloc(data_wa->size);
		unsigned bytes = as_compile(as, data_buf, data_wa->size);
		retval = target_write_buffer(target, data_wa->address, bytes,
				data_buf);
		free(data_buf);
		if (retval != ERROR_OK) {
			LOG_ERROR("Failed to write data to " TARGET_ADDR_FMT ": %d",
					data_wa->address, retval);
			goto exit;
		}

		retval = target_run_algorithm(target, 0, NULL, 2, reg_params,
				algorithm_wa->address, algorithm_wa->address + 4,
				10000, NULL);
		if (retval != ERROR_OK) {
			LOG_ERROR("Failed to execute algorithm at " TARGET_ADDR_FMT ": %d",
					algorithm_wa->address, retval);
			goto exit;
		}
	}

exit:
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[0]);
	return retval;
}

static int nuspi_write(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	struct nuspi_flash_bank *nuspi_info = bank->driver_priv;
	uint32_t cur_count, page_size, page_offset;
	int sector;
	int retval = ERROR_OK;

	LOG_DEBUG("%s: offset=0x%08" PRIx32 " count=0x%08" PRIx32,
			__func__, offset, count);

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (offset + count > nuspi_info->dev->size_in_bytes) {
		LOG_WARNING("Write past end of flash. Extra data discarded.");
		count = nuspi_info->dev->size_in_bytes - offset;
	}

	/* Check sector protection */
	for (sector = 0; sector < bank->num_sectors; sector++) {
		/* Start offset in or before this sector? */
		/* End offset in or behind this sector? */
		if ((offset <
					(bank->sectors[sector].offset + bank->sectors[sector].size))
				&& ((offset + count - 1) >= bank->sectors[sector].offset)
				&& bank->sectors[sector].is_protected) {
			LOG_ERROR("Flash sector %d protected", sector);
			return ERROR_FAIL;
		}
	}

	struct working_area *algorithm_wa;
	if (target_alloc_working_area(target, sizeof(algorithm_bin),
				&algorithm_wa) != ERROR_OK) {
		LOG_WARNING("Couldn't allocate %zd-byte working area.",
				sizeof(algorithm_bin));
		algorithm_wa = NULL;
	} else {
		retval = target_write_buffer(target, algorithm_wa->address,
				sizeof(algorithm_bin), algorithm_bin);
		if (retval != ERROR_OK) {
			LOG_ERROR("Failed to write code to " TARGET_ADDR_FMT ": %d",
					algorithm_wa->address, retval);
			target_free_working_area(target, algorithm_wa);
			algorithm_wa = NULL;
		}
	}

	struct working_area *data_wa = NULL;
	unsigned data_wa_size = 2 * count;
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

		data_wa_size /= 2;
	}

	/* If no valid page_size, use reasonable default. */
	page_size = nuspi_info->dev->pagesize ?
		nuspi_info->dev->pagesize : SPIFLASH_DEF_PAGESIZE;

	//nuspi_txwm_wait(bank);

	/* Disable Hardware accesses*/
	if (nuspi_disable_hw_mode(bank) != ERROR_OK)
		return ERROR_FAIL;

	struct algorithm_steps *as = as_new();

	/* poll WIP */
	retval = nuspi_wip(bank, NUSPI_PROBE_TIMEOUT);
	if (retval != ERROR_OK)
		goto err;

	page_offset = offset % page_size;
	/* central part, aligned words */
	while (count > 0) {
		/* clip block at page boundary */
		if (page_offset + count > page_size)
			cur_count = page_size - page_offset;
		else
			cur_count = count;

		if (algorithm_wa)
			retval = steps_add_buffer_write(as, buffer, offset, cur_count);
		else
			retval = slow_nuspi_write_buffer(bank, buffer, offset, cur_count);
		if (retval != ERROR_OK)
			goto err;

		page_offset = 0;
		buffer += cur_count;
		offset += cur_count;
		count -= cur_count;
	}

	if (algorithm_wa)
		retval = steps_execute(as, bank, algorithm_wa, data_wa);

err:
	if (algorithm_wa) {
		target_free_working_area(target, data_wa);
		target_free_working_area(target, algorithm_wa);
	}

	as_delete(as);

	/* Switch to HW mode before return to prompt */
	if (nuspi_enable_hw_mode(bank) != ERROR_OK)
		return ERROR_FAIL;
	return retval;
}

/* Return ID of flash device */
/* On exit, SW mode is kept */
static int nuspi_read_flash_id(struct flash_bank *bank, uint32_t *id)
{
	struct target *target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	//nuspi_txwm_wait(bank);

	/* poll WIP */
#if 0
	retval = nuspi_wip(bank, NUSPI_PROBE_TIMEOUT);
	if (retval != ERROR_OK)
		return retval;
#endif

	nuspi_set_dir(bank, NUSPI_DIR_TX);

	if (nuspi_write_reg(bank, NUSPI_REG_CSID, 0x1) != ERROR_OK)
		return ERROR_FAIL;

	uint8_t rx;
	/* Send SPI command "read ID" */
	nuspi_xfer(bank, SPIFLASH_READ_ID, NULL);
	nuspi_set_dir(bank, NUSPI_DIR_RX);
	/* read ID from Receive Register */
	if (nuspi_xfer(bank, 0, &rx) != ERROR_OK)
		return ERROR_FAIL;
	*id = rx;
	if (nuspi_xfer(bank, 0, &rx) != ERROR_OK)
		return ERROR_FAIL;
	*id |= (rx << 8);
	if (nuspi_xfer(bank, 0, &rx) != ERROR_OK)
		return ERROR_FAIL;
	*id |= (rx << 16);

	if (nuspi_write_reg(bank, NUSPI_REG_CSID, 0x0) != ERROR_OK)
		return ERROR_FAIL;

	return ERROR_OK;
}

static int nuspi_probe(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct nuspi_flash_bank *nuspi_info = bank->driver_priv;
	struct flash_sector *sectors;
	uint32_t id = 0; /* silence uninitialized warning */
	const struct nuspi_target *target_device;
	int retval;
	uint32_t sectorsize;

	if (nuspi_info->probed)
		free(bank->sectors);
	nuspi_info->probed = 0;

	if (nuspi_info->ctrl_base == 0) {

		if ((target->tap->idcode & 0x00000fff) == 0xa6d)
			for (target_device = target_devices ; target_device->name ; ++target_device)
			{
				if (target_device->tap_idcode == 0xa6d)
					break;
			}
		else
			for (target_device = target_devices ; target_device->name ; ++target_device)
			{
				if (target_device->tap_idcode == target->tap->idcode)
					break;
			}

		if (!target_device->name) {
			LOG_ERROR("Device ID 0x%" PRIx32 " is not known as NuSPI capable, please apply ctrl_base argument to configuration",
					target->tap->idcode);
			return ERROR_FAIL;
		}
		else
		{
			LOG_INFO("Auto-detected Flash name '%s'", target_device->name);
		}

		nuspi_info->ctrl_base = target_device->ctrl_base;

		LOG_DEBUG("Valid NUSPI on device %s at address " TARGET_ADDR_FMT,
				target_device->name, bank->base);

	} else {
	  LOG_DEBUG("Assuming NUSPI as specified at address " TARGET_ADDR_FMT
			  " with ctrl at " TARGET_ADDR_FMT, nuspi_info->ctrl_base,
			  bank->base);
	}

#if 0
	/* Read the default CSID */
	if (nuspi_read_reg(bank, &(nuspi_info->default_csid), NUSPI_REG_CSID) != ERROR_OK)
		return ERROR_FAIL;
#endif

	/* Disable Hardware accesses*/
	if (nuspi_disable_hw_mode(bank) != ERROR_OK)
		return ERROR_FAIL;

	retval = nuspi_read_flash_id(bank, &id);

	if (nuspi_enable_hw_mode(bank) != ERROR_OK)
		return ERROR_FAIL;
	if (retval != ERROR_OK)
		return retval;

	nuspi_info->dev = NULL;
	for (const struct flash_device *p = flash_devices; p->name ; p++)
		if (p->device_id == id) {
			nuspi_info->dev = p;
			break;
		}

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
	if (bank->size > (1UL << 24))
		LOG_WARNING("device needs paging or 4-byte addresses - not implemented");

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

	for (int sector = 0; sector < bank->num_sectors; sector++) {
		sectors[sector].offset = sector * sectorsize;
		sectors[sector].size = sectorsize;
		sectors[sector].is_erased = -1;
		sectors[sector].is_protected = 0;
	}

	bank->sectors = sectors;
	nuspi_info->probed = 1;
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

static int get_nuspi_info(struct flash_bank *bank, char *buf, int buf_size)
{
	struct nuspi_flash_bank *nuspi_info = bank->driver_priv;

	if (!(nuspi_info->probed)) {
		snprintf(buf, buf_size,
				"\nNUSPI flash bank not probed yet\n");
		return ERROR_OK;
	}

	snprintf(buf, buf_size, "\nNUSPI flash information:\n"
			"  Device \'%s\' (ID 0x%08" PRIx32 ")\n",
			nuspi_info->dev->name, nuspi_info->dev->device_id);

	return ERROR_OK;
}

const struct flash_driver nuspi_flash = {
	.name = "nuspi",
	.flash_bank_command = nuspi_flash_bank_command,
	.erase = nuspi_erase,
	.protect = nuspi_protect,
	.write = nuspi_write,
	.read = default_flash_read,
	.probe = nuspi_probe,
	.auto_probe = nuspi_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = nuspi_protect_check,
	.info = get_nuspi_info,
	.free_driver_priv = default_flash_free_driver_priv
};
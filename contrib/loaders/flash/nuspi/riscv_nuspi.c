#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "../../../../src/flash/nor/spi.h"

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

/* Timeouts we use, in number of status checks. */
#define TIMEOUT						(2000)

#define NUSPI_FLAGS_32B_DAT			(1 << 0)

/* #define DEBUG to make the return error codes provide enough information to
 * reconstruct the stack from where the error occurred. This is not enabled
 * usually to reduce the program size. */
#define ERROR_STACK(x)				(x)
#define ERROR_NUSPI_TXWM_WAIT		(0x10)
#define ERROR_NUSPI_TX				(0x100)
#define ERROR_NUSPI_RX				(0x1000)
#define ERROR_NUSPI_WIP				(0x50000)

#define ERROR_OK					(0)

typedef struct
{
	uint32_t info;
	uint32_t flags;
} nuspi_info_t;

static uint32_t nuspi_read_reg(volatile uint32_t *ctrl_base, uint32_t address);
static int nuspi_txwm_wait(volatile uint32_t *ctrl_base, nuspi_info_t* nuspi_info);
static int nuspi_wip(volatile uint32_t *ctrl_base, nuspi_info_t* nuspi_info);
static int nuspi_write_buffer(volatile uint32_t *ctrl_base,
		const uint8_t *buffer, uint32_t offset, uint32_t len,
		nuspi_info_t* nuspi_info);

/* Can set bits 3:0 in result. */
/* flash_info contains:
 *   bits 7:0 -- pprog_cmd
 *   bit 8    -- 0 means send 3 bytes after pprog_cmd, 1 means send 4 bytes
 *               after pprog_cmd
 */
int flash_nuspi(volatile uint32_t *ctrl_base, uint32_t page_size,
		const uint8_t *buffer, uint32_t offset, uint32_t count,
		uint32_t flash_info)
{
	int result;
	nuspi_info_t nuspi_info = {0};
	nuspi_info.info = flash_info;
	if(nuspi_read_reg(ctrl_base, NUSPI_REG_VERSION) >= 0x00010100)
		nuspi_info.flags |= NUSPI_FLAGS_32B_DAT;

	result = nuspi_txwm_wait(ctrl_base, &nuspi_info);
	if (result != ERROR_OK)
		return result | ERROR_STACK(0x1);

	/* poll WIP */
	result = nuspi_wip(ctrl_base, &nuspi_info);
	if (result != ERROR_OK) {
		result |= ERROR_STACK(0x2);
		goto err;
	}

	/* Assume page_size is a power of two so we don't need the modulus code. */
	uint32_t page_offset = offset & (page_size - 1);

	/* central part, aligned words */
	while (count > 0) {
		uint32_t cur_count;
		/* clip block at page boundary */
		if (page_offset + count > page_size)
			cur_count = page_size - page_offset;
		else
			cur_count = count;

		result = nuspi_write_buffer(ctrl_base, buffer, offset, cur_count, &nuspi_info);
		if (result != ERROR_OK) {
			result |= ERROR_STACK(0x3);
			goto err;
		}

		page_offset = 0;
		buffer += cur_count;
		offset += cur_count;
		count -= cur_count;
	}

err:
	return result;
}

static uint32_t nuspi_read_reg(volatile uint32_t *ctrl_base, uint32_t address)
{
	return ctrl_base[address / 4];
}

static void nuspi_write_reg(volatile uint32_t *ctrl_base, uint32_t address, uint32_t value)
{
	ctrl_base[address / 4] = value;
}

/* Can set bits 7:4 in result. */
static int nuspi_txwm_wait(volatile uint32_t *ctrl_base, nuspi_info_t* nuspi_info)
{
	unsigned timeout = TIMEOUT;

	if (nuspi_info->flags & NUSPI_FLAGS_32B_DAT) {
		while (timeout--) {
			uint32_t status = nuspi_read_reg(ctrl_base, NUSPI_REG_STATUS);
			if (0 == (status & NUSPI_STAT_BUSY))
				return ERROR_OK;
		}
	} else {
		while (timeout--) {
			uint32_t ip = nuspi_read_reg(ctrl_base, NUSPI_REG_IP);
			if (ip & NUSPI_IP_TXWM)
				return ERROR_OK;
		}
	}

	return ERROR_NUSPI_TXWM_WAIT;
}

static void nuspi_set_dir(volatile uint32_t *ctrl_base, bool dir)
{
	uint32_t fmt = nuspi_read_reg(ctrl_base, NUSPI_REG_FMT);
	nuspi_write_reg(ctrl_base, NUSPI_REG_FMT,
			(fmt & ~(NUSPI_FMT_DIR(0xFFFFFFFF))) | NUSPI_FMT_DIR(dir));
}

/* Can set bits 11:8 in result. */
static int nuspi_tx(volatile uint32_t *ctrl_base, uint8_t in, uint32_t flags)
{
	unsigned timeout = TIMEOUT;
	if (flags & NUSPI_FLAGS_32B_DAT) {
		while (timeout--) {
			if (!(nuspi_read_reg(ctrl_base, NUSPI_REG_STATUS) & NUSPI_STAT_TXFULL)) {
				nuspi_write_reg(ctrl_base, NUSPI_REG_TXDATA, in);
				return ERROR_OK;
			}
		}
	} else {
		while (timeout--) {
			uint32_t txfifo = nuspi_read_reg(ctrl_base, NUSPI_REG_TXDATA);
			if (!(txfifo >> 31)) {
				nuspi_write_reg(ctrl_base, NUSPI_REG_TXDATA, in);
				return ERROR_OK;
			}
		}
	}
	return ERROR_NUSPI_TX;
}

/* Can set bits 15:12 in result. */
static int nuspi_rx(volatile uint32_t *ctrl_base, uint8_t *out, uint32_t flags)
{
	unsigned timeout = TIMEOUT;
	if (flags & NUSPI_FLAGS_32B_DAT) {
		while (timeout--) {
			if (!(nuspi_read_reg(ctrl_base, NUSPI_REG_STATUS) & NUSPI_STAT_RXEMPTY)) {
				uint32_t value = nuspi_read_reg(ctrl_base, NUSPI_REG_RXDATA);
				if (out)
					*out = value & 0xff;
				return ERROR_OK;
			}
		}
	} else {
		while (timeout--) {
			uint32_t value = nuspi_read_reg(ctrl_base, NUSPI_REG_RXDATA);
			if (!(value >> 31)) {
				if (out)
					*out = value & 0xff;
				return ERROR_OK;
			}
		}
	}
	return ERROR_NUSPI_RX;
}

/* Can set bits 19:16 in result. */
static int nuspi_wip(volatile uint32_t *ctrl_base, nuspi_info_t* nuspi_info)
{
	nuspi_set_dir(ctrl_base, NUSPI_DIR_RX);

	nuspi_write_reg(ctrl_base, NUSPI_REG_CSMODE, NUSPI_CSMODE_HOLD);

	int result = nuspi_tx(ctrl_base, SPIFLASH_READ_STATUS, nuspi_info->flags);
	if (result != ERROR_OK)
		return result | ERROR_STACK(0x10000);
	result = nuspi_rx(ctrl_base, NULL, nuspi_info->flags);
	if (result != ERROR_OK)
		return result | ERROR_STACK(0x20000);

	unsigned timeout = TIMEOUT;
	while (timeout--) {
		result = nuspi_tx(ctrl_base, 0, nuspi_info->flags);
		if (result != ERROR_OK)
			return result | ERROR_STACK(0x30000);
		uint8_t rx;
		result = nuspi_rx(ctrl_base, &rx, nuspi_info->flags);
		if (result != ERROR_OK)
			return result | ERROR_STACK(0x40000);
		if ((rx & SPIFLASH_BSY_BIT) == 0) {
			nuspi_write_reg(ctrl_base, NUSPI_REG_CSMODE, NUSPI_CSMODE_AUTO);
			nuspi_set_dir(ctrl_base, NUSPI_DIR_TX);
			return ERROR_OK;
		}
	}

	return ERROR_NUSPI_WIP;
}

/* Can set bits 23:20 in result. */
static int nuspi_write_buffer(volatile uint32_t *ctrl_base,
		const uint8_t *buffer, uint32_t offset, uint32_t len,
		nuspi_info_t* nuspi_info)
{
	int result = nuspi_tx(ctrl_base, SPIFLASH_WRITE_ENABLE, nuspi_info->flags);
	if (result != ERROR_OK)
		return result | ERROR_STACK(0x100000);
	result = nuspi_txwm_wait(ctrl_base, nuspi_info);
	if (result != ERROR_OK)
		return result | ERROR_STACK(0x200000);

	nuspi_write_reg(ctrl_base, NUSPI_REG_CSMODE, NUSPI_CSMODE_HOLD);

	result = nuspi_tx(ctrl_base, nuspi_info->info & 0xff, nuspi_info->flags);
	if (result != ERROR_OK)
		return result | ERROR_STACK(0x300000);

	if (nuspi_info->info & 0x100) {
		result = nuspi_tx(ctrl_base, offset >> 24, nuspi_info->flags);
		if (result != ERROR_OK)
			return result | ERROR_STACK(0x400000);
	}
	result = nuspi_tx(ctrl_base, offset >> 16, nuspi_info->flags);
	if (result != ERROR_OK)
		return result | ERROR_STACK(0x400000);
	result = nuspi_tx(ctrl_base, offset >> 8, nuspi_info->flags);
	if (result != ERROR_OK)
		return result | ERROR_STACK(0x500000);
	result = nuspi_tx(ctrl_base, offset, nuspi_info->flags);
	if (result != ERROR_OK)
		return result | ERROR_STACK(0x600000);

	for (unsigned i = 0; i < len; i++) {
		result = nuspi_tx(ctrl_base, buffer[i], nuspi_info->flags);
		if (result != ERROR_OK)
			return result | ERROR_STACK(0x700000);
	}

	result = nuspi_txwm_wait(ctrl_base, nuspi_info);
	if (result != ERROR_OK)
		return result | ERROR_STACK(0x800000);

	nuspi_write_reg(ctrl_base, NUSPI_REG_CSMODE, NUSPI_CSMODE_AUTO);

	result = nuspi_wip(ctrl_base, nuspi_info);
	if (result != ERROR_OK)
		return result | ERROR_STACK(0x900000);
	return ERROR_OK;
}

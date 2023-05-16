/***************************************************************************
 *
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

#include "imp.h"
#include <helper/binarybuffer.h>
#include <target/algorithm.h>

/* CM32M4XXR register locations */
#define FLASH_REG_BASE  (0x40000000 + 0x18000 +  0xA000)

#define FLASH_BASE				(0x08000000)
#define FLASH_PAGE_SIZE			(2048)
#define FLASH_TOTAL_KB			(512)
#define FLASH_PG_UNIT			(4)

#define FMC_AC          0x00
#define FMC_KEY         0x04
#define FMC_OPTKEY      0x08
#define FMC_STS         0x0C
#define FMC_CTRL        0x10
#define FMC_ADD         0x14
#define FMC_OB          0x1C
#define FMC_WRP         0X20
#define FMC_ECC         0x24
#define FMC_CAH         0x30

/* option byte location */

/******************  Bit definition for FLASH_STS register  *******************/
#define FLASH_STS_BUSY_Pos						(0U)
#define FLASH_STS_BUSY_Msk						(0x1UL << FLASH_STS_BUSY_Pos)
#define FLASH_STS_BUSY							FLASH_STS_BUSY_Msk				/*!< Busy */
#define FLASH_STS_PGERR_Pos						(2U)
#define FLASH_STS_PGERR_Msk						(0x1UL << FLASH_STS_PGERR_Pos)
#define FLASH_STS_PGERR							FLASH_STS_PGERR_Msk				/*!< Programming Error */
#define FLASH_STS_PVERR_Pos						(3U)
#define FLASH_STS_PVERR_Msk						(0x1UL << FLASH_STS_PVERR_Pos)
#define FLASH_STS_PVERR							FLASH_STS_PVERR_Msk				/*!< Programming Verify ERROR after program */
#define FLASH_STS_WRPEER_Pos					(4U)
#define FLASH_STS_WRPEER_Msk					(0x1UL << FLASH_STS_WRPEER_Pos)
#define FLASH_STS_WRPEER						FLASH_STS_WRPEER_Msk			/*!< Write Protection Error */
#define FLASH_STS_EOP_Pos						(5U)
#define FLASH_STS_EOP_Msk						(0x1UL << FLASH_STS_EOP_Pos)
#define FLASH_STS_EOP							FLASH_STS_EOP_Msk				/*!< End of operation */
#define FLASH_STS_EVERR_Pos						(6U)
#define FLASH_STS_EVERR_Msk						(0x1UL << FLASH_STS_EVERR_Pos)
#define FLASH_STS_EVERR							FLASH_STS_EVERR_Msk				/*!< Erase Verify ERROR after page erase */
#define FLASH_STS_ECCERR_Pos					(7U)
#define FLASH_STS_ECCERR_Msk					(0x1UL << FLASH_STS_ECCERR_Pos)
#define FLASH_STS_ECCERR						FLASH_STS_ECCERR_Msk			/*!< ECC ERROR when Flash Reading */

/******************  Bit definition for FLASH_CTRL register  ******************/
#define FLASH_CTRL_PG_Pos						(0U)
#define FLASH_CTRL_PG_Msk						(0x1UL << FLASH_CTRL_PG_Pos)
#define FLASH_CTRL_PG							FLASH_CTRL_PG_Msk				/*!< Programming */
#define FLASH_CTRL_PER_Pos						(1U)
#define FLASH_CTRL_PER_Msk						(0x1UL << FLASH_CTRL_PER_Pos)
#define FLASH_CTRL_PER							FLASH_CTRL_PER_Msk				/*!< Page Erase */
#define FLASH_CTRL_MER_Pos						(2U)
#define FLASH_CTRL_MER_Msk						(0x1UL << FLASH_CTRL_MER_Pos)
#define FLASH_CTRL_MER							FLASH_CTRL_MER_Msk				/*!< Mass Erase */
#define FLASH_CTRL_OPTPG_Pos					(4U)
#define FLASH_CTRL_OPTPG_Msk					(0x1UL << FLASH_CTRL_OPTPG_Pos)
#define FLASH_CTRL_OPTPG						FLASH_CTRL_OPTPG_Msk			/*!< Option Byte Programming */
#define FLASH_CTRL_OPTER_Pos					(5U)
#define FLASH_CTRL_OPTER_Msk					(0x1UL << FLASH_CTRL_OPTER_Pos)
#define FLASH_CTRL_OPTER						FLASH_CTRL_OPTER_Msk			/*!< Option Byte Erase */
#define FLASH_CTRL_START_Pos					(6U)
#define FLASH_CTRL_START_Msk					(0x1UL << FLASH_CTRL_START_Pos)
#define FLASH_CTRL_START						FLASH_CTRL_START_Msk			/*!< Start */
#define FLASH_CTRL_LOCK_Pos						(7U)
#define FLASH_CTRL_LOCK_Msk						(0x1UL << FLASH_CTRL_LOCK_Pos)
#define FLASH_CTRL_LOCK							FLASH_CTRL_LOCK_Msk				/*!< Lock */
#define FLASH_CTRL_SMPSEL_Pos					(8U)
#define FLASH_CTRL_SMPSEL_Msk					(0x1UL << FLASH_CTRL_SMPSEL_Pos)
#define FLASH_CTRL_SMPSEL						FLASH_CTRL_SMPSEL_Msk			/*!< Flash Program Option Select */
#define FLASH_CTRL_OPTWE_Pos					(9U)
#define FLASH_CTRL_OPTWE_Msk					(0x1UL << FLASH_CTRL_OPTWE_Pos)
#define FLASH_CTRL_OPTWE						FLASH_CTRL_OPTWE_Msk			/*!< Option Bytes Write Enable */
#define FLASH_CTRL_ERRITE_Pos					(10U)
#define FLASH_CTRL_ERRITE_Msk					(0x1UL << FLASH_CTRL_ERRITE_Pos)
#define FLASH_CTRL_ERRITE						FLASH_CTRL_ERRITE_Msk			/*!< Error Interrupt Enable */
#define FLASH_CTRL_FERRITE_Pos					(11U)
#define FLASH_CTRL_FERRITE_Msk					(0x1UL << FLASH_CTRL_FERRITE_Pos)
#define FLASH_CTRL_FERRITE						FLASH_CTRL_FERRITE_Msk			/*!< EVERR PVERR Error Interrupt Enable */
#define FLASH_CTRL_EOPITE_Pos					(12U)
#define FLASH_CTRL_EOPITE_Msk 					(0x1UL << FLASH_CTRL_EOPITE_Pos)
#define FLASH_CTRL_EOPITE						FLASH_CTRL_EOPITE_Msk			/*!< End of operation Interrupt Enable */
#define FLASH_CTRL_ECCERRITE_Pos				(13U)
#define FLASH_CTRL_ECCERRITE_Msk				(0x1UL << FLASH_CTRL_ECCERRITE_Pos)
#define FLASH_CTRL_ECCERRITE					FLASH_CTRL_ECCERRITE_Msk			/*!< ECC Error Interrupt Enable */


#define FLASH_FLAG_BUSY             ((uint32_t) 0x00000001) /*!< FLASH Busy flag */
#define FLASH_FLAG_PGERR            ((uint32_t) 0x00000004) /*!< FLASH Program error flag */
#define FLASH_FLAG_PVERR            ((uint32_t) 0x00000008) /*!< FLASH Program Verify ERROR flag after program */
#define FLASH_FLAG_WRPERR           ((uint32_t) 0x00000010) /*!< FLASH Write protected error flag */
#define FLASH_FLAG_EOP              ((uint32_t) 0x00000020) /*!< FLASH End of Operation flag */
#define FLASH_FLAG_EVERR            ((uint32_t) 0x00000040) /*!< FLASH Erase Verify ERROR flag after page erase */
#define FLASH_FLAG_OBERR            ((uint32_t) 0x00000001) /*!< FLASH Option Byte error flag */
#define FLASH_STS_CLRFLAG           (FLASH_FLAG_PGERR | FLASH_FLAG_PVERR | FLASH_FLAG_WRPERR | FLASH_FLAG_EOP | FLASH_FLAG_EVERR)

/* FMC_OBSTAT bit definitions (reading) */

/* Flash Control Register bits */
#define CR_Set_PG       	FLASH_CTRL_PG
#define CR_Reset_PG     	(~FLASH_CTRL_PG)
#define CR_Set_PER      	FLASH_CTRL_PER
#define CR_Reset_PER   		(~FLASH_CTRL_PER)
#define CR_Set_MER      	FLASH_CTRL_MER
#define CR_Reset_MER    	(~FLASH_CTRL_MER)
#define CR_Set_OPTPG    	FLASH_CTRL_OPTPG
#define CR_Reset_OPTPG  	(~FLASH_CTRL_OPTPG)
#define CR_Set_OPTER    	FLASH_CTRL_OPTER
#define CR_Reset_OPTER  	(~FLASH_CTRL_OPTER)
#define CR_Set_START    	FLASH_CTRL_START
#define CR_Set_LOCK     	FLASH_CTRL_LOCK
#define CR_Reset_OPTWE     	(~FLASH_CTRL_OPTWE)

/* register unlock keys */

#define UNLOCK_KEY0			0x45670123
#define UNLOCK_KEY1			0xCDEF89AB

/* timeout values */

#define FLASH_WRITE_TIMEOUT 0x00002000
#define FLASH_ERASE_TIMEOUT 0x000B0000

struct cm32m4xxr_options {
	uint16_t RDP;
	uint16_t user_options;
	uint16_t user_data;
	uint16_t protection[4];
};

struct cm32m4xxr_flash_bank {
	struct cm32m4xxr_options option_bytes;
	int ppage_size;
	int probed;

	bool has_dual_banks;
	/* used to access flash bank cm32m4xxr */
	uint32_t register_base;
	uint16_t default_rdp;
	int user_data_offset;
	int option_offset;
	uint32_t user_bank_size;
};

static int cm32m4xxr_mass_erase(struct flash_bank *bank);
static int get_cm32m4xxr_info(struct flash_bank *bank, struct command_invocation *cmd);
static int cm32m4xxr_write_block(struct flash_bank *bank, const uint8_t *buffer,
    uint32_t offset, uint32_t count);

/* flash bank cm32m4xxr <base> <size> 0 0 <target#>
 */
FLASH_BANK_COMMAND_HANDLER(cm32m4xxr_flash_bank_command)
{
	struct cm32m4xxr_flash_bank *cm32m4xxr_info;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	cm32m4xxr_info = malloc(sizeof(struct cm32m4xxr_flash_bank));

	/* The flash write must be aligned to a word (4-bytes) boundary.
	 * Ask the flash infrastructure to ensure required alignment */
	bank->write_start_alignment = bank->write_end_alignment = 4;

	bank->driver_priv = cm32m4xxr_info;
	cm32m4xxr_info->probed = 0;
	cm32m4xxr_info->has_dual_banks = false;
	cm32m4xxr_info->register_base = FLASH_REG_BASE;
	cm32m4xxr_info->user_bank_size = bank->size;

	return ERROR_OK;
}

static inline int cm32m4xxr_get_flash_reg(struct flash_bank *bank, uint32_t reg)
{
	struct cm32m4xxr_flash_bank *cm32m4xxr_info = bank->driver_priv;
	return reg + cm32m4xxr_info->register_base;
}

static inline int cm32m4xxr_get_flash_status(struct flash_bank *bank, uint32_t *status)
{
	struct target *target = bank->target;
	return target_read_u32(target, cm32m4xxr_get_flash_reg(bank, FMC_STS), status);
}

static int cm32m4xxr_wait_status_busy(struct flash_bank *bank, int timeout)
{
	struct target *target = bank->target;
	uint32_t status;
	int retval = ERROR_OK;

	/* wait for busy to clear */
	for (;;) {
		retval = cm32m4xxr_get_flash_status(bank, &status);
		if (retval != ERROR_OK)
			return retval;
		LOG_DEBUG("status: 0x%" PRIx32 "", status);
		if ((status & FLASH_FLAG_BUSY) == 0)
			break;
		if (timeout-- <= 0) {
			LOG_ERROR("timed out waiting for flash");
			return ERROR_FAIL;
		}
		alive_sleep(1);
	}

	if (status & FLASH_FLAG_WRPERR) {
		LOG_ERROR("cm32m4xxr device protected");
		retval = ERROR_FAIL;
	}

	if (status & FLASH_FLAG_PGERR) {
		LOG_ERROR("cm32m4xxr device programming failed");
		retval = ERROR_FAIL;
	}

	/* Clear but report errors */
	if (status & (FLASH_FLAG_WRPERR | FLASH_FLAG_PGERR)) {
		/* If this operation fails, we ignore it and report the original
		 * retval
		 */
		target_write_u32(target, cm32m4xxr_get_flash_reg(bank, FMC_STS),
				FLASH_FLAG_WRPERR | FLASH_FLAG_PGERR);
	}
	return retval;
}

static int cm32m4xxr_erase(struct flash_bank *bank, unsigned first, unsigned last)
{
	struct target *target = bank->target;
	// uint32_t optiondata;
	// uint32_t obstat;
	uint32_t ctrl_reg;

	// struct cm32m4xxr_flash_bank *cm32m4xxr_info = NULL;
	// cm32m4xxr_info = bank->driver_priv;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}
	//todo: write protect
	if ((first == 0) && (last == (bank->num_sectors - 1)))
		return cm32m4xxr_mass_erase(bank);

	/* unlock flash registers */
	int retval = target_write_u32(target, cm32m4xxr_get_flash_reg(bank, FMC_KEY), UNLOCK_KEY0);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, cm32m4xxr_get_flash_reg(bank, FMC_KEY), UNLOCK_KEY1);
	if (retval != ERROR_OK)
		return retval;

	retval = cm32m4xxr_wait_status_busy(bank, FLASH_ERASE_TIMEOUT);
	if (retval != ERROR_OK)
		return retval;

	for (unsigned i = first; i <= last; i++) {
		//clear flash pending flags
		retval = target_write_u32(target, cm32m4xxr_get_flash_reg(bank, FMC_STS), FLASH_STS_CLRFLAG);
		if (retval != ERROR_OK)
			return retval;
		retval = target_write_u32(target, cm32m4xxr_get_flash_reg(bank, FMC_CTRL), CR_Set_PER);
		if (retval != ERROR_OK)
			return retval;
		retval = target_write_u32(target, cm32m4xxr_get_flash_reg(bank, FMC_ADD),
				bank->base + bank->sectors[i].offset);
		if (retval != ERROR_OK)
			return retval;
		retval = target_write_u32(target,
				cm32m4xxr_get_flash_reg(bank, FMC_CTRL), CR_Set_PER | CR_Set_START);
		if (retval != ERROR_OK)
			return retval;
		retval = cm32m4xxr_wait_status_busy(bank, FLASH_ERASE_TIMEOUT);
		if (retval != ERROR_OK)
			return retval;

		bank->sectors[i].is_erased = 1;

    	/* Disable the PER Bit */
		retval = target_read_u32(target, cm32m4xxr_get_flash_reg(bank, FMC_CTRL), &ctrl_reg);
		if (retval != ERROR_OK)
			return retval;
		ctrl_reg &= CR_Reset_PER;
		retval = target_write_u32(target, cm32m4xxr_get_flash_reg(bank, FMC_CTRL), ctrl_reg);
		if (retval != ERROR_OK)
			return retval;
	}

	retval = target_write_u32(target, cm32m4xxr_get_flash_reg(bank, FMC_CTRL), CR_Set_LOCK);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}


static int cm32m4xxr_write_block(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct cm32m4xxr_flash_bank *cm32m4xxr_info = bank->driver_priv;
	struct target *target = bank->target;
	uint32_t buffer_size = 16384;
	struct working_area *write_algorithm;
	struct working_area *source;
	uint32_t address = bank->base + offset;
	struct reg_param reg_params[5];
	int retval = ERROR_OK;

	static const uint8_t cm32m4xxr_flash_write_code[] = {
		0x03,0x28,0x06,0x00,0x63,0x05,0x08,0x04,0x5c,0x42,0xe3,0x8b,0x07,0xff,0x05,0x48,
		0x23,0x28,0x05,0x01,0x03,0xa8,0x07,0x00,0x23,0x20,0x07,0x01,0x91,0x07,0x11,0x07,
		0x03,0x2e,0xc5,0x00,0x93,0x78,0x1e,0x00,0x05,0x48,0xe3,0x0b,0x18,0xff,0x03,0x28,
		0x05,0x01,0x13,0x78,0xe8,0xff,0x23,0x28,0x05,0x01,0x63,0xc4,0xd7,0x00,0xb2,0x87,
		0xa1,0x07,0x5c,0xc2,0xfd,0x15,0x81,0xc5,0x65,0xbf,0x01,0x45,0x48,0xc2,0x72,0x85,
		0x02,0x90,
	};
	/* flash write code */
	if (target_alloc_working_area(target, sizeof(cm32m4xxr_flash_write_code),
			&write_algorithm) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	retval = target_write_buffer(target, write_algorithm->address,
			sizeof(cm32m4xxr_flash_write_code), cm32m4xxr_flash_write_code);
	if (retval != ERROR_OK) {
		target_free_working_area(target, write_algorithm);
		return retval;
	}

	/* memory buffer */
	while (target_alloc_working_area_try(target, buffer_size, &source) != ERROR_OK) {
		buffer_size /= 2;
		buffer_size &= ~3UL; /* Make sure it's 4 byte aligned */
		if (buffer_size <= 256) {
			/* we already allocated the writing code, but failed to get a
			 * buffer, free the algorithm */
			target_free_working_area(target, write_algorithm);

			LOG_WARNING("no large enough working area available, can't do block memory writes");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
	}

	init_reg_param(&reg_params[0], "a0", 32, PARAM_IN_OUT);	/* flash base (in), status (out) */
	init_reg_param(&reg_params[1], "a1", 32, PARAM_OUT);	/* count (halfword-16bit) */
	init_reg_param(&reg_params[2], "a2", 32, PARAM_OUT);	/* buffer start */
	init_reg_param(&reg_params[3], "a3", 32, PARAM_OUT);	/* buffer end */
	init_reg_param(&reg_params[4], "a4", 32, PARAM_IN_OUT);	/* target address */


	uint32_t wp_addr = source->address;
	uint32_t rp_addr = source->address + 4;
	uint32_t fifo_start_addr = source->address + 8;
	uint32_t fifo_end_addr = source->address + source->size;

	uint32_t wp = fifo_start_addr;
	uint32_t rp = fifo_start_addr;
	uint32_t thisrun_bytes = fifo_end_addr-fifo_start_addr - 4; /* (2:block size) */

	retval = target_write_u32(target, rp_addr, rp);
	if (retval != ERROR_OK)
		return retval;

	while (count > 0) {
		retval = target_read_u32(target, rp_addr, &rp);
		if (retval != ERROR_OK) {
			LOG_ERROR("failed to get read pointer");
			break;
		}
		if (wp != rp) {
			LOG_ERROR("Failed to write flash ;;  rp = 0x%x ;;; wp = 0x%x", rp, wp);
			break;
		}
		wp = fifo_start_addr;
		rp = fifo_start_addr;
		retval = target_write_u32(target, rp_addr, rp);
		if (retval != ERROR_OK)
			break;
		/* Limit to the amount of data we actually want to write */
		if (thisrun_bytes > count * 4)
			thisrun_bytes = count * 4;

		/* Write data to fifo */
		retval = target_write_buffer(target, wp, thisrun_bytes, buffer);
		if (retval != ERROR_OK)
			break;

		/* Update counters and wrap write pointer */
		buffer += thisrun_bytes;
		count -= thisrun_bytes / 4;
		rp = fifo_start_addr;
		wp = fifo_start_addr+thisrun_bytes;

		/* Store updated write pointer to target */
		retval = target_write_u32(target, wp_addr, wp);
		if (retval != ERROR_OK)
			break;
		retval = target_write_u32(target, rp_addr, rp);
		if (retval != ERROR_OK)
			return retval;

		buf_set_u32(reg_params[0].value, 0, 32, cm32m4xxr_info->register_base);
		buf_set_u32(reg_params[1].value, 0, 32, thisrun_bytes/4);
		buf_set_u32(reg_params[2].value, 0, 32, source->address);
		buf_set_u32(reg_params[3].value, 0, 32, source->address + source->size);
		buf_set_u32(reg_params[4].value, 0, 32, address);

		retval = target_run_algorithm(target, 0, NULL, 5, reg_params,
				write_algorithm->address, 0,
				10000, NULL);

		if (retval != ERROR_OK) {
			LOG_ERROR("Failed to execute algorithm at 0x%" TARGET_PRIxADDR ": %d",
					write_algorithm->address, retval);
			return retval;
		}
		address += thisrun_bytes;

	}

	if (retval == ERROR_FLASH_OPERATION_FAILED) {
		LOG_ERROR("flash write failed at address 0x%"PRIx32,
				buf_get_u32(reg_params[4].value, 0, 32));

		if (buf_get_u32(reg_params[0].value, 0, 32) & FLASH_FLAG_PGERR) {
			LOG_ERROR("flash memory not erased before writing");
			/* Clear but report errors */
			target_write_u32(target, cm32m4xxr_get_flash_reg(bank, FMC_STS), FLASH_FLAG_PGERR);
		}

		if (buf_get_u32(reg_params[0].value, 0, 32) & FLASH_FLAG_WRPERR) {
			LOG_ERROR("flash memory write protected");
			/* Clear but report errors */
			target_write_u32(target, cm32m4xxr_get_flash_reg(bank, FMC_STS), FLASH_FLAG_WRPERR);
		}
	}

	target_free_working_area(target, source);
	target_free_working_area(target, write_algorithm);

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	destroy_reg_param(&reg_params[3]);
	destroy_reg_param(&reg_params[4]);

	return retval;
}

static int cm32m4xxr_write(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	uint8_t *new_buffer = NULL;


	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (offset & 0x1) {
		LOG_ERROR("offset 0x%" PRIx32 " breaks required 2-byte alignment", offset);
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}

	/* If there's an odd number of bytes, the data has to be padded. Duplicate
	 * the buffer and use the normal code path with a single block write since
	 * it's probably cheaper than to special case the last odd write using
	 * discrete accesses. */
	if (count % 4) {
		new_buffer = malloc(count + 3);
		if (new_buffer == NULL) {
			LOG_ERROR("odd number of bytes to write and no memory for padding buffer");
			return ERROR_FAIL;
		}
		LOG_INFO("odd number of bytes to write, padding with 0xff");
		buffer = memcpy(new_buffer, buffer, count);
		new_buffer[count++] = 0xff;
		new_buffer[count++] = 0xff;
		new_buffer[count++] = 0xff;
	}

	uint32_t words_remaining = (count + 3) / 4;
	int retval, retval2;

	/* unlock flash registers */
	retval = target_write_u32(target, cm32m4xxr_get_flash_reg(bank, FMC_KEY), UNLOCK_KEY0);
	if (retval != ERROR_OK)
		goto cleanup;
	retval = target_write_u32(target, cm32m4xxr_get_flash_reg(bank, FMC_KEY), UNLOCK_KEY1);
	if (retval != ERROR_OK)
		goto cleanup;

	//clear flash pending flags
	retval = target_write_u32(target, cm32m4xxr_get_flash_reg(bank, FMC_STS), FLASH_STS_CLRFLAG);
	if (retval != ERROR_OK)
		goto cleanup;
	/* try using a block write */
	retval = cm32m4xxr_write_block(bank, buffer, offset, words_remaining);
	if (retval == ERROR_TARGET_RESOURCE_NOT_AVAILABLE) {
		/* if block write failed (no sufficient working area),
		 * we use normal (slow) single halfword accesses */
		LOG_WARNING("couldn't use block writes, falling back to single memory accesses");

		while (words_remaining > 0) {
			uint32_t value;
			memcpy(&value, buffer, sizeof(uint32_t));

			retval = target_write_u32(target, cm32m4xxr_get_flash_reg(bank, FMC_CTRL), CR_Set_PG);
			if (retval != ERROR_OK)
				goto cleanup;

			retval = target_write_u32(target, bank->base + offset, value);
			if (retval != ERROR_OK)
				goto reset_pg_and_lock;

			retval = cm32m4xxr_wait_status_busy(bank, 100);
			if (retval != ERROR_OK)
				goto reset_pg_and_lock;

			retval = target_write_u32(target, cm32m4xxr_get_flash_reg(bank, FMC_CTRL), 0);
			if (retval != ERROR_OK)
				goto cleanup;

			words_remaining--;
			buffer += 4;
			offset += 4;
		}
	}

reset_pg_and_lock:
	retval2 = target_write_u32(target, cm32m4xxr_get_flash_reg(bank, FMC_CTRL), CR_Set_LOCK);
	if (retval == ERROR_OK)
		retval = retval2;

cleanup:
	if (new_buffer)
		free(new_buffer);

	return retval;
}

static int cm32m4xxr_get_device_id(struct flash_bank *bank, uint32_t *device_id)
{

	struct target *target = bank->target;
	uint32_t device_id_register = 0xE0040000;
	/* read cm32m4xxr device id register */
	int retval = target_read_u32(target, device_id_register, device_id);
	if (retval != ERROR_OK)
		return retval;

	return retval;
}

static int cm32m4xxr_get_flash_size(struct flash_bank *bank, uint16_t *flash_size_in_kb)
{
	*flash_size_in_kb = FLASH_TOTAL_KB;
	return ERROR_OK;
}

static int cm32m4xxr_probe(struct flash_bank *bank)
{
	struct cm32m4xxr_flash_bank *cm32m4xxr_info = bank->driver_priv;
	int i;
	uint16_t flash_size_in_kb;
	uint16_t max_flash_size_in_kb;
	uint32_t device_id;
	int page_size;
	uint32_t base_address = FLASH_BASE;

	cm32m4xxr_info->probed = 0;
	cm32m4xxr_info->register_base = FLASH_REG_BASE;
	cm32m4xxr_info->user_data_offset = 10;
	cm32m4xxr_info->option_offset = 0;

	/* default factory protection level */
	cm32m4xxr_info->default_rdp = 0x5AA5;

	/* read cm32m4xxr device id register */
	int retval = cm32m4xxr_get_device_id(bank, &device_id);
	if (retval != ERROR_OK)
		return retval;

	LOG_INFO("device id = 0x%08" PRIx32 "", device_id);
	page_size = FLASH_PAGE_SIZE;
	cm32m4xxr_info->ppage_size = FLASH_PG_UNIT;
	max_flash_size_in_kb = FLASH_TOTAL_KB;

	/* get flash size from target. */
	retval = cm32m4xxr_get_flash_size(bank, &flash_size_in_kb);
	LOG_INFO("flash_size_in_kb = 0x%08" PRIx32 "", flash_size_in_kb);
	/* failed reading flash size or flash size invalid, default to max target family */
	if (retval != ERROR_OK || flash_size_in_kb == 0xffff || flash_size_in_kb == 0) {
		LOG_WARNING("cm32m4xxr flash size failed, probe inaccurate - assuming %dk flash",
			max_flash_size_in_kb);
		flash_size_in_kb = max_flash_size_in_kb;
	}
	/* if the user sets the size manually then ignore the probed value
	 * this allows us to work around devices that have a invalid flash size register value */
	if (cm32m4xxr_info->user_bank_size) {
		LOG_INFO("ignoring flash probed value, using configured bank size");
		flash_size_in_kb = cm32m4xxr_info->user_bank_size / 1024;
	}

	LOG_INFO("flash size = %dkbytes", flash_size_in_kb);

	/* did we assign flash size? */
	assert(flash_size_in_kb != 0xffff);

	/* calculate numbers of pages */
	int num_pages = flash_size_in_kb * 1024 / page_size;

	/* check that calculation result makes sense */
	assert(num_pages > 0);

	if (bank->sectors) {
		free(bank->sectors);
		bank->sectors = NULL;
	}

	bank->base = base_address;
	bank->size = (num_pages * page_size);
	bank->num_sectors = num_pages;
	bank->sectors = malloc(sizeof(struct flash_sector) * num_pages);

	for (i = 0; i < num_pages; i++) {
		bank->sectors[i].offset = i * page_size;
		bank->sectors[i].size = page_size;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = 1;
	}

	cm32m4xxr_info->probed = 1;

	return ERROR_OK;
}

static int cm32m4xxr_auto_probe(struct flash_bank *bank)
{
	struct cm32m4xxr_flash_bank *cm32m4xxr_info = bank->driver_priv;
	if (cm32m4xxr_info->probed)
		return ERROR_OK;
	return cm32m4xxr_probe(bank);
}

static int get_cm32m4xxr_info(struct flash_bank *bank, struct command_invocation *cmd)
{
	uint32_t dbgmcu_idcode;

	/* read cm32m4xxr device id register */
	int retval = cm32m4xxr_get_device_id(bank, &dbgmcu_idcode);
	if (retval != ERROR_OK)
		return retval;

	command_print_sameline(cmd, "%s - Rev: %s\n", "CM32M4xxR", "1.0");
	return ERROR_OK;
}


static int cm32m4xxr_mass_erase(struct flash_bank *bank)
{
	struct target *target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* unlock option flash registers */
	int retval = target_write_u32(target, cm32m4xxr_get_flash_reg(bank, FMC_KEY), UNLOCK_KEY0);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, cm32m4xxr_get_flash_reg(bank, FMC_KEY), UNLOCK_KEY1);
	if (retval != ERROR_OK)
		return retval;

	/* mass erase flash memory */
	retval = target_write_u32(target, cm32m4xxr_get_flash_reg(bank, FMC_CTRL), CR_Set_MER);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, cm32m4xxr_get_flash_reg(bank, FMC_CTRL),
			CR_Set_MER | CR_Set_START);
	if (retval != ERROR_OK)
		return retval;

	retval = cm32m4xxr_wait_status_busy(bank, FLASH_ERASE_TIMEOUT);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, cm32m4xxr_get_flash_reg(bank, FMC_CTRL), CR_Set_LOCK);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

COMMAND_HANDLER(cm32m4xxr_handle_mass_erase_command)
{
	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	retval = cm32m4xxr_mass_erase(bank);
	if (retval == ERROR_OK) {
		/* set all sectors as erased */
		for (unsigned i = 0; i < bank->num_sectors; i++)
			bank->sectors[i].is_erased = 1;

		command_print(CMD, "cm32m4xxr mass erase complete");
	} else
		command_print(CMD, "cm32m4xxr mass erase failed");

	return retval;
}

static const struct command_registration cm32m4xxr_exec_command_handlers[] = {
	{
		.name = "mass_erase",
		.handler = cm32m4xxr_handle_mass_erase_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Erase entire flash device.",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration cm32m4xxr_command_handlers[] = {
	{
		.name = "cm32m4xxr",
		.mode = COMMAND_ANY,
		.help = "cm32m4xxr flash command group",
		.usage = "",
		.chain = cm32m4xxr_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

struct flash_driver cm32m4xxr_flash = {
	.name = "cm32m4xxr",
	.commands = cm32m4xxr_command_handlers,
	.flash_bank_command = cm32m4xxr_flash_bank_command,
	.erase = cm32m4xxr_erase,
	.write = cm32m4xxr_write,
	.read = default_flash_read,
	.probe = cm32m4xxr_probe,
	.auto_probe = cm32m4xxr_auto_probe,
	.erase_check = default_flash_blank_check,
	.info = get_cm32m4xxr_info,
	.free_driver_priv = default_flash_free_driver_priv,
};

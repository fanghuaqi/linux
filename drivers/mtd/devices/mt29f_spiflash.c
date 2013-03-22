/*
 * Micron SPI Nand driver for devices: MT29F1G01 / MT29F1G02
 * Copyright (C) 2012-2013 Pierrick Hascoet <pierrick.hascoet@abilis.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/math64.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/flashchip.h>
#include <linux/mtd/partitions.h>
#include <linux/of_platform.h>

#include <linux/spi/spi.h>
#include <linux/spi/flash.h>

/* mt29f low level command */
#define CMD_PAGE_READ		0x13
#define CMD_READ_RDM		0x03
#define CMD_PROG_LOAD_PAGE	0x02
#define CMD_PROG_PAGE		0x84
#define CMD_PROG_PAGE_EXC	0x10
#define CMD_ERASE_BLK		0xd8
#define CMD_WR_ENABLE		0x06
#define CMD_WR_DISABLE		0x04
#define CMD_READ_ID		0x9f
#define CMD_RESET		0xff
#define CMD_READ_REG		0x0f
#define CMD_WRITE_REG		0x1f

/* feature / status register */
#define REG_BLOCK_LOCK		0xa0
#define REG_OTP			0xb0
#define REG_STATUS		0xc0

/* status resgister value */
#define STATUS_OIP_MASK		0x01
#define STATUS_READY		0 << 0
#define STATUS_BUSY		1 << 0

#define STATUS_E_FAIL_MASK	0x04
#define STATUS_E_FAIL		1 << 2

#define STATUS_P_FAIL_MASK 	0x08
#define STATUS_P_FAIL		1 << 3

#define STATUS_ECC_MASK 	0x30
#define STATUS_ECC_1BIT_FIXED	1 << 4
#define STATUS_ECC_ERROR	2 << 4
#define STATUS_ECC_RESERVED	3 << 4

/* set features register defines */
#define OTP_ECC_EN		0x10
#define OTP_RW_EN		0x40
#define OTP_PROT_EN		0x80


/* block lock */
#define BL_ALL_LOCKED		0x38
#define BL_1_2_LOCKED		0x30
#define BL_1_4_LOCKED		0x28
#define BL_1_8_LOCKED		0x20
#define BL_1_16_LOCKED		0x18
#define BL_1_32_LOCKED		0x10
#define BL_1_64_LOCKED		0x08
#define BL_ALL_UNLOCKED		0x00

#define MAX_CMD_SIZE	5

struct mt29f_info {
	/* device model informations */
	u8 signature[4];
	u8 rev_number[2];
	u8 features[2];
	u8 optional_cmd[2];
	u8 res1[22];
	u8 manufacturer[12];
	u8 model[20];
	u8 mid[1];
	u8 dcode[2];
	u8 res2[13];
	/* page details */
	u32 page_size;
	u16 page_spare_size;
	u32 page_sub_size;
	u16 page_sub_spare_size;
	u32 pages_per_block;
	u32 blocks_per_unit;
	u8 units;
	u8 addr_cycles;
	u8 bits_per_cell;
	u16 max_bad_blocks;
	u16 block_endurance;
	/* skipped useless data */
	u8 skip[26];
	/* timing informations */
	u16 tPROG;
	u16 tBERS;
	u16 tR;
	u16 tCCS;
} __attribute__((packed));

struct mt29f_flash {
	struct spi_device *spi;
	struct mutex lock;
	struct mtd_info mtd;
	struct mt29f_info info;

	u8 command[MAX_CMD_SIZE];

	/* bad block status */
	u8 bb_status;
	u16 page_size;
	u32 erase_size;

	flstate_t state;
};

static struct nand_ecclayout mt29f_ecclayout = {
	.eccbytes = 24,
	.eccpos = {
		   1, 2, 3, 4, 5, 6,
		   17, 18, 19, 20, 21, 22,
		   33, 34, 35, 36, 37, 38,
		   49, 50, 51, 52, 53, 54, },
	.oobavail = 32,
	.oobfree = {
		{.offset = 8,
		 .length = 8},
		{.offset = 24,
		 .length = 8},
		{.offset = 40,
		 .length = 8},
		{.offset = 56,
		 .length = 8}, }
};

static inline struct mtd_info *mt29f_to_mtd(struct mt29f_flash *flash)
{
	return &flash->mtd;
}

static inline struct mt29f_flash *mtd_to_mt29f(struct mtd_info *mtd)
{
	return (struct mt29f_flash *) mtd->priv;
}

static inline u16 mt29f_addr_to_page(struct mt29f_flash *flash, loff_t addr)
{
	return (u16) div_u64(addr, flash->page_size);
}

static int mt29f_reset(struct spi_device *spi)
{
	int ret;
	u8 cmd = CMD_RESET;

	ret = spi_write_then_read(spi, &cmd, 1, NULL, 0);

	/* wait for reset completion if xfer succeed */
	if (ret == 0)
		mdelay(1);

	return ret;
}

static int mt29f_read_id(struct spi_device *spi, u8 *id)
{
	int ret;
	u8 cmd[2];

	cmd[0] = CMD_READ_ID;
	cmd[1] = 0xFF; /* dummy byte */

	ret = spi_write_then_read(spi, cmd, 2, id, 2);
	if (ret != 0)
		dev_err(&spi->dev, "error %d reading id\n", ret);

	return ret;
}

/*
 * Lock/Unlock blocks, using the set feature command.
 */
static int mt29f_lock_block(struct mt29f_flash *flash, u8 lock)
{
	int ret;

	flash->command[0] = CMD_WRITE_REG;
	flash->command[1] = REG_BLOCK_LOCK;
	flash->command[2] = lock;

	ret = spi_write_then_read(flash->spi, flash->command, 3, NULL, 0);
	if (ret != 0)
		dev_err(&flash->spi->dev,
				"error %d during lock block\n", ret);

	return ret;
}

static inline signed long mt29f_wait_delay(struct spi_device *spi,
		unsigned long usecs)
{
#if 0
	return schedule_timeout(usecs_to_jiffies(usecs));
#else
	unsigned long deadline;

	deadline = jiffies + usecs_to_jiffies(usecs);
	do {
		cond_resched();
	} while (!time_after_eq(jiffies, deadline));

	return 0;
#endif
}

static int mt29f_read_sr(struct mt29f_flash *flash, u8 *status)
{
	int ret;

	flash->command[0] = CMD_READ_REG;
	flash->command[1] = REG_STATUS;

	ret = spi_write_then_read(flash->spi, flash->command, 2, status, 1);
	if (ret)
		dev_err(&flash->spi->dev,
				"failed to read SR (err: %d)\n", ret);

	return ret;
}

static int mt29f_get_otp_register(struct mt29f_flash *flash, u8 *otp)
{
	int ret;

	flash->command[0] = CMD_READ_REG;
	flash->command[1] = REG_OTP;

	ret = spi_write_then_read(flash->spi, flash->command, 2, otp, 1);
	if (ret != 0)
		dev_err(&flash->spi->dev,
				"error %d during read otp reg\n", ret);

	return ret;
}

static int mt29f_set_otp_register(struct mt29f_flash *flash, u8 value)
{
	int ret;

	flash->command[0] = CMD_WRITE_REG;
	flash->command[1] = REG_OTP;
	flash->command[2] = value;

	ret = spi_write_then_read(flash->spi, flash->command, 3, NULL, 0);
	if (ret != 0)
		dev_err(&flash->spi->dev,
				"error %d during set otp register\n", ret);

	return ret;
}

static int mt29f_ecc_enable(struct mt29f_flash *flash)
{
	int ret;
	u8 otp = 0;

	ret = mt29f_get_otp_register(flash, &otp);
	if (ret)
		return ret;

	if ((otp & OTP_ECC_EN) == OTP_ECC_EN)
		return 0;

	/* enable ecc */
	otp |= OTP_ECC_EN;
	ret = mt29f_set_otp_register(flash, otp);
	ret = mt29f_get_otp_register(flash, &otp);
	return ret;
}

#ifdef MTD_MT29F_ENABLE_SW_ECC
static int mt29f_ecc_disable(struct mt29f_flash *flash)
{
	int ret;
	u8 otp = 0;

	ret = mt29f_get_otp_register(flash, &otp);
	if (ret)
		return ret;

	if ((otp & OTP_ECC_EN) == OTP_ECC_EN) {
		otp &= ~OTP_ECC_EN;
		ret = mt29f_set_otp_register(spi_nand, info, otp);
		ret = mt29f_get_otp_register(spi_nand, info, &otp);
	}

	return ret;
}
#endif

static inline int mt29f_write_enable(struct mt29f_flash *flash)
{
	u8 cmd = CMD_WR_ENABLE;

	return spi_write_then_read(flash->spi, &cmd, 1, NULL, 0);
}

static inline int mt29f_write_disable(struct mt29f_flash *flash)
{
	u8 cmd = CMD_WR_DISABLE;

	return spi_write_then_read(flash->spi, &cmd, 1, NULL, 0);
}

static int mt29f_read_page_to_cache(struct mt29f_flash *flash, u16 row)
{
	int ret;

	pr_debug("%s: page_id: %d\n", __func__, row);

	flash->command[0] = CMD_PAGE_READ;
	flash->command[1] = 0x00; /* dummy byte */
	flash->command[2] = (u8) ((row & 0xFF00) >> 8);
	flash->command[3] = (u8) (row & 0x00FF);

	ret = spi_write_then_read(flash->spi, flash->command, 4, NULL, 0);

	if (ret == 0) {
		int tRD = 100; /* tRD Typ: 100 usecs */
		mt29f_wait_delay(flash->spi, tRD);
	}

	return ret;
}

static int mt29f_read_from_cache(struct mt29f_flash *flash, u16 column, u16 len, u8* rbuf)
{
	struct spi_transfer t[2];
	struct spi_message m;

	spi_message_init(&m);
	memset(t, 0, sizeof(t));

	flash->command[0] = CMD_READ_RDM;
	flash->command[1] = (u8) ((column & 0xFF00) >> 8);
	flash->command[2] = (u8) (column & 0x00FF);
	flash->command[3] = 0x00; /* dummy byte */

	t[0].tx_buf = flash->command;
	t[0].len = 4;
	spi_message_add_tail(&t[0], &m);

	t[1].rx_buf = rbuf;
	t[1].len = len;
	spi_message_add_tail(&t[1], &m);

	return spi_sync(flash->spi, &m);
}

int mt29f_read_page(struct mt29f_flash *flash, u16 page_id, u16 offset, u16 len, u8* rbuf)
{
	int ret;
	u8 status = 0;

	pr_debug("%s: page_id: %d ; offset: %d ; len: %d\n",
			__func__, page_id, offset, len);

	if ((page_id / flash->info.pages_per_block) & 0x1)
		offset |= (1 << 12); /* plane select */

	ret = mt29f_read_page_to_cache(flash, page_id);
	if (ret)
		goto out;

	/* TODO: redo ! */
	while (1)
	{
		ret = mt29f_read_sr(flash, &status);
		if (ret)
			goto out;

		if ((status & STATUS_OIP_MASK) == STATUS_READY) {

			if ((status & STATUS_ECC_MASK) == STATUS_ECC_ERROR) {
				dev_err(&flash->spi->dev, "ecc error, page=%d\n", page_id);
			}
			break;
		}
	}

	ret = mt29f_read_from_cache(flash, offset, len, rbuf);

out:
	return ret;
}

static int mt29f_program_load(struct mt29f_flash *flash, u16 column, u16 len, const u8* wbuf)
{
	struct spi_transfer t[2];
	struct spi_message m;

	spi_message_init(&m);
	memset(t, 0, sizeof(t));

	flash->command[0] = CMD_PROG_LOAD_PAGE;
	flash->command[1] = (u8) ((column & 0xFF00) >> 8);
	flash->command[2] = (u8) (column & 0x00FF);

	t[0].tx_buf = flash->command;
	t[0].len = 3;
	spi_message_add_tail(&t[0], &m);

	t[1].tx_buf = wbuf;
	t[1].len = len;
	spi_message_add_tail(&t[1], &m);

	return spi_sync(flash->spi, &m);
}

static int mt29f_program_execute(struct mt29f_flash *flash, u16 row)
{
	int ret;

	flash->command[0] = CMD_PROG_PAGE_EXC;
	flash->command[1] = 0x00; /* dummy byte */
	flash->command[2] = (u8) ((row & 0xFF00) >> 8);
	flash->command[3] = (u8) (row & 0x00FF);

	ret = spi_write_then_read(flash->spi, flash->command, 4, NULL, 0);
	if (ret == 0) {
		int tPRG = 400; /* tPRG: 400 usecs */
		mt29f_wait_delay(flash->spi, tPRG);
	}

	return ret;
}

int mt29f_program_page(struct mt29f_flash *flash, u16 page_id, u16 offset, u16 len, const u8* wbuf)
{
	int ret;
	u8 status = 0;

	pr_debug("%s: page_id: %d , offset: %d , len: %d\n", __func__,
			page_id, offset, len);

	ret = mt29f_write_enable(flash);
	if (ret)
		goto out;

	if ((page_id / flash->info.pages_per_block) & 0x1)
		offset |= (1 << 12); /* plane select */

	ret = mt29f_program_load(flash, offset, len, wbuf);
	if (ret)
		goto out;

	ret = mt29f_program_execute(flash, page_id);
	if (ret)
		goto out;

	// TODO: redo
	while (1)
	{
		ret = mt29f_read_sr(flash, &status);
		if (ret<0) {
			dev_err(&flash->spi->dev, "error %d reading status register\n",
					(int) ret);
			goto out;
		}

		if ((status & STATUS_OIP_MASK) == STATUS_READY)
		{

			if ((status & STATUS_P_FAIL_MASK) == STATUS_P_FAIL)
			{
				dev_err(&flash->spi->dev, "program error, page=%d\n", page_id);
				ret = -1;
				goto out;
			}
			else
				break;
		}
	}

out:
	return ret;
}

static int mt29f_block_erase_exec(struct mt29f_flash *flash, u16 block_id)
{
	int ret;
	u16 row;

	row = block_id << 6;

	pr_debug("%s: block_id: %d ; row: %d\n", __func__,
			block_id, row);

	flash->command[0] = CMD_ERASE_BLK;
	flash->command[1] = 0x00; /* dummy byte */
	flash->command[2] = (u8) ((row & 0xff00) >>8);
	flash->command[3] = (u8) (row & 0x00ff);


	ret = spi_write_then_read(flash->spi, flash->command, 4, NULL, 0);
	if (ret == 0) {
		int tERS = 4000; /* tERS: 4000 */
		mt29f_wait_delay(flash->spi, tERS);
	}

	return ret;
}

int mt29f_block_erase(struct mt29f_flash *flash, u16 block_id)
{
	int ret;
	u8 status = 0;

	ret = mt29f_write_enable(flash);
	if (ret)
		goto out;

	ret = mt29f_block_erase_exec(flash, block_id);
	if (ret)
		goto out;

	// TODO: redo
	while (1)
	{
		ret = mt29f_read_sr(flash, &status);
		if (ret)
			break;

		if ((status & STATUS_OIP_MASK) == STATUS_READY)
		{
			if ((status & STATUS_E_FAIL_MASK) == STATUS_E_FAIL)
			{
				dev_err(&flash->spi->dev, "erase error, block=%d\n", block_id);
				ret = -1;
			}
			break;
		}
	}

out:
	return ret;
}

static int mt29f_read_otp_page(struct mt29f_flash *flash, u16 page_id,
		u16 len, u8 *buf)
{
	u8 reg = 0;
	int ret;

	ret = mt29f_get_otp_register(flash, &reg);
	if (ret)
		return ret;

	ret = mt29f_set_otp_register(flash, (reg | OTP_RW_EN));
	if (ret)
		return ret;

	ret = mt29f_read_page(flash, page_id, 0, len, buf);

	/* restore non otp read */
	reg &= ~OTP_RW_EN;
	mt29f_set_otp_register(flash, reg);

	return ret;
}

static int spinand_get_device(struct mtd_info *mtd, int new_state)
{
	return 0;
}

static void spinand_release_device(struct mtd_info *mtd)
{
}

static int mt29f_mtd_read(struct mtd_info *mtd, loff_t from, size_t len,
	size_t *retlen, u_char *buf)
{
	struct mt29f_flash *flash = mtd_to_mt29f(mtd);
	int ret = 0;
	u16 pg_start;
	uint32_t pg_offs;
	int bytes;

	//pr_debug("%s: from = 0x%llx, len = %ld\n", __func__, from, (unsigned long) len);

	if (!len)
		return 0;

	if (from + len > mtd->size)
		return -EINVAL;

	*retlen = 0;

	mutex_lock(&flash->lock);

	while (len > 0) {
		pg_start = div_u64_rem(from, flash->page_size, &pg_offs);
		bytes = flash->page_size - pg_offs;

		if (bytes > len)
			bytes = len;

#if 0
		pr_debug("%s: from = 0x%llx ; read page: %d offs: %d ; len: %d\n",
				__func__, from, pg_start, pg_offs, bytes);
#endif

		ret = mt29f_read_page(flash, pg_start, pg_offs, bytes,
				(buf + (*retlen)));
		if (ret) {
			pr_debug("%s: mt29f_read_page() failed err: %d\n", __func__, ret);
			break;
		}

		from += bytes;
		len -= bytes;
		*retlen += bytes;
	}

	mutex_unlock(&flash->lock);
	return ret;
}

static int mt29f_mtd_write(struct mtd_info *mtd, loff_t to, size_t len,
		size_t *retlen, const u_char *buf)
{
	struct mt29f_flash *flash = mtd_to_mt29f(mtd);
	int ret;
	int bytes;
	u16 pg_start;
	u32 pg_offset;

	if (!len)
		return 0;

	if (!flash)
		return -EINVAL;

	if (to + len > mtd->size)
		return -EINVAL;

	*retlen = 0;

	mutex_lock(&flash->lock);

	while (len > 0) {
		pg_start = div_u64_rem(to, flash->page_size, &pg_offset);
		bytes = flash->page_size - pg_offset;

		if (bytes > len)
			bytes = len;

		pr_debug("%s: to = 0x%llx ; write page: %d ; offs: %d ; len: %d\n",
				__func__, to, pg_start, pg_offset, bytes);

		ret = mt29f_program_page(flash, pg_start, pg_offset, bytes,
				(buf + (*retlen)));
		if (ret) {
			pr_debug("%s: mt29f_program_page() failed err: %d\n", __func__, ret);
			break;
		}

		to += bytes;
		len -= bytes;
		*retlen += bytes;
	}

	mutex_unlock(&flash->lock);
	return ret;
}
static int mt29f_mtd_read_oob(struct mtd_info *mtd, loff_t from,
			  struct mtd_oob_ops *ops)
{
	struct mt29f_flash *flash = mtd_to_mt29f(mtd);
	int ret;
	u16 page_id;

	if (!flash)
		return -EINVAL;

	if (!ops->ooblen && !ops->oobbuf)
		return -EINVAL;

	pr_debug("%s: from = 0x%llx (len: %d)\n", __func__, from, ops->ooblen);

	// TODO: check if use case exist
	if (ops->ooblen && ops->len) {
		pr_debug("%s: want to read data + oob data\n", __func__);
	}

	page_id = mt29f_addr_to_page(flash, from);

	mutex_unlock(&flash->lock);

	ret = mt29f_read_page(flash, page_id, ops->ooboffs, ops->ooblen, ops->oobbuf);

	mutex_unlock(&flash->lock);

	if (ret == 0)
		ops->oobretlen = ops->ooblen;

	return ret;
}

static int mt29f_mtd_write_oob(struct mtd_info *mtd, loff_t to,
		struct mtd_oob_ops *ops)
{
	struct mt29f_flash *flash = mtd_to_mt29f(mtd);
	int ret;
	u16 page_id;

	if (!flash)
		return -EINVAL;

	if (!ops->ooblen && !ops->oobbuf)
		return -EINVAL;

	// TODO: check if use case exist
	if (ops->ooblen && ops->len) {
		pr_debug("%s: want to write data + oob data\n", __func__);
	}

	pr_debug("%s: to = 0x%llx (len: %d)\n", __func__, to, ops->ooblen);

	page_id = mt29f_addr_to_page(flash, to);

	mutex_lock(&flash->lock);

	ret = mt29f_program_page(flash, page_id, ops->ooboffs, ops->ooblen, ops->oobbuf);

	mutex_unlock(&flash->lock);

	if (ret == 0)
		ops->oobretlen = ops->ooblen;

	return ret;
}

static int mt29f_mtd_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	struct mt29f_flash *flash = mtd_to_mt29f(mtd);
	uint32_t rem;
	int ret = 0;
	u16 block_id, blocks;

	pr_debug("%s: start = 0x%012llx, len = %llu\n", __func__,
	      (unsigned long long) instr->addr,
	      (unsigned long long) instr->len);

	/* sanity checks */
	if (instr->addr + instr->len > flash->mtd.size)
		return -EINVAL;
	div_u64_rem(instr->len, mtd->erasesize, &rem);
	if (rem)
		return -EINVAL;

	/* TODO: use block shift from new API */
	block_id = instr->addr >> 17;//info->block_shift;
	blocks = instr->len >> 17; //info->block_shift;

	mutex_lock(&flash->lock);

	while (blocks != 0) {
		pr_debug("%s: erasing block# %d\n", __func__, block_id);

		ret = mt29f_block_erase(flash, block_id);
		if (ret < 0) {
			instr->fail_addr = MTD_FAIL_ADDR_UNKNOWN;
			break;
		}
		block_id++;
		blocks--;
	}

	mutex_unlock(&flash->lock);

	/* TODO: use block shift from new API */
	if (ret > 0)
		instr->fail_addr = (block_id << 17); // (page_size * pages_per_block)

	if (ret == 0)
		instr->state = MTD_ERASE_DONE;

	mtd_erase_callback(instr);

	return ret;
}

static int mt29f_block_isbad(struct mtd_info *mtd, loff_t ofs)
{
	struct mt29f_flash *flash = mtd_to_mt29f(mtd);

	int ret;
	u16 block_id = 0, page_id = 0;

	block_id = ofs >> 17; //info->block_shift;
	page_id = block_id * flash->info.pages_per_block;

	mutex_lock(&flash->lock);

	ret = mt29f_read_page(flash, page_id, flash->info.page_size, 1,
			&flash->bb_status);
	if (!ret)
		if (flash->bb_status != 0xFF)
			ret =  1;

	mutex_unlock(&flash->lock);

	return ret;
}

static int mt29f_mtd_block_markbad(struct mtd_info *mtd, loff_t ofs)
{
	struct mt29f_flash *flash = mtd_to_mt29f(mtd);
	int ret;
	u16 page_id;


	spinand_get_device(mtd, FL_WRITING);

	page_id = mt29f_addr_to_page(flash, ofs);
	flash->bb_status = 0x00;

	ret = mt29f_program_page(flash,
			page_id,
			flash->page_size,
			1,
			&flash->bb_status);

	spinand_release_device(mtd);

	return ret;
}

int mt29f_mtd_register(struct mt29f_flash *flash,
		struct flash_platform_data *data,
		struct mtd_part_parser_data *ppdata)
{
	struct mtd_info *mtd = mt29f_to_mtd(flash);

	mtd->name = flash->info.model;
	mtd->size = (flash->info.page_size * flash->info.pages_per_block *
			flash->info.blocks_per_unit);

	mtd->erasesize = flash->erase_size;
	mtd->writesize = flash->page_size;
	mtd->writebufsize = mtd->writesize;

	mtd->owner = THIS_MODULE;
	mtd->type = MTD_NANDFLASH;
	mtd->flags = MTD_CAP_NANDFLASH;

	mtd->ecclayout = &mt29f_ecclayout;
	mtd->oobsize = mt29f_ecclayout.oobavail;

	mtd->_erase = mt29f_mtd_erase;
	mtd->_point = NULL;
	mtd->_unpoint = NULL;
	mtd->_read = mt29f_mtd_read;
	mtd->_write = mt29f_mtd_write;
	mtd->_read_oob = mt29f_mtd_read_oob;
	mtd->_write_oob = mt29f_mtd_write_oob;
	mtd->_sync = NULL; //spinand_sync;
	mtd->_lock = NULL;
	mtd->_unlock = NULL;
	mtd->_suspend = NULL;
	mtd->_resume = NULL;
	mtd->_block_isbad = mt29f_block_isbad;
	mtd->_block_markbad = mt29f_mtd_block_markbad;

	return mtd_device_parse_register(mtd, NULL, ppdata,
			data ? data->parts : NULL,
			data ? data->nr_parts : 0);
}

int mt29f_mtd_release(struct mtd_info *mtd)
{
	return mtd_device_unregister(mtd);
}


static int mt29f_probe_device(struct mt29f_flash *flash, u8 *id)
{
	struct mt29f_info *info;
	int ret;

	if (id[0] != 0x2c)
		return -ENODEV;

	ret = mt29f_read_otp_page(flash, 0x01, 256, (u8 *) &flash->info);

	if (ret == 0) {
		info = &flash->info;
		info->manufacturer[sizeof(info->manufacturer) - 1] = '\0';
		strim(info->manufacturer);

		info->model[sizeof(info->model) - 1] = '\0';
		strim(info->model);
	}

	return ret;
}

static int mt29f_probe(struct spi_device *spi)
{
	struct flash_platform_data *data;
	struct mt29f_flash *flash;
	struct mtd_part_parser_data ppdata;
	struct device_node __maybe_unused *np = spi->dev.of_node;
	int ret;

	u8 id[2]= {0};

#ifdef CONFIG_MTD_OF_PARTS
	if (!of_device_is_available(np))
		return -ENODEV;
#endif

	data = spi->dev.platform_data;

	ret = mt29f_reset(spi);
	if (ret)
		return ret;

	ret = mt29f_read_id(spi, (u8*) &id);

	if (id[0] != 0x2C )
		return -ENODEV;

	flash = kzalloc(sizeof(struct mt29f_flash), GFP_KERNEL);
	if (!flash)
		return -ENOMEM;

	flash->spi = spi;
	mutex_init(&flash->lock);

	ret = mt29f_probe_device(flash, (u8*) &id);
	if (ret)
		goto failed;

	dev_info(&spi->dev, "found %s %s (%02x%02x)\n",
			flash->info.manufacturer,
			flash->info.model,
			id[0], id[1]);

	flash->page_size = flash->info.page_size;
	flash->erase_size = flash->info.pages_per_block * flash->page_size;

	/* unlock flash */
	ret = mt29f_lock_block(flash, BL_ALL_UNLOCKED);

#ifdef MTD_MT29F_ENABLE_SW_ECC
	mt29f_ecc_disable(flash);
#else
	mt29f_ecc_enable(flash);
#endif

	ppdata.of_node = spi->dev.of_node;
	/* init mtd support */
	dev_set_drvdata(&spi->dev, &flash->mtd);
	flash->mtd.priv = flash;
	flash->mtd.dev.parent = &spi->dev;

	ret = mt29f_mtd_register(flash, data, &ppdata);

	return ret;

failed:
	if (flash)
		kfree(flash);

	return ret;
}

static int mt29f_remove(struct spi_device *spi)
{
	int ret;
	struct mtd_info *mtd;
	struct mt29f_flash *flash;

	pr_debug("%s: remove\n", dev_name(&spi->dev));

	mtd = dev_get_drvdata(&spi->dev);

	if (!mtd)
		return -ENODEV;

	flash = mtd->priv;

	ret = mt29f_mtd_release(mtd);
	if (ret == 0) {
		kfree(flash);
	}

	return ret;
}

static struct spi_driver mt29f_driver = {
	.driver = {
		.name		= "mt29f",
		.bus		= &spi_bus_type,
		.owner		= THIS_MODULE,
	},
	.probe		= mt29f_probe,
	.remove		= mt29f_remove,
};

module_spi_driver(mt29f_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Pierrick Hascoet <pierrick.hascoet@abilis.com>");
MODULE_DESCRIPTION("MTD SPI driver for Micron SPI NAND MT29FxG01 flash chips");

/*
 * Driver for MCP2515 CAN Controller
 *
 * Copyright (C) 2014 Martin Sperl
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>

#include <linux/device.h>
#include <linux/netdevice.h>

#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>

#include <linux/spi/spi.h>

#include <linux/can.h>
#include <linux/can/dev.h>
#include <linux/can/platform/mcp251x.h>

#define DRV_NAME        "mcp2515a"

static bool use_optimize = 1;
module_param(use_optimize, bool, 0);
MODULE_PARM_DESC(use_optimize,
		"Run the driver with spi_message_compile support");

#ifdef SPI_HAVE_OPTIMIZE
#define SPI_MESSAGE_OPTIMIZE(spi,message)			\
	if (use_optimize) spi_message_optimize(spi,message)
#else
#define SPI_MESSAGE_OPTIMIZE(spi,message)
#endif

static bool use_dummy_complete = 1;
module_param(use_dummy_complete,bool,0);
MODULE_PARM_DESC(use_dummy_complete,
		"run the spi_messages using dummy complete calls");
static void mcp2515a_completed_dummy (void *data) { ; }
#define USE_DUMMY_COMPLETE					\
	(use_dummy_complete) ? mcp2515a_completed_dummy : NULL

/* some functions to measure delays on a logic analyzer
 * note: needs to get run first from non-atomic context!!! */
static int debugpin = 0;
module_param(debugpin,int,0);
MODULE_PARM_DESC(debugpin,"the pin that we should toggle");
static u32* gpio=0;
static void set_low(void) {
	if (debugpin) {
		if (!gpio)
			gpio = ioremap(0x20200000, SZ_16K);
		gpio[0x28/4]=debugpin;
	}
}
static void set_high(void) {
	if (debugpin) {
		if (!gpio)
			gpio = ioremap(0x20200000, SZ_16K);
		gpio[0x1C/4]=debugpin;
	}
}

/* the MCP commands */
#define MCP2515_CMD_WRITE         0x02
#define MCP2515_CMD_READ          0x03
#define MCP2515_CMD_MODIFY        0x05
#define MCP2515_CMD_STATUS        0xA0
#define MCP2515_CMD_RXSTATUS      0xB0
#define MCP2515_CMD_RESET         0xC0
#define MCP2515_CMD_READ_RX(i)    (0x90+(i<<2))
#define MCP2515_CMD_WRITE_TX(i)   (0x40+(i<<1))
#define MCP2515_CMD_REQ2SEND(i)   (0x80+i)

/* the MCP register */
#define MCP2515_REG_CANSTAT       0x0E
#define MCP2515_REG_CANCTRL       0x0F

#define MCP2515_REG_CANCTRL_REQOP_NORMAL   (0<<5)
#define MCP2515_REG_CANCTRL_REQOP_SLEEP    (1<<5)
#define MCP2515_REG_CANCTRL_REQOP_LOOPBACK (2<<5)
#define MCP2515_REG_CANCTRL_REQOP_LISTEN   (3<<5)
#define MCP2515_REG_CANCTRL_REQOP_CONFIG   (4<<5)

#define MCP2515_REG_CANCTRL_ABORT          (1<<4)
#define MCP2515_REG_CANCTRL_OSM            (1<<3)

#define MCP2515_REG_CANCTRL_CLKEN          (1<<2)
#define MCP2515_REG_CANCTRL_CLKPRE_1       (0)
#define MCP2515_REG_CANCTRL_CLKPRE_2       (1)
#define MCP2515_REG_CANCTRL_CLKPRE_4       (2)
#define MCP2515_REG_CANCTRL_CLKPRE_8       (3)

#define MCP2515_REG_CNF3          0x28
#define MCP2515_REG_CNF2          0x29
#define MCP2515_REG_CNF1          0x2A

#define MCP2515_REG_CANINTE       0x2B
#define MCP2515_REG_CANINTF       0x2C
#define MCP2515_REG_CANINT_MERR           (1<<7)
#define MCP2515_REG_CANINT_WAKI           (1<<6)
#define MCP2515_REG_CANINT_ERRI           (1<<5)
#define MCP2515_REG_CANINT_TX2I           (1<<4)
#define MCP2515_REG_CANINT_TX1I           (1<<3)
#define MCP2515_REG_CANINT_TX0I           (1<<2)
#define MCP2515_REG_CANINT_RX1I           (1<<1)
#define MCP2515_REG_CANINT_RX0I           (1<<0)

#define MCP2515_REG_EFLG          0x2D
#define MCP2515_REG_EFLG_RX1OVR            (1<<7)
#define MCP2515_REG_EFLG_RX0OVR            (1<<6)
#define MCP2515_REG_EFLG_TXBO              (1<<5)
#define MCP2515_REG_EFLG_TXEP              (1<<4)
#define MCP2515_REG_EFLG_RXEP              (1<<3)
#define MCP2515_REG_EFLG_TXWAR             (1<<2)
#define MCP2515_REG_EFLG_RXWAR             (1<<1)
#define MCP2515_REG_EFLG_EWARN             (1<<0)

#define MCP2515_REG_RXB0CTRL      0x60
#define MCP2515_REG_RXB1CTRL      0x70

#define MCP2515_REG_RXB_RXM_ANY   (3<<5)
#define MCP2515_REG_RXB_RXM_EXT   (2<<5)
#define MCP2515_REG_RXB_RXM_STD   (1<<5)
#define MCP2515_REG_RXB_RXM_FILTER (0<<5)

#define MCP2515_REG_RXB_RXRTR     (1<<2)
#define MCP2515_REG_RXB_BUKT      (1<<2)

#define MCP2515_REG_TXB0CTRL      0x30
#define MCP2515_REG_TXB1CTRL      0x40
#define MCP2515_REG_TXB2CTRL      0x50
#define MCP2515_REG_TXBCTRL(i)    (MCP2515_REG_TXB0CTRL + i*16)

#define MCP2515_REG_TXB_ABTF      (1<<6)
#define MCP2515_REG_TXB_MLOA      (1<<5)
#define MCP2515_REG_TXB_TXERR     (1<<4)
#define MCP2515_REG_TXB_TXREQ     (1<<3)
#define MCP2515_REG_TXB_TXP_0     (0)
#define MCP2515_REG_TXB_TXP_1     (1)
#define MCP2515_REG_TXB_TXP_2     (2)
#define MCP2515_REG_TXB_TXP_3     (3)

#define MCP2515_REG_BFPCTRL       0x0c
#define MCP2515_REG_BFP_B1BFS     (1<<5) /* pin RX1 value */
#define MCP2515_REG_BFP_B0BFS     (1<<4) /* pin RX0 value */
#define MCP2515_REG_BFP_B1BFE     (1<<3) /* pin RX1 enabled */
#define MCP2515_REG_BFP_B0BFE     (1<<2) /* pin RX0 enabled */
#define MCP2515_REG_BFP_B1BFM     (1<<1) /* pin RX1 as irq on buffer full */
#define MCP2515_REG_BFP_B0BFM     (1<<0) /* pin RX0 as irq on buffer full */

#define MCP2515_REG_TXRTSCTRL     0x0d
#define MCP2515_REG_TXRTS_B2RTS   (1<<5) /* pin TX2 value */
#define MCP2515_REG_TXRTS_B1RTS   (1<<4) /* pin TX1 value */
#define MCP2515_REG_TXRTS_B0RTS   (1<<3) /* pin TX0 value */
#define MCP2515_REG_TXRTS_B2RTSM  (1<<2) /* pin TX2 as irq to send message */
#define MCP2515_REG_TXRTS_B1RTSM  (1<<1) /* pin TX1 as irq to send message */
#define MCP2515_REG_TXRTS_B0RTSM  (1<<0) /* pin TX0 as irq to send message */

#define MCP2515_REG_TEC           0x1c
#define MCP2515_REG_REC           0x1d

#define MCP2515_CMD_STATUS_RX0IF   (1<<0)
#define MCP2515_CMD_STATUS_RX1IF   (1<<1)
#define MCP2515_CMD_STATUS_TX0REQ  (1<<2)
#define MCP2515_CMD_STATUS_TX0IF   (1<<3)
#define MCP2515_CMD_STATUS_TX1REQ  (1<<4)
#define MCP2515_CMD_STATUS_TX1IF   (1<<5)
#define MCP2515_CMD_STATUS_TX2REQ  (1<<6)
#define MCP2515_CMD_STATUS_TX2IF   (1<<7)

#define MCP2515_MSG_SIDH           0
#define MCP2515_MSG_SIDH_SHIFT     3

#define MCP2515_MSG_SIDL           1
#define MCP2515_MSG_SIDL_MASK      0xe0
#define MCP2515_MSG_SIDL_SHIFT     5
#define MCP2515_MSG_SIDL_EMASK     0x03

#define MCP2515_MSG_SIDL_SRR       (1<<4)
#define MCP2515_MSG_SIDL_IDE       (1<<3)
#define MCP2515_MSG_EIDH           2
#define MCP2515_MSG_EIDL           3
#define MCP2515_MSG_DLC            4
#define MCP2515_MSG_DLC_RTR        (1<<6)
#define MCP2515_MSG_DLC_MASK       0x0f
#define MCP2515_MSG_DATA           5

#define SET_BYTE(val, byte)                     \
        (((val) & 0xff) << ((byte) * 8))

struct mcp2515a_transfers;

/* the driver structure */
struct mcp2515a_priv {
        struct can_priv    can;
        struct net_device *net;
        struct spi_device *spi;

	/* the DMA Buffer */
	dma_addr_t transfers_dma_addr;
	/* the transfers */
	struct mcp2515a_transfers *transfers;

	/* some counters for RX sent/received */
	volatile u32 status_sent_count;
	volatile u32 status_callback_count;

	/* the structures needed for TX */
	bool tx_queue_stopped;
	u8 tx_len[3];
	s8 tx_next_priority;
#define TX_PRIORITY_MAX 11

	/* some states */
	u8 carrier_off;
	/* flag to say we are shutting down */
	u8 is_shutdown;

	/* states */
	u8 sysstate;
#define SYSSTATE_WARN    (1<<0)
#define SYSSTATE_PASSIVE (1<<1)
#define SYSSTATE_BUSOFF  (1<<2)

	/* lock used to
	 * protect structure
	 * avoid races scheduling multiple spi messages in one go
	 * (without the complete triggering BEFORE
	 * the second message gets sent)
	 */
	spinlock_t lock;
};

static const struct can_bittiming_const mcp2515a_bittiming_const = {
        .name = "mcp2515",
        .tseg1_min = 2,
        .tseg1_max = 16,
        .tseg2_min = 2,
        .tseg2_max = 8,
        .sjw_max = 4,
        .brp_min = 1,
        .brp_max = 64,
        .brp_inc = 1,
};

static int mcp2515a_reset(struct spi_device *spi)
{
        const u8 reset = 0xc0;

        return spi_write(spi, &reset, sizeof(reset));
}

/* confirm that this is the "correct" device */
int mcp2515a_confirm_device(struct spi_device *spi)
{
	/* the buffer to transmit/receive */
	u8 b[4];
	int ret;

	/* send a reset - one byte */
	ret = mcp2515a_reset(spi);
	if (ret)
		return ret;
	/* now wait for 10 ms - as per documentation */
	mdelay(10);

	/* and read the canstat and other values */
	b[0] = MCP2515_CMD_READ;
	b[1] = MCP2515_REG_CANSTAT;
	b[2] = 0;
	b[3] = 0;
	ret = spi_write_then_read(spi,b,2,b+2,2);
	if (ret)
		return ret;

	/* check canstat against magic numbers */
	/* CANSTAT */
	b[2] &= 0xEE;
	if (b[2] != 0x80)
		goto mcp2515a_confirm_device_err;
	b[3] &= 0x17;
	if (b[3] != 0x07)
		goto mcp2515a_confirm_device_err;

	/* log that we found the mcp2515 */
	dev_info(&spi->dev,
		"Found expected response from mcp2515 after reset:"
		" CANSTAT 0x%02x CANCTRL 0x%02x\n",
		b[2], b[3]);

	/* ok, we have succeeded */
	return 0;

mcp2515a_confirm_device_err:
	dev_err(&spi->dev,
		"Found unexpected response from device:"
		" CANSTAT 0x%02x CANCTRL 0x%02x - not detected\n",
		b[2], b[3]);
	return -ENODEV;
}

#define TRANSFER_INIT(base, message, callback, callbackdata)		\
	spi_message_init(&base->message.msg);				\
	base->message.msg.is_dma_mapped = 1;				\
	base->message.msg.complete = callback;				\
	base->message.msg.context = callbackdata;

#define TRANSFER_CMD_STRUCT(name, fields)	   \
	struct {				   \
		u8 cmd;				   \
		fields;				   \
		struct spi_transfer trans;	   \
	} name;

#define TRANSFER_INIT_SET_ADDR(base, message, xfer, field, ptr)	\
	if (ptr) {							\
		base->message.xfer.trans.field##_buf = ptr;		\
		base->message.xfer.trans.field##_dma = base##_dma_addr	\
			+ ((void *)ptr - (void *)base);			\
	} else {							\
		base->message.xfer.trans.field##_buf = NULL;		\
		base->message.xfer.trans.field##_dma = 0;		\
	}

#define TRANSFER_INIT_CMD(base,message,xfer,command)			\
	base->message.xfer.cmd = command;				\
	TRANSFER_INIT_SET_ADDR(base, message, xfer,			\
			tx, &base->message.xfer.cmd);			\
	base->message.xfer.trans.cs_change = 1;				\
	spi_message_add_tail(&base->message.xfer.trans,			\
			&base->message.msg);				\
	TRANSFER_INIT_SET_ADDR(base, message, xfer,			\
			rx, NULL);					\


#define TRANSFER_WRITE_STRUCT(name, datalen)	\
	TRANSFER_CMD_STRUCT(name,		\
			u8 reg;			\
			u8 data[datalen];	\
		)

#define TRANSFER_INIT_WRITE(base,message,xfer,regist)			\
	TRANSFER_INIT_CMD(base,message,xfer,MCP2515_CMD_WRITE);		\
	base->message.xfer.trans.len = 2 /* CMD write + address */	\
		+ sizeof(base->message.xfer.data) /* payload size itself */; \
	base->message.xfer.reg = regist;				\

#define TRANSFER_MODIFY_STRUCT(name)		\
	TRANSFER_CMD_STRUCT(name,		\
			u8 reg;			\
			u8 mask;		\
			u8 value;		\
		)

#define TRANSFER_INIT_MODIFY(base,message,xfer,regist)			\
	TRANSFER_INIT_CMD(base,message,xfer,MCP2515_CMD_MODIFY);	\
	base->message.xfer.trans.len = 4;				\
	base->message.xfer.reg = regist;				\
	base->message.xfer.mask = 0;					\
	base->message.xfer.value = 0;

#define TRANSFER_READ_STRUCT(name,datalen)		\
	TRANSFER_CMD_STRUCT(name,			\
			u8 reg;				\
			u8 dummydata[datalen];		\
			u8 cmd_rx;			\
			u8 reg_rx;			\
			u8 data[datalen];		\
		)

#define TRANSFER_INIT_READ(base,message,xfer,regist)			\
	TRANSFER_INIT_CMD(base,message,xfer,MCP2515_CMD_READ);		\
	base->message.xfer.reg = regist;				\
	base->message.xfer.trans.len = 2 /* CMD write + address */	\
		+ sizeof(base->message.xfer.data)			\
		/* payload size itself */;				\
	TRANSFER_INIT_SET_ADDR(base, message, xfer,			\
			rx, &base->message.xfer.cmd_rx);

#define TRANSFER_QREAD_STRUCT(name,datalen)		\
	TRANSFER_CMD_STRUCT(name,			\
			u8 dummydata[datalen];		\
			u8 cmd_rx;			\
			u8 data[datalen];		\
		)

#define TRANSFER_INIT_QREAD(base,message,xfer,command)			\
	TRANSFER_INIT_CMD(base,message,xfer,command);			\
	base->message.xfer.trans.len = 1 /* CMD READ_RX_BUFFER */	\
		+ sizeof(base->message.xfer.data)			\
		/* max payload size itself */;				\
	TRANSFER_INIT_SET_ADDR(base, message, xfer,			\
			rx, &base->message.xfer.cmd_rx);

struct mcp2515a_transfers {
	/* the messages */
	struct {
		struct spi_message msg;
		TRANSFER_WRITE_STRUCT(setRXB0ctrl,1);
		TRANSFER_WRITE_STRUCT(setRXB1ctrl,1);
		TRANSFER_WRITE_STRUCT(seterrorcounter,2);
		TRANSFER_WRITE_STRUCT(changetoconfigmode,1);
		TRANSFER_WRITE_STRUCT(setpinctrl,2);
		TRANSFER_WRITE_STRUCT(setconfig,5);
		TRANSFER_WRITE_STRUCT(changetomode,1);
		TRANSFER_READ_STRUCT(readconfig,8);
	} config;
	/* the messages we schedule in response to the irq going down */
	struct {
		struct spi_message msg;
		/* read the status flags quickly */
		TRANSFER_QREAD_STRUCT(read_status, 1);
		/* read the interrupt mask, sources and the error flags */
		TRANSFER_READ_STRUCT(read_inte_intf_eflg,3);
		/* clear the interrupts */
		TRANSFER_WRITE_STRUCT(clear_inte,1);
	} read_status;
	/* the above has a callback, this second is there to allow for
	 * HW/SW concurrency while the below transfer happens, we may have
	 * the time to handle the above information in the callback(irq)
	 * handler. as the most likley result is that the interrupt source
	 * is for a receive of messages, we read the message in preparation
	 *  already, so that we only have to acknowledge it...
	 */
	struct {
		struct spi_message msg;
		TRANSFER_READ_STRUCT(read_tec_rec,2);
		TRANSFER_READ_STRUCT(read_rx0,13);
	} read_status2;
	/* the message we send from the callback
	 * - there are 8 variants of this */
#define CALLBACK_CLEAR_EFLG  (1<<0)
#define CALLBACK_CLEAR_INTF  (1<<1)
#define CALLBACK_READACK_RX1 (1<<2)
#define CALLBACK_SIZE        8
	struct callback_actions {
		struct spi_message msg;
		struct net_device *net;
		int status;
		/* clear error flags - CALLBACK_CLEAR_ERR_FLAGS */
		TRANSFER_MODIFY_STRUCT(clear_eflg);
		/* clear irq flags - CALLBACK_CLEAR_IRQ_FLAGS */
		TRANSFER_MODIFY_STRUCT(clear_intf);
		/* read+ack RX1 - CALLBACK_READACK_RX1 */
		TRANSFER_QREAD_STRUCT(readack_rx1,13);
		/* the "corresponding" irq_mask */
		TRANSFER_WRITE_STRUCT(set_irq_mask,1);
	} callback_action[CALLBACK_SIZE];
	/* the transfers */
	struct {
		struct spi_message msg;
		TRANSFER_WRITE_STRUCT(message,14);
		TRANSFER_WRITE_STRUCT(transmit,0);
	} transmit_tx[3];
};

/* the complete callbacks and interrupt handlers */
static irqreturn_t mcp2515a_interrupt_handler(int, void *);
static void mcp2515a_completed_read_status (void *);
static void mcp2515a_completed_transfers (void *);

static void mcp2515a_free_transfers(struct net_device* net)
{
	struct mcp2515a_priv *priv = netdev_priv(net);
	int i;
	/* if we have been prepared, then release us */
	if (priv->transfers) {
		/* first unprepare messages */
#ifdef SPI_HAVE_OPTIMIZE
		printk(KERN_INFO "Unoptimize status\n");
		spi_message_unoptimize(&priv->transfers->read_status.msg);
		printk(KERN_INFO "Unoptimize status2\n");
		spi_message_unoptimize(&priv->transfers->read_status2.msg);
		for(i = 0; i < CALLBACK_SIZE; i++) {
			printk(KERN_INFO "Unoptimize callback_action %i\n",i);
			spi_message_unoptimize(
				&priv->transfers->callback_action[i].msg);
		}
		for(i = 0; i < 3; i++) {
			printk(KERN_INFO "Unoptimize tx%i\n",i);
			spi_message_unoptimize(
				&priv->transfers->transmit_tx[i].msg);
		}
#endif
		/* now release the structures */
		dma_free_coherent(&priv->spi->dev,
				sizeof(struct mcp2515a_transfers),
				priv->transfers,
				priv->transfers_dma_addr);
	}
}

static int mcp2515a_init_transfers(struct net_device* net)
{
	struct mcp2515a_priv *priv = netdev_priv(net);
        struct spi_device *spi = priv->spi;
        struct device *dev = &spi->dev;
	int i;
	/* first forece the coherent mask
	   - a bit of a hack, but at least it works... */
        dev->coherent_dma_mask = 0xffffffff;

	/* allocate memory */
	priv->transfers = dma_zalloc_coherent(
		&priv->spi->dev,
		sizeof(struct mcp2515a_transfers),
		&priv->transfers_dma_addr, GFP_KERNEL);
	if (! priv->transfers) return -ENOMEM;

	/* now let us fill in the data structures */

	/* setting up receive policies for buffers */
	TRANSFER_INIT(priv->transfers,config,
		USE_DUMMY_COMPLETE,NULL);
	TRANSFER_INIT_WRITE(priv->transfers,
				config,
				setRXB0ctrl,
				MCP2515_REG_RXB0CTRL);
	priv->transfers->config.setRXB0ctrl.data[0] =
		MCP2515_REG_RXB_RXM_ANY /* receive any message */
                | MCP2515_REG_RXB_BUKT; /* rollover to RXB1*/

	TRANSFER_INIT_WRITE(priv->transfers,
				config,
				setRXB1ctrl,
				MCP2515_REG_RXB1CTRL);
	priv->transfers->config.setRXB1ctrl.data[0] =
		MCP2515_REG_RXB_RXM_ANY; /* receive any message */

	/* clear TEC/REC */
	TRANSFER_INIT_WRITE(priv->transfers,
				config,
				seterrorcounter,
				MCP2515_REG_TEC);
	priv->transfers->config.seterrorcounter.data[0] = 0; /* tec */
	priv->transfers->config.seterrorcounter.data[1] = 0; /* rec */

	/* move to config mode */
	TRANSFER_INIT_WRITE(priv->transfers,
				config,
				changetoconfigmode,
				MCP2515_REG_CANCTRL);
	priv->transfers->config.changetoconfigmode.data[0] =
		MCP2515_REG_CANCTRL_REQOP_CONFIG;

	/* clear bit ctrl */
	TRANSFER_INIT_WRITE(priv->transfers,
				config,
				setpinctrl,
				MCP2515_REG_BFPCTRL);
	priv->transfers->config.setpinctrl.data[0] =
		0; /* BFPCTRL - high impedance */
	priv->transfers->config.setpinctrl.data[1] =
		0; /* TXRTSCTRL - no functionality*/

	/* configure CNF and interrupt-registers */
	TRANSFER_INIT_WRITE(priv->transfers,
				config,
				setconfig,
				MCP2515_REG_CNF3);
	priv->transfers->config.setconfig.data[0] = 0; /* CNF3 */
	priv->transfers->config.setconfig.data[1] = 0; /* CNF2 */
	priv->transfers->config.setconfig.data[2] = 0; /* CNF1 */
	priv->transfers->config.setconfig.data[3] =
		0x3f; /* INTE - enable all interupt sources */
	priv->transfers->config.setconfig.data[4] =
		0x00; /* INTF - clear all interupt sources */

	/* and change to the final mode we want to enter */
	TRANSFER_INIT_WRITE(priv->transfers,
			config,
			changetomode,
			MCP2515_REG_CANCTRL);
	priv->transfers->config.changetomode.data[0] =
		MCP2515_REG_CANCTRL_REQOP_NORMAL;

	/* initiate read of basic configs */
	TRANSFER_INIT_READ(priv->transfers,
			config,
			readconfig,
			MCP2515_REG_CNF3);

	/* The read status transfer with the callback */
	TRANSFER_INIT(priv->transfers,
		read_status,
		mcp2515a_completed_read_status,
		net);
	/* add the STATUS read transfer
	   - we modify the length and command */
	TRANSFER_INIT_QREAD(priv->transfers,
			read_status,
			read_status,
			MCP2515_CMD_STATUS);
	priv->transfers->read_status.read_status.cmd = MCP2515_CMD_STATUS;
	priv->transfers->read_status.read_status.trans.len = 2;
	/* add the read interrupts and error registers */
	TRANSFER_INIT_READ(priv->transfers,
			read_status,
			read_inte_intf_eflg,
			MCP2515_REG_CANINTE);
	/* and clear the interrupt mask, so no more IRQ occurs */
	TRANSFER_INIT_WRITE(priv->transfers,
				read_status,
				clear_inte,
				MCP2515_REG_CANINTE);
	priv->transfers->read_status.clear_inte.data[0] = 0;
	SPI_MESSAGE_OPTIMIZE(spi,&priv->transfers->read_status.msg);

	/* the second status message that gets scheduled */
	TRANSFER_INIT(priv->transfers,
		read_status2,
		USE_DUMMY_COMPLETE,
		NULL
		);
	/* we read the error-count */
	TRANSFER_INIT_READ(priv->transfers,
			read_status2,
			read_tec_rec,
			MCP2515_REG_TEC);
	/* and we read the RX0 buffer... */
	TRANSFER_INIT_READ(priv->transfers,
			read_status2,
			read_rx0,
			MCP2515_REG_RXB0CTRL+1);
	SPI_MESSAGE_OPTIMIZE(spi,&priv->transfers->read_status2.msg);

	/* and the callback action transfer in all variants - we want to
	 *  prepare the statements...
	 * note that we will need to change the order in which we run
	 * this dependent on status flags from above
	 */
	for ( i=0 ; i<CALLBACK_SIZE; i++) {
		TRANSFER_INIT(priv->transfers,
			callback_action[i],
			mcp2515a_completed_transfers,
			&priv->transfers->callback_action[i]);
		priv->transfers->callback_action[i].net = net;
		/* acknowledge the buffer overflows */
		if (i & CALLBACK_CLEAR_INTF) {
			TRANSFER_INIT_MODIFY(priv->transfers,
						callback_action[i],
						clear_intf,
						MCP2515_REG_CANINTF);
		}
		/* acknowledge the buffer overflows */
		if (i & CALLBACK_CLEAR_EFLG) {
			TRANSFER_INIT_MODIFY(priv->transfers,
					callback_action[i],
					clear_eflg,
					MCP2515_REG_EFLG);
		}
		/* read and acknowledge rx1 */
		if (i & CALLBACK_READACK_RX1) {
			TRANSFER_INIT_QREAD(priv->transfers,
					callback_action[i],
					readack_rx1,
					MCP2515_CMD_READ_RX(1)
				);
		}
		/* and set the IRQ mask back again */
		TRANSFER_INIT_WRITE(priv->transfers,
				callback_action[i],
				set_irq_mask,
				MCP2515_REG_CANINTE);
		priv->transfers->callback_action[i].set_irq_mask.data[0] =
			0x3f; /* no MERRE, WAKIE */
		/* and prepare the message */
		SPI_MESSAGE_OPTIMIZE(spi,
				&priv->transfers->callback_action[i].msg);
	}

	/* setup TX0,1,2 */
	for ( i=0 ; i<3; i++) {

		TRANSFER_INIT(priv->transfers,
			transmit_tx[i],
			USE_DUMMY_COMPLETE,
			NULL);
		TRANSFER_INIT_WRITE(priv->transfers,
				transmit_tx[i],
				message,
				MCP2515_REG_TXBCTRL(i));
		TRANSFER_INIT_WRITE(priv->transfers,
				transmit_tx[i],
				transmit,
				0 /* not needed */);
		priv->transfers->transmit_tx[i].transmit.cmd =
			MCP2515_CMD_REQ2SEND((1<<i));
		priv->transfers->transmit_tx[i].transmit.trans.len = 1;
		SPI_MESSAGE_OPTIMIZE(
			spi,&priv->transfers->transmit_tx[i].msg);
	}
	/* TODO ERROR HANDLING of optimize and others */

	/* and return ok */
	return 0;
}

static int mcp2515a_config(struct net_device *net)
{
        struct mcp2515a_priv *priv = netdev_priv(net);
        struct spi_device *spi = priv->spi;
        struct can_bittiming *bt = &priv->can.bittiming;
	int ret = 0;
	u8 mode = 0;

	/* select the mode we want */
	if (priv->can.ctrlmode & CAN_CTRLMODE_LOOPBACK) {
		mode = MCP2515_REG_CANCTRL_REQOP_LOOPBACK;
        } else if (priv->can.ctrlmode & CAN_CTRLMODE_LISTENONLY) {
		mode = MCP2515_REG_CANCTRL_REQOP_LISTEN;
        } else {
		mode = MCP2515_REG_CANCTRL_REQOP_NORMAL;
	}
	/* set one-shot mode if needed */
	if (priv->can.ctrlmode & CAN_CTRLMODE_ONE_SHOT)
		mode |= MCP2515_REG_CANCTRL_OSM;
	priv->transfers->config.changetomode.data[0]=mode;

	/* the Bit speed config */
	/* CNF3 */
	priv->transfers->config.setconfig.data[0] =
		bt->phase_seg2 - 1;
	/* CNF2 */
	priv->transfers->config.setconfig.data[1] =
		(priv->can.ctrlmode & CAN_CTRLMODE_3_SAMPLES ? 0xc0 : 0x80)
		| (bt->phase_seg1 - 1) << 3
		| (bt->prop_seg - 1)
		;
	/* CNF1 */
	priv->transfers->config.setconfig.data[2] =
		(bt->sjw - 1) << 6
		| (bt->brp - 1);

	/* execute transfer */
	ret = spi_sync(spi,&priv->transfers->config.msg);
	if (ret)
		return ret;
	/* dump the CNF */
	netdev_info(net, "configured CTRL: 0x%02x"
		" CNF: 0x%02x 0x%02x 0x%02x\n",
		priv->transfers->config.readconfig.data[7],
		priv->transfers->config.readconfig.data[2],
		priv->transfers->config.readconfig.data[1],
		priv->transfers->config.readconfig.data[0]
		);

	/* and return */
	return 0;
}

static int mcp2515a_do_set_mode(struct net_device *net, enum can_mode mode)
{
	if (mode == CAN_MODE_START)
		return mcp2515a_config(net);
	/* otherwise send message that it is not supported... */
	return -EOPNOTSUPP;
}

static void mcp2515a_queue_rx_message(struct mcp2515a_priv *priv, char* data)
{
        struct sk_buff *skb;
        struct can_frame *frame;

	/* increment packet counter */
	priv->net->stats.rx_packets++;

	/* allocate message buffer */
	skb = alloc_can_skb(priv->net, &frame);
        if (!skb) {
                dev_err(&priv->net->dev, "cannot allocate RX skb\n");
                priv->net->stats.rx_dropped++;
                return;
        }

	/* parse the can_id */
	if (data[MCP2515_MSG_SIDL] & MCP2515_MSG_SIDL_IDE) {
		/* Extended ID format */
		frame->can_id = CAN_EFF_FLAG;
		frame->can_id |=
			/* Extended ID part */
			SET_BYTE(data[MCP2515_MSG_SIDL] & MCP2515_MSG_SIDL_EMASK, 2) |
			SET_BYTE(data[MCP2515_MSG_EIDH], 1) |
			SET_BYTE(data[MCP2515_MSG_EIDL], 0) |
			/* Standard ID part */
			(((data[MCP2515_MSG_SIDH] << MCP2515_MSG_SIDH_SHIFT)
				|(data[MCP2515_MSG_SIDL] >> MCP2515_MSG_SIDL_SHIFT)
				) << 18);
		/* Remote transmission request */
		if (data[MCP2515_MSG_DLC] & MCP2515_MSG_DLC_RTR)
			frame->can_id |= CAN_RTR_FLAG;
	} else {
		/* Standard ID format */
		frame->can_id =
			(data[MCP2515_MSG_SIDH] << MCP2515_MSG_SIDH_SHIFT) |
			(data[MCP2515_MSG_SIDL] >> MCP2515_MSG_SIDL_SHIFT);
		if (data[MCP2515_MSG_SIDL] & MCP2515_MSG_SIDL_SRR)
			frame->can_id |= CAN_RTR_FLAG;
	}

	/* get data length */
	frame->can_dlc = get_can_dlc(
		data[MCP2515_MSG_DLC] & MCP2515_MSG_DLC_MASK);

	/* copy data */
	memcpy(frame->data,
		data + MCP2515_MSG_DATA,
		frame->can_dlc);

	/* increment byte counters */
        priv->net->stats.rx_bytes += frame->can_dlc;

	/* call can_led_event */
	can_led_event(priv->net, CAN_LED_EVENT_RX);

	/* and schedule packet */
	netif_rx_ni(skb);
}

static inline void mcp2515a_completed_transmit(
	struct net_device *net, struct mcp2515a_priv *priv, u8 tx)
{
	u8 len;
	unsigned long flags;

	spin_lock_irqsave(&priv->lock,flags);
	/* get and clear SKB - free happens later */
	len = priv->tx_len[tx];
	priv->tx_len[tx] = 0;

	/* if all tx are empty, then reset priority */
	if ((priv->tx_len[0] == 0)
		&& (priv->tx_len[1] == 0)
		&& (priv->tx_len[2] == 0)
		) {
		/* reset priorities to max */
		priv->tx_next_priority = TX_PRIORITY_MAX;
	}

	/* wake tx-queue if stopped and if priority is positive */
	if (priv->tx_queue_stopped) {
		/* wake if priority >=0 */
		if (priv->tx_next_priority >= 0) {
			priv->tx_queue_stopped = 0;
			netif_wake_queue(net);
		}
	}

	/* increment counters */
	net->stats.tx_packets++;
	net->stats.tx_bytes += len & 0x7f;

	/* return unlocked */
	spin_unlock_irqrestore(&priv->lock,flags);
}

static void mcp2515a_completed_read_status (void* context)
{
	struct net_device *net = context;
	struct mcp2515a_priv *priv = netdev_priv(net);
	struct mcp2515a_transfers *trans = priv->transfers;
	/* get the status bits */
	u8 status = trans->read_status.read_status.data[0];
	u8 inte = trans->read_status.read_inte_intf_eflg.data[0];
	u8 intf = trans->read_status.read_inte_intf_eflg.data[1];
	u8 eflg = trans->read_status.read_inte_intf_eflg.data[2];
	/* structure to use*/
	u8 structure = 0;
	u8 clear_intf = 0;
	u8 clear_eflg = 0;
	u32 err_id = 0;
	u8 err_detail = 0;
	u8 state = 0;
	u8 last_state = priv->sysstate;

	int ret=0;
	unsigned long flags;
	struct sk_buff *skb;
	struct can_frame *frame;

	/* return early if we are shutdown */
	if (priv->is_shutdown)
		return;

	/* handle errors */
	if (eflg) {
		structure |= CALLBACK_CLEAR_EFLG;
		if (eflg & MCP2515_REG_EFLG_RX0OVR) {
			clear_eflg = MCP2515_REG_EFLG_RX0OVR;
			/* increment fifo counter */
			net->stats.rx_fifo_errors++;
		}
		if (eflg & MCP2515_REG_EFLG_RX1OVR) {
			clear_eflg = MCP2515_REG_EFLG_RX1OVR;
			/* increment overflow from RX1 */
			net->stats.rx_over_errors++;
			net->stats.rx_errors++;
			/* and schedule error message to send */
			err_id     |= CAN_ERR_CRTL;
			err_detail |= CAN_ERR_CRTL_RX_OVERFLOW;
		}
		/* handle Counter WARNING errors */
		if (eflg & MCP2515_REG_EFLG_EWARN) {
			clear_eflg = MCP2515_REG_EFLG_EWARN;
			/* probably need to clean some more */
			priv->can.can_stats.error_warning++;
			/* and schedule error message to send */
			state |= SYSSTATE_WARN;
			if (state>last_state) {
				err_id     |= CAN_ERR_CRTL;
				err_detail |= CAN_ERR_CRTL_RX_WARNING
					| CAN_ERR_CRTL_TX_WARNING;
			}
		}
		/* handle passive */
		if (eflg & MCP2515_REG_EFLG_TXEP) {
			clear_eflg = MCP2515_REG_EFLG_TXEP;
			/* need to do some more stuff - taking offline */
			priv->can.can_stats.error_passive++;
			state |= SYSSTATE_PASSIVE;
			if (state>last_state) {
				err_id     |= CAN_ERR_CRTL;
				err_detail |= CAN_ERR_CRTL_TX_PASSIVE;
			}
		}
		if (eflg & MCP2515_REG_EFLG_RXEP) {
			clear_eflg = MCP2515_REG_EFLG_RXEP;
			/* need to do some more stuff - taking offline */
			priv->can.can_stats.error_passive++;
			state |= SYSSTATE_PASSIVE;
			if (state>last_state) {
				err_id     |= CAN_ERR_CRTL ;
				err_detail |= CAN_ERR_CRTL_RX_PASSIVE;
			}
		}
		/* handle Bus off */
		if (eflg & MCP2515_REG_EFLG_TXBO) {
			clear_eflg = MCP2515_REG_EFLG_TXBO;
			/* need to do some more stuff - taking offline */
			dev_err(&net->dev,
				"CAN-Bus shut down because of"
				" too many errors");
			priv->net->stats.tx_carrier_errors++;
			can_bus_off(net);
			state |= SYSSTATE_BUSOFF;
			if (state>last_state)
				err_id|=CAN_ERR_BUSOFF ;
		}
	}
	/* and check the intf flag for error - need to clean it */
	if (intf & MCP2515_REG_CANINT_ERRI) {
		structure |= CALLBACK_CLEAR_INTF;
		clear_intf |= MCP2515_REG_CANINT_ERRI;
	}
	/* handle RX buffers */
	if (status & MCP2515_CMD_STATUS_RX0IF ) {
		structure |= CALLBACK_CLEAR_INTF;
		clear_intf |= MCP2515_REG_CANINT_RX0I;
	}
	if (status & MCP2515_CMD_STATUS_RX1IF )
		structure |= CALLBACK_READACK_RX1;

	/* handle TX buffer interrupts */
	if (status & MCP2515_CMD_STATUS_TX2IF) {
		structure |= CALLBACK_CLEAR_INTF;
		clear_intf |= MCP2515_REG_CANINT_TX2I;
	}
	if (status & MCP2515_CMD_STATUS_TX1IF) {
		structure |= CALLBACK_CLEAR_INTF;
		clear_intf |= MCP2515_REG_CANINT_TX1I;
	}
	if (status & MCP2515_CMD_STATUS_TX0IF) {
		structure |= CALLBACK_CLEAR_INTF;
		clear_intf |= MCP2515_REG_CANINT_TX0I;
	}

	/* now set the clear_intf bitmask */
	trans->callback_action[structure].clear_intf.mask = clear_intf;
	trans->callback_action[structure].clear_eflg.mask = clear_eflg;
	trans->callback_action[structure].set_irq_mask.data[0] = inte;

	/* and keep the status for postprocessing */
	trans->callback_action[structure].status = status;

	/* set systemstate to the current state */
	priv->sysstate = state;

	/* and enable our own interrupts before actually scheduling the
	 * transfer - this is happening here avoiding a race condition...
	 */
	enable_irq(priv->spi->irq);

	/* schedule the transfer */
	set_high();
	spin_lock_irqsave(&priv->lock,flags);
	set_low();
	ret = spi_async(priv->spi,&trans->callback_action[structure].msg);
	set_high();
	spin_unlock_irqrestore(&priv->lock,flags);
	set_low();
	if (ret) {
		/* disable interrupts again
		 * to avoid that the system falls over
		 * and croak abaout this fact */
		disable_irq_nosync(priv->spi->irq);
		netdev_err(net,"error scheduling spi messages with err=%i"
			" - interrupts disabled\n",ret);
		/* and shut down transmit */
	}

	/* and mark the tx-messages as released
	 * ( in the correct priority order )
	 * we handle this only here, as we want to dispatch
	 * the spi message as soon as possible
	 */
	if (status & MCP2515_CMD_STATUS_TX2IF) {
		mcp2515a_completed_transmit(net,priv,2);
	}
	if (status & MCP2515_CMD_STATUS_TX1IF) {
		mcp2515a_completed_transmit(net,priv,1);
	}
	if (status & MCP2515_CMD_STATUS_TX0IF) {
		mcp2515a_completed_transmit(net,priv,0);
	}

	/* send can_error frame if needed */
	if (err_id) {
		skb = alloc_can_err_skb(priv->net, &frame);
		if (!skb) {
			netdev_err(priv->net,
				"cannot allocate error message\n");
		} else {
			/* assign data to it */
			frame->can_id |= err_id;
			frame->data[1] = err_detail;
			/* and deliver it */
			netif_rx_ni(skb);
		}
	}
}

void mcp2515a_completed_transfers (void *context)
{
	struct callback_actions *ca = context;
	struct net_device *net = ca->net;
	struct mcp2515a_priv *priv = netdev_priv(net);
	struct mcp2515a_transfers *trans = priv->transfers;
	/* get the status bits */
	u8 status = ca->status;

	/* return early if we are shutdown */
	if (priv->is_shutdown)
		return;
	/* and schedule RX0 to network stack */
	if (status & MCP2515_CMD_STATUS_RX0IF ) {
		mcp2515a_queue_rx_message(
			priv,trans->read_status2.read_rx0.data);
	}
	/* and schedule RX1 to network stack */
	if (status & MCP2515_CMD_STATUS_RX1IF ) {
		priv->net->stats.rx_fifo_errors++;
		mcp2515a_queue_rx_message(
			priv,
			ca->readack_rx1.data);
	}
}

/* the interrupt-handler for this device */
static irqreturn_t mcp2515a_interrupt_handler(int irq, void *dev_id)
{
        struct net_device *net = dev_id;
        struct mcp2515a_priv *priv = netdev_priv(net);
        struct spi_device *spi = priv->spi;
	int err = 0;
	unsigned long flags;

	/* disable the interrupt - one of the handlers will reenable it */
	disable_irq_nosync(irq);

	/* we will just schedule the 2 status transfers (of which
	 * the first generates a callback, while the second is just
	 *  pending...) */
	if (!priv->is_shutdown) {
		/* here there is a potential race if we are interrupted
		 * between the 2 transfers - this is not an issue on
		 * generic workqueue spi drivers, but it _can_ be an issue
		 * when we are get interrupt by a DMA interrupt.
		 * then reordering may occur which can be fatal
		 * for the message scheduled via complete_read_status
		 */
		spin_lock_irqsave(&priv->lock,flags);
		err = spi_async(spi,&priv->transfers->read_status.msg);
		err = spi_async(spi,&priv->transfers->read_status2.msg);
		spin_unlock_irqrestore(&priv->lock,flags);
		/* increment sent counter */
		priv->status_sent_count++;
	}

	/* return with a andled interrupt */
        return IRQ_HANDLED;
}

static netdev_tx_t mcp2515a_start_xmit(struct sk_buff *skb,
				struct net_device *net)
{
	struct mcp2515a_priv *priv = netdev_priv(net);
	struct can_frame *frame = (struct can_frame *)skb->data;
	int tx = -1;
	u8 prio = -1;
	u8 *buf;
	unsigned long flags;


	/* check some messages */
        if (can_dropped_invalid_skb(net, skb))
                return NETDEV_TX_OK;

	/* scheduling is as simple as this:
	 * * we make use of TX2 before TX1 and finally TX0.
	 * * we also make use of decreasing priority configs
	 * both means that we run the following sequence
	 * * 11 - TX2-P3
	 * * 10 - TX1-P3
	 * *  9 - TX0-P3
	 * *  8 - TX2-P2
	 * *  7 - TX1-P2
	 * *  6 - TX0-P2
	 * *  5 - TX2-P1
	 * *  4 - TX1-P1
	 * *  3 - TX0-P1
	 * *  2 - TX2-P0
	 * *  1 - TX1-P0
	 * *  0 - TX0-P0
	 *
	 * now we have to wait until the messages buffer TX0 has been
	 * transmitted before we may continue.
	 * this introduces a "delivery" latency every 12 packets where the
	 * queue has to fully empty.
	 * but then - as soon as TX2,TX1,TX0 are all "empty" the Priority
	 * gets reset back to highest, which should allow to delay the
	 * full idle-delay delay for a lot longer...
	 * this is the drawback of making sure that TX packets get sent
	 * out in sequence under any circumstances...
	 * we could provide an option to avoid this penalty by adding
	 * a module parameter to ignore these delays and schedule
	 * immediately...
	 *
	 * measured delay is 22us between 2 messages on the CAN bus
	 * but for the step from p0 to P11 the delay is about 104us.
	 * that is on a RPI (with 8MHz SPI Bus speed).
	 *
	 * part comes from the fact that the IRQ handler is scheduling
	 * the "normal" parts first and only then the messages data can
	 * get scheduled and transmitted
	 *
	 * possibly this can get reduce by changing the code to schedule
	 * a pending TX message first, but that may complicate things
	 * especially RX buffer overflows becomes more liklely...
	 *
	 * the other observation is that we still use too many interrupts
	 * possibly we could handle more transfers together and only trigger
	 * an irq on every second message - would half the interrupts by 2.
	 * the question is: is it worth the effort and complexity ?
	 */
	spin_lock_irqsave(&priv->lock,flags);

	/* if we are stopped then return busy */
	if (priv->tx_queue_stopped)
		goto return_busy;

	/* if next priority is -1, then we have to wait for all TX to
	 * clear, so stop the queue and return
	 */
	if (priv->tx_next_priority < 0)
		goto stop_queue;

	/* check the channel */
	tx = priv->tx_next_priority % 3;

	/* get priority to use */
	prio = (int)(priv->tx_next_priority / 3);

	/* check if the tx channel is available
	 * if not, then return with queue disabled
	 */
	if (priv->tx_len[tx])
		goto stop_queue;

	/* otherwise we fill in the message - to get released in irq */
	priv->tx_len[tx] = 0x80 + frame->can_dlc;

	/* calculate next priority */
	priv->tx_next_priority--;

	/* if priority is now -1, then stop queue */
	if (priv->tx_next_priority < 0) {
		priv->tx_queue_stopped = 1;
		netif_stop_queue(net);
	}

	/* and release lock continuing the more "mundane" stuff */
	spin_unlock_irqrestore(&priv->lock,flags);

	/* so fill in that buffer */
	buf = priv->transfers->transmit_tx[tx].message.data;
	/* set the TX priority */
	buf[0] = prio;

	/* transform the CAN message ID */
        if (frame->can_id & CAN_EFF_FLAG) {
                buf[1] = frame->can_id >> 21;
                buf[2] = (frame->can_id >> 13 & 0xe0) | 8 |
                        (frame->can_id >> 16 & 3);
                buf[3] = frame->can_id >> 8;
                buf[4] = frame->can_id;
        } else {
                buf[1] = frame->can_id >> 3;
                buf[2] = frame->can_id << 5;
                buf[3] = 0;
                buf[4] = 0;
        }
	/* set ID length */
        if (frame->can_id & CAN_RTR_FLAG)
                buf[5] = frame->can_dlc | 0x40;
        else
                buf[5] = frame->can_dlc;

	/* and copy the payload */
        memcpy(buf + 6, frame->data, frame->can_dlc);

	/* set length only if we optimize and support vary */
#ifdef SPI_HAVE_OPTIMIZE
	if (
		(priv->transfers->transmit_tx[tx].msg.is_optimized)
		&&
		(priv->transfers->transmit_tx[tx].message.trans.vary
			& SPI_OPTIMIZE_VARY_LENGTH)
		)
#endif
		priv->transfers->transmit_tx[tx].message.trans.len =
		  2 /* 2 bytes: write cmd + address TXBXCTRL */
		  + 6 /* 6 bytes: TXBXCTRL, TXBXSIDH, TXBXSIDL, TXBXESID8,
			 TXBXESID0 and TXBXDLC */
		  + frame->can_dlc /* real payload length */;

	/* transfer and forget */
	spi_async(priv->spi, &priv->transfers->transmit_tx[tx].msg);

	/* forget skb */
	kfree_skb(skb);

	/* and return message as delivered */
	return NETDEV_TX_OK;

stop_queue:
	/* stop queue */
	priv->tx_queue_stopped = 1;
	netif_stop_queue(net);

	/* unlock and return BUSY */
return_busy:
	spin_unlock_irqrestore(&priv->lock,flags);
	return NETDEV_TX_BUSY;
}

static int mcp2515a_get_berr_counter(const struct net_device *net,
				struct can_berr_counter *bec)
{
        struct mcp2515a_priv *priv = netdev_priv(net);

	bec->txerr = priv->transfers->read_status2.read_tec_rec.data[0];
	bec->rxerr = priv->transfers->read_status2.read_tec_rec.data[1];

	return 0;
}

static int mcp2515a_open(struct net_device *net)
{
        struct mcp2515a_priv *priv = netdev_priv(net);
        struct spi_device *spi = priv->spi;
        int ret;

	/* only open when the bitrate is defined
	 * - avoids unneccessary messages
	 */
	if (!priv->can.bittiming.tq && !priv->can.bittiming.bitrate)
		return -EINVAL;

	/* try to open the can device */
        ret = open_candev(net);
        if (ret)
                return ret;

	/* request the IRQ with the above flags */
        ret = request_irq(spi->irq,
			mcp2515a_interrupt_handler,
			IRQF_TRIGGER_FALLING,
			net->name,
			net);
        if (ret)
                goto err_candev;

	/* now configure the device */
        ret = mcp2515a_config(net);
        if (ret)
                goto err_irq;
	/* and wake the queue, telling it, that we are receving data */
        netif_wake_queue(net);

	/* and log a message */
	dev_info(&net->dev,
		"Started mcp2515 device with irq %i\n",
		spi->irq);

	/* and return */
        return 0;

err_irq:
	/* and free the IRQ */
        free_irq(spi->irq, net);
	/* send a syncronous reset */
	mcp2515a_reset(spi);
err_candev:
	dev_err(&net->dev,
		"MCP2515 Got an error opening the device - %i\n",ret);
	/* and close the device */
	close_candev(net);
	/* and return */
        return ret;
}

static int mcp2515a_stop(struct net_device *dev)
{
        struct mcp2515a_priv *priv = netdev_priv(dev);
        struct spi_device *spi = priv->spi;
	/* mark us as shut down */
	priv->is_shutdown = 1;

	/* first free the interrupt */
        free_irq(spi->irq, dev);

	/* send a syncronous reset
	 * - anything that is in-flight should stop after this...*/
        mcp2515a_reset(spi);

	/* close the device */
        close_candev(dev);
	/* and return OK */
        return 0;
}

/* the respective device operations */
static const struct net_device_ops mcp2515a_netdev_ops = {
        .ndo_open       = mcp2515a_open,
        .ndo_stop       = mcp2515a_stop,
        .ndo_start_xmit = mcp2515a_start_xmit,
};

/* the initialization and probing for this device */
static int mcp2515a_probe(struct spi_device *spi)
{
        struct net_device *net;
        struct mcp251x_platform_data *pdata = spi->dev.platform_data;
	struct mcp2515a_priv *priv;
        int ret;

	set_high();
	/* Platform data is required for osc freq */
	if (!pdata)
		return -ENODEV;

	/* check for device to be present */
	ret=mcp2515a_confirm_device(spi);
	if (ret)
		return ret;

	/* allocate the CAN structure */
	net = alloc_candev(sizeof(struct mcp2515a_priv),1);
        if (!net)
		return -ENOMEM;

	/* copy the driver data */
	dev_set_drvdata(&spi->dev, net);

	/* setting up as netdev */
	SET_NETDEV_DEV(net, &spi->dev);
	/*  link in the device ops */
	net->netdev_ops = &mcp2515a_netdev_ops;

	/* get the private data and fill in details */
	priv = netdev_priv(net);
	memset(priv,0,sizeof(*priv));

	/* fill in private data */
	spin_lock_init(&priv->lock);
	priv->spi = spi;
	priv->net = net;
        priv->can.clock.freq = pdata->oscillator_frequency / 2;
	priv->can.bittiming_const = & mcp2515a_bittiming_const;
        priv->can.ctrlmode_supported =
		CAN_CTRLMODE_3_SAMPLES
                | CAN_CTRLMODE_LOOPBACK
		| CAN_CTRLMODE_BERR_REPORTING
		| CAN_CTRLMODE_LISTENONLY
		| CAN_CTRLMODE_ONE_SHOT ;
        priv->can.do_set_mode = mcp2515a_do_set_mode;
	priv->can.do_get_berr_counter = mcp2515a_get_berr_counter;

	/* default TX-Priority */
	priv->tx_next_priority = TX_PRIORITY_MAX;

	/* allocate some buffers from DMA space */
	ret=mcp2515a_init_transfers(net);
	if (ret)
		goto error_out;

	/* and register the device */
        ret = register_candev(net);
        if (ret)
		goto error_transfers;

	/* return without errors */
	return 0;

error_transfers:
	mcp2515a_free_transfers(net);
error_out:
	free_candev(net);
	return ret;
}

static int mcp2515a_remove(struct spi_device *spi)
{
        struct net_device *net = dev_get_drvdata(&spi->dev);
	/* unregister the device */
        unregister_candev(net);
	/* free the memory for generated SPI messages*/
	mcp2515a_free_transfers(net);
	/* clear driver data */
        dev_set_drvdata(&spi->dev, NULL);
	/* free the device finally */
        free_candev(net);
	/* and return OK */
        return 0;
}

/* for Device tree matching */
static const struct of_device_id mcp2515a_match[] = {
        { .compatible = "mcp2515", },
        {}
};
MODULE_DEVICE_TABLE(of, mcp2515a_match);

/* and "normal" aliases */
static const struct spi_device_id mcp2515a_id_table[] = {
        { "mcp2515",   2515 },
        { "mcp2515a",  2515 },
        { },
};

/* power management */
static int mcp2515a_can_suspend(struct device *dev)
{
	return 0;
}

static int mcp2515a_can_resume(struct device *dev)
{
	return 0;
}

static SIMPLE_DEV_PM_OPS(mcp2515a_powermanagement_ops,
			mcp2515a_can_suspend,
			mcp2515a_can_resume);

static struct spi_driver mcp2515a_spi_driver = {
        .driver = {
                .name = DRV_NAME,
                .owner = THIS_MODULE,
		.of_match_table = mcp2515a_match,
#if CONFIG_PM_SLEEP
		.pm = &mcp2515a_powermanagement_ops,
#endif
        },
        .probe    = mcp2515a_probe,
        .remove   = mcp2515a_remove,
	.id_table = mcp2515a_id_table,
};

static int mcp2515a_init(void)
{
        return spi_register_driver(&mcp2515a_spi_driver);
}
module_init(mcp2515a_init);

static void mcp2515a_exit(void)
{
        spi_unregister_driver(&mcp2515a_spi_driver);
}
module_exit(mcp2515a_exit);

MODULE_DESCRIPTION("Driver for Microchip MCP2515 SPI CAN controller"
		" using asyncronous SPI-calls");
MODULE_AUTHOR("Martin Sperl");
MODULE_LICENSE("GPL");

/*
 * Driver for Broadcom BCM2835 SPI Controllers
 *
 * Copyright (C) 2013 Martin Sperl
 *
 * This driver is inspired by:
 * spi-ath79.c, Copyright (C) 2009-2011 Gabor Juhos <juhosg@openwrt.org>
 * spi-atmel.c, Copyright (C) 2006 Atmel Corporation
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

/* the MCP commands */
#define MCP2515_CMD_WRITE         0x02
#define MCP2515_CMD_READ          0x03
#define MCP2515_CMD_MODIFY        0x05
#define MCP2515_CMD_STATUS        0xA0
#define MCP2515_CMD_RXSTATUS      0xB0
#define MCP2515_CMD_RESET         0xC0
#define MCP2515_CMD_READ_RX(i)    (0x50+i<<2)
#define MCP2515_CMD_WRITE_TX(i)   (0x40+i<<1)
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
#define MCP2515_REG_EFLG          0x2D

#define MCP2515_REG_RXB0CTRL      0x60
#define MCP2515_REG_RXB1CTRL      0x70

#define MCP2515_REG_RXB_RXM_ANY   (3<<6)
#define MCP2515_REG_RXB_RXM_EXT   (2<<6)
#define MCP2515_REG_RXB_RXM_STD   (1<<6)
#define MCP2515_REG_RXB_RXM_FILTER (0<<6)

#define MCP2515_REG_RXB_BUKT      (1<<2)

#define MCP2515_REG_TXB0CTRL      0x30
#define MCP2515_REG_TXB1CTRL      0x40
#define MCP2515_REG_TXB2CTRL      0x50

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
#define MCP2515_REG_BFP_B1BFM     (1<<1) /* pin RX1 as interrupt on buffer full */
#define MCP2515_REG_BFP_B0BFM     (1<<0) /* pin RX0 as interrupt on buffer full */

#define MCP2515_REG_TXRTSCTRL     0x0d
#define MCP2515_REG_TXRTS_B2RTS   (1<<5) /* pin TX2 value */
#define MCP2515_REG_TXRTS_B1RTS   (1<<4) /* pin TX1 value */
#define MCP2515_REG_TXRTS_B0RTS   (1<<3) /* pin TX0 value */
#define MCP2515_REG_TXRTS_B2RTSM  (1<<2) /* pin TX2 as interrupt to send message */
#define MCP2515_REG_TXRTS_B1RTSM  (1<<1) /* pin TX1 as interrupt to send message */
#define MCP2515_REG_TXRTS_B0RTSM  (1<<0) /* pin TX0 as interrupt to send message */

#define MCP2515_REG_TEC           0x1c
#define MCP2515_REG_REC           0x1d


struct mcp2515a_transfers;

/* the driver structure */
struct mcp2515a_priv {
        struct can_priv    can;
        struct net_device *net;
        struct spi_device *spi;

	/* the DMA Buffer */
	dma_addr_t transfers_dma_addr;
	struct mcp2515a_transfers *transfers;
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
	if ((ret=mcp2515a_reset(spi)))
		return ret;
	/* now wait for 10 ms - as per documentation */
	mdelay(10);
	/* and read the canstat and other values */
	b[0]=MCP2515_CMD_READ;
	b[1]=MCP2515_REG_CANSTAT;
	b[2]=0;
	b[3]=0;
	if ((ret=spi_write_then_read(spi,b,2,b+2,2)))
		return ret;

	/* check canstat against magic numbers */
	/* CANSTAT */
	b[2]&=0xEE;
	if (b[2]!=0x80)
		goto mcp2515a_confirm_device_err;
	b[3]&=0x17;
	if (b[3]!=0x07)
		goto mcp2515a_confirm_device_err;
	/* ok, we have succeeded */
	return 0;

mcp2515a_confirm_device_err:
	dev_err(&spi->dev,
		"Found unexpected response from device: CANSTAT 0x%02x CANCTRL 0x%02x - not detected\n", 
		b[2], b[3]);
	return -ENODEV;
}

#define TRANSFER_WRITE_STRUCT(name,datalen) \
	struct { \
		u8 cmd; \
		u8 reg; \
		u8 data[datalen]; \
		struct spi_transfer trans;\
	} name;
#define TRANSFER_INIT(base,message) \
	spi_message_init(&base->message.msg);				\
	base->message.msg.is_dma_mapped=1;
#define TRANSFER_INIT_WRITE(base,message,xfer,regist)			\
	base->message.xfer.data;					\
	base->message.xfer.cmd = MCP2515_CMD_WRITE;			\
	base->message.xfer.reg = regist;				\
	base->message.xfer.trans.len=2+sizeof(base->message.xfer.data);	\
	base->message.xfer.trans.tx_buf=&base->message.xfer.cmd;	\
	base->message.xfer.trans.rx_buf=NULL;				\
	base->message.xfer.trans.tx_dma=base##_dma_addr			\
		+offsetof(struct mcp2515a_transfers,message.xfer.cmd);	\
	base->message.xfer.trans.rx_dma=0;				\
	base->message.xfer.trans.cs_change=1;				\
	spi_message_add_tail(&base->message.xfer.trans,&base->message.msg);
#define TRANSFER_READ_STRUCT(name,datalen) \
	struct { \
		u8 cmd; \
		u8 reg; \
		u8 data[datalen]; \
		struct spi_transfer trans_tx;\
		struct spi_transfer trans_rx;\
	} name;
#define TRANSFER_INIT_READ(base,message,xfer,regist)			\
	base->message.xfer.data;					\
	base->message.xfer.cmd = MCP2515_CMD_READ;			\
	base->message.xfer.reg = regist;				\
	base->message.xfer.trans_tx.len=2;				\
	base->message.xfer.trans_tx.tx_buf=&base->message.xfer.cmd; \
	base->message.xfer.trans_tx.rx_buf=NULL;			\
	base->message.xfer.trans_tx.tx_dma=base##_dma_addr		\
		+offsetof(struct mcp2515a_transfers,message.xfer.cmd); \
	base->message.xfer.trans_rx.rx_dma=0;				\
	spi_message_add_tail(&base->message.xfer.trans_tx,&base->message.msg); \
	base->message.xfer.trans_rx.len=sizeof(base->message.xfer.data); \
	base->message.xfer.trans_rx.rx_buf=&base->message.xfer.data; \
	base->message.xfer.trans_rx.tx_buf=NULL;			\
	base->message.xfer.trans_rx.rx_dma=base##_dma_addr		\
		+offsetof(struct mcp2515a_transfers,message.xfer.data); \
	base->message.xfer.trans_rx.rx_dma=0;				\
	base->message.xfer.trans_rx.cs_change=1;			\
	spi_message_add_tail(&base->message.xfer.trans_rx,&base->message.msg);

struct mcp2515a_transfers {
	/* the respective DMA buffers for RX/TX */
	char rx0_buffer[13];
	char rx1_buffer[13];
	char tx0_buffer[13];
	char tx1_buffer[13];
	char tx2_buffer[13];
	  /* the messages */
	struct {
		struct spi_message msg;
		TRANSFER_WRITE_STRUCT(setRXB0ctrl,1);
		TRANSFER_WRITE_STRUCT(setRXB1ctrl,1);
		TRANSFER_WRITE_STRUCT(setTXB0ctrl,1);
		TRANSFER_WRITE_STRUCT(setTXB1ctrl,1);
		TRANSFER_WRITE_STRUCT(setTXB2ctrl,1);
		TRANSFER_WRITE_STRUCT(seterrorcounter,2);
		TRANSFER_WRITE_STRUCT(changetoconfigmode,1);
		TRANSFER_WRITE_STRUCT(setpinctrl,2);
		TRANSFER_WRITE_STRUCT(setconfig,5);
		TRANSFER_WRITE_STRUCT(changetomode,1);
		TRANSFER_READ_STRUCT(readconfig,8);
	} config;
	struct {
		struct spi_message msg;
		TRANSFER_READ_STRUCT(status,1);/* this actually wastes some memory, but it is minimal, if it really matters at all with structure alignments*/
		TRANSFER_READ_STRUCT(rxstatus,1);
	} read_status;
	struct {
		struct spi_message msg;
		TRANSFER_READ_STRUCT(interr,3);
		TRANSFER_READ_STRUCT(errcount,2);
	} read_caninterr;
	struct {
		struct spi_message msg;
		TRANSFER_READ_STRUCT(rx0,13);
	} read_rx0;
	struct {
		struct spi_message msg;
		TRANSFER_WRITE_STRUCT(rx0,2); /* this actually wastes some memory, but it is minimal, if it really matters at all with structure alignments*/
	} clear_rx0;
};

static void mcp2515a_free_transfers(struct net_device* net)
{
	struct mcp2515a_priv *priv = netdev_priv(net);
	if (priv->transfers)
		dma_free_coherent(&priv->spi->dev,
				sizeof(struct mcp2515a_transfers), 
				priv->transfers,
				priv->transfers_dma_addr);
}

static int mcp2515a_init_transfers(struct net_device* net)
{
	struct mcp2515a_priv *priv = netdev_priv(net);
        struct device *dev = &priv->spi->dev;
	u8* data;
	/* first forece the coherent mask 
	   - a bit of a hack, but at least it works... */
        dev->coherent_dma_mask = 0xffffffff;

	/* allocate memory */
	priv->transfers = dma_zalloc_coherent(&priv->spi->dev, sizeof(struct mcp2515a_transfers), &priv->transfers_dma_addr, GFP_KERNEL);
	if (! priv->transfers) return -ENOMEM;

	/* now let us fill in the data structures */

	/* setting up receive policies for buffers */
	TRANSFER_INIT(priv->transfers,config);
	data=TRANSFER_INIT_WRITE(priv->transfers,config,setRXB0ctrl       ,MCP2515_REG_RXB0CTRL);
	data[0]=MCP2515_REG_RXB_RXM_ANY /* receive any message */
                | MCP2515_REG_RXB_BUKT; /* rollover to RXB1*/
	data=TRANSFER_INIT_WRITE(priv->transfers,config,setRXB1ctrl       ,MCP2515_REG_RXB1CTRL);
	data[0]=MCP2515_REG_RXB_RXM_ANY; /* receive any message */

	/* setting up transmit policies for buffers - and resetting pending transmits */
	data=TRANSFER_INIT_WRITE(priv->transfers,config,setTXB0ctrl       ,MCP2515_REG_TXB0CTRL);
	data[0]=MCP2515_REG_TXB_TXP_0;
	data=TRANSFER_INIT_WRITE(priv->transfers,config,setTXB1ctrl       ,MCP2515_REG_TXB1CTRL);
	data[0]=MCP2515_REG_TXB_TXP_1;
	data=TRANSFER_INIT_WRITE(priv->transfers,config,setTXB2ctrl       ,MCP2515_REG_TXB2CTRL);
	data[0]=MCP2515_REG_TXB_TXP_2;

	/* clear TEC/REC */
	data=TRANSFER_INIT_WRITE(priv->transfers,config,seterrorcounter   ,MCP2515_REG_TEC);
	data[0]=0; /* tec */
	data[1]=0; /* rec */

	/* move to config mode */
	data=TRANSFER_INIT_WRITE(priv->transfers,config,changetoconfigmode,MCP2515_REG_CANCTRL);
	data[0]=MCP2515_REG_CANCTRL_REQOP_CONFIG;

	/* clear bit ctrl */
	data=TRANSFER_INIT_WRITE(priv->transfers,config,setpinctrl        ,MCP2515_REG_BFPCTRL);
	data[0]=0; /* BFPCTRL - high impedance */
	data[1]=0; /* TXRTSCTRL - no functionality*/

	/* configure CNF and interrupt-registers */
	data=TRANSFER_INIT_WRITE(priv->transfers,config,setconfig     ,MCP2515_REG_CNF3);
	data[0]=data[1]=data[2]=0; /* CNF3,CNF2,CNF1 */
	data[3]=0xff; /* INTE - enable all interupt sources */
	data[4]=0; /* INTF - clear all interupt sources */

	/* and change to the final mode */
	data=TRANSFER_INIT_WRITE(priv->transfers,config,changetomode     ,MCP2515_REG_CANCTRL);
	data[0]=MCP2515_REG_CANCTRL_REQOP_NORMAL; /* the mode we want to enter */

	/* initiate read of basic configs */
	data=TRANSFER_INIT_READ(priv->transfers,config,readconfig        ,MCP2515_REG_CNF3);

	/* The read status transfer */
	TRANSFER_INIT(priv->transfers,read_status);
	priv->transfers->read_status.msg.complete=NULL;
	data=TRANSFER_INIT_READ(priv->transfers,read_status,status       ,MCP2515_REG_CANCTRL);
	priv->transfers->read_status.status.cmd=MCP2515_CMD_STATUS;
	priv->transfers->read_status.status.trans_tx.len=1;
	data=TRANSFER_INIT_READ(priv->transfers,read_status,rxstatus     ,MCP2515_REG_CANCTRL);
	priv->transfers->read_status.rxstatus.cmd=MCP2515_CMD_RXSTATUS;
	priv->transfers->read_status.rxstatus.trans_tx.len=1;

	/* the caninterr values */
	TRANSFER_INIT(priv->transfers,read_caninterr);
	/* possibly also set interrupts disabled here - where exactly? - there could be a race condition !*/
	data=TRANSFER_INIT_READ(priv->transfers,read_caninterr,interr         ,MCP2515_REG_CANINTE);
	data=TRANSFER_INIT_READ(priv->transfers,read_caninterr,errcount       ,MCP2515_REG_TEC);

	/* and return ok */
	return 0;
}

static int mcp2515a_config(struct net_device *net)
{
        struct mcp2515a_priv *priv = netdev_priv(net);
        struct spi_device *spi = priv->spi;
        struct can_bittiming *bt = &priv->can.bittiming;
	int ret=0;
	u8 mode=0;

	/* select the mode we want */
	if (priv->can.ctrlmode & CAN_CTRLMODE_LOOPBACK) {
		mode=MCP2515_REG_CANCTRL_REQOP_LOOPBACK;
        } else if (priv->can.ctrlmode & CAN_CTRLMODE_LISTENONLY) {
		mode=MCP2515_REG_CANCTRL_REQOP_LISTEN;
        } else {
		mode=MCP2515_REG_CANCTRL_REQOP_NORMAL;
	}
	/* set one-shot mode if needed */
	if (priv->can.ctrlmode & CAN_CTRLMODE_ONE_SHOT)
		mode|=MCP2515_REG_CANCTRL_OSM;
	priv->transfers->config.changetomode.data[0]=mode;

	/* the Bit speed config */
	/* CNF3 */
	priv->transfers->config.setconfig.data[0]=
		bt->phase_seg2 - 1;
	/* CNF2 */
	priv->transfers->config.setconfig.data[1]=
		(priv->can.ctrlmode & CAN_CTRLMODE_3_SAMPLES ? 0xc0 : 0x80)
		| (bt->phase_seg1 - 1) << 3 
		| (bt->prop_seg - 1)
		;
	/* CNF1 */
	priv->transfers->config.setconfig.data[2]=
		(bt->sjw - 1) << 6 
		| (bt->brp - 1);

	/* execute transfer */
	ret=spi_sync(spi,&priv->transfers->config.msg);
	if (ret)
		return ret;
	/* dump the CNF */
	netdev_info(net, "configured CTRL: 0x%02x CNF: 0x%02x 0x%02x 0x%02x\n",
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
	if (mode==CAN_MODE_START)
		return mcp2515a_config(net);
	/* otherwise send message that it is not supported... */
	return -EOPNOTSUPP;
}

/* the interrupt-handler for this device */
static irqreturn_t mcp2515a_interrupt_handler(int irq, void *dev_id)
{
        //struct net_device *net = dev_id;
        //struct mcp2515a_priv *priv = netdev_priv(net);
	printk(KERN_DEBUG "Interrupt handled\n");
	/* we will just schedule a transfer */
	/* disable the interrupt - one of the handlers we reenable it*/
	disable_irq_nosync(irq);
	/* return with a andled interrupt */
        return IRQ_HANDLED;
}

static netdev_tx_t mcp2515a_start_xmit(struct sk_buff *skb,
                                      struct net_device *dev)
{
	/* need to fill in some code... */
	return NETDEV_TX_BUSY;
}

static int mcp2515a_open(struct net_device *net)
{
        struct mcp2515a_priv *priv = netdev_priv(net);
        struct spi_device *spi = priv->spi;
	struct mcp251x_platform_data *pdata = spi->dev.platform_data;
        int ret;
	int flags=IRQF_ONESHOT;

	/* just in case identify the device again */
        ret = mcp2515a_confirm_device(spi);
        if (ret)
                return ret;

	/* try to open the can device */
        ret = open_candev(net);
        if (ret)
                return ret;

	/* the interrupt flag calculation */
	if (pdata->irq_flags)
		flags |= pdata->irq_flags;
	else
		flags |= IRQF_TRIGGER_FALLING;

	/* request the IRQ with the above flags */
        ret = request_irq(spi->irq, 
			mcp2515a_interrupt_handler,
			flags,
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
	dev_info(&net->dev,"Started device with irq %i and irq flags=0x%x\n",spi->irq,flags);

	/* and return */
        return 0;

err_irq:
	/* send a syncronous reset */
	mcp2515a_reset(spi);
	/* and free the IRQ */
        free_irq(spi->irq, net);
err_candev:
	/* and close the device */
	close_candev(net);
	/* and return */
        return ret;
}

static int mcp2515a_stop(struct net_device *dev)
{
        struct mcp2515a_priv *priv = netdev_priv(dev);
        struct spi_device *spi = priv->spi;

	/* send a syncronous reset */
        mcp2515a_reset(spi);
	/* close the device */
        close_candev(dev);
	/* free the interrupt */
        free_irq(spi->irq, dev);
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
	net->flags |= IFF_ECHO;

	/* get the private data and fill in details */
	priv = netdev_priv(net);

	/* fill in private data */
	priv->spi = spi;
	priv->net = net;
        priv->can.clock.freq = pdata->oscillator_frequency / 2;
	priv->can.bittiming_const = & mcp2515a_bittiming_const;
        priv->can.ctrlmode_supported = 
		CAN_CTRLMODE_3_SAMPLES
                | CAN_CTRLMODE_LOOPBACK 
		| CAN_CTRLMODE_LISTENONLY
		| CAN_CTRLMODE_ONE_SHOT ;
        priv->can.do_set_mode = mcp2515a_do_set_mode;

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
	/* free the memory */
	mcp2515a_free_transfers(net);
        unregister_candev(net);
        dev_set_drvdata(&spi->dev, NULL);
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

MODULE_DESCRIPTION("Driver for Microchip MCP2515 SPI CAN controller using asyncronous SPI-calls");
MODULE_AUTHOR("Martin Sperl");
MODULE_LICENSE("GPL");


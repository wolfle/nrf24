/*
 * Simple network device driver for nrf24l01+ the SPI rf devices
 *
 * Copyright (C) 2016 Wolfle
 *	Bob Guo <wolfle@softhome.net>
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
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */
 
#include <linux/init.h>
#include <linux/net.h>
#include <linux/in.h>
#include <linux/if.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/circ_buf.h>

#include <linux/spi/spi.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>

#include <linux/netdevice.h>
#include <linux/if_arp.h>
#include <linux/skbuff.h>

#include <asm/uaccess.h>

#include <linux/of.h>
#include <linux/of_gpio.h>


#include "nrf24l01.h"
#include "nrf24.h"

#define NRF24_DEBUG

#define HIGH_QUEUE_LEN	10
#define LOW_QUEUE_LEN	8


//in skb data, first one byte is pipe number, then the payload data	

struct nrf24_data {
	struct spi_device	*spi;
	struct net_device *dev;
	struct sk_buff_head send_queue;		/* Packets awaiting transmission */
	int			ce_gpio, irq_gpio;
	/* buf and len used for spi transfers*/
	u8			buf[32+1], len, ch, mode;//rf channel and enable flags of pipes
};

/*-------------------------------------------------------------------------*/

#define CHECKOUT(x,r,o) if((r=(x))<0)goto o;
#define CE_H 	gpio_set_value(nrf24->ce_gpio,1)
#define CE_L	gpio_set_value(nrf24->ce_gpio,0)

/*  
 * len should be the sum of reading command and the expected readin data length.
 * full duplex needed
 */
static inline ssize_t nrf24_spi_trans(struct nrf24_data* nrf24)
{
	struct spi_transfer	t = {
			.rx_buf		= nrf24->buf,
			.tx_buf		= nrf24->buf,
			.len		= nrf24->len,
		};
		
	return spi_sync_transfer(nrf24->spi,&t,1);
}

static inline ssize_t nrf24_spi_write(struct nrf24_data* nrf24)
{
	return spi_write(nrf24->spi, nrf24->buf, nrf24->len);
}

/* register read to buf[1...], status reg in buf[0], len = reg length + 1 */
static ssize_t read_register(struct nrf24_data *nrf24, u8 reg)
{
	nrf24->buf[0] = R_REGISTER | ( REGISTER_MASK & reg );
	if(reg == RX_ADDR_P0 || reg == RX_ADDR_P1 || reg == TX_ADDR)
			nrf24->len = 5+1;
	else nrf24->len = 1+1;
	return nrf24_spi_trans(nrf24);
}

static inline ssize_t read_status(struct nrf24_data *nrf24){
	nrf24->buf[0]=NOP;
	nrf24->len=1;
	return nrf24_spi_trans(nrf24);
}
//return -1 on error; 0 no receiving power; 1 receiving power detected
static inline char read_rpd(struct nrf24_data *nrf24){
	return read_register(nrf24,RPD)<0? -1 : nrf24->buf[1];
}
/* set register to val*/
static ssize_t set_register(struct nrf24_data *nrf24, u8 reg, u8 val)
{
	 if(reg == RX_ADDR_P0 || reg == RX_ADDR_P1 || reg == TX_ADDR)return -EINVAL;
	 nrf24->buf[0] = W_REGISTER | ( REGISTER_MASK & reg );
	 nrf24->buf[1] = val;
	 nrf24->len=2;
	 return nrf24_spi_write(nrf24);
}

/* set address register of P0 P1 TX to val in buf[1..5] according len */
static ssize_t set_address(struct nrf24_data *nrf24, u8 reg)
{
	nrf24->buf[0] = W_REGISTER | ( REGISTER_MASK & reg );
	nrf24->len=1+5;
	if(reg == RX_ADDR_P0 || reg == RX_ADDR_P1 || reg == TX_ADDR)
			return nrf24_spi_write(nrf24);
	else return -EINVAL;
}

static inline ssize_t flush_tx(struct nrf24_data * nrf24)
{
	nrf24->buf[0]=FLUSH_TX;
	nrf24->len=1;
	return nrf24_spi_write(nrf24);
}

static inline ssize_t flush_rx(struct nrf24_data * nrf24)
{
	nrf24->buf[0]=FLUSH_RX;
	nrf24->len=1;
	return nrf24_spi_write(nrf24);
}

static inline ssize_t activate(struct nrf24_data * nrf24)
{
	nrf24->buf[0]=ACTIVATE;
	nrf24->buf[1]=ACTIVATE_MAGIC;
	nrf24->len=2;
	return nrf24_spi_write(nrf24);
}

/* buf[0] is the pipe number; only for prx mode */
static ssize_t __write_playload(struct nrf24_data * nrf24, char * buf, size_t len)
{
		buf[0]|=W_ACK_PAYLOAD;
		if(len>33)len=33;
		return spi_write(nrf24->spi,buf,len);
}

// only the first 1-33 bytes transmitted, 0 byte is the pipe num
static ssize_t write_payload(struct nrf24_data *nrf24)
{
	ssize_t res=0;
	struct sk_buff * skb;
	while((skb=skb_dequeue(&nrf24->send_queue))){
		if(skb->len>33){
			dev_err(&nrf24->spi->dev,"Packet too big. Truncated.");
			skb->len=33;
		}
		if(skb->head[0]>5){
			dev_err(&nrf24->spi->dev,"Invalid pipe number: %u, packet dropped.",skb->head[0]);
			++nrf24->dev->stats.tx_dropped;
		}else{
			CHECKOUT(__write_playload(nrf24,skb->head,skb->len),res,out) //pipe in head[0]
			++nrf24->dev->stats.tx_packets;
		}
		//memcpy(nrf24->buf,skb->data,skb->len);
		//skb_pull(skb,skb->len);
		dev_kfree_skb_any(skb);
		CHECKOUT(read_status(nrf24),res,out)
		if(nrf24->buf[0]&TX_FULL)break;
	}
	if(nrf24->send_queue.qlen<LOW_QUEUE_LEN)netif_wake_queue(nrf24->dev); //if qlen < 8 enable the socket send traffic
out:	
	return res;
}
/* after return, payload size in buf[1], status reg in buf[0] */
static inline ssize_t get_payload_size(struct nrf24_data *nrf24)
{
	nrf24->buf[0] = R_RX_PL_WID;
	nrf24->len=2;
	return nrf24_spi_trans(nrf24);
}
//payload start at buf[1], length=len-1, status reg in buf[0]
static ssize_t read_payload(struct nrf24_data *nrf24)
{
	ssize_t res;
	
	CHECKOUT(get_payload_size(nrf24),res,out);
	if(!nrf24->buf[1]){
		dev_err(&nrf24->spi->dev,"Zero payload length, deactivated?");
		activate(nrf24);
		CHECKOUT(get_payload_size(nrf24),res,out);
	}
	if(nrf24->buf[1]>32||!nrf24->buf[1]){ //invalid payload size found
		flush_rx(nrf24);
		++nrf24->dev->stats.rx_dropped;
		if(nrf24->buf[1])dev_err(&nrf24->spi->dev,"Invalid payload length: %u, rx packets dropped.",nrf24->buf[1]);
		res=-EINVAL;
		goto out;
	}
	nrf24->buf[0]=R_RX_PAYLOAD;
	nrf24->len=1+nrf24->buf[1];
	res=nrf24_spi_trans(nrf24);
out:
	return res;
}

static inline ssize_t set_channel(struct nrf24_data * nrf24)
{
	return set_register(nrf24, RF_CH, nrf24->ch<126?nrf24->ch:126); //max channel is 127,use 126 for wide channel
}

static inline ssize_t power_switch(struct nrf24_data * nrf24, u8 powerup){
	ssize_t res;
	CHECKOUT(read_register(nrf24,CONFIG),res,out)
	if(powerup)nrf24->buf[1]|=PWR_UP;
	else nrf24->buf[1]&=~PWR_UP;
	CHECKOUT(set_register(nrf24,CONFIG,nrf24->buf[1]),res,out)
out:
	return res;
}

static netdev_tx_t nrf24_send_packet(struct sk_buff *skb,struct net_device *dev){
	struct nrf24_data *nrf24 = netdev_priv(dev);
//	if (netif_queue_stopped(dev))
//		return NETDEV_TX_BUSY;
//	netif_stop_queue (dev);

	skb_queue_tail(&nrf24->send_queue, skb);
	netif_trans_update(dev);//dev->trans_start = jiffies;  x = dev_trans_start(dev) to read the saved jiffies
	if(nrf24->send_queue.qlen>HIGH_QUEUE_LEN)netif_stop_queue(dev);
	return NETDEV_TX_OK;
}

static ssize_t receive_packet(struct nrf24_data *nrf24){
	struct sk_buff * skb;
	ssize_t res;
	u8 pipe;
	do{
		CHECKOUT(read_payload(nrf24),res,out)
		pipe=(nrf24->buf[0]>>1) & 0b111;
		nrf24->buf[0]=pipe; //set pipe number
		if (!(skb = netdev_alloc_skb(nrf24->dev,nrf24->len))) {
			dev_err(&nrf24->spi->dev,"memory squeeze, dropping packet\n");
			++nrf24->dev->stats.rx_dropped;
			res=-ENOMEM;
			break;
		} else {
			memcpy(skb_put(skb, nrf24->len), nrf24->buf, nrf24->len);
			skb->pkt_type=PACKET_HOST;
			skb->dev=nrf24->dev;
			skb->protocol = htons(ETH_P_NONE);
			skb_reset_mac_header(skb);
			skb->ip_summed=CHECKSUM_UNNECESSARY;
			netif_rx(skb);
			++nrf24->dev->stats.rx_packets;
			nrf24->dev->last_rx=jiffies;
		}
		CHECKOUT(read_status(nrf24),res,out)
	}while((nrf24->buf[0]&0b1110)!=0b1110);
out:
	return res;
}

static irqreturn_t nrf24_handler(int irq, void *dev)
{
	int res;
	u8 status;
	struct nrf24_data * nrf24 = (struct nrf24_data *)dev;
	CE_L;
	CHECKOUT(read_status(nrf24),res,err);
	status=nrf24->buf[0];
	//what happened
	if(status & TX_DS){ //tx ok, push tx payloads if available, else do nothing
		CHECKOUT(write_payload(nrf24),res,err)
	}else if(status & MAX_RT){ //tx fail, in prx mode this maybe means all ack are lost
//		flush_tx(nrf24);
//		++nrf24->dev->stats.tx_dropped;
//		--nrf24->dev->stats.tx_packets;
		dev_err(&nrf24->spi->dev,"MAX_RT happened...ghost exists");
	}
	if(status & RX_DR){ //rx ready, read out the payloads
		receive_packet(nrf24);
	}

	CHECKOUT(set_register(nrf24,STATUS,0x70),res,err); // clear status register to reset irq pin
out:
	CE_H;
	return IRQ_HANDLED;
err:
	dev_err(&nrf24->spi->dev,"SPI transfer error: %d",res);
	goto out;
}

//Only when the interface is down can the channel or addresses or mode be changed
//use ifreq.ifr_flags (short) to exchange info. IS IT OK?		
static int nrf24_ioctl(struct net_device *dev, struct ifreq *ifr, int cmd)
{
	struct nrf24_data	*nrf24=netdev_priv(dev);
	
	switch (cmd) {
	/* read requests */			
	case GET_CHANNEL:
		ifr->ifr_flags=nrf24->ch;
		break;
	case GET_PIPES:
		ifr->ifr_flags=nrf24->mode;
		break;
	case GET_RPD:
		if(dev->flags&IFF_UP)return -EBUSY;
		ifr->ifr_flags=read_rpd(nrf24);
		if(ifr->ifr_flags==-1)return -ENODEV;
		break;
	/* write requests */
	case SET_CHANNEL:
		if(dev->flags&IFF_UP)return -EBUSY;
		nrf24->ch=(u8)ifr->ifr_flags;
		if(nrf24->ch>126)nrf24->ch=126;
		break;
	case SET_PIPES:
		if(dev->flags&IFF_UP)return -EBUSY;
		nrf24->mode=(u8)ifr->ifr_flags;
		break;
	default:
		break;
	}
	return 0;
}

static int nrf24_set_mac_address(struct net_device *dev, void *addr)
{
	struct sockaddr *sa = (struct sockaddr *) addr;
	if(dev->flags&IFF_UP)return -EBUSY;
	memcpy(dev->dev_addr, sa->sa_data, dev->addr_len);
	return 0;
}

static int nrf24_open(struct net_device *dev)
{
	int			res;
	struct nrf24_data	*nrf24=netdev_priv(dev);

	if(dev->dev_addr[0]==0 || nrf24->mode == 0)return -EINVAL;
	
	//set pipe address according to dev_addr
	memcpy(nrf24->buf+1,dev->dev_addr,5);
	CHECKOUT(set_address(nrf24,RX_ADDR_P0),res,out)
	memcpy(nrf24->buf+1,dev->dev_addr+5,5);
	CHECKOUT(set_address(nrf24,RX_ADDR_P1),res,out)
	CHECKOUT(set_register(nrf24,RX_ADDR_P2,*(dev->dev_addr+10)),res,out)
	CHECKOUT(set_register(nrf24,RX_ADDR_P3,*(dev->dev_addr+11)),res,out)
	CHECKOUT(set_register(nrf24,RX_ADDR_P4,*(dev->dev_addr+12)),res,out)
	CHECKOUT(set_register(nrf24,RX_ADDR_P5,*(dev->dev_addr+13)),res,out)

	CHECKOUT(set_channel(nrf24),res,out) //set channel
	CHECKOUT(set_register(nrf24,EN_RXADDR,nrf24->mode),res,out) //set mode(enable pipes)

	CHECKOUT(power_switch(nrf24,1),res,out)
	//enable_irq(nrf24->spi->irq);
	CHECKOUT(request_threaded_irq(dev->irq, NULL, nrf24_handler, IRQF_ONESHOT|IRQF_TRIGGER_FALLING, "nRF24L01+", nrf24),res,out)
	
	netif_start_queue(dev);
	CE_H;
out:
	if(res)dev_err(&nrf24->spi->dev,"open return %d",res);
	return res;
}

static ssize_t nrf24_reset(struct nrf24_data * nrf24)
{
	int retval=0;
	
	CE_L;
	CHECKOUT(set_register(nrf24,CONFIG,0),retval,out)	
	CHECKOUT(set_register(nrf24,CONFIG,0b1001),retval,out) //8bit crc, power down, prx mode
#ifdef NRF24_DEBUG
	CHECKOUT(read_register(nrf24,CONFIG),retval,out); 
	if(nrf24->buf[1] != 0b1001) dev_err(&nrf24->spi->dev,"Set CONFIG failed! %#x",nrf24->buf[1]);
#endif

	CHECKOUT(flush_rx(nrf24),retval,out);
	
	CHECKOUT(flush_tx(nrf24),retval,out);
	
	CHECKOUT(set_register(nrf24,STATUS,0b1110000 ),retval,out);
//	CHECKOUT(read_status(nrf24),retval,out);
	CHECKOUT(read_register(nrf24,STATUS),retval,out);
	if(nrf24->buf[0] != 0b1110 || nrf24->buf[0]!=nrf24->buf[1]) {
		dev_err(&nrf24->spi->dev,"Error STATUS: %x %x\n",nrf24->buf[0],nrf24->buf[1]);
		read_status(nrf24);
		dev_err(&nrf24->spi->dev,"Error STATUS: %x\n",nrf24->buf[0]);
		retval=-ENODEV;
	}
out:
	if(retval)dev_err(&nrf24->spi->dev,"Error in reset: %d.\n",retval);
	return retval;
}

static int nrf24_stop(struct net_device *dev)
{
	struct nrf24_data	*nrf24=netdev_priv(dev);
	struct sk_buff * skb;
	int			res;
	nrf24_reset(nrf24);
	netif_stop_queue(dev);
	//disable_irq(nrf24->spi->irq);
	free_irq(dev->irq,nrf24);
	while ((skb = skb_dequeue(&nrf24->send_queue)))
		dev_kfree_skb(skb);
	
out:
	return res;
}

static struct net_device_stats *nrf24_stats(struct net_device *dev)
{
	return &dev->stats;
}

static const struct net_device_ops netdev_ops = {
	.ndo_open	     = nrf24_open,
	.ndo_stop	     = nrf24_stop,
	.ndo_start_xmit      = nrf24_send_packet,
	.ndo_do_ioctl 	     = nrf24_ioctl,
	.ndo_set_mac_address = nrf24_set_mac_address,
	.ndo_get_stats       = nrf24_stats,	
};


static ssize_t __init nrf24_initialize(struct nrf24_data * nrf24)
{
	ssize_t retval=0;
	
	CHECKOUT(nrf24_reset(nrf24),retval,out); //detect presence of nrf24l01+
	//FIXME: is it neccesary to set ard and arc for prx mode?
	CHECKOUT(set_register(nrf24,SETUP_RETR,(1 << ARD) | (0b101 << ARC)),retval,out); //500us; retransmit 5 times
#ifdef NRF24_DEBUG
	CHECKOUT(read_register(nrf24,SETUP_RETR),retval,out);
	if(nrf24->buf[1] != ((1 << ARD) | (0b101 << ARC)) )dev_err(&nrf24->spi->dev,"Set SETUP_RETR failed! %#x",nrf24->buf[1]);
#endif	

	CHECKOUT(set_register(nrf24,RF_SETUP,0b1111),retval,out); //PA max,  2Mbps
#ifdef NRF24_DEBUG
	CHECKOUT(read_register(nrf24,RF_SETUP),retval,out); 
	if(nrf24->buf[1] != 0b1111) dev_err(&nrf24->spi->dev,"Set RF_SETUP failed! %#x",nrf24->buf[1]);
#endif	
	nrf24->ch=2;
	CHECKOUT(set_channel(nrf24),retval,out);
#ifdef NRF24_DEBUG
	CHECKOUT(read_register(nrf24,RF_CH),retval,out); 
	if(nrf24->buf[1] != nrf24->ch) dev_err(&nrf24->spi->dev,"Set RF_CH failed! %#x",nrf24->buf[1]);
#endif	
	
	CHECKOUT(set_register(nrf24,FEATURE,0b111 ),retval,out);
#ifdef NRF24_DEBUG
	CHECKOUT(read_register(nrf24,FEATURE),retval,out);
	if((nrf24->buf[1]&0b111)!=0b111 )
		dev_err(&nrf24->spi->dev,"Set FEATURE after toggle failed! %#x %#x",nrf24->buf[0], nrf24->buf[1]);
#endif	

	CHECKOUT(set_register(nrf24,EN_AA,0b111111),retval,out); //enable auto ack on all pipes
#ifdef NRF24_DEBUG
	CHECKOUT(read_register(nrf24,EN_AA),retval,out);
	if(nrf24->buf[1] != 0b111111) dev_err(&nrf24->spi->dev,"Set EN_AA failed! %#x",nrf24->buf[0]);
#endif	
	
	CHECKOUT(set_register(nrf24,DYNPD,0b111111),retval,out); //enable dynamics payload length on all pipes
#ifdef NRF24_DEBUG
	CHECKOUT(read_register(nrf24,DYNPD),retval,out); 
	if(nrf24->buf[1] != 0b111111) dev_err(&nrf24->spi->dev,"Set DYNPD failed! %#x",nrf24->buf[0]);
#endif	
	nrf24->mode=0b1;//only enable pipe 0
	CHECKOUT(set_register(nrf24,EN_RXADDR,nrf24->mode),retval,out)
#ifdef NRF24_DEBUG
	CHECKOUT(read_register(nrf24,EN_RXADDR),retval,out)
	if(nrf24->buf[1] != 0b1) dev_err(&nrf24->spi->dev,"Set EN_RXADDR failed! %#x",nrf24->buf[0]);
#endif	
	
	CHECKOUT(set_register(nrf24,SETUP_AW,0b11),retval,out); // 5 bytes address
#ifdef NRF24_DEBUG
	CHECKOUT(read_register(nrf24,SETUP_AW),retval,out); 
	if(nrf24->buf[1] != 0b11) dev_err(&nrf24->spi->dev,"Set SETUP_AW failed! %#x",nrf24->buf[0]);
#endif

	CHECKOUT(set_register(nrf24,RX_PW_P0,32),retval,out); // max rx payload length
	CHECKOUT(set_register(nrf24,RX_PW_P1,32),retval,out); // max rx payload length
	CHECKOUT(set_register(nrf24,RX_PW_P2,32),retval,out); // max rx payload length
	CHECKOUT(set_register(nrf24,RX_PW_P3,32),retval,out); // max rx payload length
	CHECKOUT(set_register(nrf24,RX_PW_P4,32),retval,out); // max rx payload length
	CHECKOUT(set_register(nrf24,RX_PW_P5,32),retval,out); // max rx payload length
	
	CHECKOUT(activate(nrf24),retval,out);
	
out:
	if(retval)dev_err(&nrf24->spi->dev,"Error in initialization: %d.\n",retval);
	return retval;
}


/*-------------------------------------------------------------------------*/

static void __init dev_setup(struct net_device *dev){
	struct nrf24_data * nrf24=netdev_priv(dev);
	dev->netdev_ops = &netdev_ops;
	dev->flags           |= IFF_NOARP|IFF_POINTOPOINT;
	dev->features        |= NETIF_F_HW_CSUM;
	//dev->watchdog_timeo = TIMEOUT;
	dev->type = ARPHRD_NONE;
	dev->mtu = 32+1;
	dev->addr_len = 5+5+1+1+1+1;
	dev->tx_queue_len=10;
	nrf24->dev=dev;
	skb_queue_head_init(&nrf24->send_queue);
}

static int __init nrf24_probe(struct spi_device *spi)
{
	struct net_device *dev;
	struct nrf24_data	*nrf24;
	struct device_node *on = spi->dev.of_node;
	int			gpio_irq, gpio_ce, status=0;

	//get params from device tree
	if(!on){
		dev_err(&spi->dev,"No nrf24l01+ device node found in device tree.\n");
		return -ENODEV;
	}
	
	gpio_irq = of_get_named_gpio(on, "gpio-irq", 0);
	gpio_ce  = of_get_named_gpio(on, "gpio-ce", 0);

	if(gpio_irq==-EPROBE_DEFER || gpio_ce==-EPROBE_DEFER)
		return -EPROBE_DEFER;

	if(gpio_irq<0 || gpio_ce<0){
		dev_err(&spi->dev,"No gpio-irq or gpio-ce in device tree node.\n");
		return -ENXIO;
	}
		
	if(gpio_is_valid(gpio_ce)){
		if(devm_gpio_request_one(&spi->dev,gpio_ce,GPIOF_OUT_INIT_LOW,"nrf24 chip enable")){
			dev_err(&spi->dev, "gpio ce request failed.\n");
			return -ENXIO;
		}
	}
	if(gpio_is_valid(gpio_irq)){
		if(devm_gpio_request_one(&spi->dev,gpio_irq,GPIOF_IN,"nrf24 irq")){
			dev_err(&spi->dev, "gpio irq request failed.\n");
			status=-ENXIO;
			goto gpio;
		}
	}
	spi->irq=gpio_to_irq(gpio_irq);
	if(spi->irq<0){
		dev_err(&spi->dev,"Invalid irq for gpio-irq.\n");
		status=-ENXIO;
		goto gpio1;
	}
	// Is toggling it right?
//		gpio_direction_input(nrf24->cs_gpio);
//		if(gpio_direction_output(nrf24->cs_gpio, 0) < 0) goto out;
	/* Allocate driver data */
	dev = alloc_netdev(sizeof(struct nrf24_data), "nrf%d", NET_NAME_ENUM,dev_setup);
	if(!dev){
		dev_err(&spi->dev,"Allocate net dev failed.\n");
//		dev_dbg(&spi->dev, "probe/ENOMEM\n");
		status=-ENOMEM;
		goto gpio;
	}
	nrf24 = netdev_priv(dev);
	/* Initialize the spi driver data */

	nrf24->spi = spi;
	spi_set_drvdata(spi, nrf24);
		
	//disable_irq(nrf24->spi->irq);
	dev->irq = spi->irq;
	nrf24->ce_gpio=gpio_ce;
	nrf24->irq_gpio=gpio_irq;
	// spi init
//	spin_lock_irq(&nrf24->spi_lock);
	spi = spi_dev_get(nrf24->spi);
//	spin_unlock_irq(&nrf24->spi_lock);

	if (spi == NULL){
		status = -ESHUTDOWN;
		goto mem;
	}
	
	CHECKOUT(spi_setup(spi),status,mem);
	spi_dev_put(spi);

	CHECKOUT(nrf24_initialize(nrf24),status,mem);
	CHECKOUT(register_netdev(dev),status,mem);
	
	return status;
mem:
	free_netdev(dev);
gpio1:
	devm_gpio_free(&spi->dev, gpio_irq);
gpio:
	devm_gpio_free(&spi->dev, gpio_ce);
	return status;
}

static int __exit_call nrf24_remove(struct spi_device *spi)
{
	struct nrf24_data	*nrf24 = spi_get_drvdata(spi);
	
//	if(nrf24->dev->flags&IFF_UP)nrf24_stop(nrf24->dev); //it seems remove will auto call stop 
	activate(nrf24); //deactivate


	unregister_netdev(nrf24->dev);
	free_netdev(nrf24->dev);
	
//	spin_lock_irq(&nrf24->spi_lock);
	nrf24->spi = NULL;
	spi_set_drvdata(spi, NULL);
//	spin_unlock_irq(&nrf24->spi_lock);

	devm_gpio_free(&spi->dev, nrf24->irq_gpio);
	devm_gpio_free(&spi->dev, nrf24->ce_gpio);
	
	return 0;
}

static int __maybe_unused nrf24_suspend(struct device *dev){
	struct nrf24_data *nrf24=dev_get_drvdata(dev);
	if(netif_running(nrf24->dev)){
		CE_L;
		power_switch(nrf24,0);
	}
	return 0;
}

static int __maybe_unused nrf24_resume(struct device *dev){
	struct nrf24_data *nrf24=dev_get_drvdata(dev);
//FIXME: if the interface is opened
	if(netif_running(nrf24->dev)){
		power_switch(nrf24,1);
		CE_H; 
	}
	return 0;
}

static SIMPLE_DEV_PM_OPS(nrf24_pm, nrf24_suspend, nrf24_resume);

static const struct spi_device_id nrf24_spi_id[] = {
	{"nrf24l01+",0},
	{}
};

MODULE_DEVICE_TABLE(spi, nrf24_spi_id);

static const struct of_device_id of_nrf24_spi_match[] = {
	{ .compatible = "nordic,nrf24l01+", },
	{ .compatible = "nordic,nrf24l01p", },
	{}
};

MODULE_DEVICE_TABLE(of, of_nrf24_spi_match);

static struct spi_driver nrf24_spi_driver __refdata = { //FIXME: is this the right way to use __initdata?
	.driver = {
		.name =		"nrf24",
		.owner =	THIS_MODULE,
		.pm	=	&nrf24_pm,
		.of_match_table = of_nrf24_spi_match,
	},
	.probe =	nrf24_probe,
	.remove =	nrf24_remove,
	.id_table = nrf24_spi_id,
};

module_spi_driver(nrf24_spi_driver);

MODULE_AUTHOR("Bob Guo, <wolfle@yahoo.com>");
MODULE_DESCRIPTION("nRF24L01+ SPI device interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:nrf24");

/*
 * Simple synchronous userspace interface to SPI devices
 *
 * Copyright (C) 2006 SWAPP
 *	Andrea Paterniani <a.paterniani@swapp-eng.it>
 * Copyright (C) 2007 David Brownell (simplification, cleanup)
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
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/circ_buf.h>

#include <linux/spi/spi.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/poll.h>
#include <asm/barrier.h>

#include <asm/uaccess.h>

#include <plat/sys_config.h>

#include "nrf24l01.h"
#include "nrf24.h"

#define NRF24_DEBUG

#ifdef NRF24_DEBUG
#define nrf24_msg(...)		printk("[nrf24]: "__VA_ARGS__)
#else
#define nrf24_msg(...)
#endif

#define SPI_IOC_MAGIC     'k'

/*
 * This supports access to nRf24L01+ SPI devices using normal charactor device.
 *
 * We allocate major and minor numbers
 * dynamically using a bitmask.  You must use hotplug tools, such as udev
 * (or mdev with busybox) to create and destroy the /dev/nrf24B.C device
 * nodes, since there is no fixed association of major/minor numbers with any
 * particular nrf24 device.
 */
#define N_MINORS			32	/* ... up to 256 */

static DECLARE_BITMAP(minors, N_MINORS);


struct nrf24_data {
	dev_t			devt;
	spinlock_t		spi_lock;
	struct spi_device	*spi;
	struct list_head	device_entry;
	int			ce_gpio;
	/* buffers are NULL unless this device is open (users > 0) */
	wait_queue_head_t 		rq, wq;
	struct mutex		wlock;
	struct mutex 		rlock;
	unsigned		users;
	struct circ_buf  wbuffer,rbuffer;
	/* buf and len used for spi transfers*/
	u8			buf[32+1], len;
	u8  ch, a0[5], a1[5], a2, a3,a4,a5;
	u16 mode;
};


static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

static unsigned bufsize = 4096;
static unsigned nrf24_major =0;
static unsigned ce_gpio = 6; //FIXME

module_param(bufsize, uint, S_IRUGO);
MODULE_PARM_DESC(bufsize, "data bytes in biggest supported SPI message");

module_param(ce_gpio, uint, S_IRUGO); //FIXME
MODULE_PARM_DESC(ce_gpio, "Gpio pin number for CE");//FIXME

/*-------------------------------------------------------------------------*/

#define CHECKOUT(x,r,o) if((r=(x))<0)goto o;
#define CE_H 	gpio_set_value(nrf24->ce_gpio,1)
#define CE_L		gpio_set_value(nrf24->ce_gpio,0)

/* 
 * logical read through spi. reading command at head of the buffer. 
 * len should be the sum of reading command and the expected readin data length.
 * real full duplex needed
 */
static inline ssize_t __spi_read(struct nrf24_data* nrf24, void *buf, size_t len)
{
	struct spi_transfer	t = {
			.rx_buf		= buf,
			.tx_buf		= buf,
			.len		= len,
		};
	struct spi_message	m;
	ssize_t ret;
		
	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	ret = spi_sync(nrf24->spi, &m);
	return ret;
}

static inline ssize_t nrf24_spi_read(struct nrf24_data* nrf24)
{
	return __spi_read(nrf24, nrf24->buf, nrf24->len);
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
	return nrf24_spi_read(nrf24);
}

/* set register to val, after return status reg in buf[0] */
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

static ssize_t flush_tx(struct nrf24_data * nrf24)
{
	nrf24->buf[0]=FLUSH_TX;
	nrf24->len=1;
	return nrf24_spi_write(nrf24);
}

static ssize_t flush_rx(struct nrf24_data * nrf24)
{
	nrf24->buf[0]=FLUSH_RX;
	nrf24->len=1;
	return nrf24_spi_write(nrf24);
}
/* dynamic_payloads_enabled */
static ssize_t __write_playload(struct nrf24_data * nrf24, char * buf, size_t len, u8 pipe)
{
		ssize_t ret;
		buf[0]=!(nrf24->mode&RX_MASK)?
						(NO_ACK&nrf24->mode? W_TX_PAYLOAD_NOACK : W_TX_PAYLOAD)     :
						W_ACK_PAYLOAD|pipe ; 
		if(len>32)len=32;
		ret=spi_write(nrf24->spi,buf,len+1);
		if(ret<0)return -EFAULT;
		else return len;
}

static ssize_t write_payload(struct nrf24_data *nrf24, u8 pipe)
{
	unsigned int head = ACCESS_ONCE(nrf24->wbuffer.head);
	unsigned int tail = nrf24->wbuffer.tail;
	ssize_t res=CIRC_CNT_TO_END(head, tail, bufsize);
	//NOTE: reserve tail == 0 in the wbuffer.buf
	if ( res> 0) {
		CHECKOUT(__write_playload(nrf24,nrf24->wbuffer.buf+tail-1,res,pipe),res, out)
		tail = (tail+res) & (bufsize-1);
		if(tail==0)++tail;
		set_mb(nrf24->wbuffer.tail,tail);
	}
out:	
	return res;
}
/* after return, payload size in buf[1], status reg in buf[0] */
static ssize_t get_payload_size(struct nrf24_data *nrf24)
{
	nrf24->buf[0] = R_RX_PL_WID;
	nrf24->len=2;
	return nrf24_spi_read(nrf24);
}

static ssize_t read_payload(struct nrf24_data *nrf24, u8 pipe)
{
	unsigned int head = nrf24->rbuffer.head;
	unsigned int tail = ACCESS_ONCE(nrf24->rbuffer.tail);
	ssize_t res=0, space=CIRC_SPACE(head, tail, bufsize);
	CHECKOUT(get_payload_size(nrf24),res,out);
	if(nrf24->buf[1]>32)return flush_rx(nrf24);
	if(space<nrf24->buf[1])return 0; //blocked
	nrf24->buf[0]=R_RX_PAYLOAD;
	nrf24->len=1+nrf24->buf[1];
	CHECKOUT(nrf24_spi_read(nrf24),res,out);
	
	if(bufsize-head>=nrf24->len-1){
		memcpy(nrf24->rbuffer.buf+head,nrf24->buf+1,nrf24->len-1);
		smp_wmb();
		head += nrf24->len-1;
		head &= bufsize-1;
		nrf24->rbuffer.head = head;
	} else {
		memcpy(nrf24->rbuffer.buf+head,nrf24->buf+1,bufsize-head);
		memcpy(nrf24->rbuffer.buf, nrf24->buf+1+bufsize-head, nrf24->len - 1 - (bufsize-head));
		smp_wmb();
		nrf24->rbuffer.head = nrf24->len - 1 - (bufsize-head);
	}
	return nrf24->len;
out:
	return res;
}

static ssize_t set_channel(struct nrf24_data * nrf24, u8 channel)
{
  // TODO: This method could take advantage of the 'wide_band' calculation
  // done in setChannel() to require certain channel spacing.

	return set_register(nrf24, RF_CH, channel<126?channel:126); //max channel is 127,use 126 for wide channel
}

static irqreturn_t nrf24_handler(int irq, void *dev)
{
	int res;
	u8 status;
	struct nrf24_data * nrf24 = (struct nrf24_data *)dev;
	CE_L;
	CHECKOUT(read_register(nrf24, STATUS),res,err);//status in buf[1]
	status=nrf24->buf[1];
	CHECKOUT(set_register(nrf24,STATUS,status),res,err); // clear status register
	nrf24_msg("status=%#x\n",status);
	//what happened
	if(status & TX_DS){ //tx ok, push tx payloads if available, else do nothing
		CHECKOUT(write_payload(nrf24,0),res,err);
		if(res==0) { //nothing to send
			nrf24->mode &= ~WR_FLAG; //clear writing payload flag
		} else {
			CE_H;
			wake_up_interruptible_sync(&nrf24->wq);
		}
	}else if(status & MAX_RT){ //tx fail, what to do? mark a err status?
		if(nrf24->mode & TX_FAIL){ //failed twice, trash wbuffer data
			flush_tx(nrf24);
			nrf24->wbuffer.head=1;
			nrf24->wbuffer.tail=1;
//			nrf24->status &= ~TX_FAIL;
		} else nrf24->mode |= TX_FAIL;  // another try
		CE_H;
	}else if(status & RX_DR){ //rx ready, read out the payloads
		CHECKOUT(read_payload(nrf24,status>>1 & 0b111),res,err);
		CE_H;
		wake_up_interruptible_sync(&nrf24->rq);
	}else return IRQ_NONE;	
out:
	return IRQ_HANDLED;
err:
	nrf24->mode |= SPI_FAIL;
	goto out;
}

/*-------------------------------------------------------------------------*/
/* Read-only message with current device setup */
static ssize_t
nrf24_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	ssize_t ret, missing;
	struct nrf24_data	*nrf24 = filp->private_data;
	unsigned int head, tail;

	ret = CIRC_CNT(nrf24->rbuffer.head, nrf24->rbuffer.tail, bufsize);
	if (ret == 0) {
		if(filp->f_flags & O_NONBLOCK)return -EAGAIN;
		if(wait_event_interruptible(nrf24->rq,CIRC_CNT(nrf24->rbuffer.head, nrf24->rbuffer.tail, bufsize)>0))return -ERESTARTSYS;
	}
	mutex_lock(&nrf24->rlock);
	head = ACCESS_ONCE(nrf24->rbuffer.head);
	tail = nrf24->rbuffer.tail;
	ret=CIRC_CNT_TO_END(head,tail,bufsize);
	if(ret>count)ret=count;
	missing = copy_to_user(buf, nrf24->rbuffer.buf+tail, ret);
	if (missing == ret)	ret = -EFAULT;
	else {
		ret = ret - missing;
		set_mb(nrf24->rbuffer.tail, (tail+ret)&(bufsize-1));
	}
	mutex_unlock(&nrf24->rlock);
	return ret;
}

/* Write-only message with current device setup */
static ssize_t
nrf24_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *f_pos)
{
	ssize_t	ret,	missing;
	struct nrf24_data	*nrf24 = filp->private_data;
	unsigned int head, tail;
//	if(nrf24->status & WR_FAIL)
	if(nrf24->mode & (TX_FAIL|SPI_FAIL)) return -EFAULT;
	
	ret=CIRC_SPACE(nrf24->wbuffer.head, nrf24->wbuffer.tail, bufsize);
	if(ret==0){
		if(filp->f_flags & O_NONBLOCK)return -EAGAIN;
		if(wait_event_interruptible(nrf24->wq,CIRC_SPACE(nrf24->wbuffer.head, nrf24->wbuffer.tail, bufsize)>0))return -ERESTARTSYS;
	}
	mutex_lock(&nrf24->wlock);
	head = nrf24->wbuffer.head;
	tail = ACCESS_ONCE(nrf24->wbuffer.tail);
	ret=CIRC_SPACE_TO_END(head, tail, bufsize);
	if(count<ret)ret=count;
	missing = copy_from_user(nrf24->wbuffer.buf+head, buf, ret);
	if (missing == 0) {
		head = (head+ret)&(bufsize-1);
		if(head==0)++head; //NOTE: reserve buf[0] in txbufer
		if(WR_FLAG&nrf24->mode)	set_mb(nrf24->wbuffer.head , head);
		else{
			nrf24->wbuffer.head=head;
			ret=CIRC_CNT_TO_END(head, tail, bufsize);
			CHECKOUT(__write_playload(nrf24,nrf24->wbuffer.buf+tail-1,ret,0),ret,out)
			tail = (tail+ret) & (bufsize-1);
			if(tail==0)++tail;
			nrf24->wbuffer.tail=tail;
			if(CIRC_CNT(head,tail,bufsize)){
				nrf24->mode|=WR_FLAG;
				CE_H;
			}
			nrf24_msg("Start to transmit: %#x\n",nrf24->mode);
		}
	} else	ret = -EFAULT;
out:
	mutex_unlock(&nrf24->wlock);
	return ret;
}

unsigned int nrf24_poll(struct file * filp, poll_table *wait)
{
	struct nrf24_data * nrf24=filp->private_data;
	unsigned int mask=0;
	//TODO: lock needed
	poll_wait(filp,&nrf24->rq,wait);
	poll_wait(filp,&nrf24->wq,wait);
	if(CIRC_CNT(nrf24->rbuffer.head,nrf24->rbuffer.tail,bufsize)>0)mask|=POLLIN|POLLRDNORM;
	if(CIRC_CNT(nrf24->wbuffer.head,nrf24->wbuffer.tail,bufsize)>0)mask|=POLLOUT|POLLWRNORM;
	return mask;
}

static ssize_t nrf24_setup(struct nrf24_data * nrf24)
{
	ssize_t retval;
	if(nrf24->mode&RX_MASK){ //prx mode
		CHECKOUT(read_register(nrf24,CONFIG),retval,out);
		CHECKOUT(set_register(nrf24,CONFIG,nrf24->buf[1]&PRIM_RX),retval,out);
		CHECKOUT(set_register(nrf24,EN_RXADDR,nrf24->buf[1]&(nrf24->mode&RX_MASK)),retval,out);
	}else{ //ptx mode
		CHECKOUT(read_register(nrf24,CONFIG),retval,out);
		CHECKOUT(set_register(nrf24,CONFIG,nrf24->buf[1]&(~PRIM_RX)),retval,out);
	}
out:
	return retval;
}
		
static long
nrf24_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int			retval=0;
	struct nrf24_data	*nrf24=filp->private_data;
	u8			tmp;

	/* Check type and command number */
	if (_IOC_TYPE(cmd) != SPI_IOC_MAGIC)
		return -ENOTTY;

	/* Check access direction once here; don't repeat below.
	 * IOC_DIR is from the user perspective, while access_ok is
	 * from the kernel perspective; so they look reversed.
	 */
	if (_IOC_DIR(cmd) & _IOC_READ)
		retval = !access_ok(VERIFY_WRITE,
				(void __user *)arg, _IOC_SIZE(cmd));
	if (retval == 0 && _IOC_DIR(cmd) & _IOC_WRITE)
		retval = !access_ok(VERIFY_READ,
				(void __user *)arg, _IOC_SIZE(cmd));
	if (retval)
		return -EFAULT;


	/* use the buffer lock here for double duty:
	 *  - prevent concurrent NRF24_IOC_WR_* from morphing
	 *    data fields while NRF24_IOC_RD_* reads them;
	 *  - prevent write/read from user.
	 */
	mutex_lock(&nrf24->wlock);
	mutex_lock(&nrf24->rlock);

	switch (cmd) {
	/* read requests */			
	case NRF24_IOC_RD_MODE:
		retval = __put_user(nrf24->mode&0xff, (__u8 __user *)arg);
		break;
	case NRF24_IOC_RD_CH:
		retval = __put_user(nrf24->ch, (__u8 __user *)arg);
		break;
	case NRF24_IOC_RD_ADDR0:
		CHECKOUT(__put_user(nrf24->a0[4], (__u8 __user *)arg+0),retval,out);
		CHECKOUT(__put_user(nrf24->a0[3], (__u8 __user *)arg+1),retval,out);
		CHECKOUT(__put_user(nrf24->a0[2], (__u8 __user *)arg+2),retval,out);
		CHECKOUT(__put_user(nrf24->a0[1], (__u8 __user *)arg+3),retval,out);
		CHECKOUT(__put_user(nrf24->a0[0], (__u8 __user *)arg+4),retval,out);
		break;
	case NRF24_IOC_RD_ADDR1:
		CHECKOUT(read_register(nrf24,RX_ADDR_P1),retval,out);
		CHECKOUT(__put_user(nrf24->a1[4], (__u8 __user *)arg+0),retval,out);
		CHECKOUT(__put_user(nrf24->a1[3], (__u8 __user *)arg+1),retval,out);
		CHECKOUT(__put_user(nrf24->a1[2], (__u8 __user *)arg+2),retval,out);
		CHECKOUT(__put_user(nrf24->a1[1], (__u8 __user *)arg+3),retval,out);
		CHECKOUT(__put_user(nrf24->a1[0], (__u8 __user *)arg+4),retval,out);
		break;
	case NRF24_IOC_RD_ADDR2:
		CHECKOUT(read_register(nrf24,RX_ADDR_P2),retval,out);
		CHECKOUT(__put_user(nrf24->a2, (__u8 __user *)arg),retval,out);
		break;
	case NRF24_IOC_RD_ADDR3:
		CHECKOUT(read_register(nrf24,RX_ADDR_P3),retval,out);
		CHECKOUT(__put_user(nrf24->a3, (__u8 __user *)arg),retval,out);
		break;
	case NRF24_IOC_RD_ADDR4:
		CHECKOUT(read_register(nrf24,RX_ADDR_P4),retval,out);
		CHECKOUT(__put_user(nrf24->a4, (__u8 __user *)arg),retval,out);
		break;
	case NRF24_IOC_RD_ADDR5:
		CHECKOUT(read_register(nrf24,RX_ADDR_P5),retval,out);
		CHECKOUT(__put_user(nrf24->a5, (__u8 __user *)arg),retval,out);
		break;
	/* write requests */
	case NRF24_IOC_WR_MODE:
		CHECKOUT(__get_user(tmp, (u8 __user *)arg),retval,out);
		if(tmp^(nrf24->mode&0xff)){
			u8	save = nrf24->mode&0xff;
			retval = nrf24_setup(nrf24);
			if (retval < 0)
				nrf24->mode=save|(nrf24->mode&0xff00);
			else
				nrf24->mode=tmp|(nrf24->mode&0xff00);
				dev_dbg(&nrf24->spi->dev, "nRF24L01+ mode %02x\n", tmp);
		}
		break;
	case NRF24_IOC_WR_CH:
		CHECKOUT(__get_user(tmp, (u8 __user *)arg),retval,out);
		if(tmp > 126) retval=-EINVAL;
		else{
			CHECKOUT(set_channel(nrf24,tmp),retval,out);
			nrf24->ch=tmp;
		}
		break;
	case NRF24_IOC_WR_ADDR0:
		CHECKOUT(__get_user(nrf24->buf[5], (u8 __user *)arg+0),retval,out);
		CHECKOUT(__get_user(nrf24->buf[4], (u8 __user *)arg+1),retval,out);
		CHECKOUT(__get_user(nrf24->buf[3], (u8 __user *)arg+2),retval,out);
		CHECKOUT(__get_user(nrf24->buf[2], (u8 __user *)arg+3),retval,out);
		CHECKOUT(__get_user(nrf24->buf[1], (u8 __user *)arg+4),retval,out);
		CHECKOUT(set_address(nrf24,TX_ADDR),retval,out);
		CHECKOUT(set_address(nrf24,RX_ADDR_P0),retval,out);
		memcpy(nrf24->a0,nrf24->buf+1,5);
		break;
	case NRF24_IOC_WR_ADDR1:
		CHECKOUT(__get_user(nrf24->buf[5], (u8 __user *)arg+0),retval,out);
		CHECKOUT(__get_user(nrf24->buf[4], (u8 __user *)arg+1),retval,out);
		CHECKOUT(__get_user(nrf24->buf[3], (u8 __user *)arg+2),retval,out);
		CHECKOUT(__get_user(nrf24->buf[2], (u8 __user *)arg+3),retval,out);
		CHECKOUT(__get_user(nrf24->buf[1], (u8 __user *)arg+4),retval,out);
		CHECKOUT(set_address(nrf24,RX_ADDR_P1),retval,out);
		memcpy(nrf24->a1,nrf24->buf+1,5);
		break;
	case NRF24_IOC_WR_ADDR2:
		CHECKOUT(__get_user(tmp, (u8 __user *)arg),retval,out);
		CHECKOUT(set_register(nrf24,RX_ADDR_P2,tmp),retval,out);
		nrf24->a2=tmp;
		break;
	case NRF24_IOC_WR_ADDR3:
		CHECKOUT(__get_user(tmp, (u8 __user *)arg),retval,out);
		CHECKOUT(set_register(nrf24,RX_ADDR_P3,tmp),retval,out);
		nrf24->a3=tmp;
		break;
	case NRF24_IOC_WR_ADDR4:
		CHECKOUT(__get_user(tmp, (u8 __user *)arg),retval,out);
		CHECKOUT(set_register(nrf24,RX_ADDR_P4,tmp),retval,out);
		nrf24->a4=tmp;
		break;
	case NRF24_IOC_WR_ADDR5:
		CHECKOUT(__get_user(tmp, (u8 __user *)arg),retval,out);
		CHECKOUT(set_register(nrf24,RX_ADDR_P5,tmp),retval,out);
		nrf24->a5=tmp;
		break;
	default:
		break;
	}
out:
	mutex_unlock(&nrf24->wlock);
	mutex_unlock(&nrf24->rlock);
	return retval<0?retval:0;
}

#ifdef CONFIG_COMPAT
static long
nrf24_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	return nrf24_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#else
#define nrf24_compat_ioctl NULL
#endif /* CONFIG_COMPAT */

static ssize_t nrf24_reset(struct nrf24_data * nrf24)
{
	int retval;
	
	CE_L;
		
	CHECKOUT(set_register(nrf24,CONFIG,0b1100),retval,out); //16bit crc, power down, ptx mode
#ifdef NRF24_DEBUG
	CHECKOUT(read_register(nrf24,CONFIG),retval,out); //500us; retransmit 7 times
	if(nrf24->buf[1] != 0b1100) nrf24_msg("Set CONFIG failed! %#x",nrf24->buf[1]);
#endif

	CHECKOUT(flush_rx(nrf24),retval,out);
	
	CHECKOUT(flush_tx(nrf24),retval,out);
	
	CHECKOUT(set_register(nrf24,STATUS,0b1111110 ),retval,out);
#ifdef NRF24_DEBUG
	CHECKOUT(read_register(nrf24,STATUS),retval,out); 
	if(nrf24->buf[1] != 0b1110) nrf24_msg("Set STATUS failed! %#x",nrf24->buf[1]);
#endif	

	nrf24->mode=0;
	nrf24->rbuffer.head=0;
	nrf24->rbuffer.tail=0;
	nrf24->wbuffer.head=1;
	nrf24->wbuffer.tail=1;
out:
	return retval<0?retval:0;
}

static ssize_t __init nrf24_initialize(struct nrf24_data * nrf24)
{
	ssize_t retval;
	struct spi_device *spi;
	
	disable_irq(nrf24->spi->irq);
	// spi init
	spin_lock_irq(&nrf24->spi_lock);
	spi = spi_dev_get(nrf24->spi);
	spin_unlock_irq(&nrf24->spi_lock);

	if (spi == NULL){
		retval = -ESHUTDOWN;
		goto out;
	}
	
	CHECKOUT(spi_setup(spi),retval,out);
	spi_dev_put(spi);
	
	CHECKOUT(nrf24_reset(nrf24),retval,out);
	
	CHECKOUT(set_register(nrf24,SETUP_RETR,(1 << ARD) | (0b111 << ARC)),retval,out); //500us; retransmit 7 times
#ifdef NRF24_DEBUG
	CHECKOUT(read_register(nrf24,SETUP_RETR),retval,out); //500us; retransmit 7 times
	if(nrf24->buf[1] != ((1 << ARD) | (0b111 << ARC)) )nrf24_msg("Set SETUP_RETR failed! %#x",nrf24->buf[1]);
#endif	

	CHECKOUT(set_register(nrf24,RF_SETUP,0b1111),retval,out); //PA max,  2Mbps
#ifdef NRF24_DEBUG
	CHECKOUT(read_register(nrf24,RF_SETUP),retval,out); 
	if(nrf24->buf[1] != 0b1111) nrf24_msg("Set RF_SETUP failed! %#x",nrf24->buf[1]);
#endif	
	
	CHECKOUT(set_channel(nrf24,nrf24->ch),retval,out);
#ifdef NRF24_DEBUG
	CHECKOUT(read_register(nrf24,RF_CH),retval,out); 
	if(nrf24->buf[1] != nrf24->ch) nrf24_msg("Set RF_CH failed! %#x",nrf24->buf[1]);
#endif	
	
	CHECKOUT(set_register(nrf24,FEATURE,0b111 ),retval,out);
#ifdef NRF24_DEBUG
	CHECKOUT(read_register(nrf24,FEATURE),retval,out);
	if((nrf24->buf[1]&0b111)!=0b111 )
		nrf24_msg("Set FEATURE after toggle failed! %#x %#x",nrf24->buf[0], nrf24->buf[1]);
#endif	

	CHECKOUT(set_register(nrf24,EN_AA,0b111111),retval,out); //enable auto ack on all pipes
#ifdef NRF24_DEBUG
	CHECKOUT(read_register(nrf24,EN_AA),retval,out); //500us; retransmit 7 times
	if(nrf24->buf[1] != 0b111111) nrf24_msg("Set EN_AA failed! %#x",nrf24->buf[0]);
#endif	
	
	CHECKOUT(set_register(nrf24,DYNPD,0b111111),retval,out); //enable dynamics payload length on all pipes
#ifdef NRF24_DEBUG
	CHECKOUT(read_register(nrf24,DYNPD),retval,out); //500us; retransmit 7 times
	if(nrf24->buf[1] != 0b111111) nrf24_msg("Set DYNPD failed! %#x",nrf24->buf[0]);
#endif	
	
	CHECKOUT(set_register(nrf24,EN_RXADDR,0b1),retval,out); //only enable pipe 0
#ifdef NRF24_DEBUG
	CHECKOUT(read_register(nrf24,EN_RXADDR),retval,out); //500us; retransmit 7 times
	if(nrf24->buf[1] != 0b1) nrf24_msg("Set EN_RXADDR failed! %#x",nrf24->buf[0]);
#endif	
	
	CHECKOUT(set_register(nrf24,SETUP_AW,0b11),retval,out); // 5 bytes address
#ifdef NRF24_DEBUG
	CHECKOUT(read_register(nrf24,SETUP_AW),retval,out); //500us; retransmit 7 times
	if(nrf24->buf[1] != 0b11) nrf24_msg("Set SETUP_AW failed! %#x",nrf24->buf[0]);
#endif	
	
	memcpy(nrf24->buf+1,nrf24->a0,5);
	CHECKOUT(set_address(nrf24,RX_ADDR_P0),retval,out); 
	CHECKOUT(set_address(nrf24,TX_ADDR),retval,out);
	
	CHECKOUT(set_register(nrf24,CONFIG,0b1110),retval,out); //16bit crc, power up, ptx mode
#ifdef NRF24_DEBUG
	CHECKOUT(read_register(nrf24,CONFIG),retval,out); //500us; retransmit 7 times
	if(nrf24->buf[1] != 0b1110) nrf24_msg("Set CONFIG failed! %#x",nrf24->buf[0]);
#endif
	
	retval=request_threaded_irq(nrf24->spi->irq, NULL, nrf24_handler, IRQF_TRIGGER_FALLING, "nRF24L01+", nrf24);
	nrf24_msg("Initialized.\n");
out:
	return retval;
}

static int nrf24_open(struct inode *inode, struct file *filp)
{
	struct nrf24_data	*nrf24;
	int			ret = -ENXIO;

	mutex_lock(&device_list_lock);

	list_for_each_entry(nrf24, &device_list, device_entry) {
		if (nrf24->devt == inode->i_rdev) {
			ret = 0;
			break;
		}
	}
	if (ret == 0) {
		nrf24->users++;
		filp->private_data = nrf24;
		nonseekable_open(inode, filp);
	} else
		pr_debug("nrf24: nothing for minor %d\n", iminor(inode));

	mutex_unlock(&device_list_lock);
	return ret;
}

static int nrf24_release(struct inode *inode, struct file *filp)
{
	struct nrf24_data	*nrf24;
	int			ret = 0;

	mutex_lock(&device_list_lock);
	nrf24 = filp->private_data;
	filp->private_data = NULL;

	nrf24->users--;
	/* last close? */
	//if(nrf24->users==0)ret=nrf24_reset(nrf24);

	mutex_unlock(&device_list_lock);
	
	nrf24_msg("released. %#x users: %d\n",nrf24->mode,nrf24->users);
	return ret<0?ret:0;
}

static const struct file_operations nrf24_fops = {
	.owner =	THIS_MODULE,
	/* REVISIT switch to aio primitives, so that userspace
	 * gets more complete API coverage.  It'll simplify things
	 * too, except for the locking.
	 */
	.write =	nrf24_write,
	.read =		nrf24_read,
	.unlocked_ioctl = nrf24_ioctl,
	.compat_ioctl = nrf24_compat_ioctl,
	.open =		nrf24_open,
	.release =	nrf24_release,
	.poll	= nrf24_poll,
	.llseek =	no_llseek,
};

static struct cdev nrf24_cdev;

/*-------------------------------------------------------------------------*/

/* The main reason to have this class is to make mdev/udev create the
 * /dev/nrf24B.C character device nodes exposing our userspace API.
 * It also simplifies memory management.
 */

static struct class *nrf24_class;


static int __devinit nrf24_probe(struct spi_device *spi)
{
	struct nrf24_data	*nrf24;
	int			status=0;
	unsigned long		minor;


	struct gpio gpios[]={
		{ce_gpio,GPIOF_OUT_INIT_LOW,"nrf24 chip enable"},
	};
	
	if(spi->irq < 0 || ce_gpio < 0) return -ENODEV; //FIXME
	
	if(gpio_request_array(gpios,ARRAY_SIZE(gpios)))return -ENODEV;
	// Is toggling it right?
//		gpio_direction_input(nrf24->cs_gpio);
//		if(gpio_direction_output(nrf24->cs_gpio, 0) < 0) goto out;
	
	/* Allocate driver data */
	nrf24 = kzalloc(sizeof(*nrf24), GFP_KERNEL);
	if (!nrf24){
		dev_dbg(&nrf24->spi->dev, "probe/ENOMEM\n");
		status = -ENOMEM;
		goto gpio;
	}

	nrf24->ce_gpio=ce_gpio; //FIXME

	if (!nrf24->rbuffer.buf) {
		nrf24->rbuffer.buf = kmalloc(bufsize, GFP_KERNEL);
		if (!nrf24->rbuffer.buf) {
			dev_dbg(&nrf24->spi->dev, "probe/ENOMEM\n");
			status = -ENOMEM;
			goto mem;
		}
	}
	if (!nrf24->wbuffer.buf) {
		nrf24->wbuffer.buf = kmalloc(bufsize, GFP_KERNEL);
		if (!nrf24->wbuffer.buf) {
			dev_dbg(&nrf24->spi->dev, "probe/ENOMEM\n");
			status = -ENOMEM;
			goto mem1;
		}
	}
	
	/* Initialize the driver data */
	nrf24->spi = spi;
	spin_lock_init(&nrf24->spi_lock);
	mutex_init(&nrf24->rlock);
	mutex_init(&nrf24->wlock);

	INIT_LIST_HEAD(&nrf24->device_entry);

	/* If we can allocate a minor number, hook up this device.
	 * Reusing minors is fine so long as udev or mdev is working.
	 */
	mutex_lock(&device_list_lock);
	minor = find_first_zero_bit(minors, N_MINORS);
	if (minor < N_MINORS) {
		struct device *dev;

		nrf24->devt = MKDEV(nrf24_major, minor);
		dev = device_create(nrf24_class, &spi->dev, nrf24->devt,
				    nrf24, "nrf24%d.%d",
				    spi->master->bus_num, spi->chip_select);
		status = IS_ERR(dev) ? PTR_ERR(dev) : 0;
		if(status) goto mem2;
	} else {
		dev_dbg(&spi->dev, "no minor number available!\n");
		status = -ENODEV;
		goto mem2;
	}

	set_bit(minor, minors);
	list_add(&nrf24->device_entry, &device_list);

	mutex_unlock(&device_list_lock);

	spi_set_drvdata(spi, nrf24);
	
	nrf24->ch=76; //arbitary channel
	nrf24->mode=0; //ptx mode
	nrf24->a0[0]=nrf24->a0[1]=nrf24->a0[2]=nrf24->a0[3]=nrf24->a0[4]=0;
	nrf24->a1[0]=nrf24->a1[1]=nrf24->a1[2]=nrf24->a1[3]=nrf24->a1[4]=0;
	nrf24->a2=nrf24->a3=nrf24->a4=nrf24->a5=0;

	status = nrf24_initialize(nrf24);

	return status;

mem2:
	kfree(nrf24->wbuffer.buf);
mem1:
	kfree(nrf24->rbuffer.buf);
mem:
	kfree(nrf24);
gpio:
	gpio_free_array(gpios,ARRAY_SIZE(gpios));
	return status;
}

static int __devexit nrf24_remove(struct spi_device *spi)
{
	struct nrf24_data	*nrf24 = spi_get_drvdata(spi);

	struct gpio gpios[]={
		{nrf24->ce_gpio,GPIOF_OUT_INIT_LOW,"nrf24 chip enable"},
	};

	if (nrf24->users )return -EBUSY;
	
	nrf24_reset(nrf24);

	//free_irq(spi->irq,nrf24);
	gpio_free_array(gpios,ARRAY_SIZE(gpios));	

	kfree(nrf24->rbuffer.buf);
	kfree(nrf24->wbuffer.buf);
	
	/* make sure ops on existing fds can abort cleanly */
	spin_lock_irq(&nrf24->spi_lock);
	nrf24->spi = NULL;
	spi_set_drvdata(spi, NULL);
	spin_unlock_irq(&nrf24->spi_lock);

	/* prevent new opens */
	mutex_lock(&device_list_lock);
	list_del(&nrf24->device_entry);
	device_destroy(nrf24_class, nrf24->devt);
	clear_bit(MINOR(nrf24->devt), minors);
	kfree(nrf24);
	mutex_unlock(&device_list_lock);
	nrf24_msg("Removed.\n");
	return 0;
}

static struct spi_driver nrf24_spi_driver = {
	.driver = {
		.name =		"nrf24",
		.owner =	THIS_MODULE,
	},
	.probe =	nrf24_probe,
	.remove =	__devexit_p(nrf24_remove),

	/* NOTE:  suspend/resume methods are not necessary here.
	 * We don't do anything except pass the requests to/from
	 * the underlying controller.  The refrigerator handles
	 * most issues; the controller driver handles the rest.
	 */
};

/*-------------------------------------------------------------------------*/

static int __init nrf24_init(void)
{
	int status;
	dev_t dev;	

	/* Claim our 256 reserved device numbers.  Then register a class
	 * that will key udev/mdev to add/remove /dev nodes.  Last, register
	 * the driver which manages those device numbers.
	 */
	BUILD_BUG_ON(N_MINORS > 256);
	if(nrf24_major) {
		dev = MKDEV(nrf24_major, 0);
		status = register_chrdev_region(dev, N_MINORS, "nRF24L01+");
	} else {
		status = alloc_chrdev_region(&dev, 0, N_MINORS, "nRF24L01+");
		nrf24_major = MAJOR(dev);
	}
	if (status < 0) goto out;
	
	cdev_init(&nrf24_cdev,&nrf24_fops);
	nrf24_cdev.owner = THIS_MODULE;	
	status = cdev_add(&nrf24_cdev, dev, N_MINORS);
	if(status<0) goto out;

	nrf24_class = class_create(THIS_MODULE, "nrf24");
	if (IS_ERR(nrf24_class)) {
		unregister_chrdev_region(dev, N_MINORS);
		return PTR_ERR(nrf24_class);
	}

	status = spi_register_driver(&nrf24_spi_driver);
	if (status < 0) {
		class_destroy(nrf24_class);
		unregister_chrdev_region(dev, N_MINORS);
	}
out:
	return status;
}
module_init(nrf24_init);

static void __exit nrf24_exit(void)
{
	cdev_del(&nrf24_cdev);
	spi_unregister_driver(&nrf24_spi_driver);
	class_destroy(nrf24_class);
	if(nrf24_major){
		dev_t dev=MKDEV(nrf24_major,0);
		unregister_chrdev_region(dev, N_MINORS);
	}
}
module_exit(nrf24_exit);

MODULE_AUTHOR("Bob Guo, <wolfle@yahoo.com>");
MODULE_DESCRIPTION("nRF24L01+ SPI device interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:nrf24");

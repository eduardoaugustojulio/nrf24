#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/spi/spi.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/cdev.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/of_gpio.h>
#include <linux/ktime.h>
#include <linux/hrtimer.h>

#include <linux/nrf24.h>
#include "rf_module.h"
#include "rf_helpers.h"
#include "rf_sysfs.h"

#define N_SPI_MINORS 9

static struct class *rf_class;
static struct cdev   rf_cdev;
static        dev_t  rf_devt;   /* Holds the Major number */
static DECLARE_BITMAP(minors, N_SPI_MINORS);
static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

/*
 * Sysfs attributes
 */
static DEVICE_ATTR(registers, 0600, show_reg, store_reg);
static DEVICE_ATTR(config, 0400, show_config, NULL);
static DEVICE_ATTR(statistics, 0400, show_statistics, NULL);
static DEVICE_ATTR(scanlink, 0600, show_scanlink, store_scanlink);
static DEVICE_ATTR(rftest, 0200, NULL, store_rftest);

static struct attribute *rf_attrs[] = {
        &dev_attr_registers.attr,
        &dev_attr_config.attr,
	&dev_attr_statistics.attr,
	&dev_attr_scanlink.attr,
	&dev_attr_rftest.attr,
        NULL,
};

static struct attribute_group rf_attr_group = {
        .attrs = rf_attrs,
};

/*
 * IRQ Stuff
 */

static void prx_nonsleep(struct rf_data *rf);

static irqreturn_t rf_hard_irq(int irq, void *handle)
{
	struct rf_data *rf = handle;
	ce_low(rf->spi);
	if (!IS_SETTED(rf->config, BIT(0))) { /* in PTX */
		trace_puts("TX hard IRQ\n");
		if (rf->goto_prx) {
			rf->goto_prx = false;
			prx_nonsleep(rf);
			return IRQ_HANDLED;
		}
	} else {
		trace_puts("RX hard IRQ\n");
	}

	return IRQ_WAKE_THREAD;
}

void prx_complete(void *context);
static struct spi_message prx_nonsleep_msg;
static void prx_nonsleep(struct rf_data *rf)
{
	static struct spi_transfer t[4];
	SET_BIT(rf->config, BIT(0)); /* go to PRX */
	memset(&t, '\0', sizeof(t));
	t[0].tx_buf = "\x20"; /* W_REGISTER(CONFIG_REG) */
	t[0].len    = 1;
	t[1].tx_buf = &rf->config;
	t[1].len = 1;
	t[1].cs_change = 1;
	t[2].tx_buf = "\x27"; /* W_REGISTER(STATUS_REG) */
	t[2].len = 1;
	t[3].tx_buf = "\x70";
	t[3].len = 1;
	spi_message_init(&prx_nonsleep_msg);
	spi_message_add_tail(&t[0], &prx_nonsleep_msg);
	spi_message_add_tail(&t[1], &prx_nonsleep_msg);
	spi_message_add_tail(&t[2], &prx_nonsleep_msg);
	spi_message_add_tail(&t[3], &prx_nonsleep_msg);
	prx_nonsleep_msg.complete = prx_complete;
	prx_nonsleep_msg.context  = rf;

	trace_puts("Going to PRX start\n");
	BUG_ON(spi_async(rf->spi, &prx_nonsleep_msg));
}

void prx_complete(void *context)
{
	struct rf_data *rf = context;
	BUG_ON(prx_nonsleep_msg.status);
	complete(&rf->tx_complete);
	ce_high(rf->spi);
	trace_puts("Going to PRX complete\n");
}

static irqreturn_t rf_irq(int irq, void *data)
{
        u8 status;
        struct rf_data *rf = data;

        read_reg(rf->spi, STATUS_REG, &status, 1);

        if (IS_SETTED(status, BIT(5))) {
                _DEBUG("TX IRQ served");
                complete(&rf->tx_complete);
        }

        if (IS_SETTED(status, BIT(6))) {
                _DEBUG("RX IRQ served");
                rf->new_rx_data = 1;
                wake_up(&rf->rx_wq);
        }

	write_reg(rf->spi, STATUS_REG, &status, 1); /* clear irq flags */
        return IRQ_HANDLED;
}

/*
 * File oprations
 */
static int rf_open(struct inode *ip, struct file *fp)
{
        struct rf_data *rf;

        mutex_lock(&device_list_lock);
        list_for_each_entry(rf, &device_list, device_entry) {
                if (rf->devt == ip->i_rdev) {
                        fp->private_data = rf;
			if (rf->inuse) {
				mutex_unlock(&device_list_lock);
				return -EBUSY;
			}
			rf->inuse = true;
			pwr_up(rf->spi);
                        mutex_unlock(&device_list_lock);
                        return 0;
                }
        }
        mutex_unlock(&device_list_lock);
        return -ENODEV;
}

static int rf_release(struct inode *ip, struct file *fp)
{
        struct rf_data *rf = fp->private_data;

        if (!rf)
                return -ENODEV;
	pwr_down(rf->spi);
	rf->inuse = false;
        fp->private_data = NULL;
        return 0;
}

static ssize_t rf_write(struct file *fp,
                        const char __user *buf,
                        size_t len,
                        loff_t *off)
{
	int status;
	u8 pld[32];
	struct rf_data *rf = fp->private_data;

	BUG_ON(!rf);

	len = min(sizeof(pld), len);
	if (copy_from_user(pld, buf, len))
		return -EIO;

	rf->goto_prx = false;
	flush_tx(rf->spi);
	write_tx(rf->spi, pld, len);
	ptx_mode(rf->spi);
	rf_led_on(rf);
	reinit_completion(&rf->tx_complete);
	ce_high(rf->spi);
	status = wait_for_completion_interruptible(&rf->tx_complete);
	ce_low(rf->spi);
	rf_led_off(rf);
	if (status) {
		rf->timeout_count++;
		return status;
	}
	rf->tx_bytes_count += len;
	return len;
}

static ssize_t rf_read(struct file *fp,
                        char *buf,
                        size_t len,
                        loff_t *off)
{
	u8 feature, pldsiz;
	int status;
	char pld[32];
	struct rf_data *rf = fp->private_data;

	BUG_ON(!rf);

	if (*off > 0)
		return 0;
	if (len < 32)
		return -EINVAL;

	rf->goto_prx = false;
	rf->new_rx_data = 0;
	flush_rx(rf->spi);
	prx_mode(rf->spi);
	rf_led_on(rf);
	ce_high(rf->spi);
	status = wait_event_interruptible(rf->rx_wq,
					  rf->new_rx_data);
	ce_low(rf->spi);
	rf_led_off(rf);
	if (status) {
		rf->timeout_count++;
		return status;
	}

	read_reg(rf->spi, FEATURE_REG, &feature, 1);
	if (IS_SETTED(feature, BIT(2)))
		read_rx_pld_wid(rf->spi, &pldsiz);
	else
		read_reg(rf->spi, RX_PW_P0_REG, &pldsiz, 1);
	read_rx(rf->spi, pld, pldsiz);
	rf->rx_bytes_count += pldsiz;
	
	if (copy_to_user(buf, pld, pldsiz))
		return -EIO;
		
	return pldsiz;
}

static long rf_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
        long status = 0;
        struct rf_data *rf = fp->private_data;
        u8 rxbuf[5];

        BUG_ON(!fp);
        mutex_lock(&rf->bus_lock);

        switch (cmd) {

        case NRF24_IOCSAA:
                _DEBUG("NRF24_IOCSAA: 0x%02lx", arg);
                write_reg(rf->spi, EN_AA_REG, (u8 *)&arg, 1);
                break;

        case NRF24_IOCGAA:
                _DEBUG("NRF24_IOCGAA");
                read_reg(rf->spi, EN_AA_REG, rxbuf, 1);
                put_user(rxbuf[0], (u8 __user *)arg);
                break;

        case NRF24_IOCSCRC:     /* Bit 2,3 of CONFIG register */
                _DEBUG("NRF24_IOCSCRC: %ld", arg);
                read_reg(rf->spi, CONFIG_REG, rxbuf, 1);
                switch (arg) {
                case 0:
                        rxbuf[0] &= ~(BIT(2) | BIT(3));
                        break;
                case 1:
                        rxbuf[0] |= BIT(3);
                        rxbuf[0] &= ~(BIT(2));
                        break;
                case 2:
                        rxbuf[0] |= (BIT(2) | BIT(3));
                        break;
                }
                write_reg(rf->spi, CONFIG_REG, rxbuf, 1);
                break;

        case NRF24_IOCGCRC:
                _DEBUG("NRF24_IOCGCRC");
                read_reg(rf->spi, CONFIG_REG, rxbuf, 1);
                if (!IS_SETTED(BIT(3), rxbuf[0])) /* 0bxxxx0xxx => Disabled */
                        put_user(0, (u8 __user *)arg);
                else if (!IS_SETTED(BIT(2), rxbuf[0])) /* 0bxxxx10xx => Enabled, 1 byte*/
                        put_user(1, (u8 __user *)arg);
                else
                        put_user(2, (u8 __user *)arg); /* 0bxxxx11xx => Enabled, 2 bytes */
                break;

        case NRF24_IOCSCH:
                _DEBUG("NRF24_IOCSCH: %ld", arg);
                write_reg(rf->spi, RF_CH_REG, (u8 *)&arg, 1);
                break;

        case NRF24_IOCGCH:
                _DEBUG("NRF24_IOCGCH");
                read_reg(rf->spi, RF_CH_REG, rxbuf, 1);
                put_user(rxbuf[0], (u8 __user *)arg);
                break;

        case NRF24_IOCSPW:
                _DEBUG("NRF24_IOCSPW: %ld", arg);
                write_reg(rf->spi, RX_PW_P0_REG, (u8 *)&arg, 1);
                write_reg(rf->spi, RX_PW_P1_REG, (u8 *)&arg, 1);
                write_reg(rf->spi, RX_PW_P2_REG, (u8 *)&arg, 1);
                write_reg(rf->spi, RX_PW_P3_REG, (u8 *)&arg, 1);
                write_reg(rf->spi, RX_PW_P4_REG, (u8 *)&arg, 1);
                write_reg(rf->spi, RX_PW_P5_REG, (u8 *)&arg, 1);
                break;

        case NRF24_IOCGPW:
                _DEBUG("NRF24_IOCGPW");
                read_reg(rf->spi, RX_PW_P0_REG, rxbuf, 1);
                put_user(rxbuf[0], (u8 __user *)arg);
                break;

        case NRF24_IOCSTXADDR:
                do {
                        u8 aw;
                        read_reg(rf->spi, SETUP_AW_REG, &aw, 1);
                        aw += 2;
                        if (copy_from_user(rxbuf, (u8 __user *)arg, aw)) {
                                pr_err("copy_from_user fails");
                                status = -EFAULT;
                                break;
                        }
                        switch (aw) {
                        case 3:
                                _DEBUG("NRF24_IOCSTXADDR: %02x:%02x:%02x",
                                       rxbuf[0], rxbuf[1], rxbuf[2]);
                                break;
                        case 4:
                                _DEBUG("NRF24_IOCSTXADDR: %02x:%02x:%02x:%02x",
                                       rxbuf[0], rxbuf[1], rxbuf[2], rxbuf[3]);
                                break;
                        case 5:
                                _DEBUG("NRF24_IOCSTXADDR: %02x:%02x:%02x:%02x:%02x",
                                       rxbuf[0], rxbuf[1], rxbuf[2], rxbuf[3], rxbuf[4]);
                                break;
                        }
                        write_reg(rf->spi, TX_ADDR_REG, rxbuf, aw);
                } while (0);
                break;

        case NRF24_IOCGTXADDR:
                _DEBUG("NRF24_IOCGTXADDR");
                do {
                        u8 aw;
                        read_reg(rf->spi, SETUP_AW_REG, &aw, 1);
                        aw += 2;
                        read_reg(rf->spi, TX_ADDR_REG, rxbuf, aw);
                        if (copy_to_user((u8 __user *)arg, rxbuf, aw)) {
                                pr_err("copy_to_user fails");
                                status = -EFAULT;
                                break;
                        }
                } while (0);
                break;

        case NRF24_IOCSRXADDR:
                do {
                        u8 aw;
                        read_reg(rf->spi, SETUP_AW_REG, &aw, 1);
                        aw += 2;
                        if (copy_from_user(rxbuf, (u8 __user *)arg, aw)) {
                                pr_err("copy_from_user fails");
                                status = -EFAULT;
                                break;
                        }

                        switch (aw) {
                        case 3:
                                _DEBUG("NRF24_IOCSRXADDR: %02x:%02x:%02x",
                                       rxbuf[0], rxbuf[1], rxbuf[2]);
                                break;
                        case 4:
                                _DEBUG("NRF24_IOCSRXADDR: %02x:%02x:%02x:%02x",
                                       rxbuf[0], rxbuf[1], rxbuf[2], rxbuf[3]);
                                break;
                        case 5:
                                _DEBUG("NRF24_IOCSRXADDR: %02x:%02x:%02x:%02x:%02x",
                                       rxbuf[0], rxbuf[1], rxbuf[2], rxbuf[3], rxbuf[4]);
                                break;
                        }
                        write_reg(rf->spi, RX_ADDR_P0_REG, rxbuf, aw);
                } while (0);
                break;
	
	case NRF24_IOCSRXADDR_P1:
		do {
                        u8 aw;
                        read_reg(rf->spi, SETUP_AW_REG, &aw, 1);
                        aw += 2;
                        if (copy_from_user(rxbuf, (u8 __user *)arg, aw)) {
                                pr_err("copy_from_user fails");
                                status = -EFAULT;
                                break;
                        }

                        switch (aw) {
                        case 3:
                                _DEBUG("NRF24_IOCSRXADDR_P1: %02x:%02x:%02x",
                                       rxbuf[0], rxbuf[1], rxbuf[2]);
                                break;
                        case 4:
                                _DEBUG("NRF24_IOCSRXADDR_P1: %02x:%02x:%02x:%02x",
                                       rxbuf[0], rxbuf[1], rxbuf[2], rxbuf[3]);
                                break;
                        case 5:
                                _DEBUG("NRF24_IOCSRXADDR_P1: %02x:%02x:%02x:%02x:%02x",
                                       rxbuf[0], rxbuf[1], rxbuf[2], rxbuf[3], rxbuf[4]);
                                break;
                        }
                        write_reg(rf->spi, RX_ADDR_P1_REG, rxbuf, aw);
		} while (0);
		break;
	
	case NRF24_IOCSRXADDR_P2:
		_DEBUG("NRF24_IOCSRXADDR_P2: %02lx", arg);	
		write_reg(rf->spi, RX_ADDR_P2_REG, (u8 *)&arg, 1);
		break;

	case NRF24_IOCSRXADDR_P3:
		_DEBUG("NRF24_IOCSRXADDR_P3: %02lx", arg);
		write_reg(rf->spi, RX_ADDR_P3_REG, (u8 *)&arg, 1);
		break;
	
	case NRF24_IOCSRXADDR_P4:
		_DEBUG("NRF24_IOCSRXADDR_P4: %02lx", arg);
		write_reg(rf->spi, RX_ADDR_P4_REG, (u8 *)&arg, 1);
		break;

	case NRF24_IOCSRXADDR_P5:
		_DEBUG("NRF24_IOCSRXADDR_P5: %02lx", arg);
		write_reg(rf->spi, RX_ADDR_P5_REG, (u8 *)&arg, 1);
		break;
	
        case NRF24_IOCGRXADDR:
                _DEBUG("NRF24_IOCGRXADDR");
                do {
                        u8 aw;
                        read_reg(rf->spi, SETUP_AW_REG, &aw, 1);
                        aw += 2;
                        read_reg(rf->spi, RX_ADDR_P0_REG, rxbuf, aw);
                        if (copy_to_user((u8 __user *)arg, rxbuf, aw)) {
                                pr_err("copy_to_user fails");
                                status = -EFAULT;
                                break;
                        }
                } while (0);
                break;
	
	case NRF24_IOCGRXADDR_P1:
		_DEBUG("NRF24_IOCGRXADDR_P1");
                do {
                        u8 aw;
                        read_reg(rf->spi, SETUP_AW_REG, &aw, 1);
                        aw += 2;
                        read_reg(rf->spi, RX_ADDR_P1_REG, rxbuf, aw);
                        if (copy_to_user((u8 __user *)arg, rxbuf, aw)) {
                                pr_err("copy_to_user fails");
                                status = -EFAULT;
                                break;
                        }
                } while (0);
		break;

	case NRF24_IOCGRXADDR_P2:
		_DEBUG("NRF24_IOCGRXADDR_P2");
		read_reg(rf->spi, RX_ADDR_P2_REG, rxbuf, 1);
		put_user(rxbuf[0], (u8 __user *)arg);
		break;

	case NRF24_IOCGRXADDR_P3:
		_DEBUG("NRF24_IOCGRXADDR_P3");
		read_reg(rf->spi, RX_ADDR_P3_REG, rxbuf, 1);
		put_user(rxbuf[0], (u8 __user *)arg);
		break;

	case NRF24_IOCGRXADDR_P4:
		_DEBUG("NRF24_IOCGRXADDR_P4");
		read_reg(rf->spi, RX_ADDR_P4_REG, rxbuf, 1);
		put_user(rxbuf[0], (u8 __user *)arg);
		break;

	case NRF24_IOCGRXADDR_P5:
		_DEBUG("NRF24_IOCGRXADDR_P5");
		read_reg(rf->spi, RX_ADDR_P5_REG, rxbuf, 1);
		put_user(rxbuf[0], (u8 __user *)arg);
		break;

	case NRF24_IOCSENRXADDR:
		_DEBUG("NRF24_IOCSENRXADDR: %02lx", arg);
		write_reg(rf->spi, EN_RXADDR_REG, (u8 *)&arg, 1);
		break;
	
	case NRF24_IOCGENRXADDR:
		_DEBUG("NRF24_IOCGENRXADDR");
		read_reg(rf->spi, EN_RXADDR_REG, rxbuf, 1);
		put_user(rxbuf[0], (u8 __user *)arg);
		break;

        case NRF24_IOCSAW:
                _DEBUG("NRF24_IOCSAW: %ld", arg);
                arg -= 2;
                write_reg(rf->spi, SETUP_AW_REG, (u8 *)&arg, 1);
                break;

        case NRF24_IOCGAW:
                _DEBUG("NRF24_IOCGAW");
                read_reg(rf->spi, SETUP_AW_REG, rxbuf, 1);
                rxbuf[0] += 2;
                put_user(rxbuf[0], (u8 __user *)arg);
                break;

        case NRF24_IOCSRETR:
                _DEBUG("NRF24_IOCSRETR: 0x%02lx", arg);
                write_reg(rf->spi, SETUP_RETR_REG, (u8 *)&arg, 1);
                break;

        case NRF24_IOCGRETR:
                _DEBUG("NRF24_IOCGRETR");
                read_reg(rf->spi, SETUP_RETR_REG, rxbuf, 1);
                put_user(rxbuf[0], (u8 __user *)arg);
                break;

        case NRF24_IOCSRF:
                _DEBUG("NRF24_IOCSRF: 0x%02lx", arg);
                write_reg(rf->spi, RF_SETUP_REG, (u8 *)&arg, 1);
                break;

        case NRF24_IOCGRF:
                _DEBUG("NRF24_IOCGRF");
                read_reg(rf->spi, RF_SETUP_REG, rxbuf, 1);
                put_user(rxbuf[0], (u8 __user *)arg);
                break;

        case NRF24_IOCSDYNPLD:
                _DEBUG("NRF24_IOCSDYNPLD: 0x%02lx", arg);
		write_reg(rf->spi, DYNPD_REG, (u8 __user *)&arg, 1);
                break;

        case NRF24_IOCGDYNPLD:
                _DEBUG("NRF24_IOCGDYNPLD");
                read_reg(rf->spi, DYNPD_REG, rxbuf, 1);
                put_user(rxbuf[0], (u8 __user *)arg);
                break;

	case NRF24_IOCSFEATURE:
		_DEBUG("NRF24_IOCSFEATURE: 0x%02lx", arg);
		write_reg(rf->spi, FEATURE_REG, (u8 __user *)&arg, 1);
		break;

	case NRF24_IOCGFEATURE:
		_DEBUG("NRF24_IOCGFEATURE");
		read_reg(rf->spi, FEATURE_REG, rxbuf, 1);
		put_user(rxbuf[0], (u8 __user *)arg);
		break;

        case NRF24_IOTTR:
                _DEBUG("NRF24_IOTTR");
                do {
                        u8 tx_pld[32];
                        u8 rx_pld[32];
                        struct nrf_exch krn;
                        struct nrf_exch *usr = (struct nrf_exch *)arg;
                        int n = copy_from_user(&krn, usr, sizeof(krn));

                        if (n) {
                                status = -EIO;
                                break;
                        }

                        n = copy_from_user(tx_pld, krn.tx, min(sizeof(tx_pld), krn.tx_siz));
                        if (n) {
                                status = -EIO;
                                break;
                        }

			status = exchange_ttr(rf, tx_pld, krn.tx_siz,
					rx_pld, &krn.rx_siz, krn.timeout);
                        if (status == 0) {
                                status = -ETIMEDOUT;
                                break;
                        } else if (status < 0)
                                break; /* return status error */

                        _DEBUG("TTR received %d bytes", krn.rx_siz);

                        n = copy_to_user(krn.rx, rx_pld, krn.rx_siz);
                        if (n) {
                                status = -EIO;
                                break;
                        }

                        put_user(krn.rx_siz, &usr->rx_siz);

                        status = 0;
                        _DEBUG("Returning 0");
                } while (0);
                break;

        case NRF24_IOREADMULTICEIVER:
		_DEBUG("NRF24_IOREADMULTICEIVER");
		do {
			u8 rx_pld[32];
			struct nrf_read_multiceiver *usr = (struct nrf_read_multiceiver *)arg;
			struct nrf_read_multiceiver krn;
			int n = copy_from_user(&krn, usr, sizeof(krn));

			if (n) {
				pr_err("copy_from_user fails");
				status = -EIO;
				break;
			}

			rf->goto_prx = false;
			rf->new_rx_data = 0;
			flush_rx(rf->spi);
			prx_mode(rf->spi);
			write_reg(rf->spi, STATUS_REG, "\x70", 1);
			rf_led_on(rf);
			ce_high(rf->spi);
			if (krn.timeout > 0) {
				ktime_t ktimeout = ktime_set(0, (krn.timeout * 1000000) % 1000000000);
				_DEBUG("User timeout: %u", krn.timeout);
				status = wait_event_interruptible_hrtimeout(
					rf->rx_wq,
					rf->new_rx_data,
					ktimeout);
			} else {
				_DEBUG("No user timeout");
				status = wait_event_interruptible(rf->rx_wq, rf->new_rx_data);
			}
			ce_low(rf->spi);
			rf_led_off(rf);

			if (!status) {
				u8 aw, status, pldsiz, feature, dynpd, pipe_no, rxaddr[5], pipe_reg = 0, pipe_addr_reg = 0;

				/* retrieve the pipe number */
				read_reg(rf->spi, STATUS_REG, &status, 1);
				pipe_no = (status & 0b1110) >> 1; 
				_DEBUG("Received at pipe %d", pipe_no);
				switch (pipe_no) {
				case 0: pipe_reg = RX_PW_P0_REG; pipe_addr_reg = RX_ADDR_P0_REG; break;
				case 1: pipe_reg = RX_PW_P1_REG; pipe_addr_reg = RX_ADDR_P1_REG; break;
				case 2: pipe_reg = RX_PW_P2_REG; pipe_addr_reg = RX_ADDR_P2_REG; break;
				case 3: pipe_reg = RX_PW_P3_REG; pipe_addr_reg = RX_ADDR_P3_REG; break;
				case 4: pipe_reg = RX_PW_P4_REG; pipe_addr_reg = RX_ADDR_P4_REG; break;
				case 5: pipe_reg = RX_PW_P5_REG; pipe_addr_reg = RX_ADDR_P5_REG; break;
				}

				/* retrieve the payload size */
				read_reg(rf->spi, FEATURE_REG, &feature, 1);
				read_reg(rf->spi, DYNPD_REG, &dynpd, 1);
				if (IS_SETTED(feature, BIT(2)) && IS_SETTED(dynpd, BIT(pipe_no)))
					read_rx_pld_wid(rf->spi, &pldsiz);
				else
					read_reg(rf->spi, pipe_reg, &pldsiz, 1);

				/* retrieve the address width */
				read_reg(rf->spi, SETUP_AW_REG, &aw, 1);
				aw += 2;

				/* retrieve the pipe's address */
				switch (pipe_addr_reg) {
				case RX_ADDR_P0_REG:
				case RX_ADDR_P1_REG:
					read_reg(rf->spi, pipe_addr_reg, rxaddr, aw);
					break;
				case RX_ADDR_P2_REG:
				case RX_ADDR_P3_REG:
				case RX_ADDR_P4_REG:
				case RX_ADDR_P5_REG:
					read_reg(rf->spi, RX_ADDR_P1_REG, rxaddr, aw);
					read_reg(rf->spi, pipe_addr_reg, &rxaddr[aw-1], 1);
					break;
				}
				/* read payload */
				read_rx(rf->spi, rx_pld, pldsiz);

				/* update user stuff */
				if (copy_to_user(krn.rx, rx_pld, pldsiz)) {
					pr_err("copy_to_user fails");
					status = -EIO;
					break;
				}

				if (copy_to_user(krn.rx_addr, rxaddr, aw)) {
					pr_err("copy_to_user fails");
					status = -EIO;
					break;
				}

				put_user(pldsiz, &usr->rx_siz);
				put_user(aw,     &usr->rx_addr_len);
				status = 0;
			}
		} while (0);
                break;

        default:
                pr_warn("Unsupported ioctl used on file %s\n", fp->f_path.dentry->d_name.name);
                status = -ENOTTY;
		break;
        }

        mutex_unlock(&rf->bus_lock);
        return status;
}

static struct file_operations fops = {
        .owner = THIS_MODULE,
        .read = rf_read,
        .write = rf_write,
        .open = rf_open,
        .unlocked_ioctl = rf_ioctl,
        .release = rf_release,
};

/*
 * Driver operations
 */
static int rf_probe(struct spi_device *spi)
{
        int status;
        unsigned long minor;
        struct device *devp;
        struct rf_data *rf;
        struct device_node *np = spi->dev.of_node;
        int gpio_irq;

	dev_info(&spi->dev, "nrf24.ko: device probe");

        if (!np) {
		dev_err(&spi->dev, "No device tree, aborting");
                return -ENODEV;
	}

        rf = kzalloc(sizeof (struct rf_data), GFP_KERNEL);
        if (!rf)
                return -ENOMEM;

        rf->spi = spi;
        rf->new_rx_data = 0;
        rf->gpio_ce = of_get_named_gpio(np, "gpio-ce", 0);
        if (rf->gpio_ce < 0) {
		dev_err(&spi->dev, "No gpio-ce, aborting");
                return -ENODEV;
	}
	rf->gpio_led = of_get_named_gpio(np, "gpio-led", 0);
        INIT_LIST_HEAD(&rf->device_entry);
        mutex_init(&rf->bus_lock);
        init_completion(&rf->tx_complete);
        init_waitqueue_head(&rf->rx_wq);
        scnprintf(rf->devname, DEVNAME_MAX, "nrf24-%d.%d", spi->master->bus_num, spi->chip_select);

        gpio_irq = of_get_named_gpio(np, "gpio-irq", 0);
        spi->irq = gpio_to_irq(gpio_irq);
        if (spi->irq < 0) {
		dev_err(&spi->dev, "No IRQ configured, aborting");
                return -ENODEV;
	}
	spi->bits_per_word = 8;
	spi->mode = SPI_MODE_0;
        spi_setup(spi);
        spi_set_drvdata(spi, rf);

        status = request_threaded_irq(spi->irq, rf_hard_irq, rf_irq, IRQF_TRIGGER_FALLING, rf->devname, rf);
        if (status) {
		dev_err(&spi->dev, "request_threaded_irq failed, aborting");
                goto kfree_out;
	}

	status = gpio_request(rf->gpio_ce, rf->devname);
	if (status) {
		dev_err(&spi->dev, "Error while requesting CE gpio, aborting");
		goto free_irq_out;
	}
	gpio_direction_output(rf->gpio_ce, 0);

	if (rf->gpio_led > 0) {
		status = gpio_request(rf->gpio_led, "nRF24L01+ debug led");
		if (status) {
			dev_err(&spi->dev, "Error while requesting led gpio, aborting");
			goto gpio_free_out;
		}
		gpio_direction_output(rf->gpio_led, 1); /* 1=off, 0=on */
	}

        /* Default setup */
        write_reg(spi, EN_RXADDR_REG, "\x01", 1); /* enable only data pipe 0 */

        /* Sysfs attributes */
        status = sysfs_create_group(&spi->dev.kobj, &rf_attr_group);
        if (status) {
		dev_err(&spi->dev, "Error while creating sysfs files, aborting");
                goto gpio_free_out;
	}

        /* Device creation */
        mutex_lock(&device_list_lock);
        minor = find_first_zero_bit(minors, N_SPI_MINORS);
        if (minor > N_SPI_MINORS) {
                mutex_unlock(&device_list_lock);
                status = -ENODEV;
		dev_err(&spi->dev, "Max supported devices reached, aborting");
                goto sysfs_out;
        }

        rf->devt = MKDEV(MAJOR(rf_devt), MINOR(minor));
        devp = device_create(rf_class, &spi->dev, rf->devt, rf, "%s", rf->devname);
        if (IS_ERR(devp)) {
                mutex_unlock(&device_list_lock);
                status = PTR_ERR(devp);
		dev_err(&spi->dev, "Error while creating device file, aborting");
                goto sysfs_out;
        }
        set_bit(minor, minors);
        list_add(&rf->device_entry, &device_list);
        mutex_unlock(&device_list_lock);

	dev_info(&spi->dev, "Device probed with success: %s", rf->devname);
        return 0;
sysfs_out:
        sysfs_remove_group(&spi->dev.kobj, &rf_attr_group);
gpio_free_out:
	if (rf->gpio_led > 0)
		gpio_free(rf->gpio_led);
        gpio_free(rf->gpio_ce);
free_irq_out:
        free_irq(spi->irq, rf);
kfree_out:
        kfree(rf);
        return status;
}

static int rf_remove(struct spi_device *spi)
{
        struct rf_data *rf = spi_get_drvdata(spi);

        _DEBUG("");

        BUG_ON(!rf);
        
        mutex_lock(&device_list_lock);
        list_del(&rf->device_entry);
        clear_bit(MINOR(rf->devt), minors);
        device_destroy(rf_class, rf->devt);
        spi_set_drvdata(spi, NULL);
        mutex_unlock(&device_list_lock);

        free_irq(spi->irq, rf);
        gpio_free(rf->gpio_ce);
	if (rf->gpio_led > 0)
		gpio_free(rf->gpio_led);

        sysfs_remove_group(&spi->dev.kobj, &rf_attr_group);

        kfree(rf);
        return 0;
}

static const struct of_device_id rf_of_ids[] = {
        { .compatible = "nrf24" },
        {},
};

static struct spi_driver rf_driver = {
        .driver = {
                .name  = "nrf24",
                .owner = THIS_MODULE,
                .of_match_table = rf_of_ids,
        },
        .probe  = rf_probe,
        .remove = rf_remove,
};

static int __init rf_init(void)
{
        int status;

	pr_info("nrf24.ko: init");
        status = alloc_chrdev_region(&rf_devt, 0, N_SPI_MINORS, "nrf24");
        if (status)
                return status;

        cdev_init(&rf_cdev, &fops);
        status = cdev_add(&rf_cdev, rf_devt, N_SPI_MINORS);
        if (status) 
                goto unregister_chrdev_region_out;

        rf_class = class_create(THIS_MODULE, "nrf24");
        if (IS_ERR(rf_class)) {
                status = PTR_ERR(rf_class);
                goto cdev_del_out;
        }

        status = spi_register_driver(&rf_driver);
        if (status)
                goto class_destroy_out;

        return 0;

class_destroy_out:
        class_destroy(rf_class);
cdev_del_out:
        cdev_del(&rf_cdev);
unregister_chrdev_region_out:
        unregister_chrdev_region(rf_devt, N_SPI_MINORS);
        return status;
}

static void __exit rf_exit(void)
{
        _DEBUG("");
        spi_unregister_driver(&rf_driver);
        class_destroy(rf_class);
        cdev_del(&rf_cdev);
        unregister_chrdev_region(rf_devt, N_SPI_MINORS);
}

module_init(rf_init);
module_exit(rf_exit);
MODULE_AUTHOR("Daniel Hilst Selli <danielhilst@gmail.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("nRF24L01+ Driver");

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
static DEVICE_ATTR(statistics, 0400, show_statistics, NULL);
static DEVICE_ATTR(config, 0400, show_config, NULL);

static struct attribute *rf_attrs[] = {
        &dev_attr_registers.attr,
        &dev_attr_statistics.attr,
        &dev_attr_config.attr,
        NULL,
};

static struct attribute_group rf_attr_group = {
        .attrs = rf_attrs,
};

/*
 * IRQ Stuff
 */
static irqreturn_t rf_hard_irq(int irq, void *handle)
{
	return IRQ_WAKE_THREAD;
}

static irqreturn_t rf_irq(int irq, void *data)
{
        u8 status;
        struct rf_data *rf = data;

        read_reg(rf->spi, STATUS_REG, &status, 1);

        if (IS_SETTED(status, BIT(5))) {
                _DEBUG("TX IRQ served");
                complete(&rf->tx_complete);
                rf->txcount++;
        }

        if (IS_SETTED(status, BIT(6))) {
                _DEBUG("RX IRQ served");
                rf->rxcount++;
                rf->new_rx_data = 1;
                wake_up(&rf->rx_wq);
        }
        return IRQ_HANDLED;
}

/*
 * File oprations
 */
static int rf_open(struct inode *ip, struct file *fp)
{
        struct rf_data *rf;

        if (fp->private_data)
                return 0;

        mutex_lock(&device_list_lock);
        list_for_each_entry(rf, &device_list, device_entry) {
                if (rf->devt == ip->i_rdev) {
                        fp->private_data = rf;
                        SB1_MODE(rf->spi);
                        mutex_unlock(&device_list_lock);
                        udelay(150);                             /* power up delay */
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

        PD_MODE(rf->spi);
        fp->private_data = NULL;
        return 0;
}

static ssize_t rf_write(struct file *fp,
                        const char __user *buf,
                        size_t len,
                        loff_t *off)
{
        return -ENOSYS;
}

static ssize_t rf_read(struct file *fp,
                        char *buf,
                        size_t len,
                        loff_t *off)
{
        return -ENOSYS;
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
                                
#if defined(DEBUG)
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
#endif
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

#if defined(DEBUG)
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
#endif
                        write_reg(rf->spi, RX_ADDR_P0_REG, rxbuf, aw);
                } while (0);
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
                _DEBUG("NRF24_IOCSDYNPLD: %ld", arg);
                if (arg) {      /* enable */
                        read_reg(rf->spi, FEATURE_REG, rxbuf, 1);
                        rxbuf[0] |= BIT(2);
                        write_reg(rf->spi, FEATURE_REG, rxbuf, 1); /* assert EN_DPL bit */
                        write_reg(rf->spi, DYNPD_REG, "\x01", 1); /* assert DPL_P0 bit */
                } else {        /* disable */
                        read_reg(rf->spi, FEATURE_REG, rxbuf, 1);
                        rxbuf[0] &= ~BIT(2);
                        write_reg(rf->spi, FEATURE_REG, rxbuf, 1); /* clear EN_DPL bit */
                        write_reg(rf->spi, DYNPD_REG, "\x01", 1); /* clear DPL_P0 bit */
                }
                break;

        case NRF24_IOCGDYNPLD:
                _DEBUG("NRF24_IOCGDYNPLD");
                read_reg(rf->spi, DYNPD_REG, rxbuf, 1);
                put_user((rxbuf[0] & BIT(0)), (u8 __user *)arg);
                break;

        case NRF24_IOTTR:
                _DEBUG("NRF24_IOTTR");
                do {
                        u8 tx_pld[32];
                        u8 rx_pld[32];
                        struct nrf_exch krn;
                        struct nrf_exch *usr = (struct nrf_exch *)arg;
                        int n = copy_from_user(&krn, usr, sizeof(krn));
                        unsigned long tout;

                        if (n) {
                                status = -EIO;
                                break;
                        }
                        _DEBUG("User timeout %u", krn.timeout);
                        tout = krn.timeout * HZ / 1000;

                        n = copy_from_user(tx_pld, krn.tx, min(sizeof(tx_pld), krn.tx_siz));
                        if (n) {
                                status = -EIO;
                                break;
                        }

                        status = TX(rf, tx_pld, krn.tx_siz, tout);
                        if (status <= 0) {
                                _DEBUG("TX timedout");
                                status = -ETIMEDOUT;
                                break;
                        };
                        _DEBUG("TTR transmited");

                        status = RX(rf, rx_pld, krn.rx_siz, tout);
                        if (status <= 0) {
                                _DEBUG("RX timedout");
                                status = -ETIMEDOUT;
                                break;
                        }
                        _DEBUG("TTR received");
                                        
                        n = copy_to_user(usr->rx, rx_pld, sizeof(rx_pld));
                        if (n) {
                                status = -EIO;
                                break;
                        }

                        put_user(krn.rx_siz, &usr->rx_siz);

                        status = 0;
                        _DEBUG("Returning 0");
                } while (0);
                break;

        case NRF24_IORTT:
                _DEBUG("NRF24_IORTT");
                do {
                        u8 tx_pld[32];
                        u8 rx_pld[32];
                        struct nrf_exch krn;
                        struct nrf_exch *usr = (struct nrf_exch *)arg;
                        int n = copy_from_user(&krn, usr, sizeof(krn));
                        unsigned long tout;

                        tout = krn.timeout * HZ / 1000;

                        if (n) {
                                status = -EIO;
                                break;
                        }

                        status = RX(rf, rx_pld, krn.rx_siz, tout);
                        if (status <= 0) {
                                _DEBUG("RX timedout");
                                status = -ETIMEDOUT;
                                break;
                        }
                        _DEBUG("RTT received");

                        n = copy_from_user(tx_pld, krn.tx, sizeof(tx_pld));
                        if (n) {
                                status = -EIO;
                                break;
                        }

                        status = TX(rf, tx_pld, krn.tx_siz, tout);
                        if (status <= 0) {
                                _DEBUG("TX timedout");
                                status = -ETIMEDOUT;
                                break;
                        };
                        _DEBUG("RTT transmited");

                        n = copy_to_user(usr->rx, rx_pld, sizeof(rx_pld));
                        if (n) {
                                status = -EIO;
                                break;
                        }

                        status = 0;
                        _DEBUG("Returning 0");
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

static int rf_statistics_thread(void *data)
{
        struct rf_data *rf = data;
        u8 pldsiz;
        unsigned long tx_dif, rx_dif;

        read_reg(rf->spi, RX_PW_P0_REG, &pldsiz, 1);
        /* do the first read */
        tx_dif = rf->txcount;
        rx_dif = rf->rxcount;
        msleep(1000);

        /* do first read */
        while (!kthread_should_stop()) {
                rf->tx_kbps = (rf->txcount - tx_dif) * pldsiz * 8 / 1000;
                rf->rx_kbps = (rf->rxcount - rx_dif) * pldsiz * 8 / 1000;
                tx_dif = rf->txcount;
                rx_dif = rf->rxcount;
                msleep(1000);   /* wait one sec */
        }

        return 0;
}

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

        _DEBUG("");

        if (!np)
                return -ENODEV;

        rf = kzalloc(sizeof (struct rf_data), GFP_KERNEL);
        if (!rf)
                return -ENOMEM;

        rf->spi = spi;
        rf->new_rx_data = 0;
        rf->gpio_ce = of_get_named_gpio(np, "gpio-ce", 0);
        if (rf->gpio_ce < 0)
                return -ENODEV;
        INIT_LIST_HEAD(&rf->device_entry);
        mutex_init(&rf->bus_lock);
        init_completion(&rf->tx_complete);
        init_waitqueue_head(&rf->rx_wq);
        scnprintf(rf->devname, DEVNAME_MAX, "nrf24-%d.%d", spi->master->bus_num, spi->chip_select);

        gpio_irq = of_get_named_gpio(np, "gpio-irq", 0);
        spi->irq = gpio_to_irq(gpio_irq);
        if (spi->irq < 0)
                return -ENODEV;
        spi_setup(spi);
        spi_set_drvdata(spi, rf);

        status = request_threaded_irq(spi->irq, rf_hard_irq, rf_irq, IRQF_TRIGGER_FALLING, rf->devname, rf);
        if (status)
                goto kfree_out;

        status = gpio_request(rf->gpio_ce, rf->devname);
        if (status)
                goto free_irq_out;
        gpio_direction_output(rf->gpio_ce, 0);

        /* Default setup */
        write_reg(spi, EN_AA_REG, "\x00", 1); /* Disable autoack for faster IRQ */
        write_reg(spi, RX_PW_P0_REG, "\x20", 1); /* setup 32bytes of payload by default */
        write_reg(spi, STATUS_REG, "\x70", 1); /* Reset status register */
        write_reg(spi, RF_SETUP_REG, "\x26", 1); /* setup speed & ouput power */
        write_reg(spi, EN_RXADDR_REG, "\x01", 1); /* enable only data pipe 0 */
        disable_irqs(spi);                        /* disable irqs (are enabled on transmitting/receiving */

        /* Sysfs attributes */
        status = sysfs_create_group(&spi->dev.kobj, &rf_attr_group);
        if (status)
                goto gpio_free_out;

        /* Statistics */
        rf->statistics_tsk = kthread_run(rf_statistics_thread, rf, "nrf24-statistics");
        if (!rf->statistics_tsk) {
                status = -ENODEV;
                goto sysfs_out;
        }

        /* Device creation */
        mutex_lock(&device_list_lock);
        minor = find_first_zero_bit(minors, N_SPI_MINORS);
        if (minor > N_SPI_MINORS) {
                mutex_unlock(&device_list_lock);
                status = -ENODEV;
                goto stop_stat_thread;
        }

        rf->devt = MKDEV(MAJOR(rf_devt), MINOR(minor));
        devp = device_create(rf_class, &spi->dev, rf->devt, rf, "%s", rf->devname);
        if (IS_ERR(devp)) {
                mutex_unlock(&device_list_lock);
                status = PTR_ERR(devp);
                goto stop_stat_thread;
        }
        set_bit(minor, minors);
        list_add(&rf->device_entry, &device_list);
        mutex_unlock(&device_list_lock);

        return 0;
stop_stat_thread:
        kthread_stop(rf->statistics_tsk);
sysfs_out:
        sysfs_remove_group(&spi->dev.kobj, &rf_attr_group);
gpio_free_out:
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
        
        kthread_stop(rf->statistics_tsk);
        mutex_lock(&device_list_lock);
        list_del(&rf->device_entry);
        clear_bit(MINOR(rf->devt), minors);
        device_destroy(rf_class, rf->devt);
        spi_set_drvdata(spi, NULL);
        mutex_unlock(&device_list_lock);

        free_irq(spi->irq, rf);
        gpio_free(rf->gpio_ce);

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

        _DEBUG("");

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

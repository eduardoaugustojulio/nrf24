#ifndef RF_MODULE_H
#define RF_MODULE_H

#include <asm/uaccess.h>

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/spi/spi.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/cdev.h>
#include <linux/gpio.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/wait.h>

#include <linux/nrf24.h>

#define MODULE_NAME "nrf24"
#define _DEBUG(fmt, msg...)                                             \
        do {                                                            \
                pr_debug("%s:%s@%d: "                                   \
                         fmt "\n", MODULE_NAME, __func__, __LINE__, ##msg);  \
        } while (0);

#define IS_SETTED(bit, mask) (mask & bit)

/* Commands */
#define R_REGISTER(addr) (addr)
#define W_REGISTER(addr) ((addr) + 0x20)
#define W_ACK_PAYLOAD(p) (0xa8 | (0x07 & p))  /* write ack payload */

#define R_RX_PAYLOAD 0x61
#define W_TX_PAYLOAD 0xa0
#define FLUSH_TX     0xe1
#define FLUSH_RX     0xe2
#define REUSE_TX_PL  0xe3
#define ACTIVATE     0x50
#define R_RX_PL_WID  0x60
#define W_TX_PAYLOAD_NO_ACK 0xb0
#define NOP_CMD      0xff 

/* Registers Address */
#define CONFIG_REG      0x00
#define EN_AA_REG       0x01
#define EN_RXADDR_REG   0x02
#define SETUP_AW_REG    0x03
#define SETUP_RETR_REG  0x04
#define RF_CH_REG       0x05
#define RF_SETUP_REG    0x06
#define STATUS_REG      0x07
#define OBSERVE_TX_REG  0x08
#define CD_REG          0x09
#define RX_ADDR_P0_REG  0x0a
#define RX_ADDR_P1_REG  0x0b
#define RX_ADDR_P2_REG  0x0c
#define RX_ADDR_P3_REG  0x0d
#define RX_ADDR_P4_REG  0x0e
#define RX_ADDR_P5_REG  0x0f
#define TX_ADDR_REG     0x10
#define RX_PW_P0_REG    0x11
#define RX_PW_P1_REG    0x12
#define RX_PW_P2_REG    0x13
#define RX_PW_P3_REG    0x14
#define RX_PW_P4_REG    0x15
#define RX_PW_P5_REG    0x16
#define FIFO_STATUS_REG 0x17
#define DYNPD_REG       0x1c
#define FEATURE_REG     0x1d

/* IRQ Sources */
#define IRQ_MASK_RX_DR BIT(6)
#define IRQ_MASK_TX_DS BIT(5)
#define IRQ_MASK_MAX_RT BIT(4)

struct rf_data {
        dev_t devt;
        struct spi_device *spi;
        struct list_head device_entry;
        struct mutex bus_lock;
        unsigned int gpio_ce;
#define DEVNAME_MAX 16
        char devname[DEVNAME_MAX];
        struct completion tx_complete;
        wait_queue_head_t rx_wq;
        int new_rx_data;
        /* spi_async stuff */
        spinlock_t          async_lck;
        struct spi_message  async_msg;
        struct spi_transfer async_tsf[2];
        u8 async_cmd;
        u8 async_arg[32];
        /* statistics */
        unsigned long txcount;
        unsigned long rxcount;
        unsigned long tx_kbps;
        unsigned long rx_kbps;
        struct task_struct *statistics_tsk;
};

#endif 

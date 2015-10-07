#ifndef RF_HELPERS_H
#define RF_HELPERS_H

#include <linux/spi/spi.h>
#include <linux/types.h>

/* Avoid undefined function of TRACING isn't enabled */
#if !defined(CONFIG_TRACING)
#define trace_puts(x)
#endif

/* power up is assumed */
#define TX(rf, payload, timeout)                                        \
        ({                                                              \
                int status;                                             \
                                                                        \
                _DEBUG("Transmitting, timeout %lu", timeout);           \
                flush_tx((rf)->spi);                                    \
                write_tx((rf)->spi, (payload), 32);                     \
                write_reg((rf)->spi, STATUS_REG, "\x70", 1);            \
                ptx_mode((rf)->spi);                                    \
                trace_puts("Waiting TX IRQ\n");                         \
                enable_irqs((rf)->spi);                                 \
                ce_high((rf)->spi);                                     \
                status = wait_for_completion_interruptible_timeout(     \
                        &(rf)->tx_complete,                             \
                        timeout);                                       \
                ce_low((rf)->spi);                                      \
                disable_irqs((rf)->spi);                                \
                status;                                                 \
        })

/* power up is assumed */
#define RX(rf, payload, timeout)                                        \
        ({                                                              \
                int status;                                             \
                                                                        \
                _DEBUG("Receiving, timeout %lu", timeout);              \
                flush_rx((rf)->spi);                                    \
                write_reg((rf)->spi, STATUS_REG, "\x70", 1);            \
                prx_mode((rf)->spi);                                    \
                (rf)->new_rx_data = 0;                                  \
                trace_puts("Waiting RX IRQ\n");                         \
                enable_irqs((rf)->spi);                                 \
                ce_high((rf)->spi);                                     \
                status = wait_event_interruptible_timeout(              \
                        (rf)->rx_wq,                                    \
                        (rf)->new_rx_data,                              \
                        timeout);                                       \
                ce_low((rf)->spi);                                      \
                disable_irqs((rf)->spi);                                \
                if (status > 0) {                                       \
                        unsigned long _flags;                           \
                                                                        \
                        trace_puts("Copying received frame\n");         \
                        spin_lock_irqsave(&(rf)->async_lck, _flags);    \
                        memcpy((payload), rf->async_arg, ARRAY_SIZE(payload)); \
                        spin_unlock_irqrestore(&(rf)->async_lck, _flags); \
                }                                                       \
                                                                        \
                                                                        \
                status;                                                 \
})

#define PD_MODE(spi)                                    \
        do {                                            \
                _DEBUG("Change to POWER-DOWN MODE");    \
                ce_low((spi));                          \
                pwr_down((spi));                        \
        } while (0)

#define SB1_MODE(spi)                                   \
        do {                                            \
                _DEBUG("Change to STANDBY-I MODE");     \
                pwr_up((spi));                          \
                ce_low((spi));                          \
        } while (0)


int write_reg(struct spi_device *spi, u8 reg, u8 *val, int len);
int read_reg(struct spi_device *spi, u8 reg, u8 *val, int len);
int write_tx(struct spi_device *spi, u8 *val, int len);
void ce_high(struct spi_device *spi);
void ce_low(struct spi_device *spi);
void pwr_up(struct spi_device *spi);
void pwr_down(struct spi_device *spi);
void prx_mode(struct spi_device *spi);
void ptx_mode(struct spi_device *spi);
int flush_rx(struct spi_device *spi);
int flush_tx(struct spi_device *spi);
int is_rx_empty(struct spi_device *spi);
int read_rx(struct spi_device *spi, u8 *payload, int len);

int read_reg_nosleep(struct spi_device *spi,
                     u8 reg,
                     int len,
                     void (*complete)(void *),
                     void *context);

int write_reg_nosleep(struct spi_device *spi,
                      u8 reg,
                      u8 *val,
                      int len,
                      void (*complete)(void *),
                      void *context);

int read_rx_nosleep(struct spi_device *spi,
                    void (*complete)(void *),
                    void *context);

void enable_irqs(struct spi_device *spi);
void disable_irqs(struct spi_device *spi);

#endif

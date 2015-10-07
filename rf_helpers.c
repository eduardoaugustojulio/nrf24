#include "rf_module.h"
#include "rf_helpers.h"

int write_reg(struct spi_device *spi, u8 reg, u8 *val, int len)
{
        int status;
        u8 cmd = W_REGISTER(reg);
        struct spi_message m;
        struct spi_transfer t[2] = {
                [0] = {
                        .tx_buf = &cmd,
                        .len = 1,
                },
                [1] = {
                        .tx_buf = val,
                        .len = len,
                }
        };

        spi_message_init(&m);
        spi_message_add_tail(&t[0], &m);
        spi_message_add_tail(&t[1], &m);
        status = spi_sync(spi, &m);
        if (status)
                dev_err(&spi->dev, "Error while wrinting on SPI");
        
        return status;
}

int read_reg(struct spi_device *spi, u8 reg, u8 *val, int len)
{
        int status;
        u8 cmd = R_REGISTER(reg);
        struct spi_message m;
        struct spi_transfer t[2] = {
                [0] = {
                        .tx_buf = &cmd,
                        .len = 1,
                },
                [1] = {
                        .rx_buf = val,
                        .len = len,
                }
        };

        spi_message_init(&m);
        spi_message_add_tail(&t[0], &m);
        spi_message_add_tail(&t[1], &m);
        status = spi_sync(spi, &m);
        if (status)
                dev_err(&spi->dev, "Error while reading from SPI");
        
        return status;
}

int write_tx(struct spi_device *spi, u8 *val, int len)
{
        int status;
        u8 cmd = W_TX_PAYLOAD;
        struct spi_message m;
        struct spi_transfer t[2] = {
                [0] = {
                        .tx_buf = &cmd,
                        .len = 1,
                },
                [1] = {
                        .tx_buf = val,
                        .len = len,
                }
        };

        spi_message_init(&m);
        spi_message_add_tail(&t[0], &m);
        spi_message_add_tail(&t[1], &m);
        status = spi_sync(spi, &m);
        if (status)
                dev_err(&spi->dev, "Error while writing TX payload on SPI");
        
        return status;
}

void ce_high(struct spi_device *spi)
{
        struct rf_data *rf = spi_get_drvdata(spi);
        
        BUG_ON(!rf);
        gpio_set_value(rf->gpio_ce, 1);
}

void ce_low(struct spi_device *spi)
{
        struct rf_data *rf = spi_get_drvdata(spi);
        
        BUG_ON(!rf);
        gpio_set_value(rf->gpio_ce, 0);
}
        
void pwr_up(struct spi_device *spi)
{
        u8 config;
        
        read_reg(spi, CONFIG_REG, &config, 1);
        if (IS_SETTED(BIT(1), config))
                return;
        config |= BIT(1);       /* set PWR_UP */
        write_reg(spi, CONFIG_REG, &config, 1);
}
        
void pwr_down(struct spi_device *spi)
{
        u8 config;
        
        read_reg(spi, CONFIG_REG, &config, 1);
        if (!IS_SETTED(BIT(1), config))
                return;
        config &= ~(BIT(1));    /* clear PWR_UP */
        write_reg(spi, CONFIG_REG, &config, 1);
}

void enable_irqs(struct spi_device *spi)
{
        u8 config;

        read_reg(spi, CONFIG_REG, &config, 1);
        config &= ~(BIT(4) | BIT(5) | BIT(6));
        write_reg(spi, CONFIG_REG, &config, 1);
}

void disable_irqs(struct spi_device *spi)
{
        u8 config;

        read_reg(spi, CONFIG_REG, &config, 1);
        config |= (BIT(4) | BIT(5) | BIT(6));
        write_reg(spi, CONFIG_REG, &config, 1);
}

void prx_mode(struct spi_device *spi)
{
        u8 config;
        
        read_reg(spi, CONFIG_REG, &config, 1);
        config |= BIT(0);
        write_reg(spi, CONFIG_REG, &config, 1);
}

void ptx_mode(struct spi_device *spi)
{
        u8 config;
        
        read_reg(spi, CONFIG_REG, &config, 1);
        config &= ~(BIT(0));    /* clear PRIM_RX */
        write_reg(spi, CONFIG_REG, &config, 1);
}

int flush_rx(struct spi_device *spi)
{
        int status;
        u8 cmd = FLUSH_RX;

        struct spi_message m;
        struct spi_transfer t = {
                .tx_buf = &cmd,
                .len = 1,
        };


        spi_message_init(&m);
        spi_message_add_tail(&t, &m);
        status = spi_sync(spi, &m);
        if (status)
                dev_err(&spi->dev, "Error while sending FLUSH_RX command on SPI");
        
        return status;
}

int flush_tx(struct spi_device *spi)
{
        int status;
        u8 cmd = FLUSH_TX;

        struct spi_message m;
        struct spi_transfer t = {
                .tx_buf = &cmd,
                .len = 1,
        };


        spi_message_init(&m);
        spi_message_add_tail(&t, &m);
        status = spi_sync(spi, &m);
        if (status)
                dev_err(&spi->dev, "Error while sending FLUSH_TX command on SPI");
        
        return status;
}

int is_rx_empty(struct spi_device *spi)
{
        u8 fifo_status;

        read_reg(spi, FIFO_STATUS_REG, &fifo_status, 1);
        return IS_SETTED(BIT(0), fifo_status);
}

int read_rx(struct spi_device *spi, u8 *payload, int len)
{
        int status;
        u8 cmd = R_RX_PAYLOAD;
        struct spi_message m;
        struct spi_transfer t[2] = {
                [0] = {
                        .tx_buf = &cmd,
                        .len = 1,
                },
                [1] = {
                        .rx_buf = payload,
                        .len = len,
                }
        };

        spi_message_init(&m);
        spi_message_add_tail(&t[0], &m);
        spi_message_add_tail(&t[1], &m);
        status = spi_sync(spi, &m);
        if (status)
                dev_err(&spi->dev, "Error while reading RX payload from SPI");

        return status;
}

int read_reg_nosleep(struct spi_device *spi,
                     u8 reg,
                     int len,
                     void (*complete)(void *),
                     void *context)
{
        struct rf_data *rf = spi_get_drvdata(spi);

        BUG_ON(!spi);
        BUG_ON(len <= 0);
        BUG_ON(!complete);

        memset(rf->async_tsf, '\0', ARRAY_SIZE(rf->async_tsf));
        rf->async_tsf[0].tx_buf = &rf->async_cmd;
        rf->async_tsf[0].len    = 1;
        rf->async_tsf[1].rx_buf = &rf->async_arg;
        rf->async_tsf[1].len    = len;
        rf->async_cmd           = R_REGISTER(reg);

        spi_message_init(&rf->async_msg);
        spi_message_add_tail(&rf->async_tsf[0], &rf->async_msg);
        spi_message_add_tail(&rf->async_tsf[1], &rf->async_msg);
        rf->async_msg.complete = complete;
        rf->async_msg.context  = context;
        return spi_async(spi, &rf->async_msg);
}

int write_reg_nosleep(struct spi_device *spi,
                      u8 reg,
                      u8 *val,
                      int len,
                      void (*complete)(void *),
                      void *context)
{
        struct rf_data *rf = spi_get_drvdata(spi);

        BUG_ON(!spi);
        BUG_ON(len >= 0);
        BUG_ON(!complete);

        memset(rf->async_tsf, '\0', ARRAY_SIZE(rf->async_tsf));
        rf->async_tsf[0].tx_buf = &rf->async_cmd;
        rf->async_tsf[0].len    = 1;
        rf->async_tsf[1].tx_buf = &rf->async_arg;
        rf->async_tsf[1].len    = len;
        rf->async_cmd           = W_REGISTER(reg);

        spi_message_init(&rf->async_msg);
        spi_message_add_tail(&rf->async_tsf[0], &rf->async_msg);
        spi_message_add_tail(&rf->async_tsf[1], &rf->async_msg);
        rf->async_msg.complete = complete;
        rf->async_msg.context  = context;
        return spi_async(spi, &rf->async_msg);
}

int read_rx_nosleep(struct spi_device *spi,
                    void (*complete)(void *),
                    void *context)
{
        struct rf_data *rf = spi_get_drvdata(spi);

        BUG_ON(!spi);
        BUG_ON(!complete);

        memset(rf->async_tsf, '\0', ARRAY_SIZE(rf->async_tsf));
        rf->async_tsf[0].tx_buf = &rf->async_cmd;
        rf->async_tsf[0].len    = 1;
        rf->async_tsf[1].rx_buf = &rf->async_arg;
        rf->async_tsf[1].len    = 32;
        rf->async_cmd           = R_RX_PAYLOAD;

        spi_message_init(&rf->async_msg);
        spi_message_add_tail(&rf->async_tsf[0], &rf->async_msg);
        spi_message_add_tail(&rf->async_tsf[1], &rf->async_msg);
        rf->async_msg.complete = complete;
        rf->async_msg.context  = context;
        return spi_async(spi, &rf->async_msg);
}

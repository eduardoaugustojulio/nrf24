#include "rf_module.h"
#include "rf_helpers.h"

#include <linux/ktime.h>
#include <linux/hrtimer.h>

static void invert_buffer(char *buf, char *inv, size_t len)
{
	int i;
	for (i = 0; i < len; i++)
		inv[len - 1 - i] = buf[i];
}

int write_reg(struct spi_device *spi, u8 reg, u8 *val, int len)
{
	char tmp[5];
        int status;
        u8 cmd = W_REGISTER(reg);
        struct spi_message m;
        struct spi_transfer t[2] = {
                [0] = {
                        .tx_buf = &cmd,
                        .len = 1,
                },
                [1] = {
                        .tx_buf = tmp,
                        .len = len,
                }
        };

	_DEBUG("%02x: %02x", reg, val[0]);
	invert_buffer(val, tmp, len);
        spi_message_init(&m);
        spi_message_add_tail(&t[0], &m);
        spi_message_add_tail(&t[1], &m);
        status = spi_sync(spi, &m);
        if (status)
                dev_err(&spi->dev, "Error while wrinting on SPI");
	else if (reg == CONFIG_REG) {
		struct rf_data *rf = spi_get_drvdata(spi);
		rf->config = val[0];
	}

        return status;
}

int read_reg(struct spi_device *spi, u8 reg, u8 *val, int len)
{
        int status;
	char tmp[5];
        u8 cmd = R_REGISTER(reg);
        struct spi_message m;
        struct spi_transfer t[2] = {
                [0] = {
                        .tx_buf = &cmd,
                        .len = 1,
                },
                [1] = {
                        .rx_buf = tmp,
                        .len = len,
                }
        };

	_DEBUG("%02x", cmd);
        spi_message_init(&m);
        spi_message_add_tail(&t[0], &m);
        spi_message_add_tail(&t[1], &m);
        status = spi_sync(spi, &m);
        if (status)
                dev_err(&spi->dev, "Error while reading from SPI");
	else {
		invert_buffer(tmp, val, len);
		if (reg == CONFIG_REG) {
			struct rf_data *rf = spi_get_drvdata(spi);
			rf->config = val[0];
		}
	}

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

	_DEBUG("");
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

	_DEBUG("");
        BUG_ON(!rf);
        gpio_set_value(rf->gpio_ce, 1);
}

void ce_low(struct spi_device *spi)
{
        struct rf_data *rf = spi_get_drvdata(spi);

	_DEBUG("");
        BUG_ON(!rf);
        gpio_set_value(rf->gpio_ce, 0);
}

void pwr_up(struct spi_device *spi)
{
        u8 config;

	_DEBUG("");
        read_reg(spi, CONFIG_REG, &config, 1);
        if (IS_SETTED(BIT(1), config))
                return;
        config |= BIT(1);       /* set PWR_UP */
        write_reg(spi, CONFIG_REG, &config, 1);
}

void pwr_down(struct spi_device *spi)
{
        u8 config;

	_DEBUG("");
        read_reg(spi, CONFIG_REG, &config, 1);
        if (!IS_SETTED(BIT(1), config))
                return;
        config &= ~(BIT(1));    /* clear PWR_UP */
        write_reg(spi, CONFIG_REG, &config, 1);
}

void prx_mode(struct spi_device *spi)
{
        u8 config;

	_DEBUG("");
        read_reg(spi, CONFIG_REG, &config, 1);
        config |= BIT(0);
        write_reg(spi, CONFIG_REG, &config, 1);
}

void ptx_mode(struct spi_device *spi)
{
        u8 config;

	_DEBUG("");
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


	_DEBUG("");
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

	_DEBUG("");
        spi_message_init(&m);
        spi_message_add_tail(&t, &m);
        status = spi_sync(spi, &m);
        if (status)
                dev_err(&spi->dev, "Error while sending FLUSH_TX command on SPI");

        return status;
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

	_DEBUG("");
        spi_message_init(&m);
        spi_message_add_tail(&t[0], &m);
        spi_message_add_tail(&t[1], &m);
        status = spi_sync(spi, &m);
        if (status)
                dev_err(&spi->dev, "Error while reading RX payload from SPI");

        return status;
}

int read_rx_pld_wid(struct spi_device *spi,
                    u8 *pldwid)
{
        int status;
        u8 cmd = R_RX_PL_WID;
        struct spi_message m;
        struct spi_transfer t[2] = {
                [0] = {
                        .tx_buf = &cmd,
                        .len = 1,
                },
                [1] = {
                        .rx_buf = pldwid,
                        .len = 1,
                }
        };

	_DEBUG("");
        spi_message_init(&m);
        spi_message_add_tail(&t[0], &m);
        spi_message_add_tail(&t[1], &m);
        status = spi_sync(spi, &m);
        if (status)
                dev_err(&spi->dev, "Error while reading RX payload size from SPI");

        return status;
}

static int prepare_ttr(struct rf_data *rf, unsigned char *tx_pld, size_t tx_pld_siz)
{
	int i;
	struct spi_message m;
	struct spi_transfer t[] = {
		/* W_TX_PAYLOAD */
		{ .tx_buf = "\xa0", .len = 1 }, 
		{ .tx_buf = tx_pld, .len = tx_pld_siz, .cs_change = 1 },
		/* STATUS_REG */	
		{ .tx_buf = "\x27", .len = 1 }, 
		{ .tx_buf = "\x70", .len = 1, .cs_change = 1 },
		/* CONFIG_REG */
		{ .tx_buf = "\x20", .len = 1 }, 
		{ .tx_buf = &rf->config, .len = 1 },
	};
	_DEBUG("");
	rf->config &= ~(BIT(0)); /* clear PRIM_RX */
	spi_message_init(&m);
	for (i = 0; i < ARRAY_SIZE(t); i++)
		spi_message_add_tail(&t[i], &m);
	return spi_sync(rf->spi, &m);
}

int exchange_ttr(struct rf_data *rf, unsigned char *payload, size_t pldsiz,
		unsigned char *rxpayload, size_t *rxpldsiz, unsigned long timeout_ms)
{
	int status;
	ktime_t ktimeout = ktime_set(0, (timeout_ms * 1000000) % 1000000000);

	_DEBUG("Starting exchange");
	rf_led_on(rf);
	rf->goto_prx = true;
	rf->new_rx_data = 0;
	prepare_ttr(rf, payload, pldsiz);
	reinit_completion(&rf->tx_complete);
	ce_high(rf->spi); /* is lowered at IRQ */
	status = wait_for_completion_interruptible_timeout( /* TX IRQ should happen */
			&rf->tx_complete,
			3);	
	if (status == 0) {/*  timeout */
		_DEBUG("TX timedout");
		flush_tx(rf->spi);
		goto out;
	} else if (status < 0) { /* error */
		_DEBUG("TX error");
		flush_tx(rf->spi);
		goto out;
	} else
		rf->tx_bytes_count += pldsiz;

	status = wait_event_interruptible_hrtimeout( /* RX IRQ should happen */
			rf->rx_wq,
			rf->new_rx_data,
			ktimeout);
	if (!status) {   /* new data */
		u8 reg;
		u8 pldwid;
		/* check for dynamic payload*/
		read_reg(rf->spi, FEATURE_REG, &reg, 1);
		if (IS_SETTED(reg, BIT(2)))
			read_rx_pld_wid(rf->spi,
					&pldwid);
		else
			read_reg(rf->spi,
					RX_PW_P0_REG,
					&pldwid,
					1);
		read_rx(rf->spi, rxpayload, pldwid);
		*rxpldsiz = pldwid;
		rf->rx_bytes_count += pldwid;
		status = 1;
		/* ce lowered at irq */
	} else if (status == -ETIME) {
		_DEBUG("RX timedout");
		rf->timeout_count++;
		status = 0;
		flush_rx(rf->spi);
	} else if (status == -ERESTARTSYS) {
		_DEBUG("RX interrupted");
		flush_rx(rf->spi);
	}
out:
	_DEBUG("Exchange finished");
	ce_low(rf->spi);
	rf_led_off(rf);
	rf->goto_prx = false;
	return status;
}

void rf_led_on(struct rf_data *rf)
{
	if (rf->gpio_led > 0)
		gpio_set_value(rf->gpio_led, 0);
}

void rf_led_off(struct rf_data *rf)
{
	if (rf->gpio_led > 0)
		gpio_set_value(rf->gpio_led, 1);
}

/**
 * @file
 *
 * @date Nov 16, 2018
 * @author Anton Bondarev
 */
#include <util/log.h>

#include <errno.h>
#include <stddef.h>
#include <stdint.h>

#include <hal/reg.h>
#include <drivers/common/memory.h>
#include <drivers/clk/ccm_imx6.h>
#include <drivers/i2c/i2c.h>
#include <drivers/iomuxc.h>

#include <embox/unit.h>

#include "imx_i2c.h"

EMBOX_UNIT_INIT(imx_i2c_init);

static int imx_i2c_read(struct i2c_adapter *adapter, struct i2c_msg *msgs,
		int num);

static const struct i2c_algorithm imx_i2c_algo = {
		.i2c_master_xfer = imx_i2c_read,
		.i2c_functionality = NULL,
};

#define I2C1_PIN_SEL  OPTION_GET(NUMBER,i2c1_pins_select)
#define I2C2_PIN_SEL  OPTION_GET(NUMBER,i2c2_pins_select)
#define I2C3_PIN_SEL  OPTION_GET(NUMBER,i2c3_pins_select)

#define I2C1_BASE     0x21A0000
#define I2C2_BASE     0x21A4000
#define I2C3_BASE     0x21A8000

#define I2C1_IRQ_NUM  68
#define I2C2_IRQ_NUM  69
#define I2C3_IRQ_NUM  70

static struct imx_i2c imx_i2c1_priv = {
		.irq_num = I2C1_IRQ_NUM,
		.base_addr = I2C1_BASE,
};

static struct imx_i2c imx_i2c2_priv = {
		.irq_num = I2C2_IRQ_NUM,
		.base_addr = I2C2_BASE,
};

static struct imx_i2c imx_i2c3_priv = {
		.irq_num = I2C3_IRQ_NUM,
		.base_addr = I2C3_BASE,
};

static struct i2c_adapter imx_i2c1_adap = {
	.i2c_algo_data = &imx_i2c1_priv,
	.i2c_algo = &imx_i2c_algo,
};

static struct i2c_adapter imx_i2c2_adap = {
	.i2c_algo_data = &imx_i2c2_priv,
	.i2c_algo = &imx_i2c_algo,
};

static struct i2c_adapter imx_i2c3_adap = {
	.i2c_algo_data = &imx_i2c3_priv,
	.i2c_algo = &imx_i2c_algo,
};

static inline void imx_i2c3_pins_init(void) {
#if I2C3_PIN_SEL == 1
	iomuxc_set_reg(IOMUXC_SW_MUX_CTL_PAD_GPIO03, 2);
	iomuxc_set_reg(IOMUXC_I2C3_SCL_IN_SELECT_INPUT, 1);

	iomuxc_set_reg(IOMUXC_SW_MUX_CTL_PAD_GPIO06, 2);
	iomuxc_set_reg(IOMUXC_I2C3_SDA_IN_SELECT_INPUT, 1);
#elif I2C3_PIN_SEL == 2
	iomuxc_set_reg(IOMUXC_SW_MUX_CTL_PAD_GPIO05, 6);
	iomuxc_set_reg(IOMUXC_I2C3_SCL_IN_SELECT_INPUT, 2);

	iomuxc_set_reg(IOMUXC_SW_MUX_CTL_PAD_GPIO16, 6);
	iomuxc_set_reg(IOMUXC_I2C3_SDA_IN_SELECT_INPUT, 2);
#else
	iomuxc_set_reg(IOMUXC_SW_MUX_CTL_PAD_EIM_DATA17, 6);
	iomuxc_set_reg(IOMUXC_I2C3_SCL_IN_SELECT_INPUT, 0);

	iomuxc_set_reg(IOMUXC_SW_MUX_CTL_PAD_EIM_DATA18, 6);
	iomuxc_set_reg(IOMUXC_I2C3_SDA_IN_SELECT_INPUT, 0);
#endif
}

static inline void imx_i2c2_pins_init(void) {
#if I2C2_PIN_SEL
	iomuxc_set_reg(IOMUXC_SW_MUX_CTL_PAD_KEY_COL3, 4);
	iomuxc_set_reg(IOMUXC_I2C2_SCL_IN_SELECT_INPUT, 1);

	iomuxc_set_reg(IOMUXC_SW_MUX_CTL_PAD_KEY_ROW3, 4);
	iomuxc_set_reg(IOMUXC_I2C2_SDA_IN_SELECT_INPUT, 1);
#else
	iomuxc_set_reg(IOMUXC_SW_MUX_CTL_PAD_EIM_EB2_B, 6);
	iomuxc_set_reg(IOMUXC_I2C2_SCL_IN_SELECT_INPUT, 0);

	iomuxc_set_reg(IOMUXC_SW_MUX_CTL_PAD_EIM_DATA16, 6);
	iomuxc_set_reg(IOMUXC_I2C2_SDA_IN_SELECT_INPUT, 0);
#endif
}

static inline void imx_i2c1_pins_init(void) {
#if I2C1_PIN_SEL
	iomuxc_set_reg(IOMUXC_SW_MUX_CTL_PAD_CSI0_DATA09, 4);
	iomuxc_set_reg(IOMUXC_I2C1_SCL_IN_SELECT_INPUT, 1);

	iomuxc_set_reg(IOMUXC_SW_MUX_CTL_PAD_CSI0_DATA08, 4);
	iomuxc_set_reg(IOMUXC_I2C1_SDA_IN_SELECT_INPUT, 1);
#else
	iomuxc_set_reg(IOMUXC_SW_MUX_CTL_PAD_EIM_DATA21, 6);
	iomuxc_set_reg(IOMUXC_I2C1_SCL_IN_SELECT_INPUT, 0);

	iomuxc_set_reg(IOMUXC_SW_MUX_CTL_PAD_EIM_DATA28, 1);
	iomuxc_set_reg(IOMUXC_I2C1_SDA_IN_SELECT_INPUT, 0);
#endif
}

static int imx_i2c_init(void) {

	i2c_bus_register(&imx_i2c1_adap, 1, "i2c1");
	i2c_bus_register(&imx_i2c2_adap, 2, "i2c2");
	i2c_bus_register(&imx_i2c3_adap, 3, "i2c3");

//	imx_i2c2_pins_init();
//	clk_enable("i2c2");

//	REG8_STORE(I2C2_BASE + IMX_I2C_IFDR, 0x3F); /* lowest freq */
//	REG8_STORE(I2C2_BASE + IMX_I2C_IADR, 0x10);
//	REG8_STORE(I2C2_BASE + IMX_I2C_I2CR, 0 );
//	REG8_STORE(I2C2_BASE + IMX_I2C_I2SR, 0);

	return 0;
}

static void delay(int i) {
	volatile int cnt;
	for (cnt = 0; cnt < i * 1000 ; cnt++) {
	}
}

int imx_i2c_bus_busy(struct imx_i2c *adapter, int for_busy) {
	uint32_t temp;
	volatile int i;

	for (i = 0; i < 100; i++) {
		temp = REG8_LOAD(adapter->base_addr + IMX_I2C_I2SR);
		/* check for arbitration lost */
		if (temp & IMX_I2C_I2SR_IAL) {
			temp &= ~IMX_I2C_I2SR_IAL;
			REG8_STORE(adapter->base_addr + IMX_I2C_I2SR, temp);
			log_error("arbitration lost");
			return -EAGAIN;
		}

		if (temp & IMX_I2C_I2SR_IBB) {
			return 0;
		}
		delay(100);
	}
	return -ETIMEDOUT;
}

static int imx_i2c_trx_complete(struct imx_i2c *adapter) {
	uint32_t temp;
	volatile int i;

	for (i = 0; i < 10000; i++) {
		temp = REG8_LOAD(adapter->base_addr + IMX_I2C_I2SR);

		if ( temp & IMX_I2C_I2SR_IIF) {
			return 0;
		}
		delay(100);
	}
	return -ETIMEDOUT;
}

int imx_i2c_send(uint16_t addr, uint8_t ch) {
	return 0;
}

static int imx_i2c_stop(struct imx_i2c *adapter) {
	uint32_t tmp;

	tmp = REG8_LOAD(adapter->base_addr + IMX_I2C_I2CR);
	tmp &= ~(IMX_I2C_I2CR_MTX | IMX_I2C_I2CR_TXAK);
	tmp = REG8_LOAD(adapter->base_addr + IMX_I2C_I2CR);
	delay(100);
	tmp &= ~(IMX_I2C_I2CR_MSTA) ;
	REG8_STORE(adapter->base_addr + IMX_I2C_I2CR, tmp);
	delay(100);
	tmp = REG8_LOAD(adapter->base_addr + IMX_I2C_I2CR);
	tmp &= ~(IMX_I2C_I2CR_IEN) ;
	REG8_STORE(adapter->base_addr + IMX_I2C_I2CR, tmp);

	return 0;
}

static int imx_i2c_start(struct imx_i2c *adapter) {
	uint32_t tmp;

	REG8_STORE(adapter->base_addr + IMX_I2C_I2SR, 0 );
	REG8_STORE(adapter->base_addr + IMX_I2C_I2CR, IMX_I2C_I2CR_IEN );

	delay(1000);

	/* Start I2C transaction */
	tmp = REG8_LOAD(adapter->base_addr + IMX_I2C_I2CR);
	tmp |= IMX_I2C_I2CR_MSTA ;
	REG8_STORE(adapter->base_addr+ IMX_I2C_I2CR, tmp);

	tmp = imx_i2c_bus_busy(adapter, 1);
	if (tmp) {
		return -1;
	}

	tmp = REG8_LOAD(adapter->base_addr + IMX_I2C_I2CR);
	tmp |= IMX_I2C_I2CR_MTX | IMX_I2C_I2CR_TXAK;
	REG8_STORE(adapter->base_addr + IMX_I2C_I2CR, tmp);

	return 0;
}
#if 0
static int imx_i2c_tx_byte(struct imx_i2c *adapter, uint8_t byte) {
	REG8_STORE(adapter->base_addr + IMX_I2C_I2SR, 0);
	REG8_STORE(adapter->base_addr + IMX_I2C_I2DR, byte);
	res = imx_i2c_trx_complete(adapter);
	if (res) {
		goto out;
	}

}
#endif

static int imx_i2c_rx(struct imx_i2c *adapter, uint16_t addr, uint8_t *buff, size_t sz) {
	int res = -1;
	int cnt;
	uint32_t tmp;

	REG8_STORE(adapter->base_addr + IMX_I2C_I2SR, 0);
	/* write slave address */
	REG8_STORE(adapter->base_addr + IMX_I2C_I2DR, (uint8_t)(((addr << 1) | 0x1) & 0xFF));
	res = imx_i2c_trx_complete(adapter);
	if (res) {
		goto out;
	}
	if (REG8_LOAD(adapter->base_addr + IMX_I2C_I2SR) &  IMX_I2C_I2SR_RXAK) {
		res = -ENODEV;
		goto out;
	}

	log_debug("ACK received %d", addr);
	tmp = REG8_LOAD(adapter->base_addr + IMX_I2C_I2CR);
	tmp &= ~(IMX_I2C_I2CR_MTX | IMX_I2C_I2CR_TXAK);
	REG8_STORE(adapter->base_addr + IMX_I2C_I2CR, tmp);

	/* dummy read */
	tmp = REG8_LOAD(adapter->base_addr + IMX_I2C_I2DR);

	res = sz;

	log_error("success %d ", sz);
	for (cnt = sz; cnt > 0; cnt--) {
		log_error("i2c rx sz (%d)", sz);
		tmp = imx_i2c_trx_complete(adapter);
		if (tmp) {
			log_error("i2c complition error");
			res = -1;
			goto out;
		}

		log_error("i2c complition ok");
		res = 1;

		if (cnt == 1) {
			/*
			 * It must generate STOP before read I2DR to prevent
			 * controller from generating another clock cycle
			 */
			tmp = REG8_LOAD(adapter->base_addr + IMX_I2C_I2CR);
			tmp &= ~(IMX_I2C_I2CR_MTX | IMX_I2C_I2CR_MSTA);
			REG8_STORE(adapter->base_addr + IMX_I2C_I2CR, tmp);
		}

		imx_i2c_bus_busy(adapter, 0);

		tmp = REG8_LOAD(adapter->base_addr + IMX_I2C_I2DR);

		*buff = (uint8_t)(tmp & 0xFF);
	}

	log_error("success");
out:
	return res;
}

static int imx_i2c_read(struct i2c_adapter *adap, struct i2c_msg *msgs, int num) {
	struct imx_i2c *adapter;
	int res = -1;

	adapter = adap->i2c_algo_data;
	if (imx_i2c_start(adapter)) {
		log_error("i2c  bus error");
		res = -1;
		goto out;
	}

	res = imx_i2c_rx(adapter, msgs->addr, msgs->buf, msgs->len);

out:
	imx_i2c_stop(adapter);

	return res;
}

static struct periph_memory_desc imx_i2c1_mem = {
	.start = I2C1_BASE,
	.len   = 0x100,
};

PERIPH_MEMORY_DEFINE(imx_i2c1_mem);

static struct periph_memory_desc imx_i2c2_mem = {
	.start = I2C2_BASE,
	.len   = 0x100,
};

PERIPH_MEMORY_DEFINE(imx_i2c2_mem);

static struct periph_memory_desc imx_i2c3_mem = {
	.start = I2C3_BASE,
	.len   = 0x100,
};

PERIPH_MEMORY_DEFINE(imx_i2c3_mem);

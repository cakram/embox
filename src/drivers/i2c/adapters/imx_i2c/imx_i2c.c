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

#include <embox/unit.h>

#include "imx_i2c.h"

EMBOX_UNIT_INIT(imx_i2c_init);

#define I2C1_PIN_SEL  OPTION_GET(NUMBER,i2c1_pins_select)
#define I2C2_PIN_SEL  OPTION_GET(NUMBER,i2c2_pins_select)
#define I2C3_PIN_SEL  OPTION_GET(NUMBER,i2c3_pins_select)

#define IOMUXC_BASE                         0x020E0000

#define IOMUXC_SW_MUX_CTL_PAD_EIM_EB2_B          0x08C
#define IOMUXC_SW_MUX_CTL_PAD_EIM_DATA16         0x090
#define IOMUXC_SW_MUX_CTL_PAD_EIM_DATA17         0x094
#define IOMUXC_SW_MUX_CTL_PAD_EIM_DATA18         0x098

#define IOMUXC_SW_MUX_CTL_PAD_EIM_DATA21         0x0A4

#define IOMUXC_SW_MUX_CTL_PAD_EIM_DATA24         0x0B4
#define IOMUXC_SW_MUX_CTL_PAD_EIM_DATA25         0x0B8

#define IOMUXC_SW_MUX_CTL_PAD_EIM_DATA28         0x0C4

#define IOMUXC_SW_MUX_CTL_PAD_KEY_COL0           0x1F8
#define IOMUXC_SW_MUX_CTL_PAD_KEY_ROW0           0x1FC

#define IOMUXC_SW_MUX_CTL_PAD_KEY_COL3           0x210
#define IOMUXC_SW_MUX_CTL_PAD_KEY_ROW3           0x214

#define IOMUXC_SW_MUX_CTL_PAD_GPIO03             0x22C

#define IOMUXC_SW_MUX_CTL_PAD_GPIO06             0x230

#define IOMUXC_SW_MUX_CTL_PAD_GPIO05             0x23C

#define IOMUXC_SW_MUX_CTL_PAD_GPIO16             0x248

#define IOMUXC_SW_MUX_CTL_PAD_CSI0_DATA08        0x278
#define IOMUXC_SW_MUX_CTL_PAD_CSI0_DATA09        0x27C

#define IOMUXC_SW_MUX_CTL_PAD_SD3_DATA7          0x2A8
#define IOMUXC_SW_MUX_CTL_PAD_SD3_DATA6          0x2AC

#define IOMUXC_I2C1_SCL_IN_SELECT_INPUT          0x898
#define IOMUXC_I2C1_SDA_IN_SELECT_INPUT          0x89C
#define IOMUXC_I2C2_SCL_IN_SELECT_INPUT          0x8A0
#define IOMUXC_I2C2_SDA_IN_SELECT_INPUT          0x8A4
#define IOMUXC_I2C3_SCL_IN_SELECT_INPUT          0x8A8
#define IOMUXC_I2C3_SDA_IN_SELECT_INPUT          0x8AC

#define IOMUXC_UART1_UART_RX_DATA_SELECT_INPUT   0x920
#define IOMUXC_UART2_UART_RX_DATA_SELECT_INPUT   0x928
#define IOMUXC_UART3_UART_RX_DATA_SELECT_INPUT   0x930
#define IOMUXC_UART4_UART_RX_DATA_SELECT_INPUT   0x938

void iomuxc_set_reg(uint32_t reg, uint32_t val) {
	REG32_STORE(IOMUXC_BASE + reg, val);
}

uint32_t iomuxc_get_reg(uint32_t reg) {
	return REG32_LOAD(IOMUXC_BASE + reg);
}


#define I2C1_BASE     0x21A0000
#define I2C2_BASE     0x21A4000
#define I2C3_BASE     0x21A8000

#define I2C1_IRQ_NUM  68
#define I2C2_IRQ_NUM  69
#define I2C3_IRQ_NUM  70

static struct imx_i2c imx_i2c1_adapter = {
		.irq_num = I2C1_IRQ_NUM,
		.base_addr = I2C1_BASE,
};

static struct imx_i2c imx_i2c2_adapter = {
		.irq_num = I2C2_IRQ_NUM,
		.base_addr = I2C2_BASE,
};

static struct imx_i2c imx_i2c3_adapter = {
		.irq_num = I2C3_IRQ_NUM,
		.base_addr = I2C3_BASE,
};

static inline void imx_i2c3_pins_init(void) {
#if I2C3_PIN_SEL == 1
	iomuxc_set_reg(IOMUXC_SW_MUX_CTL_PAD_GPIO03, 2)
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
	iomuxc_set_reg(IOMUXC_SW_MUX_CTL_PAD_KEY_COL3, 4)
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
	iomuxc_set_reg(IOMUXC_SW_MUX_CTL_PAD_CSI0_DATA09, 4)
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

	i2c_bus_register(&imx_i2c1_adapter, 1, "i2c1");
	i2c_bus_register(&imx_i2c2_adapter, 2, "i2c2");
	i2c_bus_register(&imx_i2c3_adapter, 3, "i2c3");

	//imx_i2c1_pins_init();
	//clk_enable("i2c1");

	REG8_STORE(I2C2_BASE + IMX_I2C_IFDR, 0x3F); /* lowest freq */
	REG8_STORE(I2C2_BASE + IMX_I2C_IADR, 0x17);
	REG8_STORE(I2C2_BASE + IMX_I2C_I2CR, 0 );
	REG8_STORE(I2C2_BASE + IMX_I2C_I2SR, 0);

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
			REG8_STORE(adapter->base_addr + IMX_I2C_I2SR, 0);
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
	tmp &= ~(IMX_I2C_I2CR_MSTA | IMX_I2C_I2CR_MTX) ;
	REG8_STORE(adapter->base_addr + IMX_I2C_I2CR, tmp);
	delay(100);
	tmp = REG8_LOAD(adapter->base_addr + IMX_I2C_I2CR);
	tmp &= ~(IMX_I2C_I2CR_IEN) ;
	REG8_STORE(adapter->base_addr + IMX_I2C_I2CR, tmp);

	return 0;
}

static int imx_i2c_start(struct imx_i2c *adapter) {
	uint32_t tmp;

	tmp = REG8_LOAD(adapter->base_addr + IMX_I2C_I2SR);

	REG8_STORE(adapter->base_addr + IMX_I2C_I2CR, IMX_I2C_I2CR_IEN );

	delay(100);

	/* Start I2C transaction */
	tmp = REG8_LOAD(adapter->base_addr + IMX_I2C_I2CR);
	tmp |= IMX_I2C_I2CR_MSTA | IMX_I2C_I2CR_MTX ;
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

static int imx_i2c_rx(struct imx_i2c *adapter, uint16_t addr, uint8_t *buff, size_t sz) {
	int res = -1;
	int cnt;
	uint32_t tmp;

	/* write slave address */
	REG8_STORE(adapter->base_addr + IMX_I2C_I2DR, (uint32_t)((addr << 1) | 0x1));
	res = imx_i2c_trx_complete(adapter);
	if (res) {
		goto out;
	}
	if (REG8_LOAD(adapter->base_addr + IMX_I2C_I2SR) &  IMX_I2C_I2SR_RXAK) {
		res = -1;
		goto out;
	}

	log_debug("ACK received %d", addr);
	tmp = REG8_LOAD(adapter->base_addr + IMX_I2C_I2CR);
	tmp &= ~(IMX_I2C_I2CR_MTX & IMX_I2C_I2CR_TXAK);
	REG8_STORE(adapter->base_addr + IMX_I2C_I2CR, tmp);

	/* dummy read */
	tmp = REG8_LOAD(adapter->base_addr + IMX_I2C_I2DR);

	res = sz;
	for (cnt = sz; cnt > 0; cnt--) {
		tmp = imx_i2c_trx_complete(adapter);
		if (tmp) {
			log_error("i2c complition error");
			res = -1;
			goto out;
		}

		res = 1;

		if (cnt == 1) {
			/*
			 * It must generate STOP before read I2DR to prevent
			 * controller from generating another clock cycle
			 */
			tmp = REG8_LOAD(adapter->base_addr + IMX_I2C_I2CR);
			tmp &= ~(IMX_I2C_I2CR_MTX & IMX_I2C_I2CR_MSTA);
			REG8_STORE(adapter->base_addr + IMX_I2C_I2CR, tmp);
		}

		imx_i2c_bus_busy(adapter, 0);

		tmp = REG8_LOAD(adapter->base_addr + IMX_I2C_I2DR);

		*buff = (uint8_t)(tmp & 0xFF);
	}

out:
	return res;
}

int imx_i2c_read(struct i2c_bus *bus, uint16_t addr, uint8_t *buff, size_t sz) {
	struct imx_i2c *adapter;
	int res = -1;

	log_debug("start %d", addr);
	adapter = bus->adapter_priv;
	if (imx_i2c_start(adapter)) {
		log_error("i2c  bus error");
		res = -1;
		goto out;
	}

	res = imx_i2c_rx(adapter, addr, buff, sz);
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


#if 0
/*  mapped in imx_uart */
static struct periph_memory_desc iomuxc_mem = {
	.start = IOMUXC_BASE,
	.len   = 0x1000,
};

PERIPH_MEMORY_DEFINE(iomuxc_mem);
#endif

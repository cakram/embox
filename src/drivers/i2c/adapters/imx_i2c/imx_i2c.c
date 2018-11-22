/**
 * @file
 *
 * @date Nov 16, 2018
 * @author Anton Bondarev
 */
#include <util/log.h>

#include <errno.h>

#include <hal/reg.h>
#include <drivers/common/memory.h>
#include <drivers/clk/ccm_imx6.h>
#include <embox/unit.h>

#include "imx_i2c.h"

EMBOX_UNIT_INIT(imx_i2c_init);

#define I2C1_PIN_SEL  OPTION_GET(NUMBER,i2c1_pins_select)
#define I2C2_PIN_SEL  OPTION_GET(NUMBER,i2c2_pins_select)
#define I2C3_PIN_SEL  OPTION_GET(NUMBER,i2c3_pins_select)

#define IOMUXC_BASE                         0x020E0000


#define IOMUXC_SW_MUX_CTL_PAD_EIM_DATA21         0x0A4

#define IOMUXC_SW_MUX_CTL_PAD_EIM_DATA24         0x0B4
#define IOMUXC_SW_MUX_CTL_PAD_EIM_DATA25         0x0B8

#define IOMUXC_SW_MUX_CTL_PAD_EIM_DATA28         0x0C4

#define IOMUXC_SW_MUX_CTL_PAD_KEY_COL0           0x1F8
#define IOMUXC_SW_MUX_CTL_PAD_KEY_ROW0           0x1FC

#define IOMUXC_SW_MUX_CTL_PAD_CSI0_DATA08        0x278
#define IOMUXC_SW_MUX_CTL_PAD_CSI0_DATA09        0x27C

#define IOMUXC_SW_MUX_CTL_PAD_SD3_DATA7          0x2A8
#define IOMUXC_SW_MUX_CTL_PAD_SD3_DATA6          0x2AC

#define IOMUXC_I2C1_SCL_IN_SELECT_INPUT          0x898
#define IOMUXC_I2C1_SDA_IN_SELECT_INPUT          0x89C

#define IOMUXC_UART1_UART_RX_DATA_SELECT_INPUT   0x920
#define IOMUXC_UART2_UART_RX_DATA_SELECT_INPUT   0x928
#define IOMUXC_UART3_UART_RX_DATA_SELECT_INPUT   0x930
#define IOMUXC_UART4_UART_RX_DATA_SELECT_INPUT   0x938


#define I2C1_BASE     0x21A0000
#define I2C2_BASE     0x21A4000
#define I2C3_BASE     0x21A8000

void iomuxc_set_reg(uint32_t reg, uint32_t val) {
	REG32_STORE(IOMUXC_BASE + reg, val);
}

uint32_t iomuxc_get_reg(uint32_t reg) {
	return REG32_LOAD(IOMUXC_BASE + reg);
}
#if 0
static void imx_i2c1_pins_init(void) {
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
#endif
static int imx_i2c_init(void) {
	uint32_t temp;

	temp = REG8_LOAD(I2C2_BASE + IMX_I2C_I2CR);
	log_debug("\ncr(%x)", temp);

	temp = REG8_LOAD(I2C2_BASE + IMX_I2C_I2SR);
	log_debug("\nsr(%x)", temp);

	temp = REG8_LOAD(I2C2_BASE + IMX_I2C_IFDR);
	log_debug("\nfr(%x)", temp);


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
	for (cnt = 0; cnt < i * 10000 ; cnt++) {
	}
}

int imx_i2c_bus_busy(void) {
	uint32_t temp;
	volatile int i;

	for (i = 0; i < 10; i++) {
		temp = REG8_LOAD(I2C2_BASE + IMX_I2C_I2SR);
		/* check for arbitration lost */
		if (temp & IMX_I2C_I2SR_IAL) {
			temp &= ~IMX_I2C_I2SR_IAL;
			REG8_STORE(I2C2_BASE + IMX_I2C_I2SR, temp);
			log_error("arbitration lost");
			return -EAGAIN;
		}

		if (temp & IMX_I2C_I2SR_IBB) {
			return 0;
		}
		delay(1000);
	}
	return -ETIMEDOUT;
}

static int imx_i2c_trx_complete(void) {
	uint32_t temp;
	volatile int i;

	for (i = 0; i < 10000; i++) {
		temp = REG8_LOAD(I2C2_BASE + IMX_I2C_I2SR);

		//log_debug("status(%x)", temp);
		if ( temp & IMX_I2C_I2SR_IIF) {
			REG8_STORE(I2C2_BASE + IMX_I2C_I2SR, 0);
			return 0;
		}
		delay(100);
	}
	return -ETIMEDOUT;
}

int imx_i2c_send(uint16_t addr, uint8_t ch) {
	return 0;
}

static int imx_i2c_start(void) {
	uint32_t tmp;

	tmp = REG8_LOAD(I2C2_BASE + IMX_I2C_I2SR);

	REG8_STORE(I2C2_BASE + IMX_I2C_I2CR, IMX_I2C_I2CR_IEN );

	delay(100);

	/* Start I2C transaction */
	tmp = REG8_LOAD(I2C2_BASE + IMX_I2C_I2CR);
	tmp |= IMX_I2C_I2CR_MSTA | IMX_I2C_I2CR_MTX ;
	REG8_STORE(I2C2_BASE + IMX_I2C_I2CR, tmp);

	tmp = imx_i2c_bus_busy();
	if (tmp) {
		tmp = REG8_LOAD(I2C2_BASE + IMX_I2C_I2CR);
		tmp &= ~(IMX_I2C_I2CR_MSTA | IMX_I2C_I2CR_MTX) ;
		REG8_STORE(I2C2_BASE + IMX_I2C_I2CR, tmp);
		delay(100);
		tmp = REG8_LOAD(I2C2_BASE + IMX_I2C_I2CR);
		tmp &= ~(IMX_I2C_I2CR_IEN) ;
		REG8_STORE(I2C2_BASE + IMX_I2C_I2CR, tmp);
	}

	tmp = REG8_LOAD(I2C2_BASE + IMX_I2C_I2CR);
	tmp |= IMX_I2C_I2CR_MTX | IMX_I2C_I2CR_TXAK;
	REG8_STORE(I2C2_BASE + IMX_I2C_I2CR, tmp);

	return 0;
}

int imx_i2c_receive(uint16_t addr, uint8_t *ch) {
	uint32_t tmp;
	int res = -1;

	log_debug("start %d", addr);
	if (imx_i2c_start()) {
		log_error("i2c  bus error");
		res = -1;
		goto out;
	}

	/* write slave address */
	REG8_STORE(I2C2_BASE + IMX_I2C_I2DR, (uint32_t)((addr << 1) | 0x1));
	tmp = imx_i2c_trx_complete();
	if (tmp) {
		log_error("i2c complition error");
		res = -1;
		goto out;
	}
	if (REG8_LOAD(I2C2_BASE + IMX_I2C_I2SR) &  IMX_I2C_I2SR_RXAK) {
		res = -1;
		goto out;
	}

	log_debug("ACK received %d", addr);
	tmp = REG8_LOAD(I2C2_BASE + IMX_I2C_I2CR);
	tmp &= ~(IMX_I2C_I2CR_MTX & IMX_I2C_I2CR_TXAK);
	REG8_STORE(I2C2_BASE + IMX_I2C_I2CR, tmp);

	/* dummy read */
	tmp = REG8_LOAD(I2C2_BASE + IMX_I2C_I2DR);
	tmp = imx_i2c_trx_complete();
	if (tmp) {
		log_error("i2c complition error");
		res = -1;
		goto out;
	}

	res = 1;
out:
	/*
	 * It must generate STOP before read I2DR to prevent
	 * controller from generating another clock cycle
	 */
	tmp = REG8_LOAD(I2C2_BASE + IMX_I2C_I2CR);
	tmp &= ~(IMX_I2C_I2CR_MTX & IMX_I2C_I2CR_MSTA);
	REG8_STORE(I2C2_BASE + IMX_I2C_I2CR, tmp);

	if (1 != res) {
		return res;
	}
	imx_i2c_bus_busy();

	tmp = REG8_LOAD(I2C2_BASE + IMX_I2C_I2DR);

	*ch = (uint8_t)(tmp & 0xFF);

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

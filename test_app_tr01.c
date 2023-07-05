/**
* @file test_app_tr01.c
* @brief This implementation file contains the code for testing tr01.
*
* @author Jimmy Chea
* @date 30/03/2023
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "at86rf215_bb/at86rf215_bb.h"
#include "at86rf215_bb/at86rf215_bb_regs.h"
#include "i2c/i2c.h"
#include "io_utils/io_utils.h"
#include "gpio_connector/gpio_connector.h"
#include "ftd2xx.h"
#include "libft4222.h"
#include "i2c_to_gpio/peripheral_gpio.h"
#include "rffc507x/rffc507x.h"
#include "rffc507x/rffc507x_regs.h"


void rfic_supply_voltage_2_5v_test(void);
void fpga_supply_voltage_1_8v_test(void);
void fpga_supply_voltage_1_2v_test(void);
void fpga_supply_voltage_3_3v_test(void);
void mixer_supply_voltage_3_3v_test(void);
void pa_supply_voltage_8_0v_test(void);
void lna_supply_voltage_8_0v_test(void);

void rfic_current_idle_state_test(void);
void rfic_current_tx_state_test(void);
void rfic_current_rx_state_test(void);
void mixer_current_test(void);

void rfic_tx_power_test(void);

void rfic_rx_sensitivity_test(void);

void freq_on_udc_while_tx_state_test(void);
void freq_on_udc_while_rx_state_test(void);

void adc_while_tx_state_test(void);

void temp_while_tx_state_test(void);

void i2c_test(void);

void spi_test(void);

void usb_test(void);

void rfic_interrupt_test(void);

void evt_test(void);

typedef struct {
    char *name;
    void (*test)(void);
} FunctionEntry;

FunctionEntry functionList[] = {
    // Voltage Measurement Test
    {"rfic_supply_voltage_2_5v",     &rfic_supply_voltage_2_5v_test},
    {"fpga_supply_voltage_1_8v",     &fpga_supply_voltage_1_8v_test},
    {"fpga_supply_voltage_1_2v",     &fpga_supply_voltage_1_2v_test},
    {"fpga_supply_voltage_3_3v",     &fpga_supply_voltage_3_3v_test},
    {"mixer_supply_voltage_3_3v",    &mixer_supply_voltage_3_3v_test},
    {"pa_supply_voltage_8_0v",       &pa_supply_voltage_8_0v_test},
    {"lna_supply_voltage_8_0v",      &lna_supply_voltage_8_0v_test},

    // Current Measurement Test
    {"rfic_current_idle_state",      &rfic_current_idle_state_test},
    {"rfic_current_tx_state",        &rfic_current_tx_state_test},
    {"rfic_current_rx_state",        &rfic_current_rx_state_test},
    {"mixer_current",                &mixer_current_test},

    // RFIC Tx Power Measurement Test
    {"rfic_tx_power",                &rfic_tx_power_test},

    // RFIC Rx Sensitivity Measurement Test
    {"rfic_rx_sensitivity",          &rfic_rx_sensitivity_test},

    // UDC Path Measurement Test
    {"freq_on_udc_while_tx_state",   &freq_on_udc_while_tx_state_test},
    {"freq_on_udc_while_rx_state",   &freq_on_udc_while_rx_state_test},

    // ADC Measurement Test
    {"adc_while_tx_state",           &adc_while_tx_state_test},

    // Temperature Measurement Test
    {"temp_while_tx_state",          &temp_while_tx_state_test},

    // I2C Interface Test
    {"i2c",                          &i2c_test},

    // SPI Interface Test
    {"spi",                          &spi_test},

    // USB Interface Test
    {"usb",                          &usb_test},

    // RFIC Interrupt
    {"rfic_interrupt",               &rfic_interrupt_test},

    // EVT Test
    {"evt",                          &evt_test},
};

static int mixer_3v3_rfic_2v5_fpga_1v8_power_enabled(void)
{
    int ret;

    printf("Enabled the mixer_3v3, rfic_2v5, fpga_1v8 supply voltage.\n");
    ret = gpio_rbe_enabled();
    if (ret < 0)
    {
        printf("RBE gpio enabled : Failed.\n");
    }
    else
    {
        printf("RBE gpio enabled : Passed.\n");
    }

    return ret;
}

static int mixer_3v3_rfic_2v5_fpga_1v8_power_disabled(void)
{
    int ret;

    printf("Disabled the mixer_3v3, rfic_2v5, fpga_1v8 supply voltage.\n");
    ret = gpio_rbe_disabled();
    if (ret < 0)
    {
        printf("RBE gpio disabled : Failed.\n");
    }
    else
    {
        printf("RBE gpio disabled : Passed.\n");
    }

    return ret;
}

static int fpga_1v2_3v3_power_enabled(void)
{
    int ret;

    printf("Enabled the fpga_1v2 and fpga_3v3 supply voltage.\n");
    ret = gpio_rf_1v2_en_enabled();
    if (ret < 0)
    {
        printf("RF_1V2_EN gpio enabled : Failed.\n");
    }
    else
    {
        printf("RF_1V2_EN gpio enabled : Passed.\n");
    }

    return ret;
}

static int fpga_1v2_3v3_power_disabled(void)
{
    int ret;

    printf("Disabled the fpga_1v2 and fpga_3v3 supply voltage.\n");
    ret = gpio_rf_1v2_en_disabled();
    if (ret < 0)
    {
        printf("RF_1V2_EN gpio disabled : Failed.\n");
    }
    else
    {
        printf("RF_1V2_EN gpio disabled : Passed.\n");
    }

    return ret;
}

static void print_func_start_format(const char* func_name)
{
    printf("\n");
    printf("<<< Start >>> : %s\n", func_name);
    printf("\n");
}

static void print_func_end_format(const char* func_name, int ret)
{
    printf("\n");
    printf("<<< End >>> : Completed testing with %s\n", func_name);
    printf("\n");

    if (ret < 0)
    {
        printf(">>> Test Result : < Failed >\n");
    }
    else
    {
        printf(">>> Test Result : < Passed >\n");
    }

    printf("\n");
}

void rfic_supply_voltage_2_5v_test(void)
{
    int ret = 0;

    print_func_start_format(__func__);

    ret = mixer_3v3_rfic_2v5_fpga_1v8_power_enabled();
    if (ret < 0)
    {
        goto exit;
    }

    printf("\n");
    printf("Measure your rfic supply voltage 2.5v.\n");
    printf("Once completed, press ENTER key to turn off the rfic supply voltage 2.5v.\n");

    getchar();
    ret = mixer_3v3_rfic_2v5_fpga_1v8_power_disabled();

exit:
    print_func_end_format(__func__, ret);
}

void fpga_supply_voltage_1_8v_test(void)
{
    int ret = 0;

    print_func_start_format(__func__);

    ret = mixer_3v3_rfic_2v5_fpga_1v8_power_enabled();
    if (ret < 0)
    {
        goto exit;
    }

    printf("\n");
    printf("Measure your fpga supply voltage 1.8v.\n");
    printf("Once completed, press ENTER key to turn off the fpga supply voltage 1.8v.\n");

    getchar();
    ret = mixer_3v3_rfic_2v5_fpga_1v8_power_disabled();

exit:

    print_func_end_format(__func__, ret);
}

void fpga_supply_voltage_1_2v_test(void)
{
    int ret = 0;

    print_func_start_format(__func__);

    ret = fpga_1v2_3v3_power_enabled();
    if (ret < 0)
    {
        goto exit;
    }

    printf("\n");
    printf("Measure your fpga supply voltage 1.2v.\n");
    printf("Once completed, press ENTER key to turn off the fpga supply voltage 1.2v.\n");

    getchar();
    ret = fpga_1v2_3v3_power_disabled();

exit:

    print_func_end_format(__func__, ret);
}

void fpga_supply_voltage_3_3v_test(void)
{
    int ret = 0;

    print_func_start_format(__func__);

    ret = fpga_1v2_3v3_power_enabled();
    if (ret < 0)
    {
        goto exit;
    }

    printf("\n");
    printf("Measure your fpga supply voltage 3.3v.\n");
    printf("Once completed, press ENTER key to turn off the fpga supply voltage 3.3v.\n");

    getchar();
    ret = fpga_1v2_3v3_power_disabled();

exit:

    print_func_end_format(__func__, ret);
}

void mixer_supply_voltage_3_3v_test(void)
{
    int ret = 0;

    print_func_start_format(__func__);

    ret = mixer_3v3_rfic_2v5_fpga_1v8_power_enabled();
    if (ret < 0)
    {
        goto exit;
    }

    printf("\n");
    printf("Measure your mixer supply voltage 3.3v.\n");
    printf("Once completed, press ENTER key to turn off the mixer supply voltage 3.3v.\n");

    getchar();
    ret = mixer_3v3_rfic_2v5_fpga_1v8_power_disabled();

exit:

    print_func_end_format(__func__, ret);
}

// Temp: Need to revisit
void pa_supply_voltage_8_0v_test(void)
{
    int ret = 0;

    print_func_start_format(__func__);

    peri_gpio_t dev;

    peri_gpio_init(&dev, "dev/i2c-3", I2C_GPIO_ADDRESS);

    ret = peri_gpio_pa_enable(&dev, 1);
    if (ret < 0)
    {
        printf("Peripheral gpio pa enabled : Failed.\n");
        goto exit;
    }
    else
    {
        printf("Peripheral gpio pa enabled : Passed.\n");
    }

    printf("\n");
    printf("Measure your pa supply voltage 8v.\n");
    printf("Once completed, press ENTER key to turn off the peripheral gpio pa.\n");

    getchar();
    printf("Disabled the peripheral gpio pa.\n");
    ret = peri_gpio_pa_enable(&dev, 0);
    if (ret < 0)
    {
        printf("Peripheral gpio pa disabled : Failed.\n");
    }
    else
    {
        printf("Peripheral gpio pa disabled : Passed.\n");
    }

exit:

    peri_gpio_close(&dev);

    print_func_end_format(__func__, ret);
}

// Temp: Need to revisit
void lna_supply_voltage_8_0v_test(void)
{
    int ret = 0;

    print_func_start_format(__func__);

    peri_gpio_t dev;

    peri_gpio_init(&dev, "dev/i2c-3", I2C_GPIO_ADDRESS);

    ret = peri_gpio_lna_enable(&dev, 1);
    if (ret < 0)
    {
        printf("Peripheral gpio lna enabled : Failed.\n");
        goto exit;
    }
    else
    {
        printf("Peripheral gpio lna enabled : Passed.\n");
    }

    printf("\n");
    printf("Measure your lna supply voltage 8v.\n");
    printf("Once completed, press ENTER key to turn off the peripheral gpio lna.\n");

    getchar();
    printf("Disabled the peripheral gpio lna.\n");
    ret = peri_gpio_lna_enable(&dev, 0);
    if (ret < 0)
    {
        printf("Peripheral gpio lna disabled : Failed.\n");
    }
    else
    {
        printf("Peripheral gpio lna disabled : Passed.\n");
    }

exit:

    peri_gpio_close(&dev);

    print_func_end_format(__func__, ret);
}

static int check_rfic_idle_state()
{
    int ret = 0;

    return ret;
}

void rfic_current_idle_state_test(void)
{
    int ret = 0;

    print_func_start_format(__func__);

    ret = mixer_3v3_rfic_2v5_fpga_1v8_power_enabled();
    if (ret < 0)
    {
        goto exit;
    }

    printf("\n");
    printf("Waiting the rfic swith to IDLE state.\n");
    if (!check_rfic_idle_state())
    {
        return;
    }
    
    printf("Measure your 4 rfic current in IDLE state individually.\n");
    printf("Once completed, press ENTER key to turn off the rfic supply voltage 2.5v.\n");

    getchar();
    ret = mixer_3v3_rfic_2v5_fpga_1v8_power_disabled();

exit:

    print_func_end_format(__func__, ret);
}

static int check_rfic_tx_state()
{
    int ret = 0;

    return ret;
}

void rfic_current_tx_state_test(void)
{
    int ret = 0;

    print_func_start_format(__func__);

    ret = mixer_3v3_rfic_2v5_fpga_1v8_power_enabled();
    if (ret < 0)
    {
        goto exit;
    }

    printf("\n");
    printf("Waiting the rfic swith to TX state.\n");
    if (!check_rfic_tx_state())
    {
        return;
    }
    
    printf("Measure your 4 rfic current in TX state individually.\n");
    printf("Once completed, press ENTER key to turn off the rfic supply voltage 2.5v.\n");

    getchar();
    ret = mixer_3v3_rfic_2v5_fpga_1v8_power_disabled();

exit:

    print_func_end_format(__func__, ret);
}

static int check_rfic_rx_state()
{
    int ret = 0;

    return ret;
}

void rfic_current_rx_state_test(void)
{
    int ret = 0;

    print_func_start_format(__func__);

    ret = mixer_3v3_rfic_2v5_fpga_1v8_power_enabled();
    if (ret < 0)
    {
        goto exit;
    }

    printf("\n");
    printf("Waiting the rfic swith to TX state.\n");
    if (!check_rfic_rx_state())
    {
        return;
    }
    
    printf("Measure your 4 rfic current in TX state individually.\n");
    printf("Once completed, press ENTER key to turn off the rfic supply voltage 2.5v.\n");

    getchar();
    ret = mixer_3v3_rfic_2v5_fpga_1v8_power_disabled();

exit:

    print_func_end_format(__func__, ret);
}

void mixer_current_test(void)
{
    int ret = 0;

    print_func_start_format(__func__);

    ret = mixer_3v3_rfic_2v5_fpga_1v8_power_enabled();
    if (ret < 0)
    {
        goto exit;
    }

    printf("\n");
    printf("Measure your mixer current.\n");
    printf("Once completed, press ENTER key to turn off the mixer supply voltage 3.3v.\n");

    getchar();
    ret = mixer_3v3_rfic_2v5_fpga_1v8_power_disabled();

exit:

    print_func_end_format(__func__, ret);
}

void rfic_tx_power_test(void)
{
    int ret = 0;

    print_func_start_format(__func__);

    ret = mixer_3v3_rfic_2v5_fpga_1v8_power_enabled();
    if (ret < 0)
    {
        goto exit;
    }

    printf("\n");
    printf("Waiting the rfic swith to TX state.\n");
    if (!check_rfic_tx_state())
    {
        return;
    }
    
    //Temp: To do setup the TX power for 4 RFIC
    
    printf("Measure your 4 rfic TX power in TX state individually.\n");
    printf("Once completed, press ENTER key to turn off the rfic supply voltage 2.5v.\n");

    getchar();
    ret = mixer_3v3_rfic_2v5_fpga_1v8_power_disabled();

exit:

    print_func_end_format(__func__, ret);
}

//Temp: Need to revisit
void rfic_rx_sensitivity_test(void)
{
    int ret = 0;

    print_func_start_format(__func__);

    ret = mixer_3v3_rfic_2v5_fpga_1v8_power_enabled();
    if (ret < 0)
    {
        goto exit;
    }

    printf("\n");
    printf("Waiting the rfic swith to RX state.\n");
    if (!check_rfic_rx_state())
    {
        return;
    }
    
    printf("Measure your 4 rfic RX sensitivity individually.\n");

    //Temp: To do ???

    printf("Once completed, press ENTER key to turn off the rfic supply voltage 2.5v.\n");

    getchar();
    ret = mixer_3v3_rfic_2v5_fpga_1v8_power_disabled();

exit:

    print_func_end_format(__func__, ret);
}

void freq_on_udc_while_tx_state_test(void)
{
    int ret = 0;

    print_func_start_format(__func__);

    ret = mixer_3v3_rfic_2v5_fpga_1v8_power_enabled();
    if (ret < 0)
    {
        goto exit;
    }

    printf("\n");
    printf("Waiting the rfic swith to TX state.\n");
    if (!check_rfic_tx_state())
    {
        return;
    }
    
    printf("Measure your frequency on UDC while in TX state.\n");
    
    //Temp: To do ????

    printf("Once completed, press ENTER key to turn off the rfic supply voltage 2.5v.\n");

    getchar();
    ret = mixer_3v3_rfic_2v5_fpga_1v8_power_disabled();

exit:

    print_func_end_format(__func__, ret);
}

void freq_on_udc_while_rx_state_test(void)
{
    int ret = 0;

    print_func_start_format(__func__);

    ret = mixer_3v3_rfic_2v5_fpga_1v8_power_enabled();
    if (ret < 0)
    {
        goto exit;
    }

    printf("\n");
    printf("Waiting the rfic swith to RX state.\n");
    if (!check_rfic_rx_state())
    {
        return;
    }
    
    printf("Measure your frequency on UDS while in RX state individually.\n");

    //Temp: To do ???

    printf("Once completed, press ENTER key to turn off the rfic supply voltage 2.5v.\n");

    getchar();
    ret = mixer_3v3_rfic_2v5_fpga_1v8_power_disabled();

exit:

    print_func_end_format(__func__, ret);
}

void adc_while_tx_state_test(void)
{
    int ret = 0;

    print_func_start_format(__func__);

    ret = mixer_3v3_rfic_2v5_fpga_1v8_power_enabled();
    if (ret < 0)
    {
        goto exit;
    }

    printf("\n");
    printf("Waiting the rfic swith to TX state.\n");
    if (!check_rfic_tx_state())
    {
        return;
    }
    
    printf("Measure your ADC while in TX state.\n");
    
    //Temp: To do ????

    printf("Once completed, press ENTER key to turn off the rfic supply voltage 2.5v.\n");

    getchar();
    ret = mixer_3v3_rfic_2v5_fpga_1v8_power_disabled();

exit:

    print_func_end_format(__func__, ret);
}

void temp_while_tx_state_test(void)
{
    int ret = 0;

    print_func_start_format(__func__);

    ret = mixer_3v3_rfic_2v5_fpga_1v8_power_enabled();
    if (ret < 0)
    {
        goto exit;
    }

    printf("\n");
    printf("Waiting the rfic swith to TX state.\n");
    if (!check_rfic_tx_state())
    {
        return;
    }
    
    printf("Measure your temperature while in TX state.\n");
    
    //Temp: To do ????

    printf("Once completed, press ENTER key to turn off the rfic supply voltage 2.5v.\n");

    getchar();
    ret = mixer_3v3_rfic_2v5_fpga_1v8_power_disabled();

exit:

    print_func_end_format(__func__, ret);
}

static uint16_t rffc507x_regs_default[RFFC507X_NUM_REGS] =
{
	0xbefa,   /* 00 */
	0x4064,   /* 01 */
	0x9055,   /* 02 */
	0x2d02,   /* 03 */
	0xacbf,   /* 04 */
	0xacbf,   /* 05 */
	0x0028,   /* 06 */
	0x0028,   /* 07 */
	0xff00,   /* 08 */
	0x8220,   /* 09 */
	0x0202,   /* 0A */
	0x4800,   /* 0B */
	0x1a94,   /* 0C */
	0xd89d,   /* 0D */
	0x8900,   /* 0E */
	0x1e84,   /* 0F */
	0x89d8,   /* 10 */
	0x9d00,   /* 11 */
	0x2a20,   /* 12 */
	0x0000,   /* 13 */
	0x0000,   /* 14 */
	0x0000,   /* 15h / 21d <== SDI_CTRL - SDI Control        */
	0x0000,   /* 16h / 22d <== GPO - General Purpose Outputs */
	0x4900,   /* 17 */
	0x0281,   /* 18 */
	0xf00f,   /* 19 */
	0x0000,   /* 1A */
	0x0000,   /* 1B */
	0xc840,   /* 1C */
	0x1000,   /* 1D */
	0x0005,   /* 1E */
};

void i2c_test(void)
{
    int ret = 0;

    print_func_start_format(__func__);

    // I2C write and read RFFC Mixer
    rffc507x_st rffc_dev;

    printf("Initializing RFFC507x driver\n");
	memcpy(&rffc_dev.rffc507x_regs, rffc507x_regs_default, sizeof(rffc_dev.rffc507x_regs));
	rffc_dev.rffc507x_regs_dirty = 0x7fffffff;

	printf("Setting up device I2C\n");

	int fd = i2c_open("/dev/i2c-2");
    if (fd < 0)
    {
        printf("unable to open /dev/i2c-2\n");
        return;
    }

	rffc_dev.fd = fd;
	rffc_dev.slave_addr = I2C_SPI_RFFC_ADDRESS;
	rffc_dev.slave_select = rffc507x_1_ss;

	if (rffc_dev.slave_select == rffc507x_1_ss)
    {
		rffc_dev.slave_select_read_req_cmd = rffc507x_1_ss_read_req;
		rffc_dev.slave_select_read_cmd = rffc507x_1_ss_read;
		rffc_dev.slave_select_write_cmd = rffc507x_1_ss_write;
    }
	else if (rffc_dev.slave_select == rffc507x_2_ss)
	{
		rffc_dev.slave_select_read_req_cmd = rffc507x_2_ss_read_req;
		rffc_dev.slave_select_read_cmd = rffc507x_2_ss_read;
		rffc_dev.slave_select_write_cmd = rffc507x_2_ss_write;
	}

    rffc507x_device_id_st did = {0};
	rffc507x_device_status_st stat = {0};
	rffc507x_readback_status(&rffc_dev, &did, &stat);
	rffc507x_print_dev_id(&did);

    // I2C to enable all GPIO
    peri_gpio_t gpio;
    memset(&gpio, 0, sizeof(peri_gpio_t));

    peri_gpio_init(&gpio, "/dev/i2c-2", I2C_GPIO_ADDRESS);
    peri_gpio_cntl1(&gpio, 1);
    peri_gpio_cntl2(&gpio, 1);
    peri_gpio_cntl3(&gpio, 1);
    peri_gpio_cntl4(&gpio, 1);
    peri_gpio_cntl5(&gpio, 1);
    peri_gpio_cntl6(&gpio, 1);
    peri_gpio_switch_v1_1(&gpio, 1);
    peri_gpio_switch_v2_1(&gpio, 1);
    peri_gpio_switch_v1_2(&gpio, 1);
    peri_gpio_switch_v2_2(&gpio, 1);
    
    peri_gpio_close(&gpio);
    i2c_close(fd);

    print_func_end_format(__func__, ret);
}

void spi_test(void)
{
    int ret = 0;
    struct at86rf215 rfic_dev;
    spi_t spi;
    uint8_t val;

    print_func_start_format(__func__);

    // Peform Reset
    gpio_rst_rf_b_enabled();
    usleep(1000000);
    gpio_rst_rf_b_disabled();

    // SPI write and read RFIC

    ret = spi_init(&spi, "/dev/spidev1.0", 0, 0, 2500000);
    if (ret)
    {
        printf("Unabled to init SPI.\n");
        return;
    }

    int fd = i2c_open("/dev/i2c-2");
    if (fd < 0)
    {
        printf("Unable to open /dev/i2c-2\n");
        return;
    }

    ret = evt_i2c_write(fd, 0x26, 0x01);
    if (ret < 0)
    {
        printf("Unable to perform I2C Write.\n");
        return;
    }

    uint8_t ret_value = evt_i2c_read(fd, 0x26);
    printf("ret_value = %d\n", ret_value);
    if (ret_value < 0)
    {
        printf("Unable to perform I2C Read.\n");
        return;
    }

/*
    // rffc SPI write
    uint8_t chunk_tx_w[3] = {0};
    uint8_t address_w = 0x1D;
    uint16_t value_w = 0x0000;

    chunk_tx_w[0] = address_w & 0x7F;
    chunk_tx_w[1] = (value_w >> 8) & 0xFF;
    chunk_tx_w[2] = value_w & 0xFF;

    ret = spi_write(&spi, chunk_tx_w, 3);
    if (ret < 0)
    {
        printf("Unable to perform SPI write to RFFC\n");
        return;
    }

    // rffc SPI read
    uint8_t chunk_tx[3] = {0};
    uint8_t chunk_rx[3] = {0};
    uint8_t address_r = 0x1D;
    uint16_t value_r = 0x0000;

    chunk_tx[0] = (address_r & 0x7F) | 0x80;

    ret = spi_exchange(&spi, chunk_rx, chunk_tx, 3);
    if (ret < 0)
    {
        printf("Unable to perform SPI read to RFFC\n");
        return;
    }

    value_r = chunk_rx[1];
    value_r = (value_r << 8) | chunk_rx[2];
    
    printf("Value = %d\n", value_r);
*/


    memset(&rfic_dev, 0, sizeof(struct at86rf215));
    memcpy(&(rfic_dev.spidev), &spi, sizeof(spi));

    //evt_at86rf215_reg_write_8(&rfic_dev, AT86RF215_CMD_RF_RESET, REG_RF_RST);
    evt_at86rf215_reg_write_8(&rfic_dev, 0x3F, REG_RF09_IRQM);
	usleep(100000);

    ret = evt_at86rf215_reg_read_8(&rfic_dev, &val, REG_RF09_IRQM);
    if (ret)
    {
        printf("Unable to read RFIC model.\n");
        return;
    }

    printf("Value = %d\n", val);

    ret = 0;
    val = 0;

    ret = evt_at86rf215_reg_read_8(&rfic_dev, &val, REG_RF_PN);
    if (ret)
    {
        printf("Unable to read RFIC model.\n");
        return;
    }

    printf("Value = %d\n", val);
    switch (val) {
		case AT86RF215:
            printf("RFIC is AT86RF215.\n");
			break;
		case AT86RF215IQ:
            printf("RFIC is AT86RF215IQ.\n");
			break;
		case AT86RF215M:
			printf("RFIC is AT86RF215M.\n");
			break;
		default:
            printf("Unable to find the RFIC model.\n");
			return;
	}

    i2c_close(fd);
    spi_free(&spi);

    print_func_end_format(__func__, ret);
}

static void showVersion(DWORD locationId)
{
    FT_STATUS            ftStatus;
    FT_HANDLE            ftHandle = (FT_HANDLE)NULL;
    FT4222_STATUS        ft4222Status;
    FT4222_Version       ft4222Version;

    ftStatus = FT_OpenEx((PVOID)(uintptr_t)locationId, 
                         FT_OPEN_BY_LOCATION, 
                         &ftHandle);
    if (ftStatus != FT_OK)
    {
        printf("FT_OpenEx failed (error %d)\n", 
               (int)ftStatus);
        return;
    }

    // Get version of library and chip.    
    ft4222Status = FT4222_GetVersion(ftHandle,
                                     &ft4222Version);
    if (FT4222_OK != ft4222Status)
    {
        printf("FT4222_GetVersion failed (error %d)\n",
               (int)ft4222Status);
    }
    else
    {
        printf("  Chip version: %08X, LibFT4222 version: %08X\n",
               (unsigned int)ft4222Version.chipVersion,
               (unsigned int)ft4222Version.dllVersion);
    }

    (void)FT_Close(ftHandle);
}

void usb_test(void)
{
    print_func_start_format(__func__);

    int ret = 0;
    FT_STATUS                 ftStatus;
    FT_DEVICE_LIST_INFO_NODE *devInfo = NULL;
    DWORD                     numDevs = 0;
    int                       i;
    int                       retCode = 0;
    int                       found4222 = 0;
    
    ftStatus = FT_CreateDeviceInfoList(&numDevs);
    if (ftStatus != FT_OK) 
    {
        printf("FT_CreateDeviceInfoList failed (error code %d)\n", 
               (int)ftStatus);
        retCode = -10;
        goto exit;
    }
    
    if (numDevs == 0)
    {
        printf("No devices connected.\n");
        retCode = -20;
        goto exit;
    }

    /* Allocate storage */
    devInfo = calloc((size_t)numDevs,
                     sizeof(FT_DEVICE_LIST_INFO_NODE));
    if (devInfo == NULL)
    {
        printf("Allocation failure.\n");
        retCode = -30;
        goto exit;
    }
    
    /* Populate the list of info nodes */
    ftStatus = FT_GetDeviceInfoList(devInfo, &numDevs);
    if (ftStatus != FT_OK)
    {
        printf("FT_GetDeviceInfoList failed (error code %d)\n",
               (int)ftStatus);
        retCode = -40;
        goto exit;
    }

    for (i = 0; i < (int)numDevs; i++) 
    {
        if (devInfo[i].Type == FT_DEVICE_4222H_0  ||
            devInfo[i].Type == FT_DEVICE_4222H_1_2)
        {
            // In mode 0, the FT4222H presents two interfaces: A and B.
            // In modes 1 and 2, it presents four interfaces: A, B, C and D.

            size_t descLen = strlen(devInfo[i].Description);
            
            if ('A' == devInfo[i].Description[descLen - 1])
            {
                // Interface A may be configured as an I2C master.
                printf("\nDevice %d: '%s'\n",
                       i,
                       devInfo[i].Description);
                showVersion(devInfo[i].LocId);
            }
            else
            {
                // Interface B, C or D.
                // No need to repeat version info of same chip.
            }
            
            found4222++;
        }
         
        if (devInfo[i].Type == FT_DEVICE_4222H_3)
        {
            // In mode 3, the FT4222H presents a single interface.  
            printf("\nDevice %d: '%s'\n",
                   i,
                   devInfo[i].Description);
            showVersion(devInfo[i].LocId);

            found4222++;
        }
    }

    if (found4222 == 0)
    {
        printf("No FT4222H detected.\n");
        ret = -1;
    }

exit:
    free(devInfo);

    print_func_end_format(__func__, ret);
}

void rfic_interrupt_test(void)
{
    int ret = 0;

    print_func_start_format(__func__);

    print_func_end_format(__func__, ret);
}

void evt_test(void)
{
    int ret = 0;
    struct at86rf215 rfic_dev, rfic_dev2;
    spi_t spi, spi2;
    uint8_t val;
    uint8_t ret_value = 0;
    uint8_t i2c_to_gpio_address_1 = 0x24;
    uint8_t i2c_to_gpio_address_2 = 0x25;
    uint8_t i2c_to_spi_chip_select = 0x26; 

    print_func_start_format(__func__);

    // Peform Reset
    gpio_rst_rf_b_enabled();
    usleep(1000000);
    gpio_rst_rf_b_disabled();

    // SPI Init
    ret = spi_init(&spi, "/dev/spidev1.0", 0, 0, 2500000);
    if (ret)
    {
        printf("Unabled to init SPI.\n");
        return;
    }

    ret = spi_init(&spi2, "/dev/spidev2.0", 0, 0, 2500000);
    if (ret)
    {
        printf("Unabled to init SPI.\n");
        return;
    }

    // I2C Init
    int fd = i2c_open("/dev/i2c-2");
    if (fd < 0)
    {
        printf("Unable to open /dev/i2c-2\n");
        return;
    }

    ret = 0;

    // I2C-to-GPIO Testing
    for (int i=0; i < 8; i++)
    {
        switch(i)
        {
            case 0:
                printf("Testing 8V0_LNA_EN pin.\n");
                break;
            case 1:
                printf("Testing 8V0_PA_EN pin.\n");
                break;
            case 2:
                printf("Testing CNTL_1 pin.\n");
                break;
            case 3:
                printf("Testing CNTL_2 pin.\n");
                break;
            case 4:
                printf("Testing CNTL_3 pin.\n");
                break;
            case 5:
                printf("Testing CNTL_4 pin.\n");
                break;
            case 6:
                printf("Testing CNTL_5 pin.\n");
                break;
            case 7:
                printf("Testing CNTL_6 pin.\n");
                break;
            default:
                printf("Unable to enable the pin.\n");
                ret = -1;
                goto exit;
                break;
        }

        ret = evt_i2c_write(fd, i2c_to_gpio_address_1, 1<<i);
        if (ret < 0)
        {
            printf("Unable to perform I2C Write.\n");
            goto exit;
        }

        ret_value = evt_i2c_read(fd, i2c_to_gpio_address_1);
        printf("I2C-to-GPIO read back register value = 0x%X\n", ret_value);
        if (ret_value < 0)
        {
            printf("Unable to perform I2C Read.\n");
            ret = -1;
            goto exit;
        }

        printf("\n");
        printf("Please verify the pin and PRESS \"ENTER\" to continue.\n");
        getchar();
    }

    ret = 0;
    ret_value = 0;

    for (int i=0; i < 6; i++)
    {
        switch(i)
        {
            case 0:
                printf("Testing Mixer1 MODE 1 pin.\n");
                break;
            case 1:
                printf("Testing Mixer2 MODE 2 pin.\n");
                break;
            case 2:
                printf("Testing V1_1 pin.\n");
                break;
            case 3:
                printf("Testing V2_1 pin.\n");
                break;
            case 4:
                printf("Testing V1_2 pin.\n");
                break;
            case 5:
                printf("Testing V2_2 pin.\n");
                break;
            default:
                printf("Unable to enable the pin.\n");
                ret = -1;
                goto exit;
                break;
        }

        ret = evt_i2c_write(fd, i2c_to_gpio_address_2, 1<<i);
        if (ret < 0)
        {
            printf("Unable to perform I2C Write.\n");
            goto exit;
        }

        ret_value = evt_i2c_read(fd, i2c_to_gpio_address_2);
        printf("I2C-to-GPIO read back register value = 0x%X\n", ret_value);
        if (ret_value < 0)
        {
            printf("Unable to perform I2C Read.\n");
            ret = -1;
            goto exit;
        }

        printf("\n");
        printf("Please verify the pin and PRESS \"ENTER\" to continue.\n");
        getchar();
    }

    ret = 0;
    ret_value = 0;

    // Test all RFIC using 1 SPI channel
    for (int i=0; i<4; i++)
    {
        switch(i)
        {
            case 0:
                printf("Testing RFIC 1.\n");
                printf("---------------\n");
                break;
            case 1:
                printf("Testing RFIC 2.\n");
                printf("---------------\n");
                break;
            case 2:
                printf("Testing RFIC 3.\n");
                printf("---------------\n");
                break;
            case 3:
                printf("Testing RFIC 4.\n");
                printf("---------------\n");
                break;
            default:
                printf("Unable to test RFIC.\n");
                ret = -1;
                goto exit;
                break;
        }

        ret = evt_i2c_write(fd, i2c_to_spi_chip_select, 1<<i);
        if (ret < 0)
        {
            printf("Unable to perform I2C Write.\n");
            goto exit;
        }

        ret_value = evt_i2c_read(fd, i2c_to_spi_chip_select);
        printf("I2C-to-GPIO read back register value = 0x%X\n", ret_value);
        if (ret_value < 0)
        {
            printf("Unable to perform I2C Read.\n");
            ret = -1;
            goto exit;
        }

        memset(&rfic_dev, 0, sizeof(struct at86rf215));
        memcpy(&(rfic_dev.spidev), &spi, sizeof(spi));

        //evt_at86rf215_reg_write_8(&rfic_dev, AT86RF215_CMD_RF_RESET, REG_RF_RST);
        usleep(100000);

        printf("Writing the 0x3F value into REG_RF09_IRQM.\n");
        evt_at86rf215_reg_write_8(&rfic_dev, 0x3F, REG_RF09_IRQM);
        usleep(100000);

        ret = evt_at86rf215_reg_read_8(&rfic_dev, &val, REG_RF09_IRQM);
        if (ret)
        {
            printf("Unable to read RFIC model.\n");
            return;
        }

        printf("REG_RF09_IRQM: Read back value = 0x%X\n", val);

        ret = 0;
        val = 0;

        ret = evt_at86rf215_reg_read_8(&rfic_dev, &val, REG_RF_PN);
        if (ret)
        {
            printf("Unable to read RFIC model.\n");
            goto exit;
        }

        printf("REG_RF_PN: Read back value = 0x%X\n", val);
        switch (val) {
            case AT86RF215:
                printf("RFIC is AT86RF215.\n");
                break;
            case AT86RF215IQ:
                printf("RFIC is AT86RF215IQ.\n");
                break;
            case AT86RF215M:
                printf("RFIC is AT86RF215M.\n");
                break;
            default:
                printf("Unable to find the RFIC model.\n");
                ret = -1;
                goto exit;
        }

        printf("\n");
    }

    ret = 0;
    ret_value = 0;

    // Test RFIC 1 using SPI channel 1, RFIC 4 using SPI channel 2
    printf("Testing RFIC 1 and RFIC 4 concurrently.\n");
    printf("---------------\n");

    ret = evt_i2c_write(fd, i2c_to_spi_chip_select, 0x81);
    if (ret < 0)
    {
        printf("Unable to perform I2C Write.\n");
        goto exit;
    }

    ret_value = evt_i2c_read(fd, i2c_to_spi_chip_select);
    printf("I2C-to-GPIO read back register value = 0x%X\n", ret_value);
    if (ret_value < 0)
    {
        printf("Unable to perform I2C Read.\n");
        ret = -1;
        goto exit;
    }

    memset(&rfic_dev, 0, sizeof(struct at86rf215));
    memcpy(&(rfic_dev.spidev), &spi, sizeof(spi));

    memset(&rfic_dev2, 0, sizeof(struct at86rf215));
    memcpy(&(rfic_dev2.spidev), &spi2, sizeof(spi2));


    printf("RFIC 1 --> Writing the 0x3F value into REG_RF09_IRQM.\n");
    evt_at86rf215_reg_write_8(&rfic_dev, 0x3F, REG_RF09_IRQM);
    printf("RFIC 4 --> Writing the 0x3F value into REG_RF09_IRQM.\n");
    evt_at86rf215_reg_write_8(&rfic_dev2, 0x3F, REG_RF09_IRQM);
    usleep(100000);

    ret = evt_at86rf215_reg_read_8(&rfic_dev, &val, REG_RF09_IRQM);
    if (ret)
    {
        printf("Unable to read RFIC model.\n");
        return;
    }

    printf("RFIC 1 --> REG_RF09_IRQM: Read back value = 0x%X\n", val);

    ret = 0;
    val = 0;

    ret = evt_at86rf215_reg_read_8(&rfic_dev2, &val, REG_RF09_IRQM);
    if (ret)
    {
        printf("Unable to read RFIC model.\n");
        return;
    }

    printf("RFIC 4 --> REG_RF09_IRQM: Read back value = 0x%X\n", val);

    ret = 0;
    val = 0;

    ret = evt_at86rf215_reg_read_8(&rfic_dev, &val, REG_RF_PN);
    if (ret)
    {
        printf("Unable to read RFIC model.\n");
        goto exit;
    }

    printf("REG_RF_PN: Read back value = %d\n", val);
    switch (val) {
        case AT86RF215:
            printf("RFIC is AT86RF215.\n");
            break;
        case AT86RF215IQ:
            printf("RFIC is AT86RF215IQ.\n");
            break;
        case AT86RF215M:
            printf("RFIC is AT86RF215M.\n");
            break;
        default:
            printf("Unable to find the RFIC model.\n");
            ret = -1;
            goto exit;
    }

    ret = 0;
    val = 0;

    ret = evt_at86rf215_reg_read_8(&rfic_dev2, &val, REG_RF_PN);
    if (ret)
    {
        printf("Unable to read RFIC model.\n");
        goto exit;
    }

    printf("REG_RF_PN: Read back value = %d\n", val);
    switch (val) {
        case AT86RF215:
            printf("RFIC is AT86RF215.\n");
            break;
        case AT86RF215IQ:
            printf("RFIC is AT86RF215IQ.\n");
            break;
        case AT86RF215M:
            printf("RFIC is AT86RF215M.\n");
            break;
        default:
            printf("Unable to find the RFIC model.\n");
            ret = -1;
            goto exit;
    }

    printf("\n");

    ret = 0;
    val = 0;

    // rffc SPI write
    for (int i = 4; i < 6; i++)
    {
        switch(i)
        {
            case 4:
                printf("Testing RFFC Mixer 1.\n");
                printf("---------------\n");
                break;
            case 5:
                printf("Testing RFFC Mixer 2.\n");
                printf("---------------\n");
                break;
            default:
                printf("Unable to test RFFC Mixer.\n");
                ret = -1;
                goto exit;
                break;
        }

        ret = evt_i2c_write(fd, i2c_to_spi_chip_select, 1<<i);
        if (ret < 0)
        {
            printf("Unable to perform I2C Write.\n");
            goto exit;
        }

        ret_value = evt_i2c_read(fd, i2c_to_spi_chip_select);
        printf("I2C-to-GPIO read back register value = %d\n", ret_value);
        if (ret_value < 0)
        {
            printf("Unable to perform I2C Read.\n");
            ret = -1;
            goto exit;
        }

        uint8_t chunk_tx_w[3] = {0};
        uint8_t address_w = 0x00;
        uint16_t value_w = 0xFFFF;

        chunk_tx_w[0] = address_w & 0x7F;
        chunk_tx_w[1] = (value_w >> 8) & 0xFF;
        chunk_tx_w[2] = value_w & 0xFF;

        ret = spi_write(&spi, chunk_tx_w, 3);
        if (ret < 0)
        {
            printf("Unable to perform SPI write to RFFC\n");
            ret = -1;
            goto exit;
        }

        printf("\n");
    }

exit:
    i2c_close(fd);
    spi_free(&spi);

    print_func_end_format(__func__, ret);
}

static void print_help(void)
{
    printf("\n\n");
    printf("USAGE: ./test_app_tr01 [argument]\n");
    printf("Example: ./test_app_tr01 10\n");
    printf("\n");
    printf("Argument :\n");
    printf("\n");

    for (int i=0; i < sizeof(functionList) / sizeof(functionList[0]); i++)
    {
        printf("%d\t - %s\n", (i+1), functionList[i].name);
    }
    printf("\n");
}

int main (int argc, char* argv[])
{
    if (argc > 2)
    {
        printf("Invalid Argument. Please type --help to look for command usage.\n");
        return -1;
    }

    if (!(strcmp(argv[1], "--help")))
    {
        print_help();
        return 0;
    }

    int func_num = atoi(argv[1]);
    if (func_num > 0 && func_num < 23)
    {
        functionList[func_num - 1].test();
        return 0;
    }

    printf("Invalid test name. Please type --help to look for available test case.\n");
    return 0;
}
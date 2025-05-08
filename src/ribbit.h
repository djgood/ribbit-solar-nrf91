/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/byteorder.h>


#define LED_YELLOW_NODE 	DT_NODELABEL(led_green)
#define SBUS_LS_NODE 		DT_NODELABEL(sbus_ls)
#define CHARGE_EN_NODE 		DT_NODELABEL(charge_enable)

#define BQ25798_NODE		DT_NODELABEL(bq25798)
#define SCD30_NODE			DT_NODELABEL(scd30)
#define DPS310_NODE			DT_NODELABEL(dps310)

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED_YELLOW_NODE, gpios);
static const struct gpio_dt_spec sbus_ls = GPIO_DT_SPEC_GET(SBUS_LS_NODE, gpios);
static const struct gpio_dt_spec charge_enable = GPIO_DT_SPEC_GET(CHARGE_EN_NODE, gpios);

static const struct i2c_dt_spec pmic_i2c = I2C_DT_SPEC_GET(BQ25798_NODE);
static const struct i2c_dt_spec sbus_scd30_i2c = I2C_DT_SPEC_GET(SCD30_NODE);
static const struct i2c_dt_spec sbus_dps310_i2c = I2C_DT_SPEC_GET(DPS310_NODE);

int turn_on_sbus(void) {
	int ret;

	// Set up SBUS power
	if (!gpio_is_ready_dt(&sbus_ls)) {
		return -1;
	}

	ret = gpio_pin_configure_dt(&sbus_ls, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return -1;
	}

	ret = gpio_pin_set_dt(&sbus_ls, 1);
	if (ret < 0) {
		return -1;
	}

	LOG_INF("SBUS enabled");
	return ret;
}

int test_pmic_i2c(void)
{
	// Use low level I2C APIs to do a basic test of each device on the power bus
	int ret;
	if (!device_is_ready(pmic_i2c.bus)) {
		LOG_INF("I2C bus %s is not ready!\n\r",pmic_i2c.bus->name);
		return -1;
	}

	uint8_t data;
	uint8_t addr = 0x48;
	ret = i2c_reg_read_byte_dt(&pmic_i2c, addr, &data);
	if(ret != 0){
		LOG_INF("Failed to read byte from I2C device address %x at Reg. %x \n\r", pmic_i2c.addr, addr);
	}

	LOG_INF("Read BQ25798 I2C byte @ %x: %x", addr, data);
	if (data != 0x19) {
		LOG_INF("BQ25798 I2C failed");
	} else {
		LOG_INF("BQ25798 I2C passed");
	}

	return ret;
}

int test_sbus_i2c(void)
{
	// Use low level I2C APIs to do a basic test of each device on the sensor bus
	int ret;

	// -- SCD30 --

	if (!device_is_ready(sbus_scd30_i2c.bus)) {
		LOG_INF("I2C bus %s is not ready!\n\r",sbus_scd30_i2c.bus->name);
		return -1;
	}

	uint8_t data[3] = {0};
	uint8_t cmd[] = {0xD1, 0x00};
	ret = i2c_write_dt(&sbus_scd30_i2c, cmd, sizeof(cmd));
	if(ret != 0){
		LOG_INF("Failed to read bytes from I2C device address %x at Reg. %x %x", sbus_scd30_i2c.addr, cmd[0], cmd[1]);
	}

	// required, see scd30 interface datasheet
	k_sleep(K_MSEC(3));

	ret = i2c_read_dt(&sbus_scd30_i2c, data, sizeof(data));

	uint8_t expected[3] = {0x03, 0x42, 0};
	expected[2] = crc8(expected, 2, 0x31, 0xFF, false);

	bool scd30_pass = true;
	LOG_INF("Read SCD30 I2C @ d100: ");
	for (int i = 0; i < sizeof(expected); i++) {
		LOG_INF("%x", data[i]);
		if (data[i] != expected[i]) {
			scd30_pass = false;
		} 
	}

	if (scd30_pass) {
		LOG_INF("\nSCD30 I2C passed");
	} else {
		LOG_INF("\nSCD30 I2C failed");
	}

	// -- DPS310 --

	if (!device_is_ready(sbus_dps310_i2c.bus)) {
		LOG_INF("I2C bus %s is not ready!\n\r",sbus_dps310_i2c.bus->name);
		return 0;
	}

	uint8_t dps_data;
	uint8_t addr = 0x0D;
	ret = i2c_reg_read_byte_dt(&sbus_dps310_i2c, addr, &dps_data);
	if(ret != 0){
		LOG_INF("Failed to read byte from I2C device address %x at Reg. %x \n\r", sbus_dps310_i2c.addr, addr);
	}

	LOG_INF("Read DPS310 I2C byte @ %x: %x", addr, dps_data);
	if (dps_data != 0x10) {
		LOG_INF("DPS310 I2C failed");
	} else {
		LOG_INF("DPS310 I2C passed");
	}
	
	return ret;
}

int set_co2_low_sample_rate()
{
	// first read what it is
	uint8_t data[3] = {0};
	uint8_t cmd[] = {0x46, 0x00};
	int ret = i2c_write_dt(&sbus_scd30_i2c, cmd, sizeof(cmd));
	if(ret != 0){
		LOG_ERR("Failed to write bytes to I2C device address %x at Reg. %x %x", sbus_scd30_i2c.addr, cmd[0], cmd[1]);
		return -1;
	}

	// required, see scd30 interface datasheet
	k_sleep(K_MSEC(3));

	ret = i2c_read_dt(&sbus_scd30_i2c, data, sizeof(data));
	if (ret != 0) {
		LOG_ERR("Failed to read bytes from I2C device address %x", sbus_scd30_i2c.addr);
		return -1;
	}

	if (data[0] != 0x00 || data[1] != 0x2D) {
		LOG_INF("Setting CO2 sample rate to 45 seconds");
		uint8_t int_cmd[] = {0x46, 0x00, 0x00, 0x2D, 0x00};
		int_cmd[4] = crc8(int_cmd + 2, 2, 0x31, 0xFF, false);
		int ret = i2c_write_dt(&sbus_scd30_i2c, int_cmd, sizeof(int_cmd));
		if(ret != 0){
			LOG_ERR("Failed to write bytes to I2C device address %x at Reg. %x %x", sbus_scd30_i2c.addr, int_cmd[0], int_cmd[1]);
			return -1;
		}	

		// required, see scd30 interface datasheet
		k_sleep(K_MSEC(3));
	}

	return 0;
}

int read_co2(float * co2)
{
	uint8_t rdy_data[3] = {0};
	uint8_t rdy_cmd[] = {0x02, 0x02};
	int ret = i2c_write_dt(&sbus_scd30_i2c, rdy_cmd, sizeof(rdy_cmd));
	if(ret != 0){
		LOG_ERR("Failed to write bytes to I2C device address %x at Reg. %x %x", sbus_scd30_i2c.addr, rdy_cmd[0], rdy_cmd[1]);
		return -1;
	}

	// required, see scd30 interface datasheet
	k_sleep(K_MSEC(3));

	ret = i2c_read_dt(&sbus_scd30_i2c, rdy_data, sizeof(rdy_data));
	if (ret != 0) {
		LOG_ERR("Failed to read bytes to I2C device address %x", sbus_scd30_i2c.addr);
		return -1;
	}

	if (rdy_data[1] != 0x01) {
		LOG_WRN("SCD30 not ready yet");
		return -2;
	}

	uint8_t data[6] = {0};
	uint8_t cmd[] = {0x03, 0x00};
	ret = i2c_write_dt(&sbus_scd30_i2c, cmd, sizeof(cmd));
	if(ret != 0){
		LOG_ERR("Failed to write bytes to I2C device address %x at Reg. %x %x", sbus_scd30_i2c.addr, cmd[0], cmd[1]);
		return -1;
	}

	// required, see scd30 interface datasheet
	k_sleep(K_MSEC(3));

	ret = i2c_read_dt(&sbus_scd30_i2c, data, sizeof(data));
	if (ret != 0) {
		LOG_ERR("Failed to read bytes to I2C device address %x", sbus_scd30_i2c.addr);
		return -1;
	}

	// if (data[3] != crc8(data, 2, 0x31, 0xFF, false)) {
	// 	LOG_ERR("CRC FAILED!");
	// }

	// float conversion
	uint8_t raw_bytes[] = {data[0], data[1], data[3], data[4]};
	uint32_t raw = sys_get_be32(raw_bytes);
	if (raw == 0) {
		LOG_WRN("SCD30 returned 0, unlikely result skipped");
		return -2;
	}
    memcpy(co2, &raw, sizeof(float));
	return 0;
}

int infinite_blink(void)
{
	int ret;
	if (!gpio_is_ready_dt(&led)) {
		return -1;
	}

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return -1;
	}

	while (1) {
		ret = gpio_pin_toggle_dt(&led);
		if (ret < 0) {
			return -1;
		}

		k_sleep(K_MSEC(1000));
	}
}

int enable_charging(void)
{
	int ret;

	// Set up SBUS power
	if (!gpio_is_ready_dt(&charge_enable)) {
		return -1;
	}

	ret = gpio_pin_configure_dt(&charge_enable, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return -1;
	}

	ret = gpio_pin_set_dt(&charge_enable, 1);
	if (ret < 0) {
		return -1;
	}

	LOG_INF("nCE enabled");
	return ret;
}

#define REG_Minimum_System_Voltage      0x00
#define REG_Charge_Voltage_Limit        0x01
#define REG_Charge_Current_Limit        0x03
#define REG_Input_Voltage_Limit         0x05
#define REG_Input_Current_Limit         0x06
#define REG_Charger_Control_0           0x0F
#define REG_Precharge_Control           0x08
#define REG_Termination_Control         0x09
#define REG_Charger_Control_1           0x10
#define REG_Charger_Control_5           0x14
#define REG_MPPT_Control               	0x15
#define REG_Charger_Status_0            0x1B
#define REG_Charger_Status_1            0x1C
#define REG_Charger_Status_2            0x1D
#define REG_Charger_Status_3            0x1E
#define REG_Charger_Status_4            0x1F
#define REG_Charger_Flag_0              0x22
#define REG_ADC_Control             	0x2E
#define REG_ADC_Function_Disable_1	    0x30

static int i2c_reg_write_word_dt(const struct i2c_dt_spec *i2c, uint8_t reg_addr, uint16_t value)
{
    uint8_t bytes[3];

    bytes[0] = reg_addr;
    bytes[1] = (uint8_t)(value >> 8);
    bytes[2] = (uint8_t)(value & 0xFF);

    return i2c_write_dt(i2c, bytes, sizeof(bytes));
}

static int i2c_reg_read_word_dt(const struct i2c_dt_spec * i2c, uint8_t reg_addr, uint16_t * value)
{
    uint8_t buf[2];
    int ret = i2c_write_read_dt(i2c, &reg_addr, 1, buf, 2);
    if (ret != 0) {
        return ret;
    }

    *value = ((uint16_t)buf[0] << 8) | buf[1];
    return 0;
}

int program_charge_params(void)
{
	// reset
	if (i2c_reg_write_byte_dt(&pmic_i2c, REG_Termination_Control, 0x40)) {
		goto failed;
	}	

	k_sleep(K_MSEC(20));

	// bq.disable_watchdog()
	if (i2c_reg_write_byte_dt(&pmic_i2c, REG_Charger_Control_1, 0x80)) {
		goto failed;
	}

    // bq.config_vsysmin(2500)
	// 0 * 250 mV + 2500 mV == 2500 mV
	if (i2c_reg_write_byte_dt(&pmic_i2c, REG_Minimum_System_Voltage, 0)) {
		goto failed;
	}

	uint8_t byte_val;
	if (i2c_reg_read_byte_dt(&pmic_i2c, REG_Minimum_System_Voltage, &byte_val)) {
		goto failed;
	}
	LOG_INF("Minimum System Voltage = %d mV", (byte_val * 250) + 2500);

	uint16_t word_val;
    // bq.config_charge_voltage_limit(3550)
	if (i2c_reg_write_word_dt(&pmic_i2c, REG_Charge_Voltage_Limit, 355)) {
		goto failed;
	}
	
	if (i2c_reg_read_word_dt(&pmic_i2c, REG_Charge_Voltage_Limit, &word_val)) {
		goto failed;
	}

	LOG_INF("Charge Voltage Limit = %d mV", word_val * 10);

    // bq.config_charge_current_limit(500)
	if (i2c_reg_write_word_dt(&pmic_i2c, REG_Charge_Current_Limit, 50)) {
		goto failed;
	}
	
	if (i2c_reg_read_word_dt(&pmic_i2c, REG_Charge_Current_Limit, &word_val)) {
		goto failed;
	}

	LOG_INF("Charge Current Limit = %d mA", word_val * 10);

    // bq.config_precharge_current(40)
	if (i2c_reg_write_byte_dt(&pmic_i2c, REG_Precharge_Control, 1)) {
		goto failed;
	}
	
	if (i2c_reg_read_byte_dt(&pmic_i2c, REG_Precharge_Control, &byte_val)) {
		goto failed;
	}

	LOG_INF("Precharge Current Limit = %d mA", byte_val * 40);

    // bq.config_charge_term_current(40)
	if (i2c_reg_write_byte_dt(&pmic_i2c, REG_Termination_Control, 2)) {
		goto failed;
	}
	
	if (i2c_reg_read_byte_dt(&pmic_i2c, REG_Termination_Control, &byte_val)) {
		goto failed;
	}

	LOG_INF("Termination Current Limit = %d mA", byte_val * 40);

	// re-run poor source qualification
    // bq.recheck_vbus()
	if (i2c_reg_read_byte_dt(&pmic_i2c, REG_Charger_Control_0, &byte_val)) {
		goto failed;
	}

	// Force EN_HIZ
	// byte_val &= ~(1 << 2);

	// if (i2c_reg_write_byte_dt(&pmic_i2c, REG_Charger_Control_0, byte_val)) {
	// 	goto failed;
	// }

	// LOG_INF("Forcing re-check");

	// In theory, MPPT could be turned on whenever we detect the solar power present

	// bq.enable_mppt()
	if (i2c_reg_write_byte_dt(&pmic_i2c, REG_MPPT_Control, 0x4B)) {
		goto failed;
	}

	// LOG_INF("Enabled MPPT");

	// bq.configure_control5()
	// disable EN_EXTILIM, enable EN_IBAT
	if (i2c_reg_write_byte_dt(&pmic_i2c, REG_Charger_Control_5, 0x3c)) {
		goto failed;
	}
	
	if (i2c_reg_read_byte_dt(&pmic_i2c, REG_Input_Voltage_Limit, &byte_val)) {
		goto failed;
	}

	LOG_INF("Disable IINDPM limit from external pin");
	LOG_INF("Enable IBAT discharge sensing");

	// bq.config_vindpm(3600)
	if (i2c_reg_write_byte_dt(&pmic_i2c, REG_Input_Voltage_Limit, 36)) {
		goto failed;
	}
	
	if (i2c_reg_read_byte_dt(&pmic_i2c, REG_Input_Voltage_Limit, &byte_val)) {
		goto failed;
	}

	LOG_INF("Input Voltage Limit = %d mV", byte_val * 100);

 	// bq.config_iindpm(100)
	if (i2c_reg_write_word_dt(&pmic_i2c, REG_Input_Current_Limit, 50)) {
		goto failed;
	}
	
	if (i2c_reg_read_word_dt(&pmic_i2c, REG_Input_Current_Limit, &word_val)) {
		goto failed;
	}

	LOG_INF("Input Current Limit = %d mA", word_val * 10);

	// Enable ADC
	if (i2c_reg_write_byte_dt(&pmic_i2c, REG_ADC_Control, 0x80)) {
		LOG_INF("ERROR DURING ADC ENABLE");
		return -1;
	}

	LOG_INF("Enabled ADC continuous conversion");

	return 0;
failed:
	LOG_INF("ERROR DURING BQ25 COMMANDS");
	return 1;
}

int print_adc_values(void)
{
	// Read conversions
	char * names[9] = {"ibus", "ibat", "vbus", "vac1", "vac2", "vbat", "vsys", "ts", "tdie"};
	int16_t vals[9] = {0};
	for (uint8_t i = 0; i < 9; i++) {
		if (i2c_reg_read_word_dt(&pmic_i2c, 0x31 + (2*i), vals + i)) {
			return -1;
		}
	}

	for (int i = 0; i < 9; i++) {
		LOG_INF("%s : %d", names[i], vals[i]);
	}

	return 0;
}

int read_ibus(int16_t * ibus) {
	if (i2c_reg_read_word_dt(&pmic_i2c, 0x31, ibus)) {
		LOG_ERR("IBUS READ FAILED");
		return -1;
	}

	return 0;
}

int read_ibat(int16_t * ibat) {
	if (i2c_reg_read_word_dt(&pmic_i2c, 0x33, ibat)) {
		LOG_ERR("IBAT READ FAILED");
		return -1;
	}

	return 0;
}

int ribbit_main(void)
{
	if (turn_on_sbus()) {
		return 0;
	}

	test_pmic_i2c();
	test_sbus_i2c();

	// assumes a ~ 1500 mA lifepo4 cell
	program_charge_params();

	enable_charging();

	// Ensure we're charging from panel
	// if (i2c_reg_write_byte_dt(&pmic_i2c, 0x13, 0x80)) {
	// 	LOG_INF("ERROR DURING ADC ENABLE");
	// 	return -1;
	// }

	while (true) {
		print_adc_values();
		k_sleep(K_MSEC(1000));
	}

	return 0;
}

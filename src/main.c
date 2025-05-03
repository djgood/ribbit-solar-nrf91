/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/crc.h>

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

	printk("SBUS enabled\n");
	return ret;
}

int test_pmic_i2c(void)
{
	// Use low level I2C APIs to do a basic test of each device on the power bus
	int ret;
	if (!device_is_ready(pmic_i2c.bus)) {
		printk("I2C bus %s is not ready!\n\r",pmic_i2c.bus->name);
		return -1;
	}

	uint8_t data;
	uint8_t addr = 0x48;
	ret = i2c_reg_read_byte_dt(&pmic_i2c, addr, &data);
	if(ret != 0){
		printk("Failed to read byte from I2C device address %x at Reg. %x \n\r", pmic_i2c.addr, addr);
	}

	printk("Read BQ25798 I2C byte @ %x: %x\n", addr, data);
	if (data != 0x19) {
		printk("BQ25798 I2C failed\n");
	} else {
		printk("BQ25798 I2C passed\n");
	}

	return ret;
}

int test_sbus_i2c(void)
{
	// Use low level I2C APIs to do a basic test of each device on the sensor bus
	int ret;

	// -- SCD30 --

	if (!device_is_ready(sbus_scd30_i2c.bus)) {
		printk("I2C bus %s is not ready!\n\r",sbus_scd30_i2c.bus->name);
		return -1;
	}

	uint8_t data[3] = {0};
	uint8_t cmd[] = {0xD1, 0x00};
	ret = i2c_write_dt(&sbus_scd30_i2c, cmd, sizeof(cmd));
	if(ret != 0){
		printk("Failed to read bytes from I2C device address %x at Reg. %x %x\n", sbus_scd30_i2c.addr, cmd[0], cmd[1]);
	}

	// required, see scd30 interface datasheet
	k_sleep(K_MSEC(3));

	ret = i2c_read_dt(&sbus_scd30_i2c, data, sizeof(data));

	uint8_t expected[3] = {0x03, 0x42, 0};
	expected[2] = crc8(expected, 2, 0x31, 0xFF, false);

	bool scd30_pass = true;
	printk("Read SCD30 I2C @ d100: ");
	for (int i = 0; i < sizeof(expected); i++) {
		printk("%x", data[i]);
		if (data[i] != expected[i]) {
			scd30_pass = false;
		} 
	}

	if (scd30_pass) {
		printk("\nSCD30 I2C passed\n");
	} else {
		printk("\nSCD30 I2C failed\n");
	}

	// -- DPS310 --

	if (!device_is_ready(sbus_dps310_i2c.bus)) {
		printk("I2C bus %s is not ready!\n\r",sbus_dps310_i2c.bus->name);
		return 0;
	}

	uint8_t dps_data;
	uint8_t addr = 0x0D;
	ret = i2c_reg_read_byte_dt(&sbus_dps310_i2c, addr, &dps_data);
	if(ret != 0){
		printk("Failed to read byte from I2C device address %x at Reg. %x \n\r", sbus_dps310_i2c.addr, addr);
	}

	printk("Read DPS310 I2C byte @ %x: %x\n", addr, dps_data);
	if (dps_data != 0x10) {
		printk("DPS310 I2C failed\n");
	} else {
		printk("DPS310 I2C passed\n");
	}
	
	return ret;
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

	printk("nCE enabled\n");
	return ret;
}

int program_charge_params()
{
	return 0;
}

int main(void)
{
	if (turn_on_sbus()) {
		return 0;
	}

	test_pmic_i2c();
	test_sbus_i2c();

	program_default_params();

	enable_charging();

	infinite_blink(); // should not return
	return 0;
}

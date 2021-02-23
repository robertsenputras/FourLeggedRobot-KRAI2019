/*
 * mbed library for I2C LCD
 * Copyright (c) 2011 Hiroshi Suga
 * Released under the MIT License: http://mbed.org/license/mit
 *
 * This product includes:
 * mbed TextLCD Library, for a 4-bit LCD based on HD44780
 * Copyright (c) 2007-2010, sford
 */

/** @file I2CLCD.cpp
 * @brief I2C LCD library (mbed Phone Platform)
 */

#include "mbed.h"
#include "I2CLCD.h"

/**
 * @brief put character to LCD
 * @param value ASCII character code
 * @retval value
 */
int I2CLCD::_putc (int value) {

    if (value == '\n') {
        x = 0;
        y ++;
        if (y >= rows()) {
            y = 0;
            lcd_out(address(x, y), 0);
        }

    } else {

        lcd_out(address(x, y), 0);
        lcd_out(value, 1);
        x ++;
        if (x >= cols()) {
            x = 0;
            y ++;
            if (y >= rows()) {
                y = 0;
                lcd_out(address(x, y), 0);
            }
        }
    }

    return value;
}

/**
 * @brief get character from LCD
 * @retval ASCII character code
 */
int I2CLCD::_getc() {
    return lcd_in(0);
}


/**
 * @brief put character to LCD
 * @param p_sda port of I2C SDA
 * @param p_scl port of I2C SCL
 * @param p_i2caddr I2C address
 */
I2CLCD::I2CLCD (PinName p_sda, PinName p_scl, int p_i2caddr, I2CLCDType p_type, I2CLCDConfig p_config) : i2c(p_sda, p_scl) {
    init(p_i2caddr, p_type, p_config);
}

/**
 * @brief put character to LCD
 * @param p_i2c instance of I2C class
 * @param p_i2caddr I2C address
 */
I2CLCD::I2CLCD (I2C& p_i2c, int p_i2caddr, I2CLCDType p_type, I2CLCDConfig p_config) : i2c(p_i2c) {
    init(p_i2caddr, p_type, p_config);
}

void I2CLCD::init (int p_i2caddr, I2CLCDType p_type, I2CLCDConfig p_config) {

    i2caddr = p_i2caddr;
    type = p_type;

    lcd_cfg(p_config);

    wait_ms(500);
    lcd_out(0x30, 0);
    wait_ms(5);
    lcd_out(0x30, 0);
    wait_ms(2);
    lcd_out(0x30, 0);

    lcd_out(0x38, 0); // func 
    lcd_out(0x10, 0); // shift
    lcd_out(0x0c, 0); // display
    lcd_out(0x06, 0); // entry mode
    cls();
}

void I2CLCD::cls() {
    lcd_out(0x01, 0); // clear
    wait_ms(2);
    lcd_out(0x02, 0); // home
    wait_ms(2);
    locate(0, 0);
}

void I2CLCD::locate(int col, int row) {
    x = col;
    y = row;
    lcd_out(address(x, y), 0);
}

int I2CLCD::address(int col, int row) {
    switch (type) {
        case LCD16x1:
            return (col < 8 ? 0x80 : 0xc0) + (col & 0x03);
        case LCD16x4:
        case LCD20x4:
            switch (row) {
                case 0:
                    return 0x80 + col;
                case 1:
                    return 0xc0 + col;
                case 2:
                    return 0x94 + col;
                case 3:
                    return 0xd4 + col;
            }
        case LCD16x2B:
            return 0x80 + (row * 40) + col;
        case LCD8x2:
        case LCD16x2:
        case LCD20x2:
        default:
            return 0x80 + (row * 0x40) + col;
    }
}

int I2CLCD::cols() {
    switch (type) {
        case LCD8x2:
            return 8;
        case LCD20x4:
        case LCD20x2:
            return 20;
        case LCD16x1:
        case LCD16x2:
        case LCD16x2B:
        case LCD16x4:
        default:
            return 16;
    }
}

int I2CLCD::rows() {
    switch (type) {
        case LCD16x1:
            return 1;
        case LCD16x4:
        case LCD20x4:
            return 4;
        case LCD8x2:
        case LCD16x2:
        case LCD16x2B:
        case LCD20x2:
        default:
            return 2;
    }
}

void I2CLCD::lcd_cfg (I2CLCDConfig cfg) {
    i2c.start();
    i2c.write(i2caddr);
    i2c.write(LCDCFG_ENABLE | (cfg & 0x1f));
    i2c.stop();
}

void I2CLCD::lcd_out (char dat, char rs) {
    i2c.start();
    i2c.write(i2caddr);
    i2c.write(rs ? 0x40 : 0);
    i2c.write(dat);
    i2c.stop();
}

char I2CLCD::lcd_in (char rs) {
    char i;

    i2c.start();
    i2c.write(i2caddr);
    i2c.write(rs ? 0x40 : 0);

    i2c.start();
    i2c.write(i2caddr | 0x01);
    i = i2c.read(0);
    i2c.stop();

    return i;
}

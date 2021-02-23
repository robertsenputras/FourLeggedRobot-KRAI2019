/*
 * mbed library for I2C LCD
 * Copyright (c) 2011 Hiroshi Suga
 * Released under the MIT License: http://mbed.org/license/mit
 *
 * This product includes:
 * mbed TextLCD Library, for a 4-bit LCD based on HD44780
 * Copyright (c) 2007-2010, sford
 */

/** @file I2CLCD.h
 * @brief I2C LCD library (mbed Phone Platform)
 */
 
#ifndef I2CLCD_H
#define I2CLCD_H

#include "mbed.h"

/**
 * @brief default I2C address
 */
#define I2CLCD_ADDR 0x7c

/**
 * @brief LCD type
 */
enum I2CLCDType {
    LCD8x2,
    LCD16x1,
    LCD16x2,
    LCD16x2B,
    LCD16x4,
    LCD20x2,
    LCD20x4
};

/**
 * @brief LCD config
 */
enum I2CLCDConfig {
    LCDCFG_ENABLE   = 0x20,
    LCDCFG_PWMCOUNT = 0x10,
    LCDCFG_LED      = 0x08,
    LCDCFG_3V       = 0x04,
    LCDCFG_ADDR     = 0x02,
    LCDCFG_INIT     = 0x01
};

/**
 * @brief I2CLCD class
 */
class I2CLCD : public Stream {
public:
    I2CLCD (PinName p_sda, PinName p_scl, int p_i2caddr = I2CLCD_ADDR, I2CLCDType p_type = LCD16x2, I2CLCDConfig p_config = LCDCFG_3V);
    I2CLCD (I2C& p_i2c, int p_i2caddr = I2CLCD_ADDR, I2CLCDType p_type = LCD16x2, I2CLCDConfig p_config = LCDCFG_3V);

    void locate (int, int);
    void cls ();
    void lcd_cfg (I2CLCDConfig);

protected:
    virtual int _putc (int);
    virtual int _getc ();

    int address (int, int);
    int rows ();
    int cols ();
    void init (int, I2CLCDType, I2CLCDConfig);
    void lcd_out (char, char);
    char lcd_in (char);

    I2C i2c;
    int i2caddr;
    I2CLCDType type;
    int x, y;
};

#endif
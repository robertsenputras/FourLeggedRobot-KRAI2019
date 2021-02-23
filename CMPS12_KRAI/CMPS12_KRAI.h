/*
 * adopted from
 * CMPS03 by: Aarom Berk
 * 
 * Bismillahirahmanirrahim
 */

#ifndef CMPS12_KRAI_H
#define CMPS12_KRAI_H

/**
 * Includes
 */
#include "mbed.h"

#define CMPS_DEFAULT_I2C_ADDRESS 0xC0
//set register kompas
#define SOFTWARE_REVISION_REG    0x0
#define COMPASS_BEARING_WORD_REG 0x2
#define COMPASS_PITCH_WORD_REG 0x4
#define COMPASS_ROLL_WORD_REG 0x5

/** Interface to read compass using I2C communciation
 * compass type : CMPS 12
 */
class CMPS12_KRAI {

    I2C* i2c;
    int  i2cAddress;

public:

    /** Create CMPS12 inteface
     * 
     *
     * @param sda mbed pin to use for I2C SDA
     * @param scl mbed pin to use for I2C SCL
     * @param address I2C address of this device.
     */
    CMPS12_KRAI(PinName sda, PinName scl, int address);

    /** Membaca revisi dari kompas
     * 
     */
    char readSoftwareRevision(void);

    /** Membaca nilai dari kompas
     * 
     * Range nilai 0 - 3599
     */
    int getAngle(void);
    
    /** Membaca nilai sudut pitch dari kompas
     * 
     * Range nilai 0 - 3599
     */
    int getPitch(void);
    
    /** Membaca nilai sudut roll dari kompas
     * 
     * Range nilai (+/-90)
     */
    int getRoll(void);

    /** Kalibrasi kompas 
     *  
     */
    void calibrate(void);

    /** Stop calibrate
     * 
     */
    void stopCalibrate(void);
    
    int getAccelX(void);
};

#endif /* CMPS_KRAI_H */

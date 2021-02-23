/**
 * adopted from
 * CMPS03 by: Aarom Berk
 * 
 * Bismillahirahmanirrahim
 */

/**
 * Includes
 */
#include "CMPS12_KRAI.h"

CMPS12_KRAI::CMPS12_KRAI(PinName sda, PinName scl, int address) {
    i2c = new I2C(sda, scl);
    //CMPS11 maksimum 100kHz CMPS12 maksimum 400kHz
    i2c->frequency(100000);
    i2cAddress = address;

}

char CMPS12_KRAI::readSoftwareRevision(void){
    char registerNumber   = SOFTWARE_REVISION_REG;
    char registerContents = 0;

    //First, send the number of register we wish to read,
    //in this case, command register, number 0.
    i2c->write(i2cAddress, &registerNumber, 1);
    
    //Now, read one byte, which will be the contents of the command register.
    i2c->read(i2cAddress, &registerContents, 1);
    
    return registerContents; 
}


int CMPS12_KRAI::getAngle(void){
    char registerNumber = COMPASS_BEARING_WORD_REG;
    char registerContents[2] = {0x00, 0x00};
    
    //Mengirim register untuk address pertama dari i2c
    i2c->write(i2cAddress, &registerNumber, 1); //1
    
    //Mengambil data dari 2 address I2c
    i2c->read(i2cAddress, registerContents, 2);
    
    //Register 0 adalah 8 bit, harus di shift
    //Register 1 adalah 16 bit, bisa langsung dibaca
    int angle = ((int)registerContents[0] << 8) | ((int)registerContents[1]);
    
    //kirim data berupa sudut 0-360
    return angle;   
}

int CMPS12_KRAI::getPitch(void){
    char registerNumber = COMPASS_PITCH_WORD_REG;
    char registerContents[2] = {0x00, 0x00};
    
    //Mengirim register untuk address pertama dari i2c
    i2c->write(i2cAddress, &registerNumber, 1);
    
    //Mengambil data dari 2 address I2c
    i2c->read(i2cAddress, registerContents, 1);
    
    //simpan data ke variable pitch
    int pitch = ((int)registerContents[0] );
    
    return pitch;   
}

int CMPS12_KRAI::getRoll(void){
    char registerNumber = COMPASS_ROLL_WORD_REG;
    char registerContents[2] = {0x00, 0x00};
    
    //Mengirim register untuk address pertama dari i2c
    i2c->write(i2cAddress, &registerNumber, 1);
    
    //Mengambil data dari 2 address I2c
    i2c->read(i2cAddress, registerContents, 1);
    
    //simpan data ke variable roll
    int roll = ((int)registerContents[0] );
    
    return roll;   
}

void CMPS12_KRAI::calibrate(void){
    char registerNumber   = SOFTWARE_REVISION_REG;
    char calibrate_data1 = 0xF0;
    char calibrate_data2 = 0xF5;
    char calibrate_data3 = 0xF7;
    //kirim data 1
    //Thread::wait(25);
    i2c->write(i2cAddress, &registerNumber, 1);
    i2c->write(i2cAddress, &calibrate_data1, 1);
    //kirim data 2 delay 25ms
    //Thread::wait(25);
    i2c->write(i2cAddress, &registerNumber, 1);
    i2c->write(i2cAddress, &calibrate_data2, 1);
    //kirim data 3 delay 25ms
    //Thread::wait(25);
    i2c->write(i2cAddress, &registerNumber, 1);
    i2c->write(i2cAddress, &calibrate_data3, 1);
}

void CMPS12_KRAI::stopCalibrate(void){
    char registerNumber   = SOFTWARE_REVISION_REG;
    char calibrate_data1 = 0xF8;
    //kirim data 1
    i2c->write(i2cAddress, &registerNumber, 1);
    i2c->write(i2cAddress, &calibrate_data1, 1);
}


int CMPS12_KRAI::getAccelX(void){
    char registerNumber = COMPASS_ROLL_WORD_REG; //0x02
    char registerContents[16] = {0x00, 0x00};
    
    //Mengirim register untuk address pertama dari i2c
    i2c->write(i2cAddress, &registerNumber, 1);
    
    //Mengambil data dari 2 address I2c
    i2c->read(i2cAddress, registerContents, 16);
    
    //simpan data ke variable roll
    int accelX = ((int)registerContents[11]<<8 | (int)registerContents[12] );
    
    return accelX;   
}
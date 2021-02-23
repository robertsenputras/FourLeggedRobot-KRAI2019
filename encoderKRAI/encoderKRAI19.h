/*
 * encoderKRAI19.h
 *
 *  Created on: Dec 16, 2018
 *      Author: gatul
 */

#ifndef ENCODERKRAI19_H
#define ENCODERKRAI19_H


//Bismillahirahamnirahim

//LIBRARY
#include "mbed.h"

/**************************
 * Konstanta dan Variabel *
 **************************/

//KONSTANTA
#define PREV_MASK 0x1 //Konstanta untuk mengetahui previous direction
#define CURR_MASK 0x2 //Konstanta untuk mengetahui current direction
#define INVALID   0x3 //XORing two states where both bits have changed.

/********************************
 * Quadrature Encoder Interface *
 ********************************/

class encoderKRAI {

public:

    typedef enum Encoding {

        X2_ENCODING,
        X4_ENCODING,
		X5_ENCODING

    } Encoding;

    encoderKRAI(PinName channelA, PinName channelB, PinName channelZ, int pulsesPerRev, Encoding encoding = X2_ENCODING);
    /*******************************************
     * Membuat interface dari encoder
     * @param inA DigitalIn, out A dari encoder
     * @param inB DigitalIn, out B dari encoder
     * @param inZ DigitalIn, out Z dari encoder
     *******************************************/

    void reset(void);
    /*******************************************
     * Reset encoder.
     * Reset pembacaaan menjadi 0
     *******************************************/

    int getPulses(void);
    /*******************************************
     * Membaca pulse yang didapat oleh encoder
     * @return Nilai pulse yang telah dilalui.
     *******************************************/

    int getRevolutions(void);
    /*******************************************
     * Membaca putaran yang didapat oleh encoder
     * @return Nilai revolusi/putaran yang telah dilalui.
     *******************************************/

private:

    void encode(void);
    /*******************************************
     * Menghitung pulse
     * Digunakan setiap rising/falling edge baik channel A atau B
     * Membaca putaran CW atau CCW => mengakibatkan pertambahan/pengurangan pulse
     *******************************************/

//VARIABEL UNTUK PERHITUNGAN PULSE
    Encoding encoding_;

    InterruptIn channelA_;
    InterruptIn channelB_;
    InterruptIn channelZ_;

    int          pulsesPerRev_;
    int          prevState_;
    int          currState_;

    volatile int pulses_;
    volatile int revolutions_;


};

#endif /* ENCODERKRAI_ENCODERKRAI19_H_ */

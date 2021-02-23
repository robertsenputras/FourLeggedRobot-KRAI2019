#ifndef MBED_H
#include "mbed.h"
#endif
 
#ifndef JoystickPS3__serialDEFAULT_BAUD
#define JoystickPS3__serialDEFAULT_BAUD       115200
#endif
 
//Serial debug(USBTX,USBRX);
 
namespace JoystickPS3 {
 
class joysticknucleo {
public:
    joysticknucleo(PinName tx, PinName rx) : _serial(tx, rx)
    {
        
    }
    
// Deklarasi variabel tombol analog
    unsigned char LX, LY, RX, RY, R2, L2;
    
    unsigned char button;
    unsigned char RL;
    unsigned char button_click;
    unsigned char RL_click;
    
    void setup(){
        _serial.baud(JoystickPS3__serialDEFAULT_BAUD);
  //      debug.baud(9600);
        }
 
    /*********************************************************************************************/
    /**                                                                                         **/
    /** FUNGSI PEMBACAAN DATA                                                                   **/
    /** -   Data yang diterima dari Serial Arduino berbentuk 8-bit                              **/
    /** -   Data yang diterima diolah menjadi boolean / 1-bit untuk data tombol button dan RL   **/
    /**     karena data yang digunakan adalah 1-bit (true/false)                                **/
    /** -   Untuk analog data yang diterima tidak diolah karena rentang data yang dikirimkan    **/
    /**     memiliki rentang 0-255 / 8-bit, dan data yang akan digunakan adalah data 8-bit      **/
    /**                                                                                         **/
    /**         |------|-------|-------|------|-------|--------|-----------|----------|         **/
    /** Bit Ke  |   7  |   6   |   5   |   4  |   3   |    2   |      1    |     0    |         **/
    /**         |------|-------|-------|------|-------|--------|-----------|----------|         **/
    /** Data    | kiri | bawah | kanan | atas | kotak | silang | lingkaran | segitiga |         **/
    /**         |------|-------|-------|------|-------|--------|-----------|----------|         **/
    /**                                                                                         **/
    /** -   Penggabungan data R1, R2, L1, L2, R3, L3, START, dan SELECT disimpan dalam          **/
    /**     variabel "RL"                                                                       **/
    /** -   Urutan data pada variabel "RL" dan "RL_click" adalah                                **/
    /**     sebagai berikut                                                                     **/
    /**                                                                                         **/
    /**         |----|--------|-------|----|----|----|----|                                     **/
    /** Bit Ke  |  6 |    5   |   4   |  3 |  2 |  1 |  0 |                                     **/
    /**         |----|--------|-------|----|----|----|----|                                     **/
    /** Data    | PS | SELECT | START | L3 | L1 | R3 | R1 |                                     **/
    /**         |----|--------|-------|----|----|----|----|                                     **/
    /**                                                                                         **/
    /*********************************************************************************************/
    
    void olah_data()
    {
        // Pengolahan data dari data "button" 
        segitiga = (bool)((button >> 0) & 0x1);
        lingkaran = (bool)((button >> 1) & 0x1);
        silang = (bool)((button >> 2) & 0x1);
        kotak = (bool)((button >> 3) & 0x1);
        atas = (bool)((button >> 4) & 0x1);
        kanan = (bool)((button >> 5) & 0x1);
        bawah = (bool)((button >> 6) & 0x1);
        kiri = (bool)((button >> 7) & 0x1);
        
        // Pengolahan data dari data "RL" 
        R1 = (bool)((RL >> 0) & 0x1);
        R3 = (bool)((RL >> 1) & 0x1);
        L1 = (bool)((RL >> 2) & 0x1);
        L3 = (bool)((RL >> 3) & 0x1);
        START = (bool)((RL >> 4) & 0x1);
        SELECT = (bool)((RL >> 5) & 0x1);
        PS = (bool)((RL >> 6) & 0x1);
    
        // R2 click dan L2 click
        if (R2 > 100) {
            if ( R2sebelum) { R2_click = false;
                } else { R2_click = true;}   
            R2sebelum = true; 
        }else { 
            R2sebelum = false;
            R2_click = false;
        }            
        if (L2 > 100) {
            if ( L2sebelum) { L2_click = false;
                } else { L2_click = true;}   
            L2sebelum = true; 
        }else { L2sebelum = false;
                L2_click = false;
            }            
    
        segitiga_click = (bool)((button_click >> 0) & 0x1);
        lingkaran_click = (bool)((button_click >> 1) & 0x1);
        silang_click = (bool)((button_click >> 2) & 0x1);
        kotak_click = (bool)((button_click >> 3) & 0x1);
        atas_click = (bool)((button_click >> 4) & 0x1);
        kanan_click = (bool)((button_click >> 5) & 0x1);
        bawah_click = (bool)((button_click >> 6) & 0x1);
        kiri_click = (bool)((button_click >> 7) & 0x1);
        
        // Pengolahan data dari data "RL" 
        R1_click = (bool)((RL_click >> 0) & 0x1);
        R3_click = (bool)((RL_click >> 1) & 0x1);
        L1_click = (bool)((RL_click >> 2) & 0x1);
        L3_click = (bool)((RL_click >> 3) & 0x1);
        START_click = (bool)((RL_click >> 4) & 0x1);
        SELECT_click = (bool)((RL_click >> 5) & 0x1);
        PS_click = (bool)((RL_click >> 6) & 0x1);
    }
    
    /*********************************************************************************************/
    /**                                                                                         **/
    /** FUNGSI IDLE                                                                             **/
    /** -   Fungsi dijalankan saat Arduino mengirimkan data yang merupakan                      **/
    /**     kondisi PS3 Disconnected                                                            **/
    /** -   Fungsi membuat semua data joystik bernilai 0                                        **/
    /**                                                                                         **/
    /*********************************************************************************************/
    
    void idle(){
        // Set 0    
        button = 0;
        RL = 0;
        button_click = 0;
        RL_click = 0;
        R2_click =0;
        L2_click =0;
        R2 = 0;
        L2 = 0;
        RX = 0;
        RY = 0;
        LX = 0;
        LY = 0;
    
    }
    
    /*********************************************************************************************/
    /**                                                                                         **/
    /** FUNGSI PEMBACAAN DATA                                                                   **/
    /** -   Fungsi pembacaan data yang dikirim dari arduino                                     **/
    /** -   Data yang dikirim dari arduino merupakan paket data dengan format pengiriman        **/
    /**                                                                                         **/
    /** |------|------|--------|----|--------------|----------|----|----|----|----|----|----|   **/
    /** | 0x88 | 0x08 | button | RL | button_click | RL_click | R2 | L2 | RX | RY | LX | LY |   **/
    /** |------|------|--------|----|--------------|----------|----|----|----|----|----|----|   **/
    /**                                                                                         **/
    /** |------|------|                                                                         **/
    /** | 0x88 | 0x09 |                                                                         **/
    /** |------|------|                                                                         **/
    /**                                                                                         **/
    /** -   Jika urutan data yang diterima seperti tabel diatas, maka data tersebut akan        **/
    /**     diolah untuk input ke aktuator                                                      **/
    /**                                                                                         **/
    /*********************************************************************************************/
    
    void baca_data()
    {
        // Interrupt Serial
        if(_serial.readable()&&(_serial.getc()==0x88)) {
            // Pembacaan data dilakukan jika data awal yang diterima adalah 0x88 kemudian 0x08
            if(_serial.getc()==0x08){
                // Proses Pembacaan Data
                button = _serial.getc();
                RL = _serial.getc();
                button_click = _serial.getc();
                RL_click = _serial.getc();
                R2 = _serial.getc();
                L2 = _serial.getc();
                RX = _serial.getc();
                RY = _serial.getc();
                LX = _serial.getc();
                LY = _serial.getc();
            } else if(_serial.getc()==0x09) {
                // PS3 Disconnected
                idle();
            } else {
                idle(); }
            // Indikator - Print data pada monitor PC
        // debug.printf("%2x %2x %2x %2x %3d %3d %3d %3d %3d %3d\n\r",button, RL, button_click, RL_click, R2, L2, RX, RY, LX, LY);
        }   
    }
 
    int getData(){
            return _serial.getc();
    }
    int readable(){
        return _serial.readable();
    }
    
public:
    // Deklarasi variabel tombol joystik
    bool kiri, kanan, atas, bawah;
    bool segitiga, lingkaran, kotak, silang;
    bool L1, R1, L3, R3, START, SELECT, PS;
    
    bool kiri_click, kanan_click, atas_click, bawah_click;
    bool segitiga_click, lingkaran_click, kotak_click, silang_click;
    bool L1_click, R1_click, L3_click, R3_click, R2_click, L2_click;
    bool R2sebelum,L2sebelum;
    bool START_click, SELECT_click, PS_click;
  
protected:  
    virtual int _getc(){return _serial.getc();}
    Serial _serial;
};
 
};
 
using namespace JoystickPS3;
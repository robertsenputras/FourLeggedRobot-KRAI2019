/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  *
 *        HORShip Nasional KRAI 2018                                              *
 *                                                                              *
 *  Bismillahirahmanirrahim                                                     *
 *                                                                              *
 *  "Hai hamba-hamba-Ku yang malampaui batas terhadap diri mereka sendiri,      *
 *  janganlah kamu berputus asa dari rahmat Allah.                              *
 *  Sesungguhnya Allah mengampuni dosa-dosa semuanya.                           *
 *  Sesungguhnya Dialah Yang Maha Pengampun lagi Maha Penyayang."               *
 *  Surat Az-Zumar Ayat 53                                                      *
 *  Built by : KUDA KRAI 2019                                                   *
 *  																			*
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


//DEKLARASI LIBRARY
#include "mbed.h"
#include "millis.h"
#include "Motor.h"
#include "encoderKRAI.h"
#include "CMPS12_KRAI.h"
#include "variable.h"
#include "PID.h"
#include "TextLCD.h"
#include "eeprom.h"
//#include "encoderHAL/encoderHAL.h"


PIDT pidA(1,1,1,1,100);
Serial pc(USBTX, USBRX, 115200);
CMPS12_KRAI cmps(PB_3, PB_10, 0xc0);
//encoderKRAI encKanan(PC_14,PC_15, 538, encoderKRAI::X4_ENCODING);  //encoder motor steering
encoderKRAI encKanan(PC_2,PC_3, 538, encoderKRAI::X4_ENCODING);
encoderKRAI encKiri(PC_10,PC_11, 538, encoderKRAI::X4_ENCODING);
Motor motorSteer(PB_14, PC_4, PB_13);   //motor arah putaran CCW ++
Motor motorKi(PA_11,  PB_12,PA_7);                  //motor kanan
Motor motorKa(PA_5, PA_6, PA_12);                    //motor kiri

DigitalIn button1(PB_8,PullUp);	//HIJAU
DigitalIn button2(PA_13,PullUp);	//MERAH
DigitalIn button3(PA_10,PullUp); //KUNING
DigitalIn button4(PB_5,PullUp);	//BIRU
DigitalIn IRKi(PA_9,PullUp);
DigitalIn IRKa(PA_14,PullUp);
//AnalogIn IR1(PA_0);//GA BOLEH DI PAKE
AnalogIn IR2(PC_0);
AnalogIn IR3(PC_1);
AnalogIn IR4(PA_1);
DigitalIn IRAtas(PB_9); // SESUAIKAN PIN

//pneu benar
DigitalOut pneuGripper(PC_9);	//0tutup
DigitalOut pneuNaik(PC_8);		//0 turun
//DigitalOut pneuC(PC_9); // benar

I2C i2c_lcd(PB_4,PA_8); // SDA, SCL
TextLCD_I2C lcd(&i2c_lcd, 0x4E, TextLCD::LCD16x2); // I2C bus, PCF8574 Slaveaddress, LCD Type
EEPROM rom(PB_4,PA_8, 0, EEPROM::T24C256);

void FSM();
void gerakMotor();
void gerakRobot(float w_sp, float th_sp, float w_max, float wt_max, float mode);
void getCMPS();
void getPitch();
void GP2A1();
void GP2A2();
void GP2A3();
void initMain();
void lapanganMerah();
void lapanganBiru();
void PIDMotor();
void PIDTheta();
void printSerial();
void samplingEnc();
void stateSwitch();
void stallSafety();
void setLCD();
void tampilLCD();
void zeroEEPROM();
void assignDefaultValueEEPROM();
void assignValue();

float fabsError(float a, float b);

int main() {
	//zeroEEPROM
	//assignDefaultValueEEPROM();
	assignValue();
	setLCD();
	startMillis();
	state=0;
	last_mt_PID=1;
	theta=0;
	theta0 = cmps.getAngle()/10;
	pitch0 = cmps.getPitch();
	theta_sp=0;
	pneuGripper=0;	//0 turun
	pneuNaik=0;	//0 tutup
    while(1) {

    	if (millis() - last_mt_motor>=2){
			gerakMotor();
			last_mt_motor = millis();
		}
    	if (millis() - last_mt_PID>=9){
//    		PIDTheta();
    		PIDMotor();
    		last_mt_PID = millis();
    	}
//    		pulse_A = encKanan.getPulses();
//    		        pulse_B = encKiri.getPulses();
    	if (millis() - last_mt_PIDTheta>=15){
    		if (fabs(theta-theta_sp)>toleranceT){
				PIDTheta();
			} else {
				speedTheta=0;
			}
    		last_mt_PIDTheta = millis();
    	}
    	if (millis() - last_mt_FSM>=17){
			FSM();
			last_mt_FSM=millis();
		}
    	if (millis() - last_mt_sampling>=25){
			samplingEnc();
			last_mt_sampling=millis();
		}
    	if (millis() - last_mt_lcd >= 901){
//    		if (state==0 || state==99 ||state)
			tampilLCD();
			last_mt_lcd=millis();
    	}
//    	stateSwitch();
//    	if (state==1){
    	if (millis() - last_mt_switch>=61){
			stateSwitch();
			last_mt_switch = millis();
		}
//    	}else{
    	if (millis() - last_mt_IR>=91){
//    		toggleIR++;
//    		if (toggleIR>2)
//    			toggleIR=0;
    		toggleIR= !toggleIR;
    		if (toggleIR==0)
    			GP2A1();
    		else if (toggleIR==1)
    			GP2A3();
//    		else if (toggleIR==2)
//    			GP2A3();
			last_mt_IR = millis();
		}
//    	}
//
    	if (millis() - last_mt_pitch>=39){
    		getPitch();
			last_mt_pitch = millis();
		}
    	if (millis() - last_mt_cmps >= 53){
    		getCMPS();
    		last_mt_cmps = millis();
    	}
        if (millis()- last_mt_print >= 107){
			last_mt_print = millis();
			printSerial();
		}

//        if (millis() - last_mt_button>1500){
//        	last_mt_button = millis();
//        	theta_sp+=15;
//        }

    }
}

void FSM(){
	//target =9;
		if (state==0){
			//keadaan diam
			gerakRobot(0 ,0 ,0.9 ,0, 0); //speed_sp, theta_sp, pwmMax, speedThetaMax, mude putar
		}
//		} else if (state==1){
//			//gerak maju
//			gerakRobot(4.0 ,0 ,0.9 ,4.0, 0); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
//
//		//		if (eTheta<=8.1)
//		//			state=0;
//		} else if (state==2){
//			gerakRobot(8.0 ,1.0 ,0.9 ,12.55, 0); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
//			if (pitch>=7){
//				state=15;
//			}
//		} else if (state==3){
//			//gerak belok kanan 45
//			gerakRobot(8.0 ,12.0 ,0.9 ,3.5, 1); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
//			if (eTheta<=8.1)
//				state=14;
//		} else if (state==4){
//			gerakRobot(8.0 ,0.0 ,0.9 ,7.55,0);//4.0, 0); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
//	//		if (eTheta<=8.1)
//	//			state=0;
//		} else if (state==5){
//			//gerak naik Uukhai
//			gerakRobot(5 ,-90 ,0.9 ,4.0, 0); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
//			if (eTheta<=8.1)
//				state=0;
//		} else if (state==14){
//			gerakRobot(8.0 ,-45 ,0.9 ,3.5, 1); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
//
//		}
		else if (state==15){
			gerakRobot(0.0 ,170 ,0.9 ,3.5, 0); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
			if (eTheta<=8.1){
				state=0;
			}
		} else if (state==16){ //
			gerakRobot(0.0 ,-170 ,0.9 ,3.5, 0); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
			if (eTheta<=8.1){
				state=0;
			}
		} else if (state==17){ //
			gerakRobot(5.0 ,0 ,0.9 ,3.5, 0); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
			if (IRDepan<70.0){
				state=0;
			}
		}
		else  if (state==20){	//deteksi bdg miring
			gerakRobot(15 ,0.0 ,0.9 ,12.55, 1); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
			if (fabs(pitch)>=6.5){
				lcd.setBacklight(TextLCD::LightOff);
				state++;
			}
		}else if (state==21){//utk deteksi datar
			gerakRobot(15.0 ,0.0 ,0.9 ,12.55, 1); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
			if (fabs(pitch)<=6.0){
				lcd.setBacklight(TextLCD::LightOn);
				state = 24;
			}
		}else if (state==22){	//state utk deteksi tembok
			gerakRobot(5.5 ,0.0 ,0.9 ,12.55, 1); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
			if (IRDepan<=batasTembok-5.0)
				state++;
		}
		else if (state==23){
			gerakRobot(0,70,0.9,6.0,0);
			if(eTheta<=8.1){
				state++;
			}
		}
		else if (state==24){	//state UUKHAI
			gerakRobot(0 ,0.0 ,0.9 ,0, 0); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
			pneuNaik=1;
		}
		else if (state<200){
			lapanganMerah();
		}
		else if (state<300){
			lapanganBiru();
		}

}
void lapanganMerah(){
	target = 1*8.0;

	//////////////////
	///MODE FASTER ///
	//////////////////

	if (state==100){	//gerak untuk maju hingga belokan  >=speed dune
		gerakRobot(m_speed_maju ,0.0 ,0.9 ,12.55, 1); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (IRBlkng>155.0){//m_jarak_ir){
			state++;
		}
//		if (IRDepan < m_jarak_ir){
//			state++;
//		}
	} else if (state==101){	// belok utk sanddune + naik sanddune
		gerakRobot(m_speed_dune ,45 ,0.9 ,m_max_dune, 1); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (pitch>=7){	//saat miring ke bawah robotnya
			state++;
		}
	} else if (state==102){
		gerakRobot(m_speed_dune ,47.2 ,0.9 ,3.5, 1); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (fabs(pitch)<3.5){	//saat robot kembali di landasan datar
			state++;
		}
	} else if (state==103){	//TUNINGABLE STATE untuk belok ke tussock tanpa pivoting
		gerakRobot(m_speed_belok_tussock ,0.0 ,0.9 ,m_max_belok_tussock, 1); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (eTheta<7.8){
			state++;
		}
	}
	else if (state==104){	//untuk melewati tussock
		gerakRobot(m_speed_dune,0.0 ,0.9 ,m_max_tussock, 0); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (!IRAtas && millis() - last_mt_button>800){
			state=0;
			last_mt_button=millis();
		}
	}else if (state==105){
		gerakRobot(0 ,0.0 ,0.9 ,6.0, 0); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (!IRAtas && millis()-last_mt_button>800){
			state=0;
			last_mt_button = millis();
		}
	} else if (state==106){
		gerakRobot(0 ,-90.0 ,0.9 ,6.0, 0); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (!IRAtas && millis()-last_mt_button>800){
			state++;
			last_mt_button = millis();
		}
	}else if (state==107){
		gerakRobot(m_speed_dune ,-90.0 ,0.9 ,6.0, 0); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (fabs(pitch)>=6.0)
			state++;
	}else if (state==108){
		gerakRobot(m_speed_dune ,-90.0 ,0.9 ,6.0, 0); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (fabs(pitch)<=3.0)
			state++;
	}else if (state==109){
		gerakRobot(m_speed_dune ,-90.0 ,0.9 ,6.0, 0); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (IRDepan<=batasTembok)
			state++;
	}

	//////////////////
	///GERAK SAFETY///
	//////////////////

	else if (state==150){	// maju normal
		gerakRobot(m_speed_dune ,m_speed_maju ,0.9 ,12.55, 1); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (lastState==40){
			if (millis()-last_mt_button>2000 && (IRBwhKa<m_jarak_ir || IRDepan<m_jarak_ir ) ){ // yg d maksud IR Kiri
				state++;
			}
		}else{
			if (IRBwhKa<m_jarak_ir || IRDepan<m_jarak_ir ){ // yg d maksud IR Kiri
				state++;
			}
		}
	} else if (state==151){	// belok utk sanddune (dan naik sanddune)
		gerakRobot(0 ,45 ,0.9 ,m_max_belok_tussock, 0); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (eTheta<=8.1){
			state++;
		}
	} else if (state==152){	// belok utk sanddune (dan naik sanddune)
		gerakRobot(6.0 ,45 ,0.9 ,6.55, 0); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (pitch>=12){
			state++;
		}
	}else if (state==153){	// belok utk sanddune (dan naik sanddune)
		gerakRobot(6.0 ,48 ,0.9 ,6.55, 1); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (fabs(pitch)<=4.8){
			state++;
		}
	}else if (state==154){	// belok utk sanddune (dan naik sanddune)
		gerakRobot(m_speed_dune-2.0 ,90 ,0.9 ,m_max_belok_tussock, 0); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (eTheta<8.1){
			state++;
		}
	}else if (state==155){
		if (m_speed_dune>9.0)
			m_speedBelok=7.0;
		else
			m_speedBelok = m_speed_dune-2.0;
		gerakRobot(m_speedBelok ,90 ,0.9 ,6.55, 1); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (IRBwhKa<batasTembok || IRDepan<batasTembok){
			state++;
		}
	} else if (state==156){
		gerakRobot(m_speed_tussock,0.0 ,0.9 ,m_max_belok_tussock, 0); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (eTheta<=8.1){
			state++;
			last_mt_button=millis();
		}
	}	else if (state==157){
		gerakRobot(m_speed_dune ,0.0 ,0.9 ,12.0, 0); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (!IRAtas && millis() - last_mt_button>800){
			state=0;
			last_mt_button=millis();
		}
	}




	else if (state==157){
		gerakRobot(0 ,0.0 ,0.9 ,6.0, 0); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (!IRAtas && millis()-last_mt_button>800){
			state=0;
			last_mt_button = millis();
		}
	} else if (state==158){
		gerakRobot(0 ,-90.0 ,0.9 ,6.0, 0); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
//		if (!IRAtas && millis()-last_mt_button>800){
//			state++;
//			last_mt_button = millis();
//		}
	}

	//////////////////////////
	///GERAK RETRY SANDDUNE///
	//////////////////////////

	else if (state==171){	// belok utk sanddune (dan naik sanddune)
		gerakRobot(2.0 ,45 ,0.9 ,5.0, 1); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (eTheta<=8.1){
			state++;
		}
	} else if (state==172){	// belok utk sanddune (dan naik sanddune)
		gerakRobot(m_speed_dune ,45 ,0.9 ,2.55, 1); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (pitch>=7){
			state++;
		}
	}else if (state==173){	// belok utk sanddune (dan naik sanddune)
		gerakRobot(m_speed_dune ,48 ,0.9 ,2.0, 1); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (fabs(pitch)<3.8){
			state++;
		}
	}else if (state==174){	// belok utk sanddune (dan naik sanddune)
		gerakRobot(m_speed_dune ,90 ,0.9 ,4.0, 0); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (eTheta<8.1){
			state++;
		}
	}else if (state==175){	//belok ke tussock
		gerakRobot(m_speed_dune-2.0 ,90 ,0.9 ,2.0, 1); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (IRDepan<batasTembok){
			state++;
		}
	} else if (state==176){
		gerakRobot(m_speed_dune ,0.0 ,0.9 ,7.55, 0); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (!IRAtas && millis() - last_mt_button>800){
			state=0;
			last_mt_button=millis();
		}
	} else if (state==177){
		gerakRobot(0 ,-90.0 ,0.9 ,6.0, 0); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (!IRAtas && millis() - last_mt_button>800){
			state=0;
			last_mt_button=millis();
		}
	}else if (state==178){
		gerakRobot(m_speed_dune ,-90.0 ,0.9 ,6.0, 0); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (fabs(pitch)>=6.0)
			state++;
	}else if (state==179){
		gerakRobot(m_speed_dune ,-90.0 ,0.9 ,6.0, 0); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (fabs(pitch)<=3.0)
			state++;
	}

	else if (state==180){
		gerakRobot(m_speed_dune ,-90.0 ,0.9 ,6.0, 0); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (IRDepan<=batasTembok)
			state++;
	}

	//////////////////////////
	///GERAK RETRY TUSSOCK ///
	//////////////////////////
	else if (state==184){	// belok utk sanddune (dan naik sanddune)
		gerakRobot(m_speed_dune ,43 ,0.9 ,6.0, 1); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (eTheta<8.1){
			state++;
		}
	}else if (state==185){
		gerakRobot(m_speed_dune ,45 ,0.9 ,2.0, 1); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (IRDepan<batasTembok){
			state++;
		}
	} else if (state==186){
		gerakRobot(m_speed_dune ,-45.0 ,0.9 ,6.0, 0); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (!IRAtas && millis() - last_mt_button>800){
			state=0;
			last_mt_button=millis();
		}
	} else if (state==187){
		gerakRobot(0 ,-135.0 ,0.9 ,6.0, 0); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (!IRAtas && millis() - last_mt_button>800){
			state=0;
			last_mt_button=millis();
		}
	}else if (state==188){
		gerakRobot(m_speed_dune ,-135.0 ,0.9 ,6.0, 0); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (fabs(pitch)>=6.0)
			state++;
	}else if (state==189){
		gerakRobot(m_speed_dune ,-135.0 ,0.9 ,6.0, 0); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (fabs(pitch)<=3.0)
			state++;
	}else if (state==190){
		gerakRobot(m_speed_dune ,-135.0 ,0.9 ,6.0, 0); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (IRDepan<=batasTembok)
			state++;
	}




}
void lapanganBiru(){
	target=1*8.0;

	//////////////////
	///MODE FASTER ///
	//////////////////

	if (state==200){	// maju normal
		gerakRobot(b_speed_maju ,0 ,0.9 ,12.55,1); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (IRBlkng>155.0){
			state++;
		}
//		if (IRDepan < m_jarak_ir){
//			state++;
//		}
	} else if (state==201){	// belok utk sanddune (dan naik sanddune)
		gerakRobot(b_speed_dune ,-45 ,0.9 ,b_max_dune, 1); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (pitch>=7 && pitch < 50){	//deteksi turun
			state++;
		}
	} else if (state==202){
		gerakRobot(b_speed_dune ,-47.2 ,0.9 ,3.5, 1); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (fabs(pitch)<3.5){	//deteksi datar setelah turun
			state++;
		}
	} else if (state==203){	//deteksi IR blkng stlh sanddune
		gerakRobot(b_speed_belok_tussock , 0 , 0.9 , b_max_belok_tussock ,1); //speed_sp, theta_sp -70, pwmMax, speedThetaMax, mode putar
//		if (IRBlkng>90.0){
//			state++;
//		}
		if (eTheta<5.8){
			state++;
		}
	}
	else if (state==204){	//utk belok ke arah tussock
		gerakRobot(b_speed_dune ,0 ,0.9 ,b_max_tussock, 0); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (!IRAtas && millis() - last_mt_button>800){
					state=0;
					last_mt_button=millis();
				}
//		if (eTheta<8.1){
//			state++;
//		}
	} else if (state==205){ 	//utk lewat tussock
		gerakRobot(b_speed_tussock ,0 ,0.9 ,6.55, 0); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (!IRAtas && millis() - last_mt_button>800){
			state=0;
			last_mt_button=millis();
		}
	} else if (state==206){
		gerakRobot(0 ,90.0 ,0.9 ,6.0, 0); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (!IRAtas && millis() - last_mt_button>800){
			state=0;
			last_mt_button=millis();
		}
	}else if (state==207){
		gerakRobot(0 ,90.0 ,0.9 ,6.0, 0); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (!IRAtas && millis() - last_mt_button>800){
			state=0;
			last_mt_button=millis();
		}
	}else if (state==208){
		gerakRobot(b_speed_dune ,90.0 ,0.9 ,6.0, 0); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (fabs(pitch)>=6.0)
			state++;
	}else if (state==209){
		gerakRobot(b_speed_dune ,90.0 ,0.9 ,6.0, 0); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (fabs(pitch)<=3.5)
			state++;
	}
	else if (state==210){
		gerakRobot(b_speed_dune-1.5 ,90.0 ,0.9 ,6.0, 0); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (IRDepan<=80.0)
			state++;
	}else if (state==211){
		gerakRobot(0 ,90.0 ,0.9 ,6.0, 0); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		pneuNaik =1;
	}


	//////////////////
	///MODE SAFETY ///
	//////////////////

	else if (state==250){	// maju normal
		gerakRobot(b_speed_dune ,b_speed_maju ,0.9 ,12.55, 1); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (lastState==45){
			if( millis()-last_mt_button>2200 && (IRDepan<b_jarak_ir || IRBwhKa<b_jarak_ir)){
				state++;
			}
		}else{
			if( (IRDepan<b_jarak_ir || IRBwhKa<b_jarak_ir) ){
				state++;
			}
		}
	} else if (state==251){	// belok utk sanddune pivoting
		gerakRobot(0 ,-45 ,0.9 ,b_max_belok_tussock, 0); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (eTheta<=8.1){
			state++;
		}
	} else if (state==252){	// belok utk sanddune (dan naik sanddune)
		gerakRobot(b_speed_belok_tussock,-45 ,0.9 ,12.55, 1); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (pitch>=10){
			state++;
		}
	}else if (state==253){	// belok utk sanddune (dan naik sanddune)
		gerakRobot(b_speed_belok_tussock ,-48 ,0.9 ,2.0, 1); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (fabs(pitch)<5.8){
			state++;
		}
	}else if (state==254){	// belok utk sanddune (dan naik sanddune)
		gerakRobot(b_speed_dune,-90 ,0.9 ,b_max_belok_tussock, 0); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (eTheta<=8.1){
			state++;
		}
	}else if (state==255){
		gerakRobot(7.0,-90 ,0.9 ,12.55, 1); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if ((IRDepan<b_jarak_ir || IRBwhKa<b_jarak_ir) ){
			state++;
		}
	} else if (state==256){
		gerakRobot(b_speed_tussock,0.0 ,0.9 ,b_max_belok_tussock, 0); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (eTheta<=8.1){
			state++;
			last_mt_button=millis();
		}
	} else if (state==257){
		gerakRobot(b_speed_dune ,0.0 ,0.9 ,12.55, 0); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar

		if (!IRAtas && millis() - last_mt_button>800){
			state=0;
			last_mt_button=millis();
		}
	}else if (state==258){
		gerakRobot(b_speed_dune ,90.0 ,0.9 ,6.0, 0); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (fabs(pitch)>=6.0)
			state++;
	}else if (state==259){
		gerakRobot(b_speed_dune ,90.0 ,0.9 ,6.0, 0); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (fabs(pitch)<=3.5)
			state++;
	}
	else if (state==260){
		gerakRobot(b_speed_dune-1.5 ,90.0 ,0.9 ,6.0, 0); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (IRDepan<=80.0)
			state++;
	}else if (state==261){
		gerakRobot(0 ,90.0 ,0.9 ,6.0, 0); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		pneuNaik =1;
	}


	//////////////////////////
	///GERAK RETRY SANDDUNE///
	//////////////////////////

	else if (state==271){	// belok utk sanddune (dan naik sanddune)
		gerakRobot(2.0 ,-45 ,0.9 ,5.0, 1); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (eTheta<=8.1){
			state++;
		}
	} else if (state==272){	// belok utk sanddune (dan naik sanddune)
		gerakRobot(b_speed_dune ,-45 ,0.9 ,2.55, 1); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (pitch>=7){
			state++;
		}
	}else if (state==273){	// belok utk sanddune (dan naik sanddune)
		gerakRobot(b_speed_dune ,-48 ,0.9 ,2.0, 1); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (fabs(pitch)<3.8){
			state++;
		}
	}else if (state==274){	// belok utk sanddune (dan naik sanddune)
		gerakRobot(b_speed_dune ,-90 ,0.9 ,4.0, 0); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (eTheta<8.1){
			state++;
		}
	}else if (state==275){
		gerakRobot(b_speed_dune-1.5 ,-90 ,0.9 ,2.0, 1); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (IRDepan<batasTembok){
			state++;
		}
	} else if (state==276){
		gerakRobot(b_speed_tussock ,0.0 ,0.9 ,6.55, 0); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (!IRAtas && millis() - last_mt_button>800){
			state=0;
			last_mt_button=millis();
		}
	} else if (state==277){
		gerakRobot(0 ,90.0 ,0.9 ,6.0, 0); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (!IRAtas && millis() - last_mt_button>800){
			state++;
			last_mt_button=millis();
		}
	}else if (state==278){
		gerakRobot(b_speed_dune ,90.0 ,0.9 ,6.0, 0); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (fabs(pitch)>=6.0)
			state++;
	}else if (state==279){
		gerakRobot(b_speed_dune ,90.0 ,0.9 ,6.0, 0); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (fabs(pitch)<=3.5)
			state++;
	}
	else if (state==280){
		gerakRobot(b_speed_dune-1.5 ,90.0 ,0.9 ,6.0, 0); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (IRDepan<=80.0)
			state++;
	}else if (state==281){
		gerakRobot(0 ,90.0 ,0.9 ,6.0, 0); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		pneuNaik =1;
	}

	//////////////////////////
	///GERAK RETRY TUSSOCK ///
	//////////////////////////
	else if (state==284){	// belok utk sanddune (dan naik sanddune)
		gerakRobot(b_speed_dune ,-28 ,0.9 ,5.0, 1); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (eTheta<8.1){
			state++;
		}
	}else if (state==285){
		gerakRobot(5.0 ,-45 ,0.9 ,4.0, 1); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (IRDepan<batasTembok+10){
			state++;
		}
	} else if (state==286){
		gerakRobot(b_speed_dune ,45.0 ,0.9 ,6.55, 0); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (!IRAtas && millis() - last_mt_button>800){
			state=0;
			last_mt_button=millis();
		}
	} else if (state==287){
		gerakRobot(0 ,-135.0 ,0.9 ,6.0, 0); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
	}else if (state==288){
		gerakRobot(b_speed_dune ,-135.0 ,0.9 ,6.0, 0); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (fabs(pitch)>=6.0)
			state++;
	}else if (state==289){
		gerakRobot(b_speed_dune ,-135.0 ,0.9 ,6.0, 0); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (fabs(pitch)<=3.0)
			state++;
	}else if (state==290){
		gerakRobot(b_speed_dune-2.1 ,135.0 ,0.9 ,6.0, 0); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (IRDepan<=batasTembok)
			state++;
	}
}
void stateSwitch(){
	if (state==0){
		if (millis() - last_mt_button>445 && !button1){	//MODE MERAH
			state=1;
			last_mt_button = millis();
		}
		else if (millis() - last_mt_button>445 && !button2){	//MODE BIRU
			state=2;
			last_mt_button = millis();
		}
		else if (millis() - last_mt_button>445 && !button3){	//MODE LAIN
			state=3;
			last_mt_button = millis();
		}
		else if (millis() - last_mt_button>445 && !button4){	//MODE GRIPP + DEBUGGER klik selama 2 detik
			pneuGripper=1;
			state=99;
			last_mt_button = millis();
		}
	}
	else if (state==1){	//LAPANGAN MERAH
		theta0 = cmps.getAngle()/10;

		if (millis() - last_mt_button>445 && !button1){	//mode faster
			lastState=40;
			state=40;
			pneuGripper=1;
			last_mt_button = millis();
		}
		else if (millis() - last_mt_button>445 && !button2){	//mode safety
			state=100;
			lastState=100;
			last_mt_button = millis();
		}
		else if (millis() - last_mt_button>445 && !button3){	//mode retry sanddune
			state=150;
			lastState=150;
			last_mt_button = millis();
		}
		else if (millis() - last_mt_button>445 && !button4){	//mode retry tussock
			state=184;
			last_mt_button = millis();
		}
	}
	else if (state==2){	//LAPANGAN BIRU
		theta0 = cmps.getAngle()/10;

		if (millis() - last_mt_button>445 && !button1){	//mode faster
			state=45;
			lastState=45;
			pneuGripper=1;
			last_mt_button = millis();
		}
		else if (millis() - last_mt_button>445 && !button2){	//mode faster
			state=200;
			last_mt_button = millis();
		}
		else if (millis() - last_mt_button>445 && !button3){	//mode retry sandsune + safe (mulai 0 derajat)
			state=250;
			lastState=250;
			last_mt_button = millis();
		}
		else if (millis() - last_mt_button>445 && !button4){	//mode retry tussock safe
			state=284;
			last_mt_button = millis();
		}
	}
	else if (state==3){	//MODE LAIN
		theta0 = cmps.getAngle()/10;
		if (!IRAtas){	//naik mountain urtuu
			state=20;
		}
		if (millis() - last_mt_button>445 && !button1){	//belok kanan
			state=15;
			last_mt_button = millis();
		}
		else if (millis() - last_mt_button>445 && !button2){ //belok kiri
			state=16;
			last_mt_button = millis();
		}
		else if (millis() - last_mt_button>445 && !button3){ //belok kanan
			state=17;
			last_mt_button = millis();
		}
		else if (millis() - last_mt_button>445 && !button4){	//naik mountain urthuu
			state=20;
			last_mt_button = millis();
		}
	} else if (state==40){
		if(!IRKi || !IRKa){	//mode tercepat
			wait_ms(300);
			pneuGripper = 0;
			wait_ms(1000);
			state = 150;
			last_mt_button = millis();
		}
	} else if (state==45){
		if(!IRKa || !IRKi){	//mode tercepat
			wait_ms(300);
			pneuGripper = 0;
			wait_ms(1000);
			state = 250;
			last_mt_button = millis();
		}
	}
	else if (state==99){
		if (millis() - last_mt_button<2000 && millis()-last_mt_button >445 && !button4){
			state=999;
			last_mt_button = millis();
		}
		else if (millis() -last_mt_button>=2000){
			state=0;
			last_mt_button = millis();
		}
		else if (millis() - last_mt_button<2000 && millis()-last_mt_button >445 && !button3){
			state=888;
			last_mt_button = millis();
		}
	}
//	else if (!IRAtas && millis() - last_mt_button>445){
//		state=0;
//		last_mt_button=millis();
//	}
}
void getCMPS() { 	// dapatkan theta robot (theta robot/compass)
    prevTheta = theta;
	theta_temp = (cmps.getAngle()/10 - theta0);                //cari perubahan sudut badan
    if(theta_temp > 180.0 && theta_temp <= 360.0)             //kondisi agar pembacaan 0 - 180 derajat
        theta = (theta_temp - 360.0 );
    else if(theta_temp < -180.0 && theta_temp >= -360.0)
        theta = (theta_temp  + 360.0 );
    else
        theta = theta_temp;
    theta = theta*(-1);                                   //output theta
    theta = 0.6*theta + 0.4*prevTheta;
}
void getPitch(){	//dapatkan pitch robot
    int pitch_temp = cmps.getPitch() - pitch0;
    if (pitch_temp>= 128 && pitch_temp<=255){
        pitch = pitch_temp - 255;
    } else if (pitch_temp>= 128 && pitch_temp<=255){
        pitch = pitch_temp + 255;
    } else
        pitch = pitch_temp;
    pitch=pitch*-1;

}
void gerakMotor()
{
//	if (state==1){
//		speedA = 0.6;
//		speedB = 0.6;
//	} else if (state==2){
//		speedA = -0.6;
//		speedB = 0.6;
//	} else if (state==3){
//		speedA = 0.6;
//		speedB = -0.6;
//	}else if (state==4){
//		speedA = 0.5;
//		speedB = 0.5;
//	}
//    speedA=0.6;
//    speedB=0.6;
	if (speedA>pwmMax)
		speedA=pwmMax;
	else if (speedA<-1*pwmMax)
		speedA=-1*pwmMax;
	if (speedB>pwmMax+0.05)
		speedB=pwmMax+0.05;
	else if (speedB<-1*(pwmMax+0.05))
		speedB=-1*(pwmMax+0.05);

	if (fabs(speedB) > 0.15  && fabs(speedA) > 0.15 && state>0) {
        motorKa.speed(speedA);
        motorKi.speed(speedB);
    } else {
        motorKa.speed(0);
        motorKi.speed(0);
//        motorKa.brake(1);
//        motorKi.brake(1);
    }
}
void gerakRobot(float w_sp, float th_sp, float w_max, float wt_max, float n){
	theta_sp = th_sp;
	speed_sp = w_sp;
	eTheta = fabs(theta_sp-theta);
	//pwmMax = w_max;
	speedThetaMax = wt_max;
	mode = n;
}
void samplingEnc(){
	pulse_A = (double)encKanan.getPulses()*pulseToDeg;
	pulse_B = (double)encKiri.getPulses()*pulseToDeg;

	encKanan.reset();
	encKiri.reset();
//
//	pulseKa += pulse_A;
//	pulseKi += pulse_B;
//	if (pulseKa>360.0)
//		pulseKa-=360.0;
//	else if (pulseKa<-360.0)
//		pulseKa+=360.0;
//	if (pulseKi>360.0)
//		pulseKi-=360.0;
//	else if (pulseKi<-360.0)
//		pulseKi+=360.0;

}

void PIDMotor(){
	samplingEnc();
	if (mode==0){	//mode pivot
		if (fabs(theta_sp - theta)>=16){
			speed_sp = 0;
		}
	}

	speed_spA = speed_sp + speedTheta;
	speed_spB = speed_sp - speedTheta;
//	if (state>0){
//			speed_spA = 2.5;
//			speed_spB = 2.5;
//		}
	errA = (double) speed_spA - pulse_A;
	errB = (double) speed_spB - pulse_B;

	speedA = (double) ( prev_speedA
			+ Kp_A*(errA  - prevErrA)
			+ Ki_A*TS*(errA + prevErrA)/2 );
//			+ Kd_A*(errA - 2*prevErrA + p2_e_A) );
	speedB = (double) ( prev_speedB
			+ Kp_B*(errB  - prevErrB)
			+ Ki_B*TS*(errB + prevErrB)/2 );
	if (speedA>pwmMax)
		speedA = pwmMax;
	else if (speedA<-pwmMax)
		speedA = -pwmMax;
	if (speedB>pwmMax)
		speedB = pwmMax;
	else if (speedB<-pwmMax)
		speedB = -pwmMax;
	prev_speedA = speedA;
	prev_speedB = speedB;
	prevErrA = errA;
	prevErrB = errB;
}
void PIDTheta(){
	errorTheta = theta_sp - theta;
	speedTheta = KP*errorTheta;

	if (speedTheta>speedThetaMax){
		speedTheta = speedThetaMax;
	} else if (speedTheta<-1*speedThetaMax){
		speedTheta = -1*speedThetaMax;
	}

}
void printSerial(){
//	pc.printf(" state:%d\t speed sp:%.2f spA:%.2f spB:%.2f A:%.2f B:%.2f \t t:%.2f t_sp:%.2f\t Ka:%.2f Ki:%.2f %.2f %.2f\n",
//			 state,speed_sp,speed_spA,speed_spB,
//			speedA, speedB, theta,theta_sp,
//			pulseKa, IRDepan,//pulseKi,
//			errA,errB,
//	pc.printf("%.2f \t %.2f \t %.2f\n",
//			pulse_A, pulse_B, pulse_A-pulse_B);
	pc.printf("%.2f\t %.2f\t %.2f\t %.2f\t %.2f %d\n", IRBlkng,  IRDepan, IRBwhKa, IR3.read(),pitch, state);// IRDepan, IRBlkng, IRTest, pitch);//(double)IR1.read()*3.3,(double)IR2.read()*3.3);
//	pc.printf("%.2f\t\n", pulse_A-pulse_B);

}
void GP2A3(){
	//utk 5.5 meter 356 (2m akurat),
	m_slope3 = 70.5;//356;// 25.3;
    vDist3 = (double)IR3.read()* 3.3;
    IRBwhKa = m_slope3/vDist3;
    if (IRBwhKa>200)
    	IRBwhKa=200;
//    	IRDepan = 0.5*prevIRDepan + 0.5*IRDepan;
//        prevIRDepan = IRDepan;
}
void GP2A2(){
	//utk 5.5 meter 356 (2m akurat),
	 m_slope2 = 70.5;
    vDist2 = (double)IR4.read()* 3.3;
    IRBlkng = m_slope2/vDist2;
    if (IRBlkng>200)
		IRBlkng=200;
}
void GP2A1(){
	//utk 5.5 meter 356 (2m akurat),
	m_slope1 = 70.5;//356;// 25.3;
    vDist1 = (double)IR2.read()* 3.3;
    IRDepan = m_slope1/vDist1;
    if (IRDepan>200)
    	IRDepan=200;
//    	IRDepan = 0.5*prevIRDepan + 0.5*IRDepan;
//        prevIRDepan = IRDepan;
}


void stallSafety(){
	if (abs(pulse_A)<=0.05 && !stall){
		lastStall=millis();
		stall=1;
	}
	if (abs(pulse_A)>0.3 && stall){
		lastStall=millis();
		stall=0;
	}
	if (millis() - lastStall>2200 && stall){// && !nyangkut){
		ohstart=0;
		mode=0;
		state=-1;
		target_pulse=0;
	}
}

//Lampu nyala
void setLCD(){
	lcd.cls();
    lcd.setBacklight(TextLCD::LightOn);
}

//Prosedur Main LCD
void tampilLCD(){
	int resetmode = 0;
	if (state != 999 && state != 888){
//		if (state==150)
//	        lcd.printf("St: MerahSafe\n");
//		else if (state==250)
//	        lcd.printf("St: Biru Safe\n");
//		else if (state==184)
//			lcd.printf("St: M Tussock\n");
//		else if (state==284)
//			lcd.printf("St: B Tussock\n");
//		else
//			lcd.printf("State: %d\n");


        lcd.printf("State:%d \n",state);
        lcd.printf("Th:%.1f:%.1f\n",theta, pitch);
	}
	else if (state == 888){
		lcd.printf("Ki:%.2f\n",IRDepan);
		lcd.printf("Ka:%.2f\n",IRBwhKa);
	}
    else if (state == 999){
        lcd.printf("Mode:Tuning\n");
        lcd.printf("Mer Bir\n\n");
        while((button1||button4||button3||button4) && (millis()-last_mt_button > 200)){
            if (millis() - last_mt_button>200 && !button1){     //Lapangan Merah
                resetmode=1;
                last_mt_button = millis();
            }
            if (millis() - last_mt_button>200 && !button2){     //Lapangan Biru
                resetmode=2;
                last_mt_button = millis();
            }
        }
        lcd.cls();
        switch(resetmode){
            case 1 : {
            	//Tuning Lapangan Merah
            	// k = 0 => m_speed maju
            	// k = 1 => m_jarak_ir
            	// k = 2 => m_speed_dune
            	// k = 3 => m_max_dune
            	// k = 4 => m_speed_belok_tussock
            	// k = 5 =>	m_max_belok_tussock
            	// k = 6 => m_etheta
            	// k = 7 => m_max_tussock
            	// k = 8 => m_speed_tussock
            	// k = 9 => m_IRTembok
                int k = 0;
                while(button4 ){
                    switch(k){
                        case 0 :
                            rom.read(0,m_speed_maju);
                            lcd.printf("1.a_start= %.2f",m_speed_maju);
                            lcd.printf("\nLap Merah\n");
                        break;
                        case 1 :
                            rom.read(4,m_jarak_ir);
                            lcd.printf("2.IRMj= %.2f",m_jarak_ir);
                            lcd.printf("\nLap Merah\n");
                        break;
                        case 2 :
                            rom.read(8,m_speed_dune);
                            lcd.printf("3.Sp Cepat=%.2f",m_speed_dune);
                            lcd.printf("\nLap Merah\n");
                        break;
                        case 3 :
                            rom.read(12,m_max_dune);
                            lcd.printf("4.Bel SD=%.2f",m_max_dune);
                            lcd.printf("\nLap Merah\n");
                        break;
                        case 4 :
                            rom.read(16,m_speed_belok_tussock);
                            lcd.printf("5.Sp Pelan=%.2f",m_speed_belok_tussock);
                            lcd.printf("\nLap. Merah\n");
                        break;
                        case 5 :
                            rom.read(20,m_max_belok_tussock);
                            lcd.printf("6.Sp Belok=%.2f",m_max_belok_tussock);
                            lcd.printf("\nLap Merah\n");
                        break;
                        case 6 :
                            rom.read(24,m_etheta);
                            lcd.printf("7.IR Tuss= %.2f",m_etheta);
                            lcd.printf("\nLap Merah\n");
                        break;
                        case 7 :
                            rom.read(28,m_max_tussock);
                            lcd.printf("8.Wig Ts= %.2f\n",m_max_tussock);
                            lcd.printf("Lap Merah\n");
                        break;
                        case 8 :
							rom.read(64,m_speed_tussock);
							//lcd.cls();
							lcd.printf("9.sp Ts= %.2f\n",m_speed_tussock);
							lcd.printf("Lap Merah\n");
						break;
                        case 9 :
							rom.read(68,m_IR2);
							//lcd.cls();
							lcd.printf("10.IR2= %.2f",m_IR2);
							lcd.printf("\nLap Merah\n");
						break;
                    }
                    if(!button1 && millis()-last_mt_button >200){
                        switch(k){
							case 0 :
								m_speed_maju += 0.5;
								rom.write(0,m_speed_maju);
							break;
							case 1 :
								m_jarak_ir += 1;
								rom.write(4,m_jarak_ir);
							break;
							case 2 :
								m_speed_dune += 0.5;
								rom.write(8,m_speed_dune);
							break;
							case 3 :
								m_max_dune += 0.1;
								rom.write(12,m_max_dune);
							break;
							case 4 :
								m_speed_belok_tussock += 0.5;
								rom.write(16,m_speed_belok_tussock);
							break;
							case 5 :
								m_max_belok_tussock += 0.1;
								rom.write(20,m_max_belok_tussock);
							break;
							case 6 :
								m_etheta += 0.1;
								rom.write(24,m_etheta);
							break;
							case 7 :
								m_max_tussock += 0.1;
								rom.write(28,m_max_tussock);
							break;
							case 8 :
								m_speed_tussock += 0.5;
								rom.write(64,m_speed_tussock);
							break;
							case 9 :
								m_IR2 += 1;
								rom.write(68,m_IR2);
							break;
                        }
                        last_mt_button = millis();
                        lcd.cls();
                    }
                    else if(!button2 && millis()-last_mt_button >200){
                        switch(k){
							case 0 :
								m_speed_maju -= 0.5;
								rom.write(0,m_speed_maju);
							break;
							case 1 :
								m_jarak_ir -= 1;
								rom.write(4,m_jarak_ir);
							break;
							case 2 :
								m_speed_dune -= 0.5;
								rom.write(8,m_speed_dune);
							break;
							case 3 :
								m_max_dune -= 0.1;
								rom.write(12,m_max_dune);
							break;
							case 4 :
								m_speed_belok_tussock -= 0.5;
								rom.write(16,m_speed_belok_tussock);
							break;
							case 5 :
								m_max_belok_tussock -= 0.1;
								rom.write(20,m_max_belok_tussock);
							break;
							case 6 :
								m_etheta -= 0.1;
								rom.write(24,m_etheta);
							break;
							case 7 :
								m_max_tussock -= 0.1;
								rom.write(28,m_max_tussock);
							break;
							case 8 :
								m_speed_tussock -= 0.5;
								rom.write(64,m_speed_tussock);
							break;
							case 9 :
								m_IR2 -= 1;
								rom.write(68,m_IR2);
							break;
                        }
                        last_mt_button = millis();
                        lcd.cls();
                    }
                    else if(!button3 && millis()-last_mt_button >200){
                        k++;
                        if(k>9){
                            k=0;
                        }
                        last_mt_button = millis();
                        lcd.cls();
                    }
                }
                last_mt_button = millis();
            }
            break;
            case 2 : {
            	//Tuning Lapangan Biru
            	// k = 0 => b_speed maju
            	// k = 1 => b_jarak_ir
            	// k = 2 => b_speed_dune
            	// k = 3 => b_max_dune
            	// k = 4 => b_speed_belok_tussock
            	// k = 5 =>	b_max_belok_tussock
            	// k = 6 => b_etheta
            	// k = 7 => b_max_tussock
            	// k = 8 => b_speed_tussock
            	// k = 9 => b_IRTembok
                int k = 0;
                while(button4){
                    switch(k){
						case 0 :
							rom.read(32,b_speed_maju);
							lcd.printf("1.Sp Maj= %.2f",b_speed_maju);
							lcd.printf("\nLap Biru\n");
						break;
						case 1 :
							rom.read(36,b_jarak_ir);
							lcd.printf("2.IRMj= %.2f",b_jarak_ir);
							lcd.printf("\nLap Biru\n");
						break;
						case 2 :
							rom.read(40,b_speed_dune);
							lcd.printf("3.Sp SD= %.2f",b_speed_dune);
							lcd.printf("\nLap Biru\n");
						break;
						case 3 :
							rom.read(44,b_max_dune);
							lcd.printf("4.Bel SD= %.2f",b_max_dune);
							lcd.printf("\nLap Biru\n");
						break;
						case 4 :
							rom.read(48,b_speed_belok_tussock);
							lcd.printf("5.Sp Pelan=%.2f",b_speed_belok_tussock);
							lcd.printf("\nLap Biru\n");
						break;
						case 5 :
							rom.read(52,b_max_belok_tussock);
							lcd.printf("6.Sp Blk= %.2f",b_max_belok_tussock);
							lcd.printf("\nLap Biru\n");
						break;
						case 6 :
							rom.read(56,b_etheta);
							lcd.printf("7.IR Tuss= %.2f",b_etheta);
							lcd.printf("\nLap Biru\n");
						break;
						case 7 :
							rom.read(60,b_max_tussock);
							lcd.printf("8.Wig Ts= %.2f",b_max_tussock);
							lcd.printf("\nLap Biru\n");
						break;
                        case 8 :
							rom.read(72,b_speed_tussock);
							//lcd.cls();
							lcd.printf("9.sp Ts= %.2f\n",b_speed_tussock);
							lcd.printf("Lap Biru\n");
						break;
                        case 9 :
							rom.read(76,b_IR2);
							//lcd.cls();
							lcd.printf("10.IR2= %.2f\n",b_IR2);
							lcd.printf("Lap Biru\n");
						break;
                    }
                    if(!button1 && millis()-last_mt_button >200){
                        switch(k){
                        	case 0 :
								b_speed_maju += 0.5;
								rom.write(32,b_speed_maju);
							break;
							case 1 :
								b_jarak_ir += 1;
								rom.write(36,b_jarak_ir);
							break;
							case 2 :
								b_speed_dune += 0.5;
								rom.write(40,b_speed_dune);
							break;
							case 3 :
								b_max_dune += 0.1;
								rom.write(44,b_max_dune);
							break;
							case 4 :
								b_speed_belok_tussock += 0.5;
								rom.write(48,b_speed_belok_tussock);
							break;
							case 5 :
								b_max_belok_tussock += 0.1;
								rom.write(52,b_max_belok_tussock);
							break;
							case 6 :
								b_etheta += 0.1;
								rom.write(56,b_etheta);
							break;
							case 7 :
								b_max_tussock += 0.1;
								rom.write(60,b_max_tussock);
							break;
							case 8 :
								b_speed_tussock += 0.5;
								rom.write(72,b_speed_tussock);
							break;
							case 9 :
								b_IR2 += 1;
								rom.write(76,b_IR2);
							break;
                        }
                        last_mt_button = millis();
                        lcd.cls();
                    }
                    else if(!button2 && millis()-last_mt_button >200){
                        switch(k){
							case 0 :
								b_speed_maju -= 0.5;
								rom.write(32,b_speed_maju);
							break;
							case 1 :
								b_jarak_ir -= 1;
								rom.write(36,b_jarak_ir);
							break;
							case 2 :
								b_speed_dune -= 0.5;
								rom.write(40,b_speed_dune);
							break;
							case 3 :
								b_max_dune -= 0.1;
								rom.write(44,b_max_dune);
							break;
							case 4 :
								b_speed_belok_tussock -= 0.5;
								rom.write(48,b_speed_belok_tussock);
							break;
							case 5 :
								b_max_belok_tussock -= 0.1;
								rom.write(52,b_max_belok_tussock);
							break;
							case 6 :
								b_etheta -= 0.1;
								rom.write(56,b_etheta);
							break;
							case 7 :
								b_max_tussock -= 0.1;
								rom.write(60,b_max_tussock);
							break;
							case 8 :
								b_speed_tussock -= 0.5;
								rom.write(72,b_speed_tussock);
							break;
							case 9 :
								b_IR2 -= 1;
								rom.write(76,b_IR2);
							break;
                        }
                        last_mt_button = millis();
                        lcd.cls();
                    }
                    if(!button3 && millis()-last_mt_button >200){
                        k++;
                        if(k>9){
                            k=0;
                        }
                        last_mt_button = millis();
                        lcd.cls();
                    }
                }
                last_mt_button = millis();
            }
            break;
        }
    }
}

void zeroEEPROM(){
	uint32_t h;
	float zero =0;
	for (h=0;h<=128;h= h+4){
		rom.write(h,zero);
	}
}

void assignDefaultValueEEPROM(){
	rom.write(0,(float)8);
	rom.write(4,(float)155);
	rom.write(8,(float)8);
	rom.write(12,(float)3);
	rom.write(16,(float)8);
	rom.write(20,(float)3);
	rom.write(24,(float)7.8);
	rom.write(28,(float)7.55);

	rom.write(32,(float)8);
	rom.write(36,(float)155);
	rom.write(40,(float)8);
	rom.write(44,(float)3);
	rom.write(48,(float)8);
	rom.write(52,(float)3);
	rom.write(56,(float)7.8);
	rom.write(60,(float)7.55);
}

void assignValue(){
	rom.read(0,m_speed_maju);
	rom.read(4,m_jarak_ir);
	rom.read(8,m_speed_dune);
	rom.read(12,m_max_dune);
	rom.read(16,m_speed_belok_tussock);
	rom.read(20,m_max_belok_tussock);
	rom.read(24,m_etheta);
	rom.read(28,m_max_tussock);

	rom.read(32,b_speed_maju);
	rom.read(36,b_jarak_ir);
	rom.read(40,b_speed_dune);
	rom.read(44,b_max_dune);
	rom.read(48,b_speed_belok_tussock);
	rom.read(52,b_max_belok_tussock);
	rom.read(56,b_etheta);
	rom.read(60,b_max_tussock);
}

/*
 * MERAH:
 * 1. A_START = 5
 * 2. IR MJ = 109
 * 3. SP CEPAT = 15
 * 4. BEL SD = 3.8
 * 5. SP PELAN = 8
 * 6. SP BELOK = 4.30
 * 7. IR TUSS = 7.8
 * 8. WIG TS = 7.5
 * 9. SP_TS = 8
 * 10. IR2 = 70
 *
 * BIRU :
 * 1. SP MAJ = 0
 * 2. IR MJ = 104
 * 3. SP SD = 15
 * 4. BEL SD = 2
 * 5. SP PELAN = 7.5
 * 6. SP BLK = 4
 * 7. IR TUSS = 7.8
 * 8. WIG TS = 7.5
 * 9. SP TS = 8.0
 * 10. IR2 = 83
 *
 */

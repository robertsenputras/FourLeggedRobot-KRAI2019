/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  *
 *        HORSY Nasional KRAI 2018                                              *
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
 *  KATA KUNCI : (cari lewat search)											*
 *  - DEBUGGINGMODE                                                 			*
 *  - MERAHMODE                                                 				*
 *  - BIRUMODE                                                 					*
 *  - RETRYMODE                                                 				*
 *  - CEKFASA                                                 					*
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


//DEKLARASI LIBRARY
#include "mbed.h"
#include "millis.h"
#include "Motor.h"
#include "encoderKRAI.h"
#include "CMPS12_KRAI.h"
#include "variable.h"
#include "PID.h"
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

AnalogIn IR1(PA_0);
AnalogIn IR2(PC_0);
AnalogIn IR3(PA_1);
AnalogIn IR4(PC_1);

//pneu benar
DigitalOut pneuGripper(PC_7);
DigitalOut pneuNaik(PC_8);
//DigitalOut pneuC(PC_9); // benar

//DigitalIn switchKa(PC_0,PullUp);
//DigitalIn trigDepKa(PA_15,PullUp); //pc_0
////DigitalIn trigDepKi(PA_0,PullUp); //pd_2 PC_13
//DigitalIn trigBlkKa(PH_1,PullUp);
//DigitalIn trigBlkKi(PA_14,PullUp);
//DigitalIn C(PA_14,PullUp);
//DigitalIn F(PB_7,PullUp);
//DigitalIn Ubutton(USER_BUTTON);
//DigitalIn IRgerege(PD_2);					//ir deteksi gerege
//DigitalIn IRmount(PA_9);					//deteksi jarak depan mountain
//DigitalOut pneu1[2] = {(PC_5),(PC_8)};          //kanan belakang , kiri depan //pc7-pc5
//DigitalOut pneu2[2] = {(PC_6),(PC_9)};           //kiri belakang, kanan depan 12
//DigitalOut pneu3[2] = {(PC_7),(PB_6)};           //mount gerege
//HCSR04 pingKidep(PB_4, PA_8);

void FSM();
void gerakMotor();
void gerakRobot(float w_sp, float th_sp, float w_max, float wt_max, float mode);
void getCMPS();
void getPitch();
void GP2A1();
void GP2A2();
void initMain();
void lapanganMerah();
void lapanganBiru();
void PIDMotor();
void PIDTheta();
void printSerial();
void samplingEnc();
void stateSwitch();
void stallSafety();

float fabsError(float a, float b);

int main() {
	startMillis();
	state=0;
	last_mt_PID=1;
	theta=0;
	theta0 = cmps.getAngle()/10;
	pitch0 = cmps.getPitch();
	theta_sp=0;
	pneuGripper=0;
	pneuNaik=0;
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
//    	stateSwitch();
//    	if (state==1){
    	if (millis() - last_mt_switch>=71){
			stateSwitch();
			GP2A1();
			last_mt_switch = millis();
		}
//    	}else{
    	if (millis() - last_mt_IR>=131){
    		GP2A2();
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
        if (millis()- last_mt_print >= 207){
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
		gerakRobot(0 ,0 ,0.9 ,12.55, 0); //speed_sp, theta_sp, pwmMax, speedThetaMax, mude putar
	} else if (state==1){
		//gerak maju
		gerakRobot(4.0 ,0 ,0.9 ,4.0, 0); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar

	//		if (eTheta<=8.1)
	//			state=0;
	} else if (state==2){
		gerakRobot(8.0 ,1.0 ,0.9 ,12.55, 0); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (pitch>=7){
			state=15;
		}
	} else if (state==3){
		//gerak belok kanan 45
		gerakRobot(8.0 ,12.0 ,0.9 ,3.5, 1); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (eTheta<=8.1)
			state=14;
	} else if (state==4){
		gerakRobot(0 ,90 ,0.9 ,4.0, 0); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (eTheta<=8.1)
			state=0;
	} else if (state==5){
		//gerak naik Uukhai
		gerakRobot(5 ,-90 ,0.9 ,4.0, 0); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (eTheta<=8.1)
			state=0;
	} else if (state==14){
		gerakRobot(8.0 ,-45 ,0.9 ,3.5, 1); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar

	}else if (state==15){
		gerakRobot(5.0 ,0 ,0.9 ,3.5, 1); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (fabs(pitch)<2.0){
			state=16;
		}
	} else if (state==16){
		gerakRobot(8.0 ,0 ,0.9 ,3.5, 1); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (IRBlkng>60.0){
			state=17;
		}
	}
	else if (state==17){
		gerakRobot(8.0 ,-45 ,0.9 ,3.5, 1); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
	}
	else if (state<200){
		lapanganMerah();
	}
	else if (state<300){
		lapanganBiru();
	}

}
void lapanganMerah(){
	target=8.0;
	if (state==100){	// maju normal
		gerakRobot(target ,0 ,0.9 ,12.55, 1); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (IRDepan<170.0){
			state++;
		}
	} else if (state==101){	// belok utk sanddune (dan naik sanddune)
		gerakRobot(target ,47.2 ,0.9 ,12.55, 0); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (pitch>=7){
			state++;
		}
	} else if (state==102){
		gerakRobot(target ,47.2 ,0.9 ,3.5, 1); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (fabs(pitch)<2.0){
			state++;
		}
	} else if (state==103){
		gerakRobot(target ,45 ,0.9 ,3.5, 1); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (IRBlkng>70.0){
			state++;
		}
	}
	else if (state==104){
		gerakRobot(8.0 ,-2.0 ,0.9 ,3.5, 1); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (eTheta<8.1){
			state++;
		}
	}else if (state==105){
		gerakRobot(8.0 ,0 ,0.9 ,3.5, 0); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
	}

}
void lapanganBiru(){
	target=1*8.0;
	if (state==200){	// maju normal
		gerakRobot(target ,0 ,0.9 ,12.55,1); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (IRBlkng > 155.0){
			state++;
		}
	} else if (state==201){	// belok utk sanddune (dan naik sanddune)
		gerakRobot(target ,-45 ,0.9 ,3.0, 1); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (pitch>=7 && pitch < 50){	//deteksi turun
			state++;
		}
	} else if (state==202){
		gerakRobot(target ,-47.2 ,0.9 ,3.5, 1); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
		if (fabs(pitch)<3.5){	//deteksi datar setelah turun
			state++;
		}
	} else if (state==203){	//deteksi IR blkng stlh sanddune
		gerakRobot(target , 0 , 0.9 , 3 ,1); //speed_sp, theta_sp -70, pwmMax, speedThetaMax, mode putar
//		if (IRBlkng>90.0){
//			state++;
//		}
		if (eTheta<5.8){
			state++;
		}
	}
	else if (state==204){	//utk belok ke arah tussock
		gerakRobot(target ,0 ,0.9 ,7.55, 0); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
//		if (eTheta<8.1){
//			state++;
//		}
	}else if (state==205){
		gerakRobot(target ,0 ,0.9 ,6.55, 0); //speed_sp, theta_sp, pwmMax, speedThetaMax, mode putar
	}

}
void stateSwitch(){
	if (state==1 && millis() - last_mt_button>800 && !button1){
		state=0;
		last_mt_button = millis();
	} else if (millis() - last_mt_button>800 && !button1){
		state=1;
		last_mt_button = millis();
	}
	if (state==2 && millis() - last_mt_button>800 && !button2){
		state=12;
		last_mt_button = millis();

	} else if (millis() - last_mt_button>800 && !button2){
		state=200;
		last_mt_button = millis();
		last_mt_sp=millis();
	}
	if (state==4 && millis() - last_mt_button>800 && !button3){
		state=14;
		last_mt_button = millis();
	} else if (millis() - last_mt_button>800 && !button3){
		state=4;
		last_mt_button = millis();
	}
	if (state==5 && millis() - last_mt_button>800 && !button4){
		state=15;
		last_mt_button = millis();
	} else
	if (millis() - last_mt_button>800 && !button4){
		state=5;
		last_mt_button = millis();
	}

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
	if (speedA>0.9)
		speedA=0.9;
	else if (speedA<-0.9)
		speedA=-0.9;
	if (speedB>0.9)
		speedB=0.9;
	else if (speedB<-0.9)
		speedB=-0.9;

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
		if (fabs(theta_sp - theta)>20){
			speed_sp = 0;
		}
	}
	speed_spA = speed_sp + speedTheta;
	speed_spB = speed_sp - speedTheta;

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
////			errA,errB,
//			pulse_A, pulse_B);
	pc.printf("%.2f\t %.2f\t %.2f\t %.2f\t %.2f %d\n", IRBlkng, IR2.read(), IRDepan, IR4.read(),pitch, state);// IRDepan, IRBlkng, IRTest, pitch);//(double)IR1.read()*3.3,(double)IR2.read()*3.3);
//	pc.printf("%.2f\t\n", pulse_A-pulse_B);

}

void GP2A2(){
	//utk 5.5 meter 356 (2m akurat),
	 m_slope2 = 70.5;
    vDist2 = (double)IR4.read()* 3.3;
    IRBlkng = m_slope2/vDist2;

}
void GP2A1(){
	//utk 5.5 meter 356 (2m akurat),
	m_slope1 = 356;// 25.3;
    vDist1 = (double)IR2.read()* 3.3;
    IRDepan = m_slope1/vDist1;

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

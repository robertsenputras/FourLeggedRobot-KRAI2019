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
#define toleranceT 5.0
#define headingBiru 60
#define headingMerah -59

PIDT pidA(1,1,1,1,100);
Serial pc(USBTX, USBRX, 115200);
CMPS12_KRAI cmps(PB_3, PB_10, 0xc0);
//encoderKRAI encKanan(PC_14,PC_15, 538, encoderKRAI::X4_ENCODING);  //encoder motor steering
encoderKRAI encKanan(PC_2,PC_3, 538, encoderKRAI::X4_ENCODING);
encoderKRAI encKiri(PC_10,PC_11, 538, encoderKRAI::X4_ENCODING);
Motor motorSteer(PB_14, PC_4, PB_13);   //motor arah putaran CCW ++
Motor motorKi(PA_11,  PB_12,PA_7);                  //motor kanan
Motor motorKa(PA_5, PA_6, PA_12);                    //motor kiri

//DigitalIn switchKa(PC_0,PullUp);
//DigitalIn trigDepKa(PA_15,PullUp); //pc_0
////DigitalIn trigDepKi(PA_0,PullUp); //pd_2 PC_13
//DigitalIn trigBlkKa(PH_1,PullUp);
//DigitalIn trigBlkKi(PA_14,PullUp);

DigitalIn button1(PB_8,PullUp);	//HIJAU
DigitalIn button2(PA_13,PullUp);	//MERAH
DigitalIn button3(PA_10,PullUp); //KUNING
DigitalIn button4(PB_5,PullUp);	//BIRU

AnalogIn IR1(PA_0);
//AnalogIn IR4(PC_0);
//DigitalIn C(PA_14,PullUp);
//DigitalIn F(PB_7,PullUp);
//DigitalIn Ubutton(USER_BUTTON);
//DigitalIn IRgerege(PD_2);					//ir deteksi gerege
//DigitalIn IRmount(PA_9);					//deteksi jarak depan mountain
//DigitalOut pneu1[2] = {(PC_5),(PC_8)};          //kanan belakang , kiri depan //pc7-pc5
//DigitalOut pneu2[2] = {(PC_6),(PC_9)};           //kiri belakang, kanan depan 12
//DigitalOut pneu3[2] = {(PC_7),(PB_6)};           //mount gerege

//HCSR04 pingKidep(PB_4, PA_8);

void printSerial();
void initMain();
void PIDMotor();
void hitungParameter();
void getCMPS();
void getPitch();
void gerakMotor();
void FSM();
void stateSwitch();
void stallSafety();
void debugMode();
void GP2A();
void gerakRobot();
void PIDTheta();

int main() {
	startMillis();
	state=0;
	last_mt_PID=1;
	theta=0;
	theta0 = cmps.getAngle()/10;
	theta_sp=0;
	double tes = pidA.createpwm(5,1);
    while(1) {
    	if (millis() - last_mt_PID>=9){
//    		PIDTheta();
    		PIDMotor();
    		last_mt_PID = millis();
    	}
//    		pulse_A = encKanan.getPulses();
//    		        pulse_B = encKiri.getPulses();
    	if (millis() - last_mt_PIDTheta>=13){
    		if (fabs(theta-theta_sp)>toleranceT){
				PIDTheta();
			} else {
				speedTheta=0;
			}
    		last_mt_PIDTheta = millis();
    	}
    	if (millis() - last_mt_motor>=2){
			gerakMotor();
			last_mt_motor = millis();
		}
    	if (millis() - last_mt_cmps >= 53){
    		getCMPS();
    		last_mt_cmps = millis();
    	}
        if (millis()- last_mt_print >= 207){
			last_mt_print = millis();
			printSerial();
		}
        if (millis() - last_mt_switch>=29){
        	stateSwitch();
        	last_mt_switch = millis();
        }
//        if (millis() - last_mt_button>1500){
//        	last_mt_button = millis();
//        	theta_sp+=15;
//        }
        if (millis() - last_mt_FSM>=17){
        	FSM();
        	last_mt_FSM=millis();
        }
    }
}
void FSM(){
	int target =9;
	if (state==0){
		//keadaan diam
		speed_sp=0;
		theta_sp=0;
	} else if (state==1){
		//gerak maju
		speed_sp = 5.5;
		theta_sp = 0;
	} else if (state==2){
		//gerak maju belok kiri 45
		speed_sp = 0.0; //0; utk belok d tempat
		theta_sp = 160;
		if (fabs(theta_sp - theta)<=6.0)
			state=0;
	} else if (state==3){
		//gerak belok kanan 45
		speed_sp = 3.0;	//0; utk belok d tmpt
		theta_sp = -45.0;
	} else if (state==4){
		//gerak naik Uukhai
		speed_sp = 7;
		theta_sp = 45.0;
	} else if (state==5){
		//gerak naik Uukhai
		speed_sp = 7;
		theta_sp = -45.0;
	} else if (state==10){	// maju normal
		//gerak maju
		speed_sp = target;
		theta_sp = 0;
	} else if (state==11){	// belok utk sanddune
		//gerak maju
		speed_sp = 0;
		theta_sp = 47.0;
	} else if (state==12){	//stlh sanddune
		//
		speed_sp = 12;
		theta_sp = 0;
	} else if (state==13){	//meluruskan ke tali
		speed_sp = 0;
		theta_sp = -3;
	} else if (state==14){	//lewat tali
		speed_sp = 0;
		theta_sp = 179;
	} else if (state==15){	//lewat tali
		speed_sp = 0;
		theta_sp = -179;
	}else if (state==101){
		speed_sp=target;
		theta_sp=0;
		if (millis() - last_mt_sp>2000){
			state++;
			last_mt_sp=millis();
		}
	}else if (state==102){
		speed_sp=target;
		theta_sp=90;
		if (millis() - last_mt_sp>500 && fabs(theta_sp-theta)<8.0){
			state++;
			last_mt_sp=millis();
		}
	}else if (state==103){
		speed_sp=target;
		theta_sp=90;
		if (millis() - last_mt_sp>2000 && fabs(theta_sp-theta)<8.0){
			state++;
			last_mt_sp=millis();
		}
	}else if (state==104){
		speed_sp=target;
		theta_sp=0;
		if (millis() - last_mt_sp>500 && fabs(theta_sp-theta)<8.0){
			state++;
			last_mt_sp=millis();
		}
	} else if (state==105){
		speed_sp=target;
		theta_sp=0;
		if (millis() - last_mt_sp>2000 && fabs(theta_sp-theta)<8.0){
			state++;
			last_mt_sp=millis();
		}
	}else if (state==106){
		speed_sp=target;
		theta_sp=-90;
		if (millis() - last_mt_sp>500 && fabs(theta_sp-theta)<8.0){
			state++;
			last_mt_sp=millis();
		}
	}else if (state==107){
		speed_sp=target;
		theta_sp=-90;
		if (millis() - last_mt_sp>2000 && fabs(theta_sp-theta)<8.0){
			state++;
			last_mt_sp=millis();
		}
	} else if (state==108){
		speed_sp=-1*target;
		theta_sp=0;
		if (millis() - last_mt_sp>500 && fabs(theta_sp-theta)<8.0){
			state++;
			last_mt_sp=millis();
		}
	}else if (state==109){
		speed_sp=-1*target;
		theta_sp=0;
		if (millis() - last_mt_sp>2000 && fabs(theta_sp-theta)<8.0){
			state=0;
			last_mt_sp=millis();
		}
	} else if (state==201){
		speed_sp=target;
		theta_sp=0;
		if (millis() - last_mt_sp>1000){
			state++;
			last_mt_sp=millis();
		}
	}else if (state==202){
		speed_sp=target;
		theta_sp=45.0;
		if (millis() - last_mt_sp>500 && fabs(theta_sp-theta)<8.0){
			state++;
			last_mt_sp=millis();
		}
	}else if (state==203){
		speed_sp=target;
		theta_sp=45;
		if (millis() - last_mt_sp>1500 && fabs(theta_sp-theta)<8.0){
			state++;
			last_mt_sp=millis();
		}
	}else if (state==204){
		speed_sp=target;
		theta_sp=0;
		if (millis() - last_mt_sp>500 && fabs(theta_sp-theta)<8.0){
			state++;
			last_mt_sp=millis();
		}
	} else if (state==205){
		speed_sp=target;
		theta_sp=0;
		if (millis() - last_mt_sp>1500 && fabs(theta_sp-theta)<8.0){
			state++;
			last_mt_sp=millis();
		}
	}else if (state==206){
		speed_sp=target;
		theta_sp=-45.0;
		if (millis() - last_mt_sp>500 && fabs(theta_sp-theta)<8.0){
			state++;
			last_mt_sp=millis();
		}
	}else if (state==207){
		speed_sp=target;
		theta_sp=-45.0;
		if (millis() - last_mt_sp>1500 && fabs(theta_sp-theta)<8.0){
			state++;
			last_mt_sp=millis();
		}
	} else if (state==208){
		speed_sp=-1*target;
		theta_sp=0;
		if (millis() - last_mt_sp>500 && fabs(theta_sp-theta)<8.0){
			state++;
			last_mt_sp=millis();
		}
	}else if (state==209){
		speed_sp=-1*target;
		theta_sp=0;
		if (millis() - last_mt_sp>1500 && fabs(theta_sp-theta)<8.0){
			state=0;
			last_mt_sp=millis();
		}
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
		state=2;
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
void gerakRobot(float theta_sp,float speed_sp){
	if (fabs(theta-theta_sp)>toleranceT){
		PIDTheta();
	}
}
void PIDMotor(){
        pulse_A = (double)encKanan.getPulses()*pulseToDeg;
        pulse_B = (double)encKiri.getPulses()*pulseToDeg;
        encKanan.reset();
        encKiri.reset();

        pulseKa += pulse_A;
        pulseKi += pulse_B;
        if (pulseKa>360.0)
        	pulseKa-=360.0;
        else if (pulseKa<-360.0)
			pulseKa+=360.0;
        if (pulseKi>360.0)
			pulseKi-=360.0;
		else if (pulseKi<-360.0)
			pulseKi+=360.0;
        if (fabs(theta_sp - theta)>20){
			speed_sp = 0;
        }
        speed_spA = speed_sp + speedTheta;
        speed_spB = speed_sp - speedTheta;
        errA = (double) speed_spA - pulse_A;
        errB = (double) speed_spB - pulse_B;


        speedA = (double) ( prev_speedA
                + Kp_A*(errA  - prevErrA)
                + Ki_A*TS*(errA + prevErrA)/2 );
//                + Kd_A*(errA - 2*prevErrA + p2_e_A) );
        speedB = (double) ( prev_speedB
				+ Kp_B*(errB  - prevErrB)
				+ Ki_B*TS*(errB + prevErrB)/2 );

        if(fabs(errA)<=0.1 || fabs(errB)<=0.1){
                	speedA=0;
                	speedB=0;
                }
        if (speedA>pwmMax)
            speedA = pwmMax;
        else if (speedA<-pwmMax)
            speedA = -pwmMax;
        if (speedB>pwmMax)
			speedB = pwmMax;
		else if (speedB<-pwmMax)
			speedB = -pwmMax;
//        if (!ohstart){
//            speedA=0.0;
//            speedB=0.0;
//        }
        prev_speedA = speedA;
        prev_speedB = speedB;
        //p2_e_A = prevErrA;
        prevErrA = errA;
        prevErrB = errB;
}
void PIDTheta(){
	errorTheta = theta_sp - theta;
//	if (fabs(errorTheta)>errorThetaMax)
//		errorTheta = errorThetaMax;
//	else if (fabs(errorTheta)<-1*errorThetaMax)
//		errorTheta = -1* errorThetaMax;
	speedTheta = KP*errorTheta;

	if (fabs(speed_sp)<=0.1)
		speedThetaMax=12.5;
	else
		speedThetaMax = 8.5;
	if (speedTheta>speedThetaMax){
		speedTheta = speedThetaMax;
	} else if (speedTheta<-1*speedThetaMax){
		speedTheta = -1*speedThetaMax;
	}
//	if (state==2){
//		speedTheta=8.0;
//	}
//	speedTheta=6.0;
	//if (fabs(errorTheta)>toleranceT){	//mungkin terbalik

	//}
//	} else{
//		speed_spA = speed_sp + speedTheta;
//		speed_spB = speed_sp - speedTheta;
//	}

}
void printSerial(){
	pc.printf(" state:%d\t speed sp:%.2f spA:%.2f spB:%.2f A:%.2f B:%.2f \t t:%.2f t_sp:%.2f\t Ka:%.2f Ki:%.2f %.2f %.2f\n",
			 state,speed_sp,speed_spA,speed_spB,
			speedA, speedB, theta,theta_sp,
			pulseKa, pulseKi,
//			errA,errB,
			pulse_A, pulse_B);
//	pc.printf("%.2f\t\n", pulse_A-pulse_B);

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


void GP2A()
{
	double m_slope = 23.4;
    double vDist = (double)IR1.read()* 3.3;
    float distance = m_slope/vDist;
//    if (distance > 80) return 80;
//    else if (distance < 10) return 10;
//    else return distance;
}

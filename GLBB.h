//MESSENGER ROBOT 1
#include "mbed.h"
#include "encoderKRAI.h"
#include "CMPS12_KRAI.h"
#include "JoystickPS3.h"
#include "pinList.h"
#include "millis.h"
#include "Motor.h"

// KONSTANTA
#define PI  3.14159265359     //konstanta PI
#define RAD_TO_DEG  57.2957795131   //konstanta rad/deg
#define PULSE_TO_JARAK 0.657//0.61313625
#define MAX_SPEED 100                 //

#define TS 7.0                     //
#define L 365.0                   //roda to center of robot
#define L_ENC 139                // jarak enc ke pusat
#define MAX_W_SPEED 47700        //
#define LIMITPWM 0.9           //
#define TOLERANCET 1.5       //theta tolerance


//Mapping posisi
typedef struct map {
    int n;
    float x_pos[7];
    float y_pos[7];
    float theta_pos[7];
} mapping;

const mapping map_NULL =    {  0,
                            {  0,  0,  0,  0,  0,  0,  0},
                            {  0,  0,  0,  0,  0,  0,  0},
                            {  0,  0,  0,  0,  0,  0,  0}};
mapping map_data = map_NULL;

//deklarasi Serial
Serial pc(USBTX, USBRX, 115200);
joysticknucleo stick(PIN_TX, PIN_RX);

//deklarasi I2C CMPS12
CMPS12_KRAI compass(PB_3,PB_10, 0xC0);


//Encoder external
encoderKRAI enc_A( PC_2,PC_3, 538,   encoderKRAI::X4_ENCODING);
encoderKRAI enc_B( PA_13,PC_12, 538,   encoderKRAI::X4_ENCODING);
encoderKRAI enc_C( PC_10,PC_11, 538, encoderKRAI::X4_ENCODING);
encoderKRAI enc_D(PB_4, PA_8, 538, encoderKRAI::X4_ENCODING);


// Deklarasi Motor
Motor motorA(PA_11,  PB_12, PA_7);
Motor motorB(PA_5, PA_12, PA_6  );
Motor motorC(PB_14, PC_4, PB_13);
Motor motorD(PB_15,PB_1,PB_2);
// DEKLARASI VARIABLE
DigitalIn butt(USER_BUTTON);
//BASE
float VR_MAX =0.95   ;               // BATAS MAX DAN MIN
float VR_MIN =-0.95 ;               //
double pulse_A, pulse_B, pulse_C, pulse_D;
double t_pulseA, t_pulseB, t_pulseC, t_pulseD;
double speedA, speedB, speedC, speedD;
double prev_speedA, prev_speedB, prev_speedC, prev_speedD;
double c_e_A, c_e_B, c_e_C, c_e_D;
double p_e_A, p_e_B, p_e_C, p_e_D;
double p2_e_A, p2_e_B, p2_e_C, p2_e_D;
double Vx=0.0, Vy=0.0, x=0.0, y=0.0, theta4=0.0;
//konstanta PID
const double Kp_A=0.1004, Kd_A=0, Ki_A=6.2645e-07;
const double Kp_B=0.1004, Kd_B=0, Ki_B=6.2645e-07;
const double Kp_C=0.1004, Kd_C=0, Ki_C=6.2645e-07;
const double Kp_D=0.1004, Kd_D=0, Ki_D=6.2645e-07;
const double KP_W=2.50 ,KI_W=0.0,KD_W=400.0;
// Theta
float theta_temp, theta0, theta, theta_s = 0;
float theta_input, theta_error, sum_theta_error, theta_error_prev;
//kecepatan motor
double Vr_A, Vr_B, Vr_C, Vr_D, Vw_B, Vw_C;
double Vw, Vw_max=0.3, w;
double Vmax = 0.4;
double Va, Vb, Vc, Vd;
double a=0;
double Vr=0.0;
double Vw_tot=0.0, Vw_PID=0.0;
double t_pulse_A,t_pulse_B,t_pulse_C,t_pulse_D, target_rpm=0.0, rpm_max = 15.0, rpm_theta=0.0, theta_max=0.0;
unsigned long last_mt_pid, last_mt_print, last_mt_printtest=0, last_mt_Vw=0.0, last_mt_butt=0;
int brake_state, count, count1;
int test;
int gerak = 0;
double last_theta=0.0, error_last=0.0, last_mt_printPID=0.0, last_pwm=0.0;
double error_s=0.0, target_x = 0.0, target_y = 0.0, x_s = 0.0, y_s=0.0;;

float compute_Alpha(float x_t, float y_t, float x, float y,float theta);
void check_Point();
void hitungTheta();
void hitungOdometry4Roda();
void gerakMotor();
void hitungPIDTheta(double theta_s);
void printPulse();
void case_gerak();
void motor_ForceStop();
void PIDMotor();
void hitungParameterTarget();
void gerak_auto();

int cnt=0;
//double arX[30]={0,500,	1900, 2500,	3700,	4000,	5200,	5500,	6000, 	8000};
//double arY[30]={0,1200,	1300, 330,	100,	1000,	1300,	300,	750, 	600};
double arX[30]={0,300,	2250, 2250,	3700,	3800,	5000,	5200,	5500, 	6000};
double arY[30]={0,1500,	1450, 0,	0,		1500,	1500,	725,	725, 	725};
double arT[30]={0,0,	0,	  0,	0,		0,		0,		0,		0,		0}
int main(){
    enc_A.reset();
    enc_B.reset();
    enc_C.reset();
    enc_D.reset();
    startMillis();
    stick.setup();
    stick.idle();
    theta0 = (compass.getAngle()/10.0);
    cnt=0;
	cp=0;
    brake_state=0;
    last_mt_butt=millis();
    while (1){
        if (!(fabs(theta_s - (theta*RAD_TO_DEG))<TOLERANCET)  ){
            hitungPIDTheta(theta_s);
        }
        gerakMotor();
        if (millis() - last_mt_printPID > 7.0){
            PIDMotor();
            last_mt_printPID=millis();
        }
        if (millis() - last_mt_Theta > 3.0){
            //pc.printf("%.2f \t%.2f \t%.2f \t%.2f \t%.2f \t%.2f \t%.2f \n",  pulse_A,pulse_B,pulse_C,pulse_D, theta*RAD_TO_DEG, theta_s, error_last*RAD_TO_DEG);
            //pc.printf("%.2f \t%.2f \t%.2f \t%.2f \t%.2f \t%.2f \t%.2f \n",  x, y, error_s, target_rpm,a*RAD_TO_DEG, rpm_theta, theta*RAD_TO_DEG);
            hitungTheta();
            last_mt_Theta = millis();
        }
        if (millis() - last_mt_printtest > 5.0){
            error_last = theta - last_theta;
            last_theta=theta;
            pc.printf("x%.2f \ty%.2f \tt%.2f \tVa%.2f \tVa%.2f \tVa%.2f \tVa%.2f \t%.2f \t%.2f \t%.2f \t%d \n",  x,y,theta*RAD_TO_DEG,Va,Vb,Vc,Vd, target_x, target_y,(double) brake_state,cnt );
            last_mt_printtest = millis();
        }
        if (!butt && millis()-last_mt_butt>800){
            wait_ms(2000);
			cp++;
			cp%=2;
        	cnt=0;
			gerak_auto();
            last_mt_butt=millis();
        }
        gerak_auto();
    }
}
float compute_Alpha(float x_t, float y_t, float x, float y,float theta){
//fungsi untuk menghitung alpha sebagai arah gerak robot
    float temp = atan((y_t - y)/(x_t - x)) - theta;
    if (x_t < x) return temp + PI;
    else         return temp;
}
void hitungTheta(){
    theta_temp = (compass.getAngle()/10.0) - theta0;
    if(theta_temp > 180.0 && theta_temp <= 360.0)
        theta = ((compass.getAngle()/10.0) - 360.0 - theta0)/-RAD_TO_DEG;
    else if(theta_temp < -180.0 && theta_temp >= -360.0)
        theta = ((compass.getAngle()/10.0)  + 360.0 - theta0)/-RAD_TO_DEG;
    else
        theta = ((compass.getAngle()/10.0) - theta0)/-RAD_TO_DEG;
}

void hitungOdometry4Roda(){
    Vx = (pulse_C + pulse_D - pulse_A - pulse_B)*0.525322;
    Vy = (pulse_A + pulse_D - pulse_B - pulse_C)*0.525322;
    x += Vx*cos(theta) - Vy*sin(theta);
    y += Vx*sin(theta) + Vy*cos(theta);
    theta4 += (pulse_A + pulse_C + pulse_B + pulse_D)/(4.0*L);
}

void gerakMotor(){

    if(speedA > VR_MAX)
        Vr_A = VR_MAX;
    else if ( speedA < VR_MIN)
        Vr_A = VR_MIN;
    else
        Vr_A = speedA;

    if(speedB > VR_MAX)
        Vr_B = VR_MAX;
    else if ( speedB < VR_MIN)
        Vr_B = VR_MIN;
    else
        Vr_B = speedB;

    if(speedC > VR_MAX)
        Vr_C = VR_MAX;
    else if ( speedC < VR_MIN)
        Vr_C = VR_MIN;
    else
        Vr_C = speedC;

    if(speedD > VR_MAX)
        Vr_D = VR_MAX;
    else if ( speedD < VR_MIN)
        Vr_D = VR_MIN;
    else
        Vr_D = speedD;

    if (Vw==0)
        Vw_tot =  Vw_PID;
    else
        Vw_tot = Vw;

    Va = Vr_A;
    Vb = Vr_B;
    Vc = Vr_C;
    Vd = Vr_D;

	if (Vw_PID > Vw_max){
		Vw_PID = Vw_max;
	}
	else if ( Vw_PID < -1*Vw_max){
		Vw_PID = -1*Vw_max;
	}
   /* Va = Vr*0.7071*(sin(a)-cos(a)) + Vw_tot;
    Vb = Vr*0.7071*(-sin(a)-cos(a)) + Vw_tot;
    Vc = Vr*0.7071*(-sin(a)+cos(a)) + Vw_tot;
    Vd = Vr*0.7071*(sin(a)+cos(a)) + Vw_tot;*/

    if (brake_state==0 || fabs(Va)>0.1 || fabs(Vb)>0.1 || fabs(Vc)>0.1 || fabs(Vd)>0.1){
    motorA.speed(Va);
    motorB.speed(Vb);
    motorC.speed(Vc);
    motorD.speed(Vd);
    } else  {
		motorA.speed(0.0);
		motorB.speed(0.0);
		motorC.speed(0.0);
		motorD.speed(0.0);
		motorA.brake(BRAKE_HIGH);
		motorB.brake(BRAKE_HIGH);
		motorC.brake(BRAKE_HIGH);
		motorD.brake(BRAKE_HIGH);
	}
}
void PIDMotor(){
        //get Pulse value
	
		
        pulse_A = (double)enc_A.getPulses()*PULSE_TO_JARAK;
        pulse_B = (double)enc_B.getPulses()*PULSE_TO_JARAK;
        pulse_C = (double)enc_C.getPulses()*PULSE_TO_JARAK;
        pulse_D = (double)enc_D.getPulses()*PULSE_TO_JARAK;
        enc_A.reset();
        enc_B.reset();
        enc_C.reset();
        enc_D.reset();
        hitungOdometry4Roda();

        //a=90.0/RAD_TO_DEG;
        t_pulse_A = target_rpm*0.7071*(sin(a)-cos(a)) + rpm_theta;
        t_pulse_B = target_rpm*0.7071*(-sin(a)-cos(a)) + rpm_theta;
        t_pulse_C = target_rpm*0.7071*(-sin(a)+cos(a))+ rpm_theta;
        t_pulse_D = target_rpm*0.7071*(sin(a)+cos(a))+ rpm_theta;
        c_e_A = (double) t_pulse_A - pulse_A;
        c_e_B = (double) t_pulse_B - pulse_B;
        c_e_C = (double) t_pulse_C - pulse_C;
        c_e_D = (double) t_pulse_D - pulse_D;

        //update value
        speedA = (double) ( prev_speedA + Kp_A*(c_e_A - p_e_A) + Ki_A*TS*(c_e_A + p_e_A)/2 + Kd_A*(c_e_A - 2*p_e_A + p2_e_A) );
        speedB = (double) ( prev_speedB + Kp_B*(c_e_B - p_e_B) + Ki_B*TS*(c_e_B + p_e_B)/2 + Kd_B*(c_e_B - 2*p_e_B + p2_e_B) );
        speedC = (double) ( prev_speedC + Kp_C*(c_e_C - p_e_C) + Ki_C*TS*(c_e_C + p_e_C)/2 + Kd_C*(c_e_C - 2*p_e_C + p2_e_C) );
        speedD = (double) ( prev_speedD + Kp_D*(c_e_D - p_e_D) + Ki_D*TS*(c_e_D + p_e_D)/2 + Kd_D*(c_e_D - 2*p_e_D + p2_e_D) );

        //update value
        prev_speedA = speedA;
        prev_speedB = speedB;
        prev_speedC = speedC;
        prev_speedD = speedD;
        p2_e_A = p_e_A;
        p2_e_B = p_e_B;
        p2_e_C = p_e_C;
        p2_e_D = p_e_C;
        p_e_A = c_e_A;
        p_e_B = c_e_B;
        p_e_C = c_e_C;
        p_e_D = c_e_D;

        //Thread::wait(TS2);
        //pulse_C = (double)pulse_C*PULSE_TO_JARAK;
        //pc.printf("%.2f \t %.2f \t %.2f \n", speedA, speedB, speedC);


}
void hitungPIDTheta(double theta_s){

    //menghitung error jarak x,y terhaadap xs,ys
    theta_input = theta*RAD_TO_DEG;
    theta_error = theta_s - theta_input;

    sum_theta_error += theta_error;

    //kalkulasi PID Theta
    w = KP_W*theta_error + KI_W*TS*sum_theta_error + KD_W*(theta_error - theta_error_prev)/TS;
    Vw_PID = (w*L/MAX_W_SPEED)*LIMITPWM;

    //update
    theta_error_prev = theta_error;


}

void printPulse(){
    pc.printf("%.2f,%.2f,%.2f,%.2f,%.2f\n", pulse_A, pulse_B, pulse_C, pulse_D, theta*RAD_TO_DEG);
}


void gerak_auto(){
	check_Point();
	if (cp>0 && cp<3){
		if (cnt>=0 && cnt<map_data.n){
			
			target_x = map_data.x_pos[cnt];
			target_y = map_data.y_pos[cnt];
			theta_s  = map_data.theta_pos[cnt-1];
			
			hitungParameterTarget();
		}
		else  {
			gerak= 0;
			Vr = 0.0;
			target_rpm = 0.0;
		}
	} else {
		gerak= 0;
		Vr = 0.0;
		target_rpm = 0.0;
	}
	
}

void hitungParameterTarget(){
	x_s = (target_x - x)*(target_x - x);
	y_s = (target_y - y)*(target_y - y);
	error_s = sqrt(x_s + y_s);
	a = compute_Alpha(target_x, target_y,x,y,theta); // Kiri
	if (error_s<=100){
		cnt++;
	}
	if (error_s>=200){
		//target_rpm = 7.0;
		if (target_rpm<=rpm_max){
			if (millis()-last_pwm>=17){
				target_rpm +=0.4;
				last_pwm=millis();
			}
		} else
			target_rpm = rpm_max;
	} else if (error_s>50){
		target_rpm = rpm_max*error_s/240;
	} else
		target_rpm=0.0;	
}

void motor_ForceStop(){
//FORCEBRAKE
    motorA.brake(BRAKE_HIGH);
    motorB.brake(BRAKE_HIGH);
    motorC.brake(BRAKE_HIGH);
    motorD.brake(BRAKE_HIGH);
}

/*
vt^2 = vo^2 + 2as
a konstant
s berubah

500
0.5
0.001
*/

void gerakTes(){
	//di inisialisasi
	KpFF = 0.001;
	acc = 0.05
	
	//di input joystick
	xT = 3000;
	yT = 0;
	sign =1;
	lastS = millis();
	
	//perhitungan kecepatan
	sT = sqrt(xT*xT + yT*yT);
	s0 = sqrt(x*x + y*y);
	ds = sT - s0;
	s = acc*TS*TS/2;
	if (ds<s)
		s = ds;
	
	if (millis() - lastS<100) //waktu ff 100ms
		ff = KpFF*ds;
	else 
		ff = 0.0;

	if (vo>vMax)
		sign = -1;
	
	vt = sqrt(vo*vo + 2*sign*acc*s) + ff;
	vo = vt;
		
}
void ambilDATA(){
	ar[700];
	i=0
	while (1){
		if (millis() - last_sample>=7 && i<=700){
			pulse_A = (double)enc_A.getPulses()*PULSE_TO_JARAK;
			enc_A.reset();
			ar[i]=pulse_A;
			i++;
			last_sample=millis();
		}
		if (i<=500)
			motorA.speed(1.0);
		else	
			motorA.speed(0.0);
		if  (i>=700)
			for (j=0;j<700;j++)
				pc.printf("%.2f\n", ar[j]);	
}
void check_Point(){
//prosedur untuk mengganti map odometry
//Urutan mapping X, Y, THETA
    switch(cp){
        case 1:{
            const mapping map_cp1 = {10,
            		{0,300,	2250, 2250,	3700,	3800,	5000,	5200,	5500, 	6000},
					{0,1500,	1450, 0,	0,		1500,	1500,	725,	725, 	725},
					{0,0,	0,	  0,	0,		0,		0,		0,		0,		0}};
            map_data = map_cp1; break;
        }
        case 2:{
            const mapping map_cp2 = {1,
                                    {0},
                                    {0},
                                    {0}};
            map_data = map_cp2; break;
        }
        case 3:{
            const mapping map_cp3 = {5,
                                    {500,0,500,0,500},
                                    {0,0},
                                    {0,20,40,60,90}};
            map_data = map_cp3; break;
        }
        case 4:{
            const mapping map_cp4 = {1,
                                    {0},
                                    {1000},
                                    {0}};
            map_data = map_cp4; break;
        }
        case 5:{
            const mapping map_cp5 = {5,
                                    {0, 1060, 1060, 0,0},
                                    {0, 0,    1060, 1060,0},
                                    {0, 0,    0,    0,0}};
            map_data = map_cp5; break;
        }
        case 6:{
                    const mapping map_cp6 = {7,
                                            {0,   0,  0,   0,   0,  0,  0 },
                                            {-400, -1000, -3100, -4100, -4600, -4800, -4800},
                                            {0,     0,      0,      0,      0,      0,  0}};
                    map_data = map_cp6; break;
                }
        default:    break;
    }
}
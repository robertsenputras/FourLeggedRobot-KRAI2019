#include "mbed.h"

#define pulseToDeg 0.4444 // 0.86103// bisa ganti 17.765 utk deg/s 1/(360*538/805)

#define pwmMax 0.90
#define errorThetaMax 45.0

#define KP 0.390//0.353
//kanan
float Kp_A = 0.0088;//0.017//0.078
float Ki_A = 2.654e-4;//1.754e-4
float Kd_A = 0;
//kiri
float Kp_B = 0.0123;//0.0153//0.078
float Ki_B = 2.85e-4;//1.454e-4
float Kd_B = 0;

////BACKUP
////kanan
//float Kp_A = 0.01;//0.017//0.078
//float Ki_A = 2.754e-4;//1.754e-4
//float Kd_A = 0;
////kiri
//float Kp_B = 0.0083;//0.0153//0.078
//float Ki_B = 2.25e-4;//1.454e-4
//float Kd_B = 0;
#define TS 25//9
#define toleranceT 3.0
#define headingBiru 60
#define headingMerah -59

////kanan
//#define Kp_A 0.0017//0.078
//#define Ki_A 5.754e-4
//#define Kd_A 0
////kiri
//
//#define Kp_B 0.00153//0.078
//#define Ki_B 5.454e-4
//#define Kd_B 0

float speedThetaMax=6.0;

int stall, mode, state, ohstart;
double pulse_A, pulse_B,
	pulseKa, pulseKi, target_pulse;
unsigned long last_mt_motor, last_mt_PID,
			last_mt_print, lastStall,
			last_mt_cmps, last_mt_button,
			last_mt_switch, last_mt_PIDTheta,
			last_mt_FSM, last_mt_sp,
			last_mt_sampling, last_mt_pitch,
			last_mt_IR,last_mt_lcd;

double theta, prevTheta, theta_temp,theta0,
		pitch, pitch_temp,pitch0,
		speed_sp, theta_sp,
		speed_spA, speed_spB,
		speed,speedA, speedB,
		prev_speedA, prev_speedB,
		errA, prevErrA,
		errB, prevErrB,
		speedTheta, errorTheta;
double eTheta=0, target, IRDepan, IRBlkng,IRBwhKa, IRTest;
double vDist1, vDist2, vDist3, m_slope1, m_slope2, m_slope3;

int toggleIR=0;
int mainMode=0;
float batasTembok= 80.0;
float prevIRDepan=0;
float m_speed_maju;
float m_jarak_ir;
float m_speed_dune;
float m_max_dune;
float m_speed_belok_tussock;
float m_max_belok_tussock;
float m_etheta;
float m_max_tussock;
float m_speed_tussock;
float m_IR2;

float b_speed_maju;
float b_jarak_ir;
float b_speed_dune;
float b_max_dune;
float b_speed_belok_tussock;
float b_max_belok_tussock;
float b_etheta;
float b_max_tussock;
float b_speed_tussock;
float b_IR2;
float m_speedBelok=0,b_speedBelok=0;

int lastState=0;


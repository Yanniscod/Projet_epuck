#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <motors.h>
#include <camera/po8030.h>
#include <chprintf.h>
#include "sensors/proximity.h"
#include <pi_regulator.h>
#include <process_image.h>
	//  // MANQUE CONDITION SUR ERROR POUR PAS QU'IL TOURNE EN ROND EN BOUCLE LORSQU'IL N'Y A PLUS D'OBSTACLE //  //
	//	// CONFIGURER KP ET KI PLUS GRANDS COMME CA ON PEUTLES REGLER PLUS PRECISEMENT ET PLUS BESOIN DE FLOAT, KP=3 TROP GRAND
#define BASE_MOTOR_SPEED 400
#define ZERO 0
#define IR1 0
#define IR2 1
#define IR3 2
#define IR4 3
#define IR5 4
#define IR6 5
#define IR7 6
#define IR8 7
#define NO_OBST 8
#define OBST_THRES 450
#define OBST_THRES_SIDE 200  // smaller than obst_thres, otherwise IR2/7 keeps getting higher values -> the robot keeps rotating
#define ERROR_THRES 30
#define DEG_THRES 5 //[deg]
//#define MAX_ERROR 800
#define MAX_SUM_ERROR 10000
#define SPEED_WHEEL_45DEG_1S 320	//	step/s
#define QUART_DE_TOUR 90	// degrees
//#define QUART_DE_TOUR_G -90
#define SPEEDTOANGLE 0.0277f //1/(DEG_2_STEP/2)*THD_OBST_TIME_MS/1000
#define TIERS_DE_TOUR 58
//#define TIERS_DE_TOUR_G -58
#define DEG_2_STEP  7.21f // 7.21 step/deg, obtained experimentally
#define KP 1.50f
#define KI 0.05f
#define THD_OBST_TIME_MS 107

systime_t time_1,time_2;
messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

void SendUint8ToComputer(uint8_t* data, uint16_t size) 
{
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, size);
}

static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

int speed_2_deg_rota(int speed){ // THD_OBST_TIME_MS is the time (ms) during which the chosen wheel turns at the increased speed,
	int8_t angle = 0;	//  we have to take it into account because our angle which we want to reduce is affected by it angle = speed/(DEG_2_STEP/2) * THD_OBST_TIME_MS / 1000; // [deg] = [step/s]*[s]/[step/deg], DEG_2_STEP is float so calculation done in float, optimizable?
	angle = speed*SPEEDTOANGLE; // happens each 107ms
	chprintf((BaseSequentialStream *)&SD3,"Speed fct : %-7d Angle fct : %-7d\r\n", speed, angle);
	return angle;
}

void rotate_one_sec(int angle){
	int16_t speed = DEG_2_STEP /2 * angle;
	right_motor_set_speed(-speed);
	left_motor_set_speed(speed);
	 chprintf((BaseSequentialStream *)&SD3, "Speed : %-7d", speed);
}

void detect_obst(void){
	static int cmpt_rot_deg = 0;
	//static float sum_error = 0;
	int speed_PID = 0;
	uint16_t prox_1, prox_2, prox_7, prox_8; // for the values of the proximity sensors
	uint8_t prox_obst = 8;			// to know which sensor is the closest to the obstacle, 0 already taken by IR1, so we chose 8
	prox_1=get_calibrated_prox(IR1);
	//chThdSleepMilliseconds(10);
	prox_2=get_calibrated_prox(IR2);
	//chThdSleepMilliseconds(10);
	prox_7=get_calibrated_prox(IR7);
	//chThdSleepMilliseconds(10);
	prox_8=get_calibrated_prox(IR8);
	//chThdSleepMilliseconds(10);
	//detects where is the obstacle, only cares for sensors in front of the e-puck (17' and 49')
	if ((prox_1 > OBST_THRES) || (prox_8 > OBST_THRES)){  //if obstacle in front of IR1/8
		if (prox_1 > prox_8){	// compare both values
			prox_obst = IR1;
		} else {
			prox_obst = IR8;
		}
	} else if((prox_2 > OBST_THRES) || (prox_7 > OBST_THRES)){ //if obstacle in front of IR2/7
		if (prox_2 > prox_7){	// compare both values
			prox_obst = IR2;
		} else {
			prox_obst = IR7;
		}
	}

	//chprintf((BaseSequentialStream *)&SD3,"Capteur d'obstacle = %-7d\r\n", get_calibrated_prox(prox_obst));

	if (prox_obst != NO_OBST){ // if there is an obstacle, avoid it by rotating and save how much we rotated
		switch(prox_obst){
		case 0: // 90deg to the left
			rotate_one_sec(-QUART_DE_TOUR);
			cmpt_rot_deg += (-QUART_DE_TOUR);
			break;
		case 1: // (90-(49-17))deg = 58deg to the left
			rotate_one_sec(-TIERS_DE_TOUR);
			cmpt_rot_deg += (-TIERS_DE_TOUR);
			break;
		case 6: // 58deg to the right
			rotate_one_sec(TIERS_DE_TOUR);
			cmpt_rot_deg += TIERS_DE_TOUR;
			break;
		case 7: // 90deg to the right
			rotate_one_sec(QUART_DE_TOUR);
			cmpt_rot_deg += QUART_DE_TOUR;
			break;
		default:
			break;
		}
		chThdSleepMilliseconds(1000); // to rotate, constants were set according to 1 sec
	}

	// et là faut commencer à AUSSI regarder IR3/6 et réajuster la position jusqu'à "annuler" l'angle duquel on
	// a tourné. À ce moment-là, reprendre uniquement par rapport aux autres IR
	// PID BEGINS
	if (cmpt_rot_deg != ZERO){ // checks if we've avoided an obstacle and therefore need to rectify, threshold of 3 degrees
	int prox_near_obst = 0; // will stock the value of the concerned proximity sensor
	int error = 0; // error for PID
	static float sum_error = 0;
	if (cmpt_rot_deg<0){	// checks which values' sensor we need to look at
		prox_near_obst = get_calibrated_prox(IR3);
	} else {
		prox_near_obst = get_calibrated_prox(IR6);
		}

	error = OBST_THRES_SIDE - prox_near_obst;

	if(abs(error) < ERROR_THRES){
		error = 0;
	}

	chprintf((BaseSequentialStream *)&SD3, "Error : %-7d", error);

	sum_error += error;

	if(sum_error > MAX_SUM_ERROR){	// to avoid an uncontrolled growth
	sum_error = MAX_SUM_ERROR;
	}else if(sum_error < -MAX_SUM_ERROR){
	sum_error = -MAX_SUM_ERROR;
	}

	speed_PID = KP * error + KI * sum_error; // speed of the right or left wheel to add to the base speed
	chprintf((BaseSequentialStream *)&SD3, "Speed PID: %-7d ", speed_PID);

	if (cmpt_rot_deg < 0){	// adds the speed to the correct wheel, divides it by 2 to split it to both wheels, same result
		right_motor_set_speed(BASE_MOTOR_SPEED - speed_PID); // to go right
		left_motor_set_speed(BASE_MOTOR_SPEED + speed_PID);
		//cmpt_rot_deg += speed_2_deg(speed_PID);// translates the additional speed in degrees, to substract them from the counter
		cmpt_rot_deg += speed_PID*SPEEDTOANGLE;
	} else {
		right_motor_set_speed(BASE_MOTOR_SPEED + speed_PID); // to go left
		left_motor_set_speed(BASE_MOTOR_SPEED - speed_PID);
		//cmpt_rot_deg -= speed_2_deg(speed_PID);
		cmpt_rot_deg -= speed_PID*SPEEDTOANGLE;
			}
	//time_1 = chVTGetSystemTime(); // to obtain THD_OBST_TIME_MS experimentally
	//time_2 = chVTGetSystemTime();
	//chprintf((BaseSequentialStream *)&SD3, "time1 = %-7d time2 = %-7d\r\n",time_1, time_2); // 107ms
	chprintf((BaseSequentialStream *)&SD3,"Angle tourne : %-7d Compteur : %-7d\r\n", speed_2_deg_rota(speed_PID), cmpt_rot_deg);
	}
	else{
		right_motor_set_speed(BASE_MOTOR_SPEED);
		left_motor_set_speed(BASE_MOTOR_SPEED);
	}

	if(fabs(cmpt_rot_deg) < DEG_THRES){ //	fabs() !
		cmpt_rot_deg = 0;
	}

	//chprintf((BaseSequentialStream *)&SDU1,"IR3= %-7d IR6= %-7d\r\n", prox_3, prox_6);
	//chprintf((BaseSequentialStream *)&SD3,"IR1 = %-7d IR2 = %-7d IR3 = %-7d IR6 = %-7d IR7 = %-7d IR8 = %-7d\r\n",
	//		prox_1, prox_2, prox_3, prox_6, prox_7, prox_8);

	chThdSleepMilliseconds(100);

}

int main(void){
    RCC->AHB1ENR    |= RCC_AHB1ENR_GPIOBEN; //initiate GPIOC/B clocks
    RCC->AHB1ENR    |= RCC_AHB1ENR_GPIOCEN;
    halInit();
    chSysInit();
    messagebus_init(&bus, &bus_lock, &bus_condvar);
    serial_start();
    usb_start();
    proximity_start();
	motors_init();
	   // mpu_init();
	right_motor_set_speed(BASE_MOTOR_SPEED);
	left_motor_set_speed(BASE_MOTOR_SPEED);
	 while(1){
		 /*
		 rotate_one_sec(180);
		 chThdSleepMilliseconds(1000);
		 right_motor_set_speed(0);
		left_motor_set_speed(0);
		chThdSleepMilliseconds(1000);
		*/
		detect_obst();
	 }
    }
    /*
    //starts the camera
    dcmi_start();
	po8030_start();

	//stars the threads for the pi regulator and the processing of the image
	pi_regulator_start();
	process_image_start();
*/
    /* Infinite loop. */

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}

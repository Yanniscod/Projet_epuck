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

	//  // THD_TIME_MS TO MANUALLY CONFIGURE, NOT TO FORGET //  //
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
#define OBST_THRES 400
#define OBST_THRES_SIDE 500  // bigger than obst_thres, otherwise IR2/7 keeps getting higher values -> the robot keeps rotating
#define ERROR_THRES 30
#define DEG_THRES 2 //[deg]
#define MAX_ERROR 800
#define MAX_SUM_ERROR 800
#define SPEED_WHEEL_45DEG_1S 320	//	step/s
#define QUART_DE_TOUR_D 90	// degrees
//#define QUART_DE_TOUR_G -90
#define TIERS_DE_TOUR_D 58
//#define TIERS_DE_TOUR_G -58
#define ANG_2_SPEED  7.11f // 7.11 step/deg, obtained experimentally
#define KP 3.00f
#define KI 0.05f
#define THD_OBST_TIME_MS 100

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

int speed_2_deg(int speed){ // THD_OBST_TIME_MS is the time (ms) during which the chosen wheel turns at the increased speed,
	int8_t angle = 0;		//  we have to take it into account because our angle which we want to reduce is affected by it
	angle = speed/(ANG_2_SPEED/2) * THD_OBST_TIME_MS / 1000; // [deg] = [step/s]*[s]/[step/deg], ang_2_speed is float so calculation done in float, optimizable?
	return angle;											// could : /(1000*711) instead of /(1000000*7.11), to check with other places where ang_2_speed is called
}

void rotate_one_sec(int angle){
	int16_t speed = ANG_2_SPEED /2 * angle;
	right_motor_set_speed(-speed);
	left_motor_set_speed(speed);
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
			rotate_one_sec(-QUART_DE_TOUR_D);
			cmpt_rot_deg += (-QUART_DE_TOUR_D);
			break;
		case 1: // (90-(49-17))deg = 58deg to the left
			rotate_one_sec(-TIERS_DE_TOUR_D);
			cmpt_rot_deg += (-TIERS_DE_TOUR_D);
			break;
		case 6: // 58deg to the right
			rotate_one_sec(TIERS_DE_TOUR_D);
			cmpt_rot_deg += TIERS_DE_TOUR_D;
			break;
		case 7: // 90deg to the right
			rotate_one_sec(QUART_DE_TOUR_D);
			cmpt_rot_deg += QUART_DE_TOUR_D;
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
	chprintf((BaseSequentialStream *)&SD3, "Error : %-7d", error);
	if(abs(error) < ERROR_THRES){
		error = 0;
	}
	if(error > MAX_ERROR){	// to avoid an uncontrolled growth of the error
		error = MAX_ERROR;
	}else if(error < -MAX_ERROR){
		error = -MAX_ERROR;
	}

	sum_error += error;

	if(sum_error > MAX_SUM_ERROR){	// to avoid an uncontrolled growth
	sum_error = MAX_SUM_ERROR;
	}else if(sum_error < -MAX_SUM_ERROR){
	sum_error = -MAX_SUM_ERROR;
	}

	speed_PID = KP * error + KI * sum_error; // speed of the right or left wheel to add to the base speed
	//time_1 = chVTGetSystemTime(); // to obtain THD_OBST_TIME_MS experimentally
	chprintf((BaseSequentialStream *)&SD3, "Speed : %-7d ", speed_PID);

	if (cmpt_rot_deg<0){	// adds the speed to the correct wheel, divides it by 2 to split it to both wheels, same result
		right_motor_set_speed(BASE_MOTOR_SPEED - speed_PID); // to go right
		left_motor_set_speed(BASE_MOTOR_SPEED + speed_PID);
		} else {
		right_motor_set_speed(BASE_MOTOR_SPEED + speed_PID); // to go left
		left_motor_set_speed(BASE_MOTOR_SPEED - speed_PID);
			}
	//time_2 = chVTGetSystemTime();
	//chprintf((BaseSequentialStream *)&SD3, "time2␣=␣%dus\n",time_2);
	cmpt_rot_deg -= speed_2_deg(speed_PID);// translates the additional speed in degrees, to substract them from the counter
	chprintf((BaseSequentialStream *)&SD3,"Angle tourne : %-7d Compteur : %-7d\r\n", speed_2_deg(speed_PID), cmpt_rot_deg);
	}										// rotation to achieve (cmpt_rot_deg)

	if(fabs(cmpt_rot_deg) < DEG_THRES){
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
	 while(1){
		 right_motor_set_speed(BASE_MOTOR_SPEED);
		 left_motor_set_speed(BASE_MOTOR_SPEED);
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

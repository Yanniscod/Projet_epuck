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
#include <detect_obst.h>
#include "sensors/proximity.h"
#include <process_image.h>


	//	// CONFIGURER KP ET KI PLUS GRANDS COMME CA ON PEUTLES REGLER PLUS PRECISEMENT ET PLUS BESOIN DE FLOAT, KP=3 TROP GRAND

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
/*
int speed_2_deg_rota(int speed){ // THD_OBST_TIME_MS is the time (ms) during which the chosen wheel turns at the increased speed,
	int8_t angle = 0;	//  we have to take it into account because our angle which we want to reduce is affected by it angle = speed/(DEG_2_STEP/2) * THD_OBST_TIME_MS / 1000; // [deg] = [step/s]*[s]/[step/deg], DEG_2_STEP is float so calculation done in float, optimizable?
	angle = speed*SPEEDTOANGLE; // happens each 107ms
	//chprintf((BaseSequentialStream *)&SD3,"Speed fct : %-7d Angle fct : %-7d\r\n", speed, angle);
	return angle;
}
*/

//void detect_obst(void){


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
	//right_motor_set_speed(BASE_MOTOR_SPEED);
	//left_motor_set_speed(BASE_MOTOR_SPEED);
	move();
	detect_obst_start();
	 while(1){
		 /*
		 rotate_one_sec(180);
		 chThdSleepMilliseconds(1000);
		 right_motor_set_speed(0);
		left_motor_set_speed(0);
		chThdSleepMilliseconds(1000);
		*/
		 /* Infinite loop. */
		 chThdSleepMilliseconds(1000);
	 }
    }
    /*
    //starts the camera
    dcmi_start();
	po8030_start();

	//starts the threads for the pi regulator and the processing of the image
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

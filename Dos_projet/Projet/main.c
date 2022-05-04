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
#include <move.h>

//systime_t time_1,time_2;
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
	move();
	detect_obst_start();
	 while(1){
		 /* Infinite loop. */
		 chThdSleepMilliseconds(1000);
	 }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}

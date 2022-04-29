#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <move.h>
#include <main.h>

// 0b000
static bool go = 0; // 0 if the robot doesn't have to move, 1 if it does || 0 = stop 1 = go
static bool forward = 0; // 0 if it has to rotate, 1 if it has to go straightforward
static bool rota_type = 0; // 0 if it's the rotation to avoid the obstacle, 1 if it's to rectify the position
static int speed_rota = 0;
static int angle_rota = 0;
//static int pos_2_reach=0;

static THD_WORKING_AREA(waMove, 512);
static THD_FUNCTION(Move, arg) {

	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	systime_t time_3 = 0;
	systime_t time_4 = 0;
	while(1){
		if(go == true){
			if(forward == true){		// when going forward, the speed is set with a define
				right_motor_set_speed(BASE_MOTOR_SPEED);
				left_motor_set_speed(BASE_MOTOR_SPEED);
			}else if(rota_type == false){
					time_3 = chVTGetSystemTime();
					rotate_one_sec(angle_rota);
					chThdSleepMilliseconds(1000);
					rota_type = true;
				}else{
					//time_3 = chVTGetSystemTime();
					right_motor_set_speed(BASE_MOTOR_SPEED - speed_rota);
					left_motor_set_speed(BASE_MOTOR_SPEED + speed_rota);
					time_4 = chVTGetSystemTime(); // 101ms
					//chprintf((BaseSequentialStream *)&SD3, "Speed Rot : %-7d time_speed_move = %-7d\r\n", speed_rota, time_4);
					chprintf((BaseSequentialStream *)&SD3,"Time_1 : %-7d Time_2 %-7d\r\n",time_3, time_4);
				}
			}else{
			right_motor_set_speed(ZERO);
			left_motor_set_speed(ZERO);
		}
		chThdSleepMilliseconds(100); // thread each 100ms
	}
}

// to allow other source files to modify the speed
void set_speed_rota(int speed){
	speed_rota =speed;
}

void set_angle_rota(int angle){
	angle_rota =angle;
}

void set_bool(int bool_num, bool type){
	switch(bool_num){
	case 0:
		go = type;
		break;
	case 1:
		forward = type;
		break;
	case 2:
		rota_type = type;
		break;
	default:
		break;
	}
}

void rotate_one_sec(int angle){
	int16_t speed = DEG_2_STEP /2 * angle;
	right_motor_set_speed(-speed);
	left_motor_set_speed(speed);
	 chprintf((BaseSequentialStream *)&SD3, "Speed : %-7d", speed);
}

void move(void){
	chThdCreateStatic(waMove, sizeof(waMove), NORMALPRIO, Move, NULL);
}

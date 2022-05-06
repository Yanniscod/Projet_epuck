#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <move.h>
#include <main.h>
#include <process_image.h>
#include <motors.h>

static bool go = 0; // 0 if the robot doesn't move, 1 if it does || 0 = stop 1 = go
static bool forward = 0; // 0 if it has to rotate, 1 if it has to go straightforward
static bool rota_type = 0; // 0 if it's the rotation to avoid the obstacle, 1 if it's to rectify the initial direction
static bool tourne = true;
static int16_t speed_rota = 0;
static int8_t turns = 0;

systime_t time;
static THD_WORKING_AREA(waMove, 512); // 27? (6x1 + 16x1 + 8x2) +arg_fcts(16x1 + 4x8  + 1x1) = 38 + 49 = 87
static THD_FUNCTION(Move, arg) {

	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	while(1){
		time = chVTGetSystemTime();

		if(go == true){
			if(forward == true){
				right_motor_set_speed(BASE_MOTOR_SPEED);
				left_motor_set_speed(BASE_MOTOR_SPEED);
			}else if(rota_type == false){
					rotate(turns);
				}else{
					right_motor_set_speed(BASE_MOTOR_SPEED - speed_rota);
					left_motor_set_speed(BASE_MOTOR_SPEED + speed_rota);
				}
		}else{
			if(get_img_captured()){
			take_puck();
			}
		}
		chThdSleepUntilWindowed(time, time + MS2ST(20)); // 50 Hz, thread lasts around ms

	}
}

void set_speed_rota(int16_t speed){
	speed_rota =speed;
}

void set_nbr_rota(int8_t nbr_90_turns){
	turns = nbr_90_turns;
}

void set_bool(int8_t bool_num, bool type){
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

int8_t get_bool(int8_t bool_num){
	bool status = false;
	switch(bool_num){
	case 0:
		status = go;
		break;
	case 1:
		status = forward;
		break;
	case 2:
		status = rota_type;
		break;
	default:
		break;
	}
	return status;
}

void rotate(int8_t nbr_90_turns){
	if(nbr_90_turns >0){
		right_motor_set_speed(-BASE_MOTOR_SPEED);
		left_motor_set_speed(BASE_MOTOR_SPEED);
		if((left_motor_get_pos()) >= (STEPS_TURN*nbr_90_turns)){
			rota_type = true;
			tourne=false;
		}
	}else{
		right_motor_set_speed(BASE_MOTOR_SPEED);
		left_motor_set_speed(-BASE_MOTOR_SPEED);
		if((left_motor_get_pos()) <= (STEPS_TURN*nbr_90_turns)){
			rota_type = true;
			tourne=false;
		}
	}
}

void move(void){
	chThdCreateStatic(waMove, sizeof(waMove), NORMALPRIO, Move, NULL);
}

void take_puck(void){
	static uint8_t nbr_puck;
	static uint8_t state=1;
	static bool puck = false;

	switch (state){
	case RESET_STEPS : // steps at 0 for rotation
		right_motor_set_pos(0);
		left_motor_set_pos(0);
		if(!tourne){
			state=MOVE_TO_PUCK;
		}
		else{
		state=ROTATION;
		}
		break;
	case ROTATION:
		//rotation 90 deg
		rotate(ONE_TURN_90_DEG); //changes value of tourne
		if(!tourne){
			if(puck){	
				state=GO_TO_GOAL;
			}
			else{state=RESET_STEPS;}

		}break;

	case MOVE_TO_PUCK:
		nbr_puck=get_nbr_lines();
		right_motor_set_speed(BASE_MOTOR_SPEED);
		left_motor_set_speed(BASE_MOTOR_SPEED);

		if(left_motor_get_pos()>nbr_puck*DISTANCE_PUCK){ //check if reach puck before rotating to pick it
			right_motor_set_speed(0);
			left_motor_set_speed(0);
			tourne=!tourne;
			puck=true;
			state=RESET_STEPS;
		}
		break;

	case GO_TO_GOAL: // end of taking puck ready for obstacle avoidance

		set_bool(GO, 1);
		right_motor_set_speed(BASE_MOTOR_SPEED);
		left_motor_set_speed(BASE_MOTOR_SPEED);

		break;

	default:
		break;
	}

}


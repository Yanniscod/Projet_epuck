#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <move.h>
#include <main.h>
#include <process_image.h>
#include <motors.h>
#include <avoid_obst.h>

static bool go_dribble = 0; // 0 if the robot doesn't move towards the goal, 1 if it does
static bool forward = 0; // 0 if it has to rotate, 1 if it has to go straightforward
static bool rota_type = 0; // 0 if it's the rotation to avoid the obstacle, 1 if it's to rectify the initial direction
static bool tourne = true;
static int16_t speed_rota = 0;
static int8_t turns = 0;

systime_t time;
static THD_WORKING_AREA(waMove, 46); //  (6x1 + 8x1 + 16x2)
static THD_FUNCTION(Move, arg) {

	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	while(1){
		time = chVTGetSystemTime();
		if(go_dribble){
			if(forward){
				right_motor_set_speed(BASE_MOTOR_SPEED);
				left_motor_set_speed(BASE_MOTOR_SPEED);
			}else if(!rota_type){
					rotate(turns);
				}else{
					right_motor_set_speed(BASE_MOTOR_SPEED - speed_rota);
					left_motor_set_speed(BASE_MOTOR_SPEED + speed_rota);
				}
		}else{
			if(get_img_captured()){
			move_puck();
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
	case GO_DRIBBLE:
		go_dribble = type;
		break;
	case FORWARD:
		forward = type;
		break;
	case ROTA_TYPE:
		rota_type = type;
		break;
	default:
		break;
	}
}

int8_t get_bool(int8_t bool_num){
	bool status = false;
	switch(bool_num){
	case GO_DRIBBLE:
		status = go_dribble;
		break;
	case FORWARD:
		status = forward;
		break;
	case ROTA_TYPE:
		status = rota_type;
		break;
	default:
		break;
	}
	return status;
}

void rotate(int8_t nbr_90_turns){
	if(nbr_90_turns >ZERO_TURN){
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

void move_puck(void){
	static uint8_t nbr_puck;
	static uint8_t state=1;
	static bool puck = false;

	switch (state){
	case RESET_STEPS : // steps at 0 for rotation
		right_motor_set_pos(ZERO_POS);
		left_motor_set_pos(ZERO_POS);
		if(!tourne){
			state=MOVE_TO_PUCK;
		}
		else{
		state=ROTATION;
		}
		break;

	case ROTATION:
		if(!puck){ // first rotation to go towards puck
			rotate(ONE_TURN_90_DEG);
			if(!tourne){ // if rotation done
				state=RESET_STEPS;
			}
		}else{//puck=true
			if(get_ready_to_score()){// when detect goal rotate 180 to score
				rotate(TWO_TURN);
				if(!tourne){ // if rotation done
					state=END;
				}
			}else{ // got puck and go towards goal
				rotate(ONE_TURN_90_DEG);
				if(!tourne){ //if rotation done
					state=GO_TO_GOAL;
				}
			}
		}
		break;

	case MOVE_TO_PUCK: //move towards puck to pick it up
		nbr_puck=get_nbr_lines();
		right_motor_set_speed(BASE_MOTOR_SPEED);
		left_motor_set_speed(BASE_MOTOR_SPEED);

		if(left_motor_get_pos()>nbr_puck*DISTANCE_PUCK){ //check if reach puck before rotating to pick it
			right_motor_set_speed(ZERO_SPEED);
			left_motor_set_speed(ZERO_SPEED);
			tourne=!tourne;
			puck=true;
			state=RESET_STEPS;
		}
		break;

	case GO_TO_GOAL: // end of taking puck ready for obstacle avoidance
		set_bool(GO_DRIBBLE, 1);
		right_motor_set_speed(BASE_MOTOR_SPEED);//a enlever apres test
		left_motor_set_speed(BASE_MOTOR_SPEED);
		if(get_ready_to_score()){
			tourne=true;
			state=RESET_STEPS;
		}
		break;

	case END: //goal scored stop moving maybe LEDS or MUSIC???
		right_motor_set_speed(ZERO_SPEED);
		left_motor_set_speed(ZERO_SPEED);
		break;

	default:
		break;
	}
}

void move(void){
	chThdCreateStatic(waMove, sizeof(waMove), NORMALPRIO, Move, NULL);
}

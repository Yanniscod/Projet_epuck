#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <move.h>
#include <main.h>
#include <process_image.h>
#include <motors.h>

static bool go = 0; // 0 if the robot doesn't have to move, 1 if it does || 0 = stop 1 = go
static bool forward = 0; // 0 if it has to rotate, 1 if it has to go straightforward
static bool rota_type = 0; // 0 if it's the rotation to avoid the obstacle, 1 if it's to rectify the position
static bool tourne = true;
static int speed_rota = 0;
static int angle_rota = 0;
//static int pos_2_reach=0;
systime_t time;
static THD_WORKING_AREA(waMove, 512);
static THD_FUNCTION(Move, arg) {

	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	while(1){
		time = chVTGetSystemTime();

		if(go == true){
			if(forward == true){		// when going forward, the speed is set with a define
				right_motor_set_speed(BASE_MOTOR_SPEED);
				left_motor_set_speed(BASE_MOTOR_SPEED);
			}else if(rota_type == false){
					rotate(angle_rota);
				}else{
					right_motor_set_speed(BASE_MOTOR_SPEED - speed_rota);
					left_motor_set_speed(BASE_MOTOR_SPEED + speed_rota);
				}
		}else{
			if(get_img_captured()){
			take_puck();
			}
		}
		//chThdSleepMilliseconds(50); // thread each 100ms
		chThdSleepUntilWindowed(time, time + MS2ST(20));
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

void rotate(int angle){
	if(angle >0){
		right_motor_set_speed(-BASE_MOTOR_SPEED);
		left_motor_set_speed(BASE_MOTOR_SPEED);
		if((left_motor_get_pos()) >= (STEPS_TURN)){
			rota_type = true;
			tourne=false;
		}
	}else{
		right_motor_set_speed(BASE_MOTOR_SPEED);
		left_motor_set_speed(-BASE_MOTOR_SPEED);
		if((right_motor_get_pos()) >= (STEPS_TURN)){
			rota_type = true;
			tourne=false;
		}
	}
}

void move(void){
	chThdCreateStatic(waMove, sizeof(waMove), NORMALPRIO, Move, NULL);
}

void take_puck(void){
//	static uint8_t nbr_puck;
	static uint8_t state=1;
	static bool puck=false;

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
		rotate(90); //changes value of tourne
		if(!tourne){
			if(puck){
				state=4;
			}
			else{state=RESET_STEPS;}

		}break;

	case MOVE_TO_PUCK:
		//nbr_puck=get_nbr_lines();
		right_motor_set_speed(BASE_MOTOR_SPEED);
		left_motor_set_speed(BASE_MOTOR_SPEED);

		if(left_motor_get_pos()>/*nbr_puck**/DISTANCE_PUCK){ //check if reach puck before rotating to pick it
			right_motor_set_speed(0);
			left_motor_set_speed(0);
			tourne=!tourne;
			puck=true;
			state=RESET_STEPS;
		}
		break;

	case 4: // end of taking puck ready for obstacle avoidance

		set_bool(GO, 1);
		right_motor_set_speed(BASE_MOTOR_SPEED);
		left_motor_set_speed(BASE_MOTOR_SPEED);

		break;

	default:
		break;
	}

/*	//rotate quart de tour à droite
	static bool debut = true;
	if(debut){
		right_motor_set_pos(0);
		left_motor_set_pos(0);
		debut = false;
	}
	if(tourne){
		rotate(90);
	}
	else{

		right_motor_set_speed(BASE_MOTOR_SPEED);
		left_motor_set_speed(BASE_MOTOR_SPEED);

	}
*/


/*	switch (get_nbr_lines()){

	case 1:


		break;
	case 2:

		break;

	default:
		break;


	}

*/




}


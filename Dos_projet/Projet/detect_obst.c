#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <detect_obst.h>
#include <move.h>
#include <main.h>
#include <motors.h>
#include <process_image.h>

static THD_WORKING_AREA(waDetectObst, 300); // 176 bits needed for the initialization of our variables
static THD_FUNCTION(DetectObst, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;
    systime_t time;
    static int steps_to_reach = 0;
    static bool reset = 0;
	set_bool(GO, 1); // the robot moves
	set_bool(FORWARD, 1);


    while(1){
    	time = chVTGetSystemTime();
    	int16_t speed_PID = 0;
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
    	if (prox_obst != NO_OBST){ // if there is an obstacle, avoid it by rotating and save how much we rotated
			set_bool(FORWARD, 0);
			if (reset==false){
				right_motor_set_pos(0);
				left_motor_set_pos(0);
				reset = true;
			}
			switch(prox_obst){
			case 0: // 90deg to the left
				set_angle_rota(-QUART_DE_TOUR);
				steps_to_reach += (-STEPS_TURN);
				break;
			case 1: // (90-(49-17))deg = 58deg to the left
				set_angle_rota(-QUART_DE_TOUR);
				steps_to_reach += (-STEPS_TURN);
				break;
			case 6: // 58deg to the right
				set_angle_rota(QUART_DE_TOUR);
				steps_to_reach += STEPS_TURN;
				break;
			case 7: // 90deg to the right
				set_angle_rota(QUART_DE_TOUR);
				steps_to_reach += STEPS_TURN;
				break;
			default:
				break;
			}
			chThdSleepMilliseconds(100); // to rotate, constants were set according to 1 sec of rotation
		}

    	// PID BEGINS //compare left to right, if steps to reach > 0 left is higher
    	    if (steps_to_reach != ZERO){ // checks if we've avoided an obstacle and therefore need to rectify
    	    	uint16_t prox_near_obst = 0; // will stock the value of the concerned proximity sensor
    	    	int16_t error = 0; // error for PID
    	    	static float sum_error = 0;
    	    	if (steps_to_reach > 0){	// checks which values' sensor we need to look at, >0 so we turned right
    	    		prox_near_obst = get_calibrated_prox(IR6);
    	    	} else {
    	    		prox_near_obst = get_calibrated_prox(IR3);
    	    		}

    	    	error = OBST_THRES_SIDE - prox_near_obst;

    	    	if(abs(error) < ERROR_THRES){
    	    		error = 0;
    	    	}

    	    	sum_error += error;

    	    	if(sum_error > MAX_SUM_ERROR){	// to avoid an uncontrolled growth, to modify
    	    	sum_error = MAX_SUM_ERROR;
    	    	}else if(sum_error < -MAX_SUM_ERROR){
    	    	sum_error = -MAX_SUM_ERROR;
    	    	}

    	    	speed_PID = KP * error + KI * sum_error; // speed of the right or left wheel to add to the base speed

    	    	if (steps_to_reach > 0){	// adds the speed to the correct wheel, divides it by 2 to split it to both wheels, same result
    	    		set_speed_rota(-speed_PID);
    	    	} else {
    	    		set_speed_rota(speed_PID);
    	    	}

    	    	if(abs(left_motor_get_pos() - right_motor_get_pos()) < STEPS_THRESHOLD){
					steps_to_reach = 0;
					// robot stops rotating and goes forward
					set_bool(ROTA_TYPE, 0);
					set_bool(FORWARD, 1);
					reset = false;
				}
    	    }
    	    	chThdSleepUntilWindowed(time, time + MS2ST(20));
    	    	//chThdSleepMilliseconds(20);

    	    }
}

void detect_obst_start(void){
	chThdCreateStatic(waDetectObst, sizeof(waDetectObst), NORMALPRIO, DetectObst, NULL);
}


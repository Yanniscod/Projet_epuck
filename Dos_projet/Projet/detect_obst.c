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

static THD_WORKING_AREA(waDetectObst, 200); // 200 (80 fonctionne) bits needed (16x8 + 8x1 + 32x2 (float))
static THD_FUNCTION(DetectObst, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;
    systime_t time_1;
    static int16_t steps_to_reach = 0; //bool possible
	static float sum_error = 0.0;
	static float prev_error = 0.0;
	int16_t speed_PID = 0;
	set_bool(GO_DRIBBLE, 1);
	set_bool(FORWARD, 1);

    while(1){
    	time = chVTGetSystemTime();

    	if(get_bool(FORWARD)){
			uint16_t prox_1, prox_2, prox_7, prox_8; // for the values of the proximity sensors
			uint8_t prox_obst = 8; //NO_OBST		 // to know which sensor is the closest to the obstacle, 0 already taken by IR1, so we chose 8
			prox_1=get_calibrated_prox(IR1);
			prox_2=get_calibrated_prox(IR2);
			prox_7=get_calibrated_prox(IR7);
			prox_8=get_calibrated_prox(IR8);

			//detects where is the obstacle, only cares for sensors in front of the e-puck (17' and 49')
			if ((prox_1 > OBST_THRES) || (prox_8 > OBST_THRES)){  //if obstacle in front of IR1/8
				if (prox_1 > prox_8){
					prox_obst = IR1;
				} else {
					prox_obst = IR8;
				}
			} else if((prox_2 > OBST_THRES) || (prox_7 > OBST_THRES)){ //if obstacle in front of IR2/7
				if (prox_2 > prox_7){
					prox_obst = IR2;
				} else {
					prox_obst = IR7;
				}
			}
			// if there is an obstacle, avoids it by a first rotation of 90 deg
			if (prox_obst != NO_OBST){
				set_bool(FORWARD, 0); // goes to rotation mode
				right_motor_set_pos(ZERO_STEP);
				left_motor_set_pos(ZERO_STEP);
				 if((prox_obst == IR1) | (prox_obst == IR2)){
					set_nbr_rota(-ONE_TURN_90_DEG);
					steps_to_reach += (-STEPS_TURN); //stocks how many steps we rotated
				}else{
					set_nbr_rota(ONE_TURN_90_DEG);
					steps_to_reach += STEPS_TURN;
				}
				chThdSleepMilliseconds(50);
			}
		}
    	// PID BEGINS
    	// checks if we've avoided an obstacle and therefore need to rectify
		if ((steps_to_reach != ZERO_STEP) && (get_bool(ROTA_TYPE) == true)){
			uint16_t prox_near_obst = 0; // will stock the value of the concerned proximity sensor
			int16_t error_PID = 0; // error for PID

			if (steps_to_reach > ZERO_STEP){	// checks which values' sensor we need to look at, >0 so we turned right
				prox_near_obst = get_calibrated_prox(IR6);
			} else {
				prox_near_obst = get_calibrated_prox(IR3);
				}
			//chprintf((BaseSequentialStream *)&SD3,"Capteur : %-7d\r\n", prox_near_obst);

			error_PID = OBST_THRES_SIDE - prox_near_obst;

			if(abs(error_PID) < ERROR_THRES){
				error_PID = 0;
			}

			sum_error += error_PID;

			if(sum_error > MAX_SUM_ERROR){	// to avoid an uncontrolled growth
			sum_error = MAX_SUM_ERROR;
			}else if(sum_error < -MAX_SUM_ERROR){
			sum_error = -MAX_SUM_ERROR;
			}

			speed_PID = KP * error_PID + KI * sum_error + KD * (error_PID - prev_error); // speed of the right or left wheel to add to the base speed
			time_1 = chVTGetSystemTime();
			if (steps_to_reach > ZERO_STEP){
				set_speed_rota(-speed_PID);
			} else {
				set_speed_rota(speed_PID);
			}
			prev_error = error_PID;

			if(abs(left_motor_get_pos() - right_motor_get_pos()) < STEPS_THRESHOLD){
				steps_to_reach = 0;
				set_bool(ROTA_TYPE, 0);
				set_bool(FORWARD, 1);
				sum_error = 0;
			}
		}
			//chprintf((BaseSequentialStream *)&SD3,"Time_PID : %-7d\r\n", time_1);
			chThdSleepUntilWindowed(time, time + MS2ST(20));  // 50 Hz, thread lasts around ms
	}

}

void detect_obst_start(void){
	chThdCreateStatic(waDetectObst, sizeof(waDetectObst), NORMALPRIO+1, DetectObst, NULL);
}


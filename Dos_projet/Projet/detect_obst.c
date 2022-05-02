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

//static float kp = 0;
//static float speed2angle = 0.0;

static THD_WORKING_AREA(waDetectObst, 300); // 176 bits needed for the initialization of our variables
static THD_FUNCTION(DetectObst, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;
	//static int16_t cmpt_rot_deg = 0; // goes from -32768 -> 32767
    static int steps_to_reach = 0;
    static bool reset = 0;
	//int16_t speed_PID = 0; // max error of 0.05*MAX_SUM_ERROR + KP * MAX_ERROR = ±756, so 16 bits needed
	set_bool(GO, 1); // the robot moves
	set_bool(FORWARD, 1);

    while(1){
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
    	    			set_angle_rota(-TIERS_DE_TOUR);
    	   				steps_to_reach += (-STEPS_TURN);
    	    			break;
    	    		case 6: // 58deg to the right
    	    			set_angle_rota(TIERS_DE_TOUR);
    	    			steps_to_reach += STEPS_TURN;
    	    			break;
    	    		case 7: // 90deg to the right
    	    			set_angle_rota(QUART_DE_TOUR);
    	    			steps_to_reach += STEPS_TURN;
    	    			break;
    	    		default:
    	    			break;
    	    		}
    	    		chThdSleepMilliseconds(1000); // to rotate, constants were set according to 1 sec of rotation
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

    	    	//chprintf((BaseSequentialStream *)&SD3,"Steps : %-7d Erreur %-7d\r\n", steps_to_reach, error);

    	    	if (steps_to_reach > 0){	// adds the speed to the correct wheel, divides it by 2 to split it to both wheels, same result
    	    		set_speed_rota(-speed_PID);
    	    	} else {
    	    		set_speed_rota(speed_PID);
    	    	}
    	    }
    	    chprintf((BaseSequentialStream *)&SD3,"left : %-7d right : %-7d\r\n", left_motor_get_pos(), right_motor_get_pos());
    	    	if(abs(left_motor_get_pos() - right_motor_get_pos()) < STEPS_THRESHOLD){
    	    		steps_to_reach = 0;
    	    		// robot stops rotating and goes forward
    	    		set_bool(ROTA_TYPE, 0);
    	    		set_bool(FORWARD, 1);
    	    	}

    	    	chThdSleepMilliseconds(20);

    	    }
}
    	//chprintf((BaseSequentialStream *)&SD3,"Capteur d'obstacle = %-7d\r\n", get_calibrated_prox(prox_obst));
/*
    	if (prox_obst != NO_OBST){ // if there is an obstacle, avoid it by rotating and save how much we rotated
    		set_bool(FORWARD, 0);
    		switch(prox_obst){
    		case 0: // 90deg to the left
    			set_angle_rota(-QUART_DE_TOUR);
    			//rotate_one_sec(-QUART_DE_TOUR);
    			cmpt_rot_deg += (-QUART_DE_TOUR);
    			break;
    		case 1: // (90-(49-17))deg = 58deg to the left
    			set_angle_rota(-TIERS_DE_TOUR);
    			//rotate_one_sec(-TIERS_DE_TOUR);
    			cmpt_rot_deg += (-TIERS_DE_TOUR);
    			break;
    		case 6: // 58deg to the right
    			set_angle_rota(TIERS_DE_TOUR);
    			//rotate_one_sec(TIERS_DE_TOUR);
    			cmpt_rot_deg += TIERS_DE_TOUR;
    			break;
    		case 7: // 90deg to the right
    			set_angle_rota(QUART_DE_TOUR);
    			//rotate_one_sec(QUART_DE_TOUR);
    			cmpt_rot_deg += QUART_DE_TOUR;
    			break;
    		default:
    			break;
    		}
    		//chSysUnlock();
    		chThdSleepMilliseconds(1000); // to rotate, constants were set according to 1 sec of rotation
    	}

    	// PID BEGINS
    	if (cmpt_rot_deg != ZERO){ // checks if we've avoided an obstacle and therefore need to rectify, threshold of 5 degrees
    	uint16_t prox_near_obst = 0; // will stock the value of the concerned proximity sensor
    	int16_t error = 0; // error for PID
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

    	//chprintf((BaseSequentialStream *)&SD3, "Error : %-7d prox_obst : %-7d\r\n" , error, prox_near_obst );

    	sum_error += error;

    	if(sum_error > MAX_SUM_ERROR){	// to avoid an uncontrolled growth, to modify
    	sum_error = MAX_SUM_ERROR;
    	}else if(sum_error < -MAX_SUM_ERROR){
    	sum_error = -MAX_SUM_ERROR;
    	}

    	switch(get_selector()){
    		case 0 :
    			kp = 1;
    			break;
    		case 1 :
    			kp = 1.1;
    			break;
    		case 2 :
    			kp = 0.8;
    			break;
    		default :
    			break;
    	}

    	switch(get_selector()){
    	    		case 0 :
    	    			speed2angle = 0.0277;
    	    			break;
    	    		case 1 :
    	    			speed2angle = 0.0235;
    	    			break;
    	    		case 2 :
    	    			speed2angle = 0.02;
    	    			break;
    	    		default :
    	    			break;
    	    	}

    	speed_PID = KP * error + KI * sum_error; // speed of the right or left wheel to add to the base speed

    	//time_1 = chVTGetSystemTime();

    	if (cmpt_rot_deg < 0){	// adds the speed to the correct wheel, divides it by 2 to split it to both wheels, same result
    		set_speed_rota(speed_PID);
    		//right_motor_set_speed(BASE_MOTOR_SPEED - speed_PID); // to go right
    		//left_motor_set_speed(BASE_MOTOR_SPEED + speed_PID);
    		//cmpt_rot_deg += speed_PID*speed2angle;
    		cmpt_rot_deg += speed_PID*SPEEDTOANGLE; // translates the additional speed in degrees, to substract them from the counter
    	} else {
    		set_speed_rota(-speed_PID);
    		//right_motor_set_speed(BASE_MOTOR_SPEED + speed_PID); // to go left
    		//left_motor_set_speed(BASE_MOTOR_SPEED - speed_PID);
    		//cmpt_rot_deg -= speed_PID*speed2angle;
    		cmpt_rot_deg -= speed_PID*SPEEDTOANGLE;
    			}

    	//time_2 = chVTGetSystemTime();
    	chprintf((BaseSequentialStream *)&SD3, "Speed PID: %-7d compteur: %-7d\r\n", speed_PID, cmpt_rot_deg);
    	//chprintf((BaseSequentialStream *)&SD3, "time1 = %-7d time2 = %-7d\r\n",time_1, time_2); // 102ms
    	}

    	if(fabs(cmpt_rot_deg) < DEG_THRES){
    		cmpt_rot_deg = 0;
    		// robot stops rotating and goes forward
    		set_bool(ROTA_TYPE, 0);
    		set_bool(FORWARD, 1);
    	}

    	//chprintf((BaseSequentialStream *)&SDU1,"IR3= %-7d IR6= %-7d\r\n", prox_3, prox_6);
    	//chprintf((BaseSequentialStream *)&SD3,"IR1 = %-7d IR2 = %-7d IR3 = %-7d IR6 = %-7d IR7 = %-7d IR8 = %-7d\r\n",
    	//		prox_1, prox_2, prox_3, prox_6, prox_7, prox_8);

    	chThdSleepMilliseconds(100);
*/
        //100Hz
       // chThdSleepUntilWindowed(time, time + MS2ST(100));
   // }

void detect_obst_start(void){
	chThdCreateStatic(waDetectObst, sizeof(waDetectObst), NORMALPRIO, DetectObst, NULL);
}
/*
 static int steps_to_reach = 0;
 static bool reset = 0;
if (prox_obst != NO_OBST){ // if there is an obstacle, avoid it by rotating and save how much we rotated
    		set_bool(FORWARD, 0);
    		switch(prox_obst){
    		case 0: // 90deg to the left
    			set_angle_rota(-QUART_DE_TOUR);
    			steps_to_reach += (-STEPS_TURN);
    			break;
    		case 1: // (90-(49-17))deg = 58deg to the left
    			set_angle_rota(-TIERS_DE_TOUR);
   				steps_to_reach += (-STEPS_TURN);
    			break;
    		case 6: // 58deg to the right
    			set_angle_rota(TIERS_DE_TOUR);
    			steps_to_reach += STEPS_TURN;
    			break;
    		case 7: // 90deg to the right
    			set_angle_rota(QUART_DE_TOUR);
    			steps_to_reach += STEPS_TURN;
    			break;
    		default:
    			break;
    		}
    		chThdSleepMilliseconds(1000); // to rotate, constants were set according to 1 sec of rotation
    	}

// PID BEGINS //compare left to right, if steps to reach > 0 left is higher
    if (steps_to_reach != ZERO){ // checks if we've avoided an obstacle and therefore need to rectify
    	if (reset==false){
    		right_motor_set_pos(0);
    		left_motor_set_pos(0);
    		reset = true;
    	}
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
    }

    	if(abs(left_motor_get_pos() - right_motor_get_pos()) < STEPS_THRESHOLD){
    		steps_to_reach = 0;
    		// robot stops rotating and goes forward
    		set_bool(ROTA_TYPE, 0);
    		set_bool(FORWARD, 1);
    	}

    	chThdSleepMilliseconds(100);

    }
    */

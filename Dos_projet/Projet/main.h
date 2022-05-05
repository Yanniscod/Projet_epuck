#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"


//constants for the differents parts of the project
#define IMAGE_BUFFER_SIZE	640
#define ZERO 0
#define IR1 0
#define IR2 1
#define IR3 2
#define IR6 5
#define IR7 6
#define IR8 7
#define NO_OBST 8
#define OBST_THRES 200
#define OBST_THRES_SIDE 140
#define ERROR_THRES 20
#define MAX_SUM_ERROR 10000
#define SPEED_WHEEL_45DEG_1S 320	//	step/s
#define QUART_DE_TOUR 90	// degrees
#define SPEEDTOANGLE 0.0235f //1/(DEG_2_STEP/2)*THD_OBST_TIME_MS/1000 * corrective factor for the motor (0.8)
#define TIERS_DE_TOUR 58
#define DEG_2_STEP  7.21f // 7.21 step/deg, obtained experimentally
#define KP 1.80f
#define KI 0.1f
#define BASE_MOTOR_SPEED 350

#define STEPS_TURN 319
#define STEPS_THRESHOLD 15
#define ANGLE_TO_STEPS 3.55f //STEPS_TURN/QUART_DE_TOUR

#define MIN_LINE_WIDTH 30
#define WIDTH_SLOPE 5

#define DISTANCE_PUCK 1200 // valeur en steps à changer
#define ROTATION 2
#define RESET_STEPS 1
#define MOVE_TO_PUCK 3

#define GO 0
#define FORWARD 1
#define ROTA_TYPE 2
/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

void SendUint8ToComputer(uint8_t* data, uint16_t size);
//void detect_obst(void);
//void rotate_one_sec(int angle);
//int speed_2_deg_rota(int speed);
#ifdef __cplusplus
}
#endif

#endif

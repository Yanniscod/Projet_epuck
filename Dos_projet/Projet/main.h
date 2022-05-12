#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"

// DEFINES OF THE PROJECT //
////////////////////////////
//detect_obst.c
//each proximity sensor needed
#define IR1 0
#define IR2 1
#define IR3 2
#define IR6 5
#define IR7 6
#define IR8 7
#define NO_OBST 8
//their values' threshold, set day-to-day according to the light we have, they vary a LOT
#define OBST_THRES 200
#define OBST_THRES_SIDE 170
#define ERROR_THRES 10
//PID
#define MAX_SUM_ERROR 5000 // lower than maximum speed (1000 step/s) divided by KI, so maximum 10000 -> we chose 5000
#define KP 0.50f
#define KI 0.05f
#define KD KP

//move.c
#define GO_DRIBBLE 0
#define FORWARD 1
#define ROTA_TYPE 2
#define ZERO_SPEED 0 //[step/s]
#define BASE_MOTOR_SPEED 350 //[steps/s]
#define ZERO_TURN 0
#define ONE_TURN_90_DEG 1
#define TWO_TURN 2
// steps values set experimentally
#define STEPS_TURN 319 // number of steps needed for a 90 degrees turn
#define STEPS_THRESHOLD 15 //[steps]
#define ZERO_STEP 0 //[steps]
#define DISTANCE_PUCK 800 //[steps], distance between the pucks (=10cm)
//different states used for the first part of the project
#define ZERO_POS 0
#define RESET_STEPS 1
#define ROTATION 2
#define MOVE_TO_PUCK 3
#define GO_TO_GOAL 4
#define END 5
//process_image.c
#define IMAGE_BUFFER_SIZE	640
#define MIN_LINE_WIDTH 30
#define WIDTH_SLOPE 5
#define DETECT_LINE 1
#define DETECT_GOAL 2

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

void SendUint8ToComputer(uint8_t* data, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif

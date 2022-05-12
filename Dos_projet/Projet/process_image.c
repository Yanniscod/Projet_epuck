#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <camera/po8030.h>
#include <motors.h>
#include <process_image.h>
#include <move.h>

//static float distance_cm = 0;
static uint8_t nbr_lines=0;
static uint8_t count_img_line=0;
static bool img_captured = false;
static uint8_t img_to_detect=DETECT_LINE;
static bool ready_to_score=false;
//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;
	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 90 from the top
	po8030_advanced_config(FORMAT_RGB565, 0, 90, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1); //change line of pixel
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();
    while(1){
		dcmi_capture_start(); //starts a capture	
		wait_image_ready();//waits for the capture to be done
		chBSemSignal(&image_ready_sem);//signals an image has been captured
    }
}

static THD_WORKING_AREA(waProcessImage, 1024);
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;
    uint8_t prev_nbr_lines=0;
	uint8_t *img_buff_ptr;
	uint8_t image[IMAGE_BUFFER_SIZE] = {0};
    while(1){
        chBSemWait(&image_ready_sem);//waits until an image has been captured		   
		img_buff_ptr = dcmi_get_last_image_ptr();//gets the pointer to the array filled with the last image in RGB565 

		for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2){	//Extracts only the red pixels
			image[i/2] = (uint8_t)img_buff_ptr[i]&0xF8;
		}
		switch (img_to_detect){
		case DETECT_LINE: //capture image to get puck
			//take several image so can check later if nbr line is always the same
			if(count_img_line<10){
				nbr_lines=0;
				find_nbr_lines(image);// now need to find out what image is by determining how many lines

				if(prev_nbr_lines==nbr_lines){
					count_img_line+=1;//need 10 image in a row with same number of lines to go further
				}
				else{
					count_img_line=0;
				}
				prev_nbr_lines=nbr_lines;
				chThdSleepMilliseconds(200);
			}		
			else if((nbr_lines==1) | (nbr_lines==2)){//image captured is correct( 1 or 2 lines detected)
				img_captured=true;
				img_to_detect=DETECT_GOAL;// once image of lines captured tries to detect the goal
			}
			else{	//image does not have 1 or 2 lines -> retake image
				count_img_line=0;
			}
			break;

		case DETECT_GOAL: //detect black to score goal
			detect_goal(image);
			chThdSleepMilliseconds(50);
			break;

		default:
			break;
		}
		//send to computer to see img with python
		if(send_to_computer)
		{
		//sends to the computer the image
		SendUint8ToComputer(image, IMAGE_BUFFER_SIZE);
		}
		//invert the bool
		send_to_computer = !send_to_computer;		

    }
}
void find_nbr_lines(uint8_t *buffer){

	uint32_t mean = 0;
	uint16_t begin=0, end=0;

	for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE ; i++){//performs an average
		mean += buffer[i];
	}
	mean /= IMAGE_BUFFER_SIZE;
	for (uint16_t i=0; i< IMAGE_BUFFER_SIZE;i++){
		if(buffer[i]>mean && buffer[i+WIDTH_SLOPE]<mean)
		{
			begin=i;
		}
		if(i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE) && begin!=0)//search for end
		{
			if(buffer[i] < mean && buffer[i+WIDTH_SLOPE] > mean)
		    {
				end=i;
				if((end-begin)>MIN_LINE_WIDTH)
				{
					nbr_lines=nbr_lines+1;
					begin=0;
					end=0;
				}
			}
		}
	}		
}

//want 4 images in a row with mean intensity <25:changes with ambient light
void detect_goal (uint8_t *buffer){ 
	static uint16_t mean=0;
	static uint8_t mean_count=0;
	for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE ; i++){
		mean+=buffer[i];
	}
	mean/=IMAGE_BUFFER_SIZE;
	if(mean<DARK_PXL_MEAN){
		mean_count+=1;
	}else{
		mean_count=0;
	}
	if(mean_count>NBR_DARK_IMG){
		set_bool(GO_DRIBBLE,0);
		ready_to_score=true;
	}
}

uint8_t get_nbr_lines(void){
	return nbr_lines;
}
bool get_img_captured(void){
	return img_captured;
}
bool get_ready_to_score(void){
	return ready_to_score;
}
void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}

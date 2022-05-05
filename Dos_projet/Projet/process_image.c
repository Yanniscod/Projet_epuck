#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <camera/po8030.h>

#include <process_image.h>


//static float distance_cm = 0;
static uint8_t nbr_lines=0;
static uint8_t count_img_line=0;
static bool img_captured = false;
//semaphore

static BSEMAPHORE_DECL(image_ready_sem, TRUE);

void leds_nbr_lines(void){


	//img_captured=true;

	switch(nbr_lines){
		case 1:
			palClearPad(GPIOD, GPIOD_LED1);
		break;
		case 2:
			palClearPad(GPIOD, GPIOD_LED3);
		break;
		case 3:
			palClearPad(GPIOD, GPIOD_LED5);
		break;
		case 4:
			palClearPad(GPIOD, GPIOD_LED7);
		break;
		default:
		break;
	}

}
void find_nbr_lines(uint8_t *buffer){

	uint32_t mean = 0;
	uint16_t begin=0, end=0;
	//performs an average
	for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE ; i++){
		mean += buffer[i];
	}
	mean /= IMAGE_BUFFER_SIZE;

	for (uint16_t i=0; i< IMAGE_BUFFER_SIZE;i++){

		if(buffer[i]>mean && buffer[i+WIDTH_SLOPE]<mean)
		{
			begin=i;
		}
		//search for end
		if(i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE) && begin!=0)
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
static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 10 + 11 (minimum 2 lines because reasons)
	po8030_advanced_config(FORMAT_RGB565, 0, 10, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1); //change line of pixel
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

    while(1){
        //starts a capture
		dcmi_capture_start();
		//waits for the capture to be done
		wait_image_ready();
		//signals an image has been captured
		chBSemSignal(&image_ready_sem);
    }
}


static THD_WORKING_AREA(waProcessImage, 1024);
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;
    uint8_t prev_nbr_lines=0;
	uint8_t *img_buff_ptr;
	uint8_t image[IMAGE_BUFFER_SIZE] = {0};
	
	//bool send_to_computer = true;

    while(1){
    	//waits until an image has been captured
        chBSemWait(&image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565    
		img_buff_ptr = dcmi_get_last_image_ptr();

		//Extracts only the red pixels
		for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2){
			//extracts first 5bits of the first byte
			//takes nothing from the second byte
			image[i/2] = (uint8_t)img_buff_ptr[i]&0xF8;
		}		
		//take several image so can check later if nbr line is always the same
		if(count_img_line<10){
			
			nbr_lines=0;
			// now need to find out what image is by determining how many lines
			find_nbr_lines(image);

			if(prev_nbr_lines==nbr_lines){
				count_img_line+=1;//need 10 image in a row with same number of lines to go further
			}
			else{
				count_img_line=0;
			}
			prev_nbr_lines=nbr_lines;
			//chprintf((BaseSequentialStream *)&SD3,"nombre lignes = %-7d\r\n",nbr_lines);
			chThdSleepMilliseconds(1000);
		}
		//mettre la thread en attente si on est parti chercher le puck
		else{

			palSetPad(GPIOB,GPIOB_LED_BODY);
			//leds_nbr_lines();
			img_captured=true;
			chThdSleepMilliseconds(2000);
		}



		/*
		//send to computer to see img with python
		if(send_to_computer)
		{
		//sends to the computer the image
		SendUint8ToComputer(image, IMAGE_BUFFER_SIZE);
		}
		//invert the bool
		send_to_computer = !send_to_computer;		
		*/
    }
}



void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}
uint8_t get_nbr_lines(void){
	return nbr_lines;
}

bool get_img_captured(void){
	return img_captured;
}

#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

float get_distance_cm(void);
void process_image_start(void);
uint8_t get_nbr_lines(void);
bool get_img_captured(void);
void detect_goal_line(uint8_t *buffer);
bool get_ready_to_score(void);
#endif /* PROCESS_IMAGE_H */

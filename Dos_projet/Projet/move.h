#ifndef MOVE_H_
#define MOVE_H_

void move(void);
void rotate(int8_t nbr_90_turns);
void set_nbr_rota(int8_t nbr_90_turns);
void set_speed_rota(int16_t speed);
void set_bool(int8_t bool_num, bool type);
int8_t get_bool(int8_t bool_num);
void take_puck(void);
#endif /* MOVE_H_ */

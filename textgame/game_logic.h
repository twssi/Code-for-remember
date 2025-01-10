//================ game_logic.h ================
#ifndef GAME_LOGIC_H
#define GAME_LOGIC_H

#include "main.h"

// 함수 선언
void intro(void);
void characater_select(void);
const char* player_job(int job);
void player_data(int job);
void monster_data(void);
void battle_data(int damage, int health);
void battle_info(void);

#endif
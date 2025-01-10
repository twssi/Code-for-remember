#include <stdio.h>
#include <math.h>
#include <string.h>

// 구조체 정의
typedef struct Player {
    char            player_name[20];
    int             job;
    int             health;
    int             mana;
    unsigned int    damage;
    int             armor;
} Player;

typedef struct Monster {
    
    int health;
    int mana;
    int damage;
    int armor;
    char name;
    char drop_item;
} Monster;

typedef struct defaultitem {
    char zealot;
    char mage;
} defaultitem;

// 전역 변수 선언
extern Player player;
extern Monster monster;
extern defaultitem playeritem;



void intro(void);
void characater_select(void);
const char* player_job(int job);
void player_data(int job);
void monster_data(void);
void battle_data(int damage, int health);
void battle_info(void);

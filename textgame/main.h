#include <stdio.h>
#include <math.h>
#include <string.h>



#define player_default_level 1

// 직업별 기본 공격력
#define player_zealot_default_damage    25
#define player_mage_default_damage      10
#define player_mercenary_default_damage 20
#define player_paladin_default_damage   15
#define player_rogue_default_damage     25
#define player_inquisitor_damage        20

// 직업별 기본 체력
#define player_zealot_default_health        30
#define player_mage_default_health          15
#define player_mercenary_default_health     20
#define player_paladin_default_health       30
#define player_rogue_default_health         15
#define player_inquisitor_default_health    20 

// 직업별 기본 마나
#define player_zealot_default_mana    15
#define player_mage_default_mana      25
#define player_mercenary_default_mana 10
#define player_paladin_default_mana   20
#define player_rogue_default_mana     10
#define player_inquisitor_mana        15

// 직업별 기본 방어력
#define player_zealot_default_armor    0
#define player_mage_default_armor      5
#define player_mercenary_default_armor 10
#define player_paladin_default_armor   15
#define player_rogue_default_armor     5
#define player_inquisitor_armor        10

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

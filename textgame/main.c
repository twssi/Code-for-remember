#include "main.h"




int main(void)
{

    characater_select();

    return 0;
}



// 전역 변수 정의
Player player;
Monster monster;
defaultitem playeritem = {.zealot = 5, .mage = 5};

void monster_data(void) {
    int monsterhealth = 30;
    int monster_damage = 10;
    int monster_armor = 20;
}

void player_data(int job) {
    if(1 == job) {
        player.health = player_zealot_default_health;
        player.mana = player_zealot_default_mana;
        player.armor = player_zealot_default_armor;
        player.damage = player_zealot_default_damage;
    }
    else if(2 == job) {
        player.health = player_mage_default_health;
        player.mana = player_mage_default_mana;
        player.armor = player_mage_default_armor;
        player.damage = player_mage_default_damage;
    }
    
}

const char* player_job(int job) {
    if(1 == job) return "광신도";
    else if (2 == job) return "마법사";
    else if (3 == job) return "용병";
    else if (4 == job) return "성기사";
    else if (5 == job) return "도적";
    else if (6 == job) return "이단심문관";
    return "알 수 없는 직업";
}

void characater_select(void) {
    unsigned int job;
    char answer = 'n';
   
    while (answer == 'n' || answer == 'N') {
        printf("아바타의 이름을 입력하세요(최대 19자)");
        scanf("%19s", player.player_name);
        while (getchar() != '\n');

        printf("\n직업을 선택하세요 \n");
        printf("1.광신도 2.마법사 3.용병 4.성기사 5.도적 6.이단심문관 \n");
        scanf("%u", &job);

        player.job = job;
        player_data(player.job);

        while (getchar() != '\n');

        printf("당신의 이름은 %s\n", player.player_name);
        printf("직업은 %s 입니다.\n", player_job(player.job));
        printf("이대로 진행 하시겠습니까? (주의! 진행하면 더 이상 변경 할 수 없습니다.) y/n : \n");
        scanf("%s", &answer);
        
        while (getchar() != '\n');

        if (answer != 'y' && answer != 'Y' && answer != 'n' && answer != 'N') {
            printf("\n잘못된 입력입니다. 다시 선택하세요.\n");
            answer = 'n';
        }
    }

    if(answer == 'y' || answer == 'Y') {
        intro();
    }
}

void intro(void) {
    switch(player.job) {
        case 1: //광신도
            printf("\n 맹목적인 신앙심에 모든것을 바친자.\n");
            printf("%s은 이 세계의 신앙속에서 홀로 일어선 선지자 혹은 이단자. 마을의 이단 심판에서도 살아남았다.\n", player.player_name);
            printf("신의 계시를 받은 당신은 신이 당도함을 알리기 위해 타락한 도시에 신의 뜻을 전하려한다.\n");
            break;
        case 2: // 흑마법사
            printf("\n 어둠과 계약한 자.\n");
            printf("%s은 영혼을 대가로 금단의 지식을 얻었으나, 대가는 영원한 마력과 생명에대한 갈증이었다.\n", player.player_name);
            printf("끝없는 영혼을 고치기위해 또 다시 금단의 지식을 찾아 왕국의 타락한 도시로 향한다.\n");
            break;
        // ... 나머지 직업들의 intro
    }

    printf("도시로 향한 %s은 수십 수일의 시간이 흘러 도시 앞 그 누구도 지키고 있지 않은 문 앞에 당도 하였다.\n", player.player_name);
    printf("\n어둡고 적막한 도시의 모습만이 당신을 기다리고 있다. 무엇도 장담 할 수 없는 어두운 그곳에서 각자의 이야기를 따라 흘러가며\n");
    printf("타락한곳에서 모든것이 하나로 모이니.\n");
    printf("그대는...준비되었는가?\n");
}

void battle_data(int damage, int health) {
    // 전투 데이터 처리
}

void battle_info(void) {
    printf("당신은 기습 당했습니다! 전투를 준비하십시오!\n");
    printf("미처 판단하기도 전에 당신은 죽었습니다...\n");
}


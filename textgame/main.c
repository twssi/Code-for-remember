
#include "main.h"
#include "player.h"
#include "monster.h"

// Global objects for the game
Player player;
Monster monster;
defaultitem playeritem;


int main(void)
{

    character_select();

    return 0;
}


void monster_data(void)
{

}

void player_data(int job) 
{
    

    if(1 == job)
    {
        player.health = player_zealot_default_health;
        player.mana = player_zealot_default_mana;
        player.armor = player_zealot_default_armor;
        player.damage = player_zealot_default_damage;


    }

    else if(2 == job) 
    {
        player.health = player_mage_default_health;
        player.mana = player_mage_default_mana;
        player.armor = player_mage_default_armor;
        player.damage = player_mage_default_damage;

   
    }

    else if(3 == job)
    {
        player.health = player_mercenary_default_health;
        player.mana = player_mercenary_default_mana;
        player.armor = player_mercenary_default_armor;
        player.damage = player_mercenary_default_damage;
        

    }
    
    else if(4 == job)
    {
        player.health = player_paladin_default_health;
        player.mana = player_paladin_default_mana;
        player.armor = player_paladin_default_armor;
        player.damage = player_paladin_default_damage;
        

    }

    else if(5 == job)
    {
        player.health = player_rogue_default_health;
        player.mana = player_rogue_default_mana;
        player.armor = player_rogue_default_armor;
        player.damage = player_rogue_default_damage;

      
    }

    else if(6 == job)
    {
        player.health = player_inquisitor_default_health;
        player.mana = player_inquisitor_mana;
        player.armor = player_inquisitor_armor;
        player.damage = player_inquisitor_damage;

      
    }


}

const char* player_job(int job) 
{
    if(1 == job) return "광신도";
    else if (2 == job) return "마법사";
    else if (3 == job) return "용병";
    else if (4 == job) return "성기사";
    else if (5 == job) return "도적";
    else if (6 == job) return "이단심문관";
    return "알 수 없는 직업";
}

void character_select(void)
{
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

void intro(void)
{
    switch(player.job)
    {
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
        case 3: // 용병
            printf("\n악랄한 돈의 전사.\n");
            printf("%s은 고용한 고용주의 돈을 위하여 싸웠다. 하지만 당신을 고용한 고용주의 배신에의해 모든걸 빼앗겼다.\n", player.player_name);
            printf("아무것도 남지 않은 당신은 타락한 왕국의 도시 속에서 또 다른 기회를 찾았다. \n");
            break;
        case 4: // 성기사
            printf("\n성기사: 타락한 왕국의 성스러운 기사.\n");
            printf("%s은 왕국을 수호하겠다는 신성한 다짐을 하였지만 거대한 타락의 기운을 당신 홀로 막아낼 순 없었다.\n", player.player_name);
            printf("신성했던 도시의 모습과 왕국의 옛 영광을 재건하고 신의 뜻을 받들고자 당신은 타락의 기운이 나오는 도시로 홀로 향했다.\n");
            break;
        case 5: // 도적
            printf("\n 가진것을 빼앗고 속이는 더러운 길거리의 무법자.\n");
            printf("%s은 타락한 도시로 흘러가는 부랑자, 모험자들을 털어 재물을 챙기는 도시 속 쓰레기다.\n", player.player_name);
            printf("이 도시를 찾아오는 어리숙한 부랑자들이 줄어들자 타락의 기운이 깃든 도시속 보물을 손에 넣을 생각 뿐이다.\n");
            break;
        case 6: // 이단심문관
            printf("\n 신의 이름을 빌어 이단을 처단 하는자.\n");
            printf("%s은 신의 이름 아래에서 모든 이단자를 처벌하기로 맹세한 당신은 신의 이름아래 이단자와 마녀라는 사람들을 처단해왔다.\n", player.player_name);
            printf("이 타락한 도시 속 이단의 근원을 찾아 없애기 위해 스스로 타락한 도시속으로 향한다.\n");
            break;
   
        default:
            printf("\n알 수 없는 직업입니다.\n");
            break;
    }

    printf("도시로 향한 %s은 수십 수일의 시간이 흘러 도시 앞 그 누구도 지키고 있지 않은 문 앞에 당도 하였다.\n", player.player_name);
    printf("\n어둡고 적막한 도시의 모습만이 당신을 기다리고 있다. 무엇도 장담 할 수 없는 어두운 그곳에서 각자의 이야기를 따라 흘러가며\n");
    printf("타락한곳에서 모든것이 하나로 모이니.\n");
    printf("그대는...준비되었는가?\n");
    display_ascii_art();
}


void combat_senario(void)
{
    printf("당신은 어두운 도시의 입구에 들어 섰습니다. 눈 앞에 보이는건 희미한 불빛이 새어 나오는 술집, 무언가 형체가 보이는 골목, 저 멀리 보이는 본성으로 향하는 큰길입니다.\n");
    printf("당신은 어떤 길을 선택하시겠습니까? 1. 술집 2. 골목 3. 큰길\n");
    
}


void battle_data(Player player, Monster monster)
{


    if(player.health >0 && monster.health)
    {
        printf("적을 만났다! \n");


    }
}

void battle_info(void)
{
    printf("당신은 기습 당했습니다! 전투를 준비하십시오!\n");
    printf("미처 판단하기도 전에 당신은 죽었습니다...\n");
}

void display_ascii_art(void)
{
    printf("   O   \n");
    printf("  /|\\  \n");
    printf("  / \\  \n");
}


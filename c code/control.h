#ifndef _CONTROL_H
#define _CONTROL_H

#define NODE_NONE       0x00 //00000000

#define NODE_LEFT 		0x01 //00000001
#define NODE_RIGHT		0x02 //00000010
#define NODE_CROSS 	    0x04 //00000100
#define NODE_T          0x08 //00001000
#define NODE_TERMINAL   0x10 //00010000

#define VELO_DEFAULT 750
#define ACC_WHEEL   200000
#define ACC_ROBOT   500
#define ACC_P       400.f
#define LIMDT       9000000
#define POS_COEFF   .3f
#define PI          3.141592653589793238462643383279f
#define WHEEL_RAD   0.025f*992.5f/1000.f // Wheel radius in meter
#define NANOSEC     1000000000
#define TICK2DEG    4.75f
#define TICK_S_W    400                  // Ticks between sensors and wheels

// 
void moveTicks(int64_t ticks);

// 
void rotate(float degree);

// Go untill any node appears
int8_t goUntillNode();

// 
int initControl();

// 
void endControl();

// Return total distance from device boot.
int64_t getDistance();

/* 사용 예시
int main(){
    if(initControl()==-1){
        ERR("Cannot initialize controller.");
        return -1;
    }

    int8_t node;
    while(1){
        // 노드가 나타날 때까지 움직인다.
        LOG("MOVE");
        node = goUntillNode();

        LOG("TURN");
        // 만약 왼쪽이 검출되면 왼쪽으로 돈다.
        if(node==NODE_LEFT) rotate(90);
        // 만약 오른쪽이 검출되면 오른쪽으로 돈다.
        else if(node==NODE_RIGHT)rotate(-90);
        // 그 외의 노드가 나오면 루프 탈출 
        else break;
    }

    // 끝냄
    endControl();
    return 0;
}
*/

#endif
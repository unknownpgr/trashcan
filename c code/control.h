#ifndef _CONTROL_H
#define _CONTROL_H

#define NODE_NONE       0x00 //00000000
#define NODE_LEFT 		0x01 //00000001
#define NODE_RIGHT		0x02 //00000010
#define NODE_CROSS 	    0x04 //00000100
#define NODE_T          0x08 //00001000
#define NODE_T_RIGHT    0x10 
#define NODE_T_LEFT     0x20
#define NODE_TERMINAL   0x40 

#define VELO_DEFAULT 1000.0
#define ACC_WHEEL   2000.0
#define ACC_ROBOT   500.0
#define ACC_P       400.0
#define LIMDT       20000000.0
#define POS_COEF   .3
#define PI          3.141592653589793238462643383279
#define WHEEL_RAD   0.025*992.5/1000.0 // Wheel radius in meter
#define NANOSEC     1000000000
#define TICK2DEG    4.75
#define TICK_S_W    400                  // Ticks between sensors and wheels

// 1 tick = 1 step of stepmotor
void moveTicks(int64_t ticks);

// 
void rotate(double degree);

// Go untill any node appears. return accumulated state.
int8_t goUntillNode();

// Recognize node type from accumulated state and current sensor state.
int8_t recognizeNode(int8_t state);

// 
int initControl();

// 
void endControl();

// Return total distance from device boot.
int64_t getDistance();

/* 사용 예시
int main(){
    // Initialize controller
    if(initControl()==-1){
        ERR("Cannot initialize controller.");
        return -1;
    }

    int8_t state, node;

    while(1){
        // Move to any node
        LOG("MOVE");
        currVelo = destVelo = 0;
        align();

        // Get node type
        state = goUntillNode();
        node  = recognizeNode(state);

        // Make decision
        LOG("TURN");
        currVelo = destVelo = 0;
        if(node==NODE_LEFT)         rotate(90);
        else if(node==NODE_RIGHT)   rotate(-90);
        else break;
    }

    #define NODE_CASE(NODE) case NODE : LOG(#NODE); break;
    switch(node){
        NODE_CASE(NODE_LEFT);
        NODE_CASE(NODE_RIGHT);
        NODE_CASE(NODE_CROSS);
        NODE_CASE(NODE_T);
        NODE_CASE(NODE_T_LEFT);
        NODE_CASE(NODE_T_RIGHT);
        NODE_CASE(NODE_TERMINAL);
    }

    endControl();
    return 0;
}
*/

#endif
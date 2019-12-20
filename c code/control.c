#include <sys/shm.h>
#include <sys/types.h>
#include <stdio.h>
#include <unistd.h>
#include <sched.h>
#include <math.h>

#include "controlProtocol.h"
#include "log.h"
#include "interval.h"
#include "control.h"

#define ABS(x) (((x)>0)?(x):(-(x)))
#define SIGN(x) (((x)>0)?1:(((x)<0)?(-1):0))

#define BOOL(x) (x)?"True":"False"

int sleep_ms(int ms){
    for(int i =0;i<ms;i++){
        usleep(1000);
    }
}

/*
ACCELERATE : used in loop with delay.
curr is the variable to be controlled and dest is the destination value.
Increase or decrease the 'curr' to the slope of 'acc' until the 'curr' reaches 'dest'.
*/
#define ACCELERATE(curr, dest, acc, dt){if((curr)<(dest)){(curr) += (acc)*(dt);if((curr)>(dest))(curr)=(dest);}else if((curr)>(dest)){(curr) -= (acc)*(dt);if((curr)<(dest))(curr)=(dest);}}
double ACCELERATE_(double curr,double dest,double acc,double dt){
    if((curr)<(dest)){
        curr+=acc*dt;
        if(curr>dest) return dest;
        else          return curr;
    }else if((curr)>(dest)){
        curr-=acc*dt;
        if(curr<dest) return dest;
        else          return curr;
    }
}

/*
ABS_LIM : Limit the range of value in [-abslim,abslim].
if value is larger than abslim or smaller than -abslim, make it abslim or -abslim.
*/
#define ABS_LIM(value,abslim){if((value)>0&&(value)>(abslim))(value)=(abslim);if((value)<0&&(value)<-(abslim))(value)=-(abslim);}
double absLimF(double value,double absLim){
    if(value>0&&value>absLim)return absLim;
    if(value<0&&value<(-absLim))return -absLim;
    return value;
}

int exec(char* program){
    pid_t pid=fork();
    if (pid==0) { /* child process */
        execv(program,NULL);
        ERR("Cannot execute %s",program);
        return -1;
    }
    return 0;
}

SHM_CONTROL* control;

double destVelo = 0;
double currVelo = 0;

int main_remote(){
    exec("./comm.o");
    LOG("Server alive : %s",BOOL(control->serverAlive));

    double
        cvl,cvr,    // Current velocity
        dvl,dvr,    // Destination velocity
        dtl,dtr;    // Delta time (the reciprocal of current velocity)

    for(;;){
        sleep_ms(10);

        dvl = control->userVL;
        dvr = control->userVR;

        ACCELERATE(cvl,dvl,ACC_WHEEL,0.01f);
        ACCELERATE(cvr,dvr,ACC_WHEEL,0.01f);

        if(cvl!=0) dtl = NANOSEC/cvl;
        else dtl = 0;
        if(cvr!=0) dtr = NANOSEC/cvr;
        else dtr = 0;

        if(cvl==-1||cvr==-1)break;

        // Calculate the velocity
        // v = 1000000/dt
        // :. dt = 1000000/v
        // minv = 200
        // :. maxdt = 1000000/200
        // :. maxdt = 10000/2 = 5000

        ABS_LIM(dtl,LIMDT);
        ABS_LIM(dtr,LIMDT);

        control->dtL = (int)dtl;
        control->dtR = (int)dtr;
    }
}

int main_lineTracing(){

    INTERVAL interval;
    interval.recent = 0;
    interval.interval = 1000000;
    double   intervalSec = interval.interval*1/NANOSEC; // Interval in sec 

    // Control variable
    double veloL, veloR, dtL, dtR;
    destVelo = VELO_DEFAULT;

    INTERVAL_LOOP{
        RUN_TASK(interval,
            if(control->lineout) veloL=veloR=0;
            else{
                ACCELERATE(currVelo,destVelo,ACC_ROBOT,intervalSec);
                veloL = currVelo*(1+control->position*POS_COEFF);
                veloR = currVelo*(1-control->position*POS_COEFF);
            }

            if(veloL!=0) dtL = (NANOSEC/veloL);
            else         dtL = 0;
            if(veloR!=0) dtR = (NANOSEC/veloR);
            else         dtR = 0;

            ABS_LIM(dtL,LIMDT);
            ABS_LIM(dtR,LIMDT);

            control->dtL = (int64_t)dtL;
            control->dtR = (int64_t)dtR;
        );
    }
}

// Todo : implement it
void align(){
    // p-control value
    double   pGain    = 10.f;
    int64_t err = -control->position;
    int     errSign  = SIGN(err);
    
    // Control variable
    double veloL, veloR, dtL, dtR;

    INTERVAL interval;
    interval.recent = 0;
    interval.interval = 100000;
    double intervalSec = interval.interval*1.0/NANOSEC; // Interval in sec 

    INTERVAL_LOOP{
        RUN_TASK(interval,

            // P-control
            err = -control->position;
            destVelo = err*pGain;
            ACCELERATE(currVelo,destVelo,ACC_ROBOT/2,intervalSec);

            if(errSign*err<=0)return;

            veloL = -currVelo;
            veloR = currVelo;

            if(veloL!=0) dtL = (NANOSEC/veloL);
            else         dtL = 0;
            if(veloR!=0) dtR = (NANOSEC/veloR);
            else         dtR = 0;

            ABS_LIM(dtL,LIMDT);
            ABS_LIM(dtR,LIMDT);

            control->dtL = (int64_t)dtL;
            control->dtR = (int64_t)dtR;
        );
    }
}

// Move by given ticks. 1 tick = 2PI*WHEEL_RAD/400.
/*

*/
void moveTicks(int64_t ticks){
    // p-control value
    int64_t destTick = control->tickC+ticks*2;
    double   pGain    = .5f;

    // Control variable
    double veloL, veloR, dtL, dtR;

    INTERVAL interval;
    interval.recent = 0;
    interval.interval = 1000000;
    double   intervalSec = interval.interval*1/NANOSEC; // Interval in sec 

    INTERVAL_LOOP{
        RUN_TASK(interval,
            // P-control
            int64_t err = destTick-control->tickC;
            if(err<=0)break;
            destVelo = err*pGain;

            if(control->lineout) veloL=veloR=0;
            else{
                ACCELERATE(currVelo,destVelo,ACC_ROBOT,intervalSec);
                veloL = currVelo*(1+control->position*POS_COEFF);
                veloR = currVelo*(1-control->position*POS_COEFF);
            }

            if(veloL!=0) dtL = (NANOSEC/veloL);
            else         dtL = 0;
            if(veloR!=0) dtR = (NANOSEC/veloR);
            else         dtR = 0;

            ABS_LIM(dtL,LIMDT);
            ABS_LIM(dtR,LIMDT);

            control->dtL = (int64_t)dtL;
            control->dtR = (int64_t)dtR;
        );
    }
}

void moveMeter(double meter){
    int64_t ticks = (meter/(PI*WHEEL_RAD))*200;
    LOG("Move meter : %f, Ticks : %lld",meter,ticks);
    moveTicks(ticks);
}

void rotate(double degree){
    // p-control value
    int64_t destTick = control->tickR+(int64_t)(degree*TICK2DEG);
    double   pGain    = 1;
    int64_t err = destTick-control->tickR;
    int     errSign  = SIGN(err);
    
    // Control variable
    double veloL, veloR, dtL, dtR;

    INTERVAL interval;
    interval.recent = 0;
    interval.interval = 100000;
    double intervalSec = interval.interval*1.0/NANOSEC; // Interval in sec 

    INTERVAL_LOOP{
        RUN_TASK(interval,
            // P-control
            err = destTick-control->tickR;
            destVelo = err*pGain;
            ACCELERATE(currVelo,destVelo,ACC_ROBOT/2,intervalSec);

            if(errSign*err<=0)return;

            veloL = -currVelo;
            veloR = currVelo;

            if(veloL!=0) dtL = (NANOSEC/veloL);
            else         dtL = 0;
            if(veloR!=0) dtR = (NANOSEC/veloR);
            else         dtR = 0;

            ABS_LIM(dtL,LIMDT);
            ABS_LIM(dtR,LIMDT);

            control->dtL = (int64_t)dtL;
            control->dtR = (int64_t)dtR;
        );
    }
}

int64_t getDistance(){
    return control->tickC;
}

int8_t stop(int64_t ticks){
    INTERVAL interval;
    interval.recent = 0;
    interval.interval = 1000000;
    double   intervalSec = interval.interval*1.0/NANOSEC; // Interval in sec 

    // Control variable
    double dt;
    destVelo = VELO_DEFAULT;

    // P-control for stop
    int64_t destDist = control->tickC+ticks;
    int64_t error    = ticks; 
    double pGain = currVelo/ticks;

    int8_t accum = 0x00;

    int i = 0;

    INTERVAL_LOOP{
        RUN_TASK(interval,

            i++;
            if(!(i%100)){
                // LOG("VELO : %f",currVelo);  
            }

            error = destDist-control->tickC;
            currVelo = error*pGain;
            if(currVelo<=.5f)break;

            // ACCELERATE(currVelo,destVelo,ACC_ROBOT,intervalSec);

            if(currVelo!=0) dt = (NANOSEC/currVelo);
            else        dt = 0;
            ABS_LIM(dt,LIMDT);

            control->dtL = (int64_t)dt;
            control->dtR = (int64_t)dt;

            accum|=control->sensorState;
        );
    }

    return accum;
}

// Go untill any node appears
int8_t goUntillNode(){
    INTERVAL interval;
    interval.recent = 0;
    interval.interval = 1000000;
    double intervalSec = interval.interval*1.0/NANOSEC; // Interval in sec 
    printf("intervalSec : %f\n",intervalSec);

    // Control variable
    double veloL, veloR, dtL, dtR;
    destVelo = VELO_DEFAULT;

    int i = 0;

    INTERVAL_LOOP{
        RUN_TASK(interval,

            if(control->node!=0x00) {
                printf("%x\n",control->sensorState);
                break;
            }

            if(control->lineout){
                currVelo = ACCELERATE_(currVelo, 0, ACC_ROBOT, intervalSec);
                veloL = currVelo;
                veloR = currVelo;
            }
            else{
                currVelo = ACCELERATE_(currVelo, destVelo, ACC_ROBOT, intervalSec);
                veloL = currVelo*(1+control->position*POS_COEFF);
                veloR = currVelo*(1-control->position*POS_COEFF);
            }

            if(veloL!=0) dtL = (NANOSEC/veloL);
            else         dtL = 0;
            if(veloR!=0) dtR = (NANOSEC/veloR);
            else         dtR = 0;

            dtL = absLimF(dtL,LIMDT);
            dtR = absLimF(dtR,LIMDT);

            control->dtL = (int64_t)dtL;
            control->dtR = (int64_t)dtR;

            i++;
            if(!(i%100)){
                // printf("Lineout? %d\n",control->lineout);
                // printf("DTL : %lld DTR %lld\n",control->dtL,control->dtR);  
                LOG("VL: %f, VR: %f",veloL,veloR);
            }

        );
    }

    return stop(TICK_S_W);
}

#define STATE_LEFT      0x01 // 0x01 : 00 000001
#define STATE_RIGHT     0x20 // 0x20 : 00 100000
#define STATE_CROSS     0x3F // 0x3F : 00 111111
#define STATE_CENTER    0x1E // 0x1E : 00 011110

int initControl(){
    LOG("Start controller.");

    // Get control object from shared memory.
    control = getControlStruct();

    if((int)control<0){
        ERR("Cannot get shared memory. err code : %d",control);
        return -1;
    }

    // Exit existing child processes
    control->exit = 1;
    sleep_ms(100);
    control->exit = 0;
    sleep_ms(100);

    initControlStruct(control);
    LOG("Control structure initialized.")

    // Start child process
    exec("./motor.o");
    while(!control->motorAlive)sleep_ms(10);
    LOG("Motor activated.");

    exec("./sensor.o");
    while(!control->sensorAlive)sleep_ms(10);
    LOG("Sensor activated.");

    control->dtL = 0;
    control->dtR = 0;
    control->run = 1;

    sleep_ms(100);

    LOG("Start tracing");
    return 0;
}

void endControl(){
    LOG("Turn off motor");
    control->run=0;
    sleep_ms(100);

    LOG("Exit all subprocess");
    control->exit=1;
    sleep_ms(100);

    LOG("Motor alive : %s",BOOL(control->motorAlive));

    if(removeControlStruct(control)==-1){ERR("Cannot remove shared memory.");}
    else LOG("Shared memory removed.");

    LOG("Exit control control.");
}

int8_t recognizeNode(int8_t state){
    int8_t node = 0x00;
    int8_t line = control->sensorState&STATE_CENTER;
    if((state&STATE_CROSS)==STATE_CROSS){
        if(line) node = NODE_CROSS;
        else     node = NODE_T;
    }else{
        if(state&STATE_LEFT){
            if(line) node = NODE_T_LEFT;
            else     node = NODE_LEFT;
        }
        else if(state&STATE_RIGHT){
            if(line) node = NODE_T_RIGHT;
            else     node = NODE_RIGHT;
        }
        else         node = NODE_TERMINAL;
    }
    return node;
}

int main(){
    if(initControl()==-1){
        ERR("Cannot initialize controller.");
        return -1;
    }

    int8_t state, node;

    while(1){
     LOG("MOVE");
        currVelo = destVelo = 0;
        align();
        state = goUntillNode();
        node = recognizeNode(state);

        LOG("TURN");
        currVelo = destVelo = 0;
        if(node==NODE_LEFT) rotate(90);
        else if(node==NODE_RIGHT)rotate(-90);
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
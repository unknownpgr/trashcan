#include <stdio.h>
#include <unistd.h>
#include <sched.h>
#include <sys/shm.h>
#include <sys/types.h>
#include <math.h>

#include "controlProtocol.h"
#include "log.h"
#include "interval.h"

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

/*
ABS_LIM : Limit the range of value in [-abslim,abslim].
if value is larger than abslim or smaller than -abslim, make it abslim or -abslim.
*/
#define ABS_LIM(value,abslim){if((value)>0&&(value)>(abslim))(value)=(abslim);if((value)<0&&(value)<-(abslim))(value)=-(abslim);}

int exec(char* program){
    pid_t pid=fork();
    if (pid==0) { /* child process */
        execv(program,NULL);
        ERR("Cannot execute %s",program);
        return -1;
    }
    return 0;
}

#define VELO_DEFAULT 750
#define ACC_WHEEL   200000
#define ACC_ROBOT   500
#define ACC_P       400.f
#define LIMDT       9000000
#define POS_COEFF   .3f
#define PI          3.141592653589793238462643383279f
#define WHEEL_RAD   0.025f*992.5f/1000.f // Wheel radius in meter
#define NANOSEC     1000000000
#define TICK2DEG    5.30f

SHM_CONTROL* control;

float destVelo = 0;
float currVelo = 0;

int main_remote(){
    exec("./comm.o");
    LOG("Server alive : %s",BOOL(control->serverAlive));

    float
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

        if(cvl==-1.f||cvr==-1.f)break;

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
    float   intervalSec = interval.interval*1.f/NANOSEC; // Interval in sec 

    // Control variable
    float veloL, veloR, dtL, dtR;
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
int align(){
    INTERVAL interval;
    interval.recent = 0;
    interval.interval = 1000000;
    float   intervalSec = interval.interval*1.f/NANOSEC; // Interval in sec 

    // Control variable
    float veloL, veloR, dtL, dtR;
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

// Move by given ticks. 1 tick = 2PI*WHEEL_RAD/400.
/*

*/
void moveTicks(int64_t ticks){
    // p-control value
    int64_t destTick = control->tickC+ticks*2;
    float   pGain    = .5f;

    // Control variable
    float veloL, veloR, dtL, dtR;

    INTERVAL interval;
    interval.recent = 0;
    interval.interval = 1000000;
    float   intervalSec = interval.interval*1.f/NANOSEC; // Interval in sec 

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

void moveMeter(float meter){
    int64_t ticks = (meter/(PI*WHEEL_RAD))*200;
    LOG("Move meter : %f, Ticks : %lld",meter,ticks);
    moveTicks(ticks);
}

void rotate(float degree){
    // p-control value
    int64_t destTick = control->tickR+(int64_t)(degree*TICK2DEG);
    float   pGain    = 2.f;
    int64_t errH     = (destTick-control->tickR)/2;
    int     errSign  = SIGN(errH);
    errH = ABS(errH);

    // Control variable
    float veloL, veloR, dtL, dtR;

    INTERVAL interval;
    interval.recent = 0;
    interval.interval = 100000;

    INTERVAL_LOOP{
        RUN_TASK(interval,
            // P-control
            int64_t err = destTick-control->tickR;
            
            if(ABS(err)<=1){
                destVelo=currVelo=0;
                control->dtL = (int64_t)0;
                control->dtR = (int64_t)0;    
                break;
            }

            // destVelo = err*pGain;
            if(ABS(err)>errH){
                destVelo += ACC_P * 1.f * errSign * interval.interval / NANOSEC;
            }else{
                destVelo -= ACC_P * 0.98f * errSign * interval.interval / NANOSEC;
            }

            veloL = -destVelo;
            veloR = destVelo;

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

//Print bit of int.
void printBit8(int x){
    printf("0b");
    for(int i = 7; i>=0;i--) printf("%d",1&&(x&1<<i));
    printf("\n");
}

int8_t stop(int64_t ticks){
    INTERVAL interval;
    interval.recent = 0;
    interval.interval = 1000000;
    float   intervalSec = interval.interval*1.f/NANOSEC; // Interval in sec 

    // Control variable
    float dt;
    destVelo = VELO_DEFAULT;

    // P-control for stop
    int64_t destDist = control->tickC+ticks;
    int64_t error    = ticks; 
    float pGain = currVelo/ticks;

    int8_t accum = 0x00;

    INTERVAL_LOOP{
        RUN_TASK(interval,
            error = destDist-control->tickC;
            if(error<=0)break;
            currVelo = error*pGain;

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

void goUntillMark(){
    INTERVAL interval;
    interval.recent = 0;
    interval.interval = 1000000;
    float intervalSec = interval.interval*1.f/NANOSEC; // Interval in sec 

    // Control variable
    float veloL, veloR, dtL, dtR;
    destVelo = VELO_DEFAULT;

    INTERVAL_LOOP{
        if(control->mark!=0x00) return;
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

#define MARK_NONE       0x00 //00000000

#define MARK_LEFT 		0x01 //00000001
#define MARK_RIGHT		0x02 //00000010
#define MARK_CROSS 	    0x04 //00000100
#define MARK_T          0x08 //00001000
#define MARK_TERMINAL   0x10 //00010000

#define STATE_LEFT      0x01 // 0x01 : 00 000001
#define STATE_RIGHT     0x20 // 0x20 : 00 100000
#define STATE_CROSS     0x3F // 0x3F : 00 111111
#define STATE_CENTER    0x1E // 0x1E : 00 011110

int main(){
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
    // moveMeter(1.0f);
    // main_lineTracing();

    goUntillMark();
    int8_t state = stop(400);
    int8_t mark  = 0x00;

    if((state&STATE_CROSS)==STATE_CROSS){
        if(control->sensorState&STATE_CENTER)   mark = MARK_CROSS;
        else                                    mark = MARK_T;
    }else{
             if(state&STATE_LEFT)               mark = MARK_LEFT;
        else if(state&STATE_RIGHT)              mark = MARK_RIGHT;
        else                                    mark = MARK_TERMINAL;
    }

    #define MARK_CASE(MARK) case MARK : LOG(#MARK); break;
    switch(mark){
        MARK_CASE(MARK_LEFT);
        MARK_CASE(MARK_RIGHT);
        MARK_CASE(MARK_CROSS);
        MARK_CASE(MARK_T);
        MARK_CASE(MARK_TERMINAL);
    }

    /* Rotation test
    destVelo=currVelo=0;
    control->dtL = (int64_t)0;
    control->dtR = (int64_t)0;    
    rotate(180.f);
    LOG("TURNED");
    sleep_ms(500);
    destVelo=currVelo=0;
    control->dtL = (int64_t)0;
    control->dtR = (int64_t)0;    
    rotate(-180.f);
    */
    
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
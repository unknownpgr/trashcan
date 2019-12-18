#include <stdio.h>
#include <unistd.h>
#include <sched.h>
#include <sys/shm.h>
#include <sys/types.h>
#include <math.h>
#include "controlProtocol.h"
#include "log.h"

#define ABS(x) (((x)>0)?(x):(-(x)))
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

#define VELO        1500
#define ACC_WHEEL   200000
#define ACC_ROBOT   2000
#define LIMDT       9000000
#define POS_COEFF   .3f
#define PI          3.141592653589793238462643383279f
#define WHEEL_RAD   0.025f*992.5f/1000.f // Wheel radius in meter
#define NANOSEC     1000000000

SHM_CONTROL* control;

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

    float
        cvl,cvr,    // Current velocity
        dvl,dvr,    // Destination velocity
        dtl,dtr;    // Delta time (the reciprocal of current velocity)

    for(int i =0;;i++){
        if(control->lineout){
            dvl=dvr=0;
        }else{
            dvl = VELO*(1+control->position*POS_COEFF);
            dvr = VELO*(1-control->position*POS_COEFF);
        }

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

        // Print some values for debugging
        // if(!(i%1000000)) printf("%f, %f\n",cvl,cvr);

        control->dtL = (int)dtl;
        control->dtR = (int)dtr;
    }
}

// Do p-control with motor.
void moveTicks(int64_t ticks){

    // p-control value
    float destVelo = 0;
    float currVelo = 0;

    int64_t destTick = control->tickC+ticks*2;
    float   pGain    = .5f;

    // Interval control for accurate acceleration and deceleration
    struct timespec time;
    clock_gettime(CLOCK_REALTIME, &time);

    int32_t currTime    = time.tv_nsec;          // Current time
    int32_t recentTime  = time.tv_nsec;          // Recent task-executed time
    int32_t interval    = 1000000;               // 1000000ns = 1ms
    float   intervalSec = interval*1.f/NANOSEC; // Interval in sec 
    int32_t dt          = 0;                     // currTime - recentTime

    // Control variable
    float veloL, veloR, dtL, dtR;

    for(int i =0;;){
        clock_gettime(CLOCK_REALTIME, &time);
        currTime = time.tv_nsec;
        dt = currTime-recentTime;
        if(dt<0)dt+=NANOSEC;
        if(dt<interval){
            usleep(1);
            continue;
        }
        recentTime += interval;
        if(recentTime>NANOSEC)recentTime-=NANOSEC;
        i++;

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

        // Print some variables for debugging
        // if(!(i%500)){
        //     printf("Lineout : %d, Position : %f\n",control->lineout,control->position);
        //     printf("error : %lld, tick : %lld\n",err,control->tickC);
        //     printf("dv:%f, cv:%f\n",destVelo,currVelo);
        //     printf("vL : %f, vR : %f\n",veloL,veloR);
        //     printf("dtL:%f, dtR:%f\n",dtL,dtR);
        //     LOG("%d",i);
        // }
    }
}

void moveMeter(float meter){
    int64_t ticks = (meter/(PI*WHEEL_RAD))*200;
    LOG("Move meter : %f, Ticks : %lld",meter,ticks);
    moveTicks(ticks);
}

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
    sleep_ms(100);
    exec("./motor.o");
    sleep_ms(100);
    exec("./sensor.o"); 
    LOG("Motor alive : %s",BOOL(control->motorAlive));
    LOG("Sensor alive : %s",BOOL(control->sensorAlive));

    // control->dtL = 0;
    // control->dtR = 0;
    control->run = 1;

    sleep_ms(100);
// 
    moveMeter(1.0f);
    // main_lineTracing();
    
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
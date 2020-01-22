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

/*
 이전의 제어 방식은 position 값이 급작스럽게 변하면서 로봇이 튀는 문제가 있었다.
 따라서 부드러운 position 변화량을 가지는 제어 방식이 있다면 좋을 듯하다.
 간단히 IIR을 사용하여 LPF를 구현해도 되지만, 그것보다는 일반적으로 position을 사용하지 않는 제어에서도
 같은 값을 사용하여 일관성을 유지할 수 있으면 좋겠다.
 그러기 위해서, 다음과 같은 경우를 살핀다.
 vL = vC*(1+pos)
 vR = vC*(1-pos)
 이 식은 실수 전역에서 해를 가지는가? 내가 어떠한 vL과 vR을 지정하더라도 vC와 pos를 반환할 수 있는가?
 자명히 -1에서 그렇지 못하다.

 그렇다면 이 식을 풀어서 다음과 같이 써 보자.
 vL = vC+vD
 vR = vC-vD
 이 경우 해가 존재하는가?
 자명히 그렇다. 위 식과 아래 식을 더하면
 vC=(vL+vR)/2
 위 식에서 아래 식을 빼면
 vD=(vL-vR)/2
 그러므로 해가 반드시 존재한다. 따라서 이 제어방식은 우리 제어에 적합하다.

 그렇다면 이 제어 방식에서 position은 어떤 영향을 미치는가?
 자명히 위 식으로부터 vD = vC*position이다.
 이로부터 vC와 vD라는 두 수를 이용한 모터의 제어를 도입하여 모터를 모두 부드럽게 제어할 수 있음을 보였다.
*/

/*
 다음으로 이러한 제어 방식을 효율적으로 다루는 방법에 대해 논의한다.
 가장 간단한 방법은 Struct를 사용하여 구현하는 것이다. 그렇게 하면 향후 등가속도 추종을 할 때에도 유용하게 사용가능하다.
 그러나 지금은 이 제어 방법의 효율성이 검증되지 않았으므로, 일단 변수를 사용하여 구현하기로 하자.
*/

/*
기존에 바퀴가 튕기는 문제가 있었는데, 그 원인을 알아낸 듯하다.
바퀴가 충분히 저속으로 주행할 수 없어서 발생한 문제가 분명하다. 즉, 정지에서 출발할 때 가속도가 너무 커서 발생하는 문제인 듯.
따라서 저속 리미터를 충분히 풀되 PI제어를 이용하여 저속 구간을 최대한 줄이는 방법을 이용하기로 한다.
*/

int sleep_ms(int ms){
    for(int i =0;i<ms;i++){
        usleep(1000);
    }
}

/*
accelerate : used in loop with delay.
curr is the variable to be controlled and dest is the destination value.
Increase or decrease the 'curr' to the slope of 'acc' until the 'curr' reaches 'dest'.
*/
double accelerate(double curr,double dest,double acc,double dt){
    if(curr<dest){
        curr+=acc*dt;
        if(curr>dest) return dest;
        else          return curr;
    }else if(curr>dest){
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

double currVc=0,currVd=0;
double destVc=0,destVd=0;
double vCAccel = 500;
double vDAccel = 8000;

void updateVelocity(double dt){
    currVc = accelerate(currVc,destVc,vCAccel,dt);
    currVd = accelerate(currVd,destVd,vDAccel,dt);

    double veloL,veloR,dtL,dtR;
    veloL = currVc+currVd;
    veloR = currVc-currVd;

    if(veloL!=0) dtL = (NANOSEC/veloL);
    else         dtL = 0;
    if(veloR!=0) dtR = (NANOSEC/veloR);
    else         dtR = 0;

    // printf("%f, %f\n",currVd,dt);

    if(ABS(dtL)>LIMDT)dtL = 0;
    if(ABS(dtR)>LIMDT)dtR = 0;
    // ABS_LIM(dtL,LIMDT);
    // ABS_LIM(dtR,LIMDT);

    control->dtL = (int64_t)dtL;
    control->dtR = (int64_t)dtR;   
}

void setVelocityByWheel(double vL,double vR){
    destVc = (vL+vR)/2;
    destVd = (vL-vR)/2;
}

void setVelocityByCurvate(double vC,double curvate){
    destVc = vC;
    destVd = curvate*vC;
}

// 
int main_remote(){
    exec("./comm.o");
    sleep_ms(100);
    LOG("Server alive : %s",BOOL(control->serverAlive));

    double
        cvl,cvr,    // Current velocity
        dvl,dvr,    // Destination velocity
        dtl,dtr;    // Delta time (the reciprocal of current velocity)

    for(;;){
        sleep_ms(10);

        dvl = control->userVL;
        dvr = control->userVR;

        cvl = accelerate(cvl,dvl,ACC_WHEEL,0.01f);
        cvr = accelerate(cvr,dvr,ACC_WHEEL,0.01f);

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

// The most basic line-following control.
// Verified
int main_lineTracing(){
    INTERVAL interval;
    interval.recent = 0;
    interval.interval = 100000;
    double   intervalSec = interval.interval*1.0/NANOSEC; // Interval in sec 

    INTERVAL_LOOP{
        RUN_TASK(interval,{
            updateVelocity(intervalSec);
            if(control->lineout) setVelocityByWheel(0,0);
            else                 setVelocityByCurvate(VELO_DEFAULT,control->position*POS_COEF);
        });
    }
}

// Todo : implement it
void align(){
    // PI-control value
    double pGain    = 200;
    double iGain    = 0.1;
    double err      = control->position;
    double errI     = 0;
    double iirGain  = 0.5;

    // Interval struct
    INTERVAL interval;
    double dt = initInterval(&interval,100000);

    // Loop break control
    double  breakTime = 0.5;
    int64_t currentTick = control->tickL;
    double  stopTime = 0;

    INTERVAL_LOOP{
        RUN_TASK(interval,{
            updateVelocity(dt);

            // PI-control
            err = err*iirGain+control->position*(1-iirGain);
            errI += err;
            setVelocityByCurvate(0,err*pGain + errI*iGain);

            // Break check
            if(currentTick!=control->tickL){
                currentTick=control->tickL;
                stopTime = 0;
            }else if(stopTime>=breakTime)break;
            stopTime+=dt;
        });
    }
}

// Move by given ticks. 1 tick = 2PI*WHEEL_RAD/400.
/*

*/
void moveTicks(int64_t ticks){
    // p-control value
    int64_t destTick = control->tickC+ticks*2;
    double   pGain    = .5f;

    INTERVAL interval;
    interval.recent = 0;
    interval.interval = 1000000;
    double   intervalSec = interval.interval*1/NANOSEC; // Interval in sec 

    INTERVAL_LOOP{
        RUN_TASK(interval,{
            updateVelocity(intervalSec);

            int64_t err = destTick-control->tickC;
            if(err<=0)break;

            if(control->lineout) setVelocityByWheel(0,0);
            else                 setVelocityByCurvate(err*pGain,control->position*POS_COEF);
        });
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
    int64_t err = destTick-control->tickR;

    double  errorI = 0;
    double  pGain    = -1;
    double  iGain    = -0.00003;
    // double  errBef;

    int     errSign  = SIGN(err);

    INTERVAL interval;
    interval.recent = 0;
    interval.interval = 100000;
    double intervalSec = interval.interval*1.0/NANOSEC; // Interval in sec 

    destVc=0;

    INTERVAL_LOOP{
        RUN_TASK(interval,{
            updateVelocity(intervalSec);

            // P-control
            err = destTick-control->tickR;
            errorI+=err*iGain;
            destVd = err*pGain+errorI;
            if(errSign*err<=0)return;
        });
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

    // P-control for stop
    int64_t destDist = control->tickC+ticks;
    int64_t error    = ticks; 
    double pGain = currVc/ticks;

    int8_t accum = 0x00;

    destVd = 0;
    vCAccel = 2000;
    
    INTERVAL_LOOP{
        RUN_TASK(interval,{
            updateVelocity(intervalSec);

            error = destDist-control->tickC;
            // if(error<=0)break;

            double t = error*pGain;
            if(ABS(t)<50)t = 0;
            setVelocityByCurvate(t,0);

            if(currVc<5)break;
            accum|=control->sensorState;
        });
    }

    vCAccel = 500;

    return accum;
}

// // Go untill any node appears
int8_t goUntillNode(){
    INTERVAL interval;
    interval.recent = 0;
    interval.interval = 1000000;
    double intervalSec = interval.interval*1.0/NANOSEC; // Interval in sec 

    int i = 0;

    INTERVAL_LOOP{
        RUN_TASK(interval,{
            updateVelocity(intervalSec);

            if(control->node!=0x00) break;

            if(control->lineout) setVelocityByWheel(0,0);
            else                 setVelocityByCurvate(VELO_DEFAULT,control->position*POS_COEF);

            // i++;
            // if(!(i%100)){
            //     printf("Lineout? %d\n",control->lineout);
            //     printf("DTL : %lld DTR %lld\n",control->dtL,control->dtR);  
            //     LOG("VL: %f, VR: %f",veloL,veloR);
            // }
        });
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

    // main_remote();
    // main_lineTracing();

    while(1){
        LOG("ALIGN");
        align();
        LOG("Go");
        state = goUntillNode();
        node  = recognizeNode(state);

        LOG("Turn")
        if(node==NODE_LEFT) rotate(90);
        if(node==NODE_RIGHT) rotate(-90);
        if(node==NODE_T) rotate(90);
        if(node==NODE_CROSS) rotate(-180);
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

    LOG("END");

    endControl();
    return 0;
}
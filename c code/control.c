#include <stdio.h>
#include <unistd.h>
#include <sched.h>
#include <sys/shm.h>
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

#define VELO 1500
#define ACC 200000
#define LIMDT 3000000

int main_remote(){
    LOG("Start controller.");

    // Get control object from shared memory.
    SHM_CONTROL* control = getControlStruct();
    if((int)control<0){
        ERR("Cannot get shared memory. err code : %d",control);
        return -1;
    }

    // Exit existing child processes
    control->exit = 1;
    sleep_ms(100);
    control->exit = 0;

    // Start child process
    exec("./comm.o");
    exec("./motor.o");

    // Check motor and server
    LOG("Motor alive : %s",BOOL(control->motorAlive));
    LOG("Server alive : %s",BOOL(control->serverAlive));
    control->run = 1;

    float
        cvl,cvr,    // Current velocity
        dvl,dvr,    // Destination velocity
        dtl,dtr;    // Delta time (the reciprocal of current velocity)

    for(;;){
        sleep_ms(10);

        dvl = control->userVL;
        dvr = control->userVR;

        ACCELERATE(cvl,dvl,ACC,0.01f);
        ACCELERATE(cvr,dvr,ACC,0.01f);

        if(cvl!=0) dtl = 1000000000/cvl;
        else dtl = 0;
        if(cvr!=0) dtr = 1000000000/cvr;
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

    LOG("Turn off motor");
    control->run=0;
    sleep_ms(10);

    LOG("Exit all subprocess");
    control->exit=1;
    sleep_ms(100);

    LOG("Motor alive : %s",BOOL(control->motorAlive));
    LOG("Server alive : %s",BOOL(control->serverAlive));

    if(removeControlStruct(control)==-1){ERR("Cannot remove shared memory.");}
    else LOG("Shared memory removed.");

    LOG("Exit control control.");
}

int main_lineTracing(){
    LOG("Start controller.");

    // Get control object from shared memory.
    SHM_CONTROL* control = getControlStruct();
    if((int)control<0){
        ERR("Cannot get shared memory. err code : %d",control);
        return -1;
    }

    // Exit existing child processes
    control->exit = 1;
    sleep_ms(100);
    control->exit = 0;

    // Start child process
    sleep_ms(1000);
    exec("./motor.o");
    sleep_ms(1000);
    exec("./sensor.o");
    sleep_ms(1000);

    // Check motor and server
    LOG("Motor alive : %s",BOOL(control->motorAlive));
    // LOG("Sensor alive : %s",BOOL(control->serverAlive));
    control->run = 1;

    float
        cvl,cvr,    // Current velocity
        dvl,dvr,    // Destination velocity
        dtl,dtr;    // Delta time (the reciprocal of current velocity)

    for(int i =0;;i++){
        #define POSITION_COEFF .3f

        if(control->lineout){
            dvl=dvr=0;
        }else{
            dvl = VELO*(1+control->position*POSITION_COEFF);
            dvr = VELO*(1-control->position*POSITION_COEFF);
        }

        ACCELERATE(cvl,dvl,ACC,0.01f);
        ACCELERATE(cvr,dvr,ACC,0.01f);

        if(cvl!=0) dtl = 1000000000/cvl;
        else dtl = 0;
        if(cvr!=0) dtr = 1000000000/cvr;
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

    LOG("Turn off motor");
    control->run=0;
    sleep_ms(10);

    LOG("Exit all subprocess");
    control->exit=1;
    sleep_ms(100);

    LOG("Motor alive : %s",BOOL(control->motorAlive));
    LOG("Server alive : %s",BOOL(control->serverAlive));

    if(removeControlStruct(control)==-1){ERR("Cannot remove shared memory.");}
    else LOG("Shared memory removed.");

    LOG("Exit control control.");
}

int main(){
    return main_lineTracing();
}
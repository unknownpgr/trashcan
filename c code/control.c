#include <stdio.h>
#include <unistd.h>
#include <sched.h>
#include <sys/shm.h>
#include <math.h>
#include "controlProtocol.h"

#define ABS(x) (((x)>0)?(x):(-(x)))

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

int main(){
    printf("Start controller.\n");

    // Get control object from shared memory.
    MOTOR_CONTROL* control = getControlStruct();
    if((int)control<0){
        printf("Cannot get shared memory. err code : %d\n",control);
        return -1;
    }

    printf("Start motor : ");
    // Get control object from shared memory.
    control->run = 0;
    sleep_ms(100);
    printf("%d\n",control->motorAlive);
    control->run = 1;

    float
        cvl,cvr,    // Current velocity
        dvl,dvr,    // Destination velocity
        dtl,dtr;    // Delta time (the reciprocal of current velocity)

    for(float t = 0;t<10;t+=0.01f){
        sleep_ms(10);

        #define VELO 1000

        dvl = sin(t*4)*VELO;
        dvr = cos(t*4)*VELO;

        #define ACC 10000
        #define LIMDT 3000000

        ACCELERATE(cvl,dvl,ACC,0.01f);
        ACCELERATE(cvr,dvr,ACC,0.01f);

        dtl = 1000000000/cvl;
        dtr = 1000000000/cvr;

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


    float velocity = 0;

    for(;0;){
        // Get velocity from user
        printf("Enter velocity. enter -128 for exit. >> ");
        scanf("%f",&velocity);
        printf("\n");

        // Clear buffer
        int c;
        while ((c = getchar()) != '\n' && c != EOF);

        if(velocity==-128.f)break;
        if(ABS(velocity)<1){
            control->run =0;
            printf("Stop motor\n");
        }
        if(ABS(velocity)<2000){
            printf("Velocity is too slow.\n");
            if(velocity<0){
                velocity = -2000;
                printf("Set velocity to -2000\n");
            }else{
                velocity = 2000;
                printf("Set velocity to 2000\n");
            }
        }
        control->run =1;
        // control->velocity = velocity;
        printf("Set velocity to %f\n",velocity);
    }

    control->run=0;
    sleep_ms(100);
    printf("MotorAlive : %d\n",control->motorAlive);

    control->exit=1;
    sleep_ms(100);
    printf("MotorAlive : %d\n",control->motorAlive);

    if(removeControlStruct(control)==-1)printf("Cannot remove shared memory.\n");
    else printf("Shared memory removed.\n");

    printf("End control.\n");
}
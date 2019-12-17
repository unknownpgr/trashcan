#define _GNU_SOURCE 
#include "controlProtocol.h"
#include "log.h"

#include "gpio.h"

#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <unistd.h>

#include <sys/stat.h>    
#include <sched.h>
#define ABS(x) (((x)>0)?(x):(-(x)))

// Typedef and define
typedef unsigned char   bool;
typedef unsigned char   byte;
#define false   0
#define true    1

// #define ENABLE_BITMASK

// Do nothing during given microsecond. use loop to prevent context switching.
// Maximum delay = 0.01s = 10ms = 10000us
void delay_us(long int delay){
	static long int start_time;
	static long int time_difference;
	static struct timespec gettime_now;

	clock_gettime(CLOCK_REALTIME, &gettime_now);
	start_time = gettime_now.tv_nsec;

	for(;;){
		clock_gettime(CLOCK_REALTIME, &gettime_now);
		time_difference = gettime_now.tv_nsec - start_time;

        /*
        The maximum value of tv_nsec maximum is 1000000000 = 1sec.
        Therefore if time_differece is negative, add 1000000000 to time differece.
        */
    	if (time_difference < 0) time_difference += 1000000000;
		if (time_difference > (delay * 1000)) break;
	}
}

int processCoreAssign(){
    // ========================================================
    //     Assign process to isolated core.
    // ========================================================

    const int isolatedCPU = 3;

    cpu_set_t mask;                                         // Mask variable that indicates isolated cpu
    CPU_ZERO(&mask);                                        // Initialize cpu mask
    CPU_SET(isolatedCPU,&mask);                             // Set mask

    int core = sched_setaffinity(0,sizeof(mask),&mask);     // Set core affinity of current process to masked core.
    if(core==-1) printf("Cannot assign core\n");
    return core;
}


void initMotor(MOTOR* motor, int pins[4], int phases[8]){

    // Set pin and pin mask
    motor->phase = 0;
    motor->pins = pins;
    motor->pinMask = getMask(pins, 4);

    // Set phase mask (which is directly used for motor set/clear)
    motor->phaseMasks = phases;
    phaseToMask(pins,4,motor->phaseMasks,8);

    // Initialize pins used for motor control
    for(int i =0;i<4;i++){
        MODE_OUT(pins[i]);
        CLR(pins[i]);
    }
}

/*
TICK(motor,dir){                                    // Used for real-time SLA7026 motor driver control
    (*GPIO_SET)=(motor).pinMask;                    // Turn on all pins
    (*GPIO_CLR)=(motor).phaseMasks[(motor).phase];  // Turn off selected pins and increase phase
    if((dir)>0){(motor).phase++;}                   // Increase or decrease phase index according to direction
    if((dir)<0){(motor).phase--;}
    (motor).phase&=7;                               // motor = motor>7?0:motor
}
*/
#define TICK(motor,dir){(GPIO->SET[0])=(motor).pinMask;(GPIO->CLR[0])=(motor).phaseMasks[(motor).phase];if((dir)>0){(motor).phase++;}if((dir)<0){(motor).phase--;}(motor).phase&=0x07;}

// Turn on all pins(because SLA7026 uses high as default.)
#define STOP(motor) ((GPIO->SET[0])=(motor).pinMask)

//Print bit of int.
void printBit(int x){
    printf("0b");
    for(int i = 31; i>=0;i--) printf("%d",1&&(x&1<<i));
    printf("\n");
}

// THREAD_LOOP interval structure. must be initialized before being used.
typedef struct{
    long interval;
    long recent;
}INTERVAL;

// Make a for statement that have __time variable which is updated in each loop.
#define THREAD_LOOP for(struct timespec __time;;clock_gettime(CLOCK_REALTIME, &__time))

/*
RUN_TASK : run given task 
*/
#define RUN_TASK(thread,task) {static long __td;__td=__time.tv_nsec-(thread).recent;__td = (__td<0)?(__td+1000000000):(__td);if(__td>(thread).interval){(thread).recent= __time.tv_nsec;{task;}}}

int main(){
    // ========================================================
    //     Assign process to isolated core.
    // ========================================================

    if(processCoreAssign()==-1) return -1;
    LOG("Motor process was assigned to isolated core.");

    // ========================================================
    //     Get shared memory or create if it does not exists.
    // ========================================================

    // Get control object from shared memory.
    MOTOR_CONTROL* control = getControlStruct();
    if((int)control<0){
        ERR("Cannot get shared memory. err code : %d",control);
        return -1;
    }
    LOG("Got the shared memory.");


    // Initialize check
    if(control->motorAlive){
        ERR("Another motor process still alive.");
        return -1;
    }
    // Set flag
    control->motorAlive = 1;

    // ========================================================
    //     Initialzie GPIO and make bitmask for control.
    // ========================================================

    // GPIO = gpio base address.
    initGpioMmap();
    if(GPIO==NULL){
        printf("GPIO memory map error.");
        return -1;
    }
    LOG("GPIO memory has been mapped.")

    // Initialize all pins
    initGPIO();
    LOG("GPIO initialized.")

    /*
    GPIO on-off wave max frequancy is about 10kHz
    This is because inaccuracy of delay_us
    */

    // Pins of motors. A,A',B,B' in order
    int pins_r[4] = {2 , 3 , 4 , 17};
    int pins_l[4] = {23, 22, 27, 18};

    // Declare phases of each motor and convert it to bitmask
    int phases_l[8] = {0x01, 0x01|0x04, 0x04, 0x04|0x02, 0x02, 0x02|0x08, 0x08, 0x08|0x01};
    int phases_r[8] = {0x01, 0x01|0x04, 0x04, 0x04|0x02, 0x02, 0x02|0x08, 0x08, 0x08|0x01};

    MOTOR motorL, motorR;
    initMotor(&motorL,pins_l,phases_l);
    initMotor(&motorR,pins_r,phases_r);
    LOG("Motor object initialized.");

    // Print bitmask and phase for check
    #ifdef ENABLE_BITMASK
        //Print phase bitmask
        for(int i =0;i<4;i++)printBit(phase_l[i]);

        LOG("Left motor gpio register bitmask :");
        printBit(motorL.pinMask);
        LOG("Right motor gpio register bitmask :");
        printBit(motorR.pinMask);
        LOG("Motor phase list :");
        for(int i =0;i<8;i++)printBit(motorL.phaseMasks[i]);
        LOG("Motor phase list :");
        for(int i =0;i<8;i++)printBit(motorR.phaseMasks[i]);
    #endif

    LOG("Start motor control loop");

    INTERVAL threadL, threadR;
    threadL.interval = threadL.recent = 0;
    threadR.interval = threadR.recent = 0;

    THREAD_LOOP{
        // Check control object.
        if(control->exit)break; // If the control is exit, break the loop.
        if(!control->run){      // If the control is stop == !run
            STOP(motorL);       // Stop both motor
            STOP(motorR);
            for(;!(control->run||control->exit);) delay_us(1000);   //Wait othre command.
        }

        threadL.interval = ABS(control->dtL);
        threadR.interval = ABS(control->dtR);

        if(control->dtL!=0){RUN_TASK(threadL,
            TICK(motorL,control->dtL);
        );}else STOP(motorL);

        if(control->dtR!=0){RUN_TASK(threadR,
            TICK(motorR,control->dtR);
        );}else STOP(motorR);
    }

    //Initialize the GPIO, set the flag and exit process.
    control->motorAlive=0;
    initGPIO();
    LOG("Motor control process successfully terminated.");
    return 0;
}
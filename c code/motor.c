#define _GNU_SOURCE 
#include "controlProtocol.h"

#include <stdlib.h>  
#include <stdio.h>
#include <time.h>
#include <unistd.h>

#include <sys/stat.h>    
#include <fcntl.h>
#include <sys/mman.h> 
#include <sched.h>
#define ABS(x) (((x)>0)?(x):(-(x)))

// Typedef and define
typedef unsigned char   bool;
typedef unsigned char   byte;
#define false   0
#define true    1

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

    return sched_setaffinity(0,sizeof(mask),&mask);        // Set core affinity of current process to masked core.
}

// GPIO memory map base address
#define   GPIO_BASE      0x3F200000

// GPIO memory map structure
typedef struct{
    int SELECT[6]; // Select register
    int reserved0;
    int SET[2];    // Set register
    int reserved1;
    int CLR[2];    // Clear register
    int reserved2;
}GPIO_t;

// GPIO memory map
GPIO_t* GPIO = NULL;

// Initialize the GPIO memory map 
void init_gpio_mmap(){
    // Open file
    int fd = open("/dev/mem", O_RDWR|O_SYNC);    
    if ( fd < 0 ){
        GPIO==NULL;
        return;
    }

    // Memory mapping
    GPIO = (GPIO_t*)mmap(0, 4096, PROT_READ|PROT_WRITE, MAP_SHARED, fd, GPIO_BASE);        
    if(GPIO == MAP_FAILED)GPIO == NULL;
}

typedef struct{
    int* pins;          // Used pins
    int  pinMask;       // Bit mask that indicate
    int* phaseMasks;    // Bit masks of each phase
    int  phase;         // Current phase = index of phaseMasks
}MOTOR;

#define MODE_IN(pin)    ((GPIO->SELECT[(pin)/10])&=(~(0x07<<(((pin)%10)*3)))) // Set gpio mode of given pin to input
#define MODE_OUT(pin)   ((GPIO->SELECT[(pin)/10])|=  (0x01<<(((pin)%10)*3)))  // Set gpio mode of given pin to output

#define SET(pin) ((GPIO->SET[0])|=(1<<(pin)))  // Set given gpio pin(If mode is input, do nothing.)
#define CLR(pin) ((GPIO->CLR[0])|=(1<<(pin)))  // Clear given gpio pin(If mode is input, do nothing.)

// Get 32bit length bitmask that bits[0<=i<len]th bits are 1 and the others are 0.
int getMask(int*bits, int len){
    int mask = 0;
    for(int i = 0; i<len; i++) mask|=0x01<<bits[i];
    return mask;
}

// pins     : array of pins to use.
// pinLen   : length of pins
// phase    : array of phases to use.
// phaseLen : length of phase
void phaseToMask(int* pins, int pinLen, int* phase, int phaseLen){
    for(int i=0;i<phaseLen;i++){
        int mask = 0;
        // If jth bit of phase[i] is 1, set pins[j]th bit of mask to 1.
        // For example, if pins = {1,2,3,5} and phase[i] is 0b1001,
        // It means that 1st and 5th bit of mask should be 1.
        for(int j = 0; j<pinLen; j++) if(phase[i]&(0x01<<j))mask|=0x01<<pins[j];
        phase[i] = mask;
    }
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

// Turn off all pins
#define STOP(motor) ((GPIO->SET[0])=(motor).pinMask)

// Initialize = Clear all gpio pin.
// Turn of all gpio and set mode to in.
void initGPIO(){
    for(int i=0;i<28;i++){
        MODE_OUT(i);
        CLR(i);
        MODE_IN(i);
    }
}

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

    if(processCoreAssign()==-1){
        printf("Cannot assign core\n");
        return -1;
    }

    // ========================================================
    //     Get shared memory or create if it does not exists.
    // ========================================================

    // Get control object from shared memory.
    MOTOR_CONTROL* control = getControlStruct();
    if((int)control<0){
        printf("Cannot get shared memory. err code : %d\n",control);
        return -1;
    }

    // ========================================================
    //     Initialzie GPIO and make bitmask for control.
    // ========================================================

    // GPIO = gpio base address.
    init_gpio_mmap();
    if(GPIO==NULL){
        printf("gpio memory map error.\n");
        return -1;
    }

    // Initialize all pins
    initGPIO();

    /*
    GPIO on-off wave max frequancy is about 10kHz
    This is because inaccuracy of delay_us
    */

    // Pins of motors. A,A',B,B' in order
    int pins_l[4] = {2 , 3 , 4 , 17};
    int pins_r[4] = {18, 27, 22, 23};

    // Declare phases of each motor and convert it to bitmask
    int phases_l[8] = {0x01, 0x01|0x04, 0x04, 0x04|0x02, 0x02, 0x02|0x08, 0x08, 0x08|0x01};
    int phases_r[8] = {0x01, 0x01|0x04, 0x04, 0x04|0x02, 0x02, 0x02|0x08, 0x08, 0x08|0x01};

    MOTOR motorL, motorR;
    initMotor(&motorL,pins_l,phases_l);
    initMotor(&motorR,pins_r,phases_r);

    // Print bitmask and phase for check
    #ifdef ENABLE_BITMASK

    for(int i =0;i<4;i++)printBit(phase_l[i]);

    printf("Left motor gpio register bitmask :\n");
    printBit(motorL.pinMask);
    printf("Right motor gpio register bitmask :\n");
    printBit(motorR.pinMask);
    printf("Motor phase list : \n");
    for(int i =0;i<8;i++)printBit(motorL.phaseMasks[i]);
    printf("Motor phase list : \n");
    for(int i =0;i<8;i++)printBit(motorR.phaseMasks[i]);

    #endif

    // Kill remaining process.
    control->exit = 1;
    sleep(0.1);
    control->exit = 0;

    // Initialize check
    if(control->motorAlive){
        // control->motorAlive is set by motor process.
        // control->exit is 1 only after motor process exited.
        printf("Memory uninitialized.\n");
    }

    // Set flag
    control->motorAlive = 1;

    printf("Start loop\n");

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

        if(control->dtL!=0) RUN_TASK(threadL,
            TICK(motorL,control->dtL);
        );

        if(control->dtR!=0) RUN_TASK(threadR,
            TICK(motorR,control->dtR);
        );
    }

    //Initialize the GPIO, set the flag and exit process.
    control->motorAlive=0;
    initGPIO();
    printf("Motor control process successfully terminated.\n");

    return 0;
}
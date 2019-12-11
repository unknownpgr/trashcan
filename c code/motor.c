#define _GNU_SOURCE 
#include <stdlib.h>  
#include <stdio.h>
#include <time.h>
#include <unistd.h>
#include <sys/types.h>    
#include <sys/stat.h>    
#include <fcntl.h>
#include <sys/mman.h> 
#include <sched.h>
#include <sys/shm.h>
#include "controlProtocol.h"
#define ABS(x) (((x)>0)?(x):(-(x)))

// GPIO memory map base address
#define   GPIO_BASE      0x3F200000

int* GPIO_SELECT0;  // Address of function select register
int* GPIO_SET;      // Address of GPIO set register
int* GPIO_CLR;      // Address of GPIO clear register

// Typedef and define
typedef unsigned char*  GPIO;
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

// Return memory mapped address of address of GPIO register
int* get_gpio_mmap(){

    // Open file
    int fd = open("/dev/mem", O_RDWR|O_SYNC);    
    if ( fd < 0 )return NULL;

    // Memory mapping
    int* gpio = (int*)mmap( 0, 4096, PROT_READ|PROT_WRITE, MAP_SHARED, fd, GPIO_BASE);        
    if ( gpio == MAP_FAILED )return NULL;

    // Set global variable
    GPIO_SELECT0 = gpio;
    GPIO_SET     = gpio + 7;
    GPIO_CLR     = gpio + 10;

    // Return mapped base address
    return gpio;
}

#define MODE_IN(pin)    (*(GPIO_SELECT0+((pin)/10))&=(~(0x07<<(((pin)%10)*3)))) // Set gpio mode of given pin to input
#define MODE_OUT(pin)   (*(GPIO_SELECT0+((pin)/10))|=  (0x01<<(((pin)%10)*3)))  // Set gpio mode of given pin to output

#define SET(pin) (*(GPIO_SET)|=(1<<(pin)))  // Set given gpio pin(If mode is input, do nothing.)
#define CLR(pin) (*(GPIO_CLR)|=(1<<(pin)))  // Clear given gpio pin(If mode is input, do nothing.)

// Initialize = Clear all gpio pin.
// Turn of all gpio and set mode to in.
void init(){
    for(int i=0;i<28;i++){
        MODE_OUT(i);
        CLR(i);
        MODE_IN(i);
    }
}

// Get 32bit length bitmask that bits[0<=i<len]th bits are 1 and the others are 0.
int getMask(int*bits, int len){
    int mask = 0;
    for(int i = 0; i<len; i++) mask|=0x01<<bits[i];
    return mask;
}

// pins : array of pins to use.
// pinLen : length of pins
// phase : array of phases to use.
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

//Print bit of int.
void printBit(int x){
    printf("0b");
    for(int i = 31; i>=0;i--) printf("%d",1&&(x&1<<i));
    printf("\n");
}

int main(){
    printf("Program started.\n");

    // ========================================================
    //     Assign process to isolated core.
    // ========================================================

    const int isolatedCPU = 3;

    cpu_set_t mask;                                         // Mask variable that indicates isolated cpu
    CPU_ZERO(&mask);                                        // Initialize cpu mask
    CPU_SET(isolatedCPU,&mask);                             // Set mask

    if(sched_setaffinity(0,sizeof(mask),&mask)==-1){        // Set core affinity of current process to masked core.
        printf("Cannot assign process to core %d.\n",isolatedCPU);
        return -1;
    }
    printf("Process assigned to core %d.\n",isolatedCPU);

    // ========================================================
    //     Get shared memory or create if it does not exists.
    // ========================================================

    key_t shmKey  = 8080;                                   // Key to identify shared memory
    int   shmSize = 1024;                                   // Size of shared memory
    int   shmid   = shmget(shmKey,shmSize,IPC_CREAT|0666);  // ID of shared memory given by kernel
    char* sharedMemory;                                     // Pointer of shared memory

    if(shmid==-1){
        printf("Cannot get/create shared memory.\n");
        return -1;
    }
    printf("Shared memory successfully assigned.\n");

    // Paramter : (Key, Address(0 = assigned by kernel),  Permission(0 = READ/WRITE))
    sharedMemory = shmat(shmid,(void*)0,0);                 // Attach shared memory to variable
    if((int)sharedMemory==-1){
        printf("Cannot attach shared memory. : \n");
        return -1;
    }

    // Print some bytes of memory to check if memory is initialized.
    printf("Memory check : \n");
    for(int i = 0;i<8;i++) printf("|%d",*(sharedMemory+i));
    printf("|\n");

    // ========================================================
    //     Initialzie GPIO and make bitmask for control.
    // ========================================================

    // GPIO = gpio base address.
    GPIO gpio = (GPIO)get_gpio_mmap();
    if(gpio==NULL){
        printf("gpio memory map error.\n");
        return -1;
    }

    // Initialize all pins
    init();

    /*
    GPIO on-off wave max frequancy is about 10kHz
    This is because inaccuracy of delay_us
    */

    // Pins of motors. A,A',B,B' in order
    int motor_l[4] = {2 , 3 , 4 , 17};
    int motor_r[4] = {18, 27, 22, 23};

    // Bitmask of motor control related pins
    int mask_l = getMask(motor_l,4);
    int mask_r = getMask(motor_r,4);

    // Declare phases of each motor and convert it to bitmask
    int phase_l[8] = {0x01, 0x01|0x04, 0x04, 0x04|0x02, 0x02, 0x02|0x08, 0x08, 0x08|0x01};
    int phase_r[8] = {0x01, 0x01|0x04, 0x04, 0x04|0x02, 0x02, 0x02|0x08, 0x08, 0x08|0x01};
    phaseToMask(motor_l,4,phase_l,8);
    phaseToMask(motor_r,4,phase_r,8);

    // Print bitmask and phase for check
    // for(int i =0;i<4;i++)printBit(phase_l[i]);
    printf("Left motor gpio register bitmask :\n");
    printBit(mask_l);
    printf("Right motor gpio register bitmask :\n");
    printBit(mask_r);

    // Initialize pins used for motor control
    for(int i =0;i<4;i++){
        MODE_OUT(motor_l[i]);
        CLR(motor_l[i]);
        MODE_OUT(motor_r[i]);
        CLR(motor_r[i]);
    }

    // Get control object from shared memory.
    MOTOR_CONTROL* control = (MOTOR_CONTROL*)sharedMemory;

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

    // Run loop for motor control
    float currentVelocity = 110;
    char dir = 1;
    for(int i =0;;i+=dir){

        // Check control object.
        if(control->exit)break;
        if(!control->run)continue;

        // Turn off all gpio pins
        (*GPIO_CLR)=mask_l;
        (*GPIO_CLR)=mask_r;

        // Set gpio pins with mask.
        (*GPIO_SET)=(mask_l&(~phase_l[i%8]));
        (*GPIO_SET)=(mask_r&(~phase_r[i%8]));

        // Calculate the velocity
        // v = 1000000/dt
        // :. dt = 1000000/v
        // minv = 200
        // :. maxdt = 1000000/200
        // :. maxdt = 10000/2 = 5000

        float dt = 1000000/ABS(currentVelocity);
        if(currentVelocity<0)dir=-1;
        else dir=1;
        if(dt>5000)dt=5000;

        delay_us((int)dt);
        if(currentVelocity<control->velocity)currentVelocity+=dt/1000;
        if(currentVelocity>control->velocity)currentVelocity-=dt/1000;
    }

    //Initialize the GPIO, set the flag and exit process.
    control->motorAlive=0;
    init();
    printf("Process successfully terminated.\n");

    return 0;
}
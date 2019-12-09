#include <stdlib.h>  
#include<stdio.h>
#include<time.h>
#include <unistd.h>
#include <sys/types.h>    
#include <sys/stat.h>    
#include <fcntl.h>
#include <sys/mman.h> 

#define   GPIO_BASE      0x3F200000
int* GPIO_SELECT0;
int* GPIO_SET;
int* GPIO_CLR;

typedef unsigned char*  GPIO;
typedef unsigned char   bool;
typedef unsigned char   byte;
#define false   0
#define true    1

long int basetime_us;

long int ms(){
	static struct timespec time;
    clock_gettime(CLOCK_REALTIME, &time);
    return time.tv_nsec-basetime_us;
}

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

        tv_nsec maximum value is 1000000000 = 1sec.
        therefore if we got negative time_difference, add 1000000000 to time differece.

        */

    	if (time_difference < 0) time_difference += 1000000000;
		if (time_difference > (delay * 1000)) break;
	}
}

int* get_gpio_mmap(){
    int fd = open("/dev/mem", O_RDWR|O_SYNC);    
    if ( fd < 0 )return NULL;
    int* gpio = (int*)mmap( 0, 4096, PROT_READ|PROT_WRITE, MAP_SHARED, fd, GPIO_BASE);        
    if ( gpio == MAP_FAILED )return NULL;

    GPIO_SELECT0 = gpio;
    GPIO_SET    = gpio + 7;
    GPIO_CLR    = gpio + 10;

    return gpio;
}


#define MODE_IN(pin)    (*(GPIO_SELECT0+((pin)/10))&=(~(0x07<<(((pin)%10)*3))))
#define MODE_OUT(pin)   (*(GPIO_SELECT0+((pin)/10))|=  (0x01<<(((pin)%10)*3)))

#define SET(pin) (*(GPIO_SET)|=(1<<(pin)))
#define CLR(pin) (*(GPIO_CLR)|=(1<<(pin)))

void init(){
    for(int i=0;i<28;i++){
        MODE_OUT(i);
        CLR(i);
        MODE_IN(i);
    }
}

int getMask(int*pins, int len){
    int mask = 0;
    for(int i = 0; i<len; i++) mask|=0x01<<pins[i];
    return mask;
}

int pin2phase(int* pins, int len, char phase){
    int r = 0;
    for(int i = 0; i<len; i++) if(phase&(0x01<<i))r|=0x01<<pins[i];
    return r;
}

void printBit(int x){
    for(int i = 31; i>=0;i--) printf("%d",1&&(x&1<<i));
    printf("\n");
}

int main(){
    printf("Program started.\n");
    GPIO gpio = (GPIO)get_gpio_mmap();

    if(gpio==NULL){
        printf("gpio memory map error\n");
        return -1;
    }

    init();

    /*

    GPIO on-off wave max frequancy = 10kHz.
    This is because inaccuracy of delay_us.

    */

    int motor_l[4] = {2 , 3 , 4 , 17};
    int motor_r[4] = {18, 27, 22, 23};

    int mask_l = getMask(motor_l,4);
    int mask_r = getMask(motor_r,4);

    int phase_l[8] = {
        pin2phase(motor_l,4,0x01),
        pin2phase(motor_l,4,0x01|0x04),
        pin2phase(motor_l,4,0x04),
        pin2phase(motor_l,4,0x04|0x02),
        pin2phase(motor_l,4,0x02),
        pin2phase(motor_l,4,0x02|0x08),
        pin2phase(motor_l,4,0x08),
        pin2phase(motor_l,4,0x08|0x01)
    };

    int phase_r[8] = {
        pin2phase(motor_r,4,0x01),
        pin2phase(motor_r,4,0x01|0x04),
        pin2phase(motor_r,4,0x04),
        pin2phase(motor_r,4,0x04|0x02),
        pin2phase(motor_r,4,0x02),
        pin2phase(motor_r,4,0x02|0x08),
        pin2phase(motor_r,4,0x08),
        pin2phase(motor_r,4,0x08|0x01)
    };

    // Print bitmask and phase for check
    // for(int i =0;i<4;i++)printBit(phase_l[i]);
    printBit(~mask_l);
    printBit(~mask_r);

    for(int i =0;i<4;i++){
        MODE_OUT(motor_l[i]);
        CLR(motor_l[i]);
        MODE_OUT(motor_r[i]);
        CLR(motor_r[i]);
    }

    // v = 1000000/dt
    // dt = 1000000/v

    float v = 110;

    for(int i =0;i<5000;i++){
        (*GPIO_CLR)=mask_l;
        (*GPIO_CLR)=mask_r;
        // (*GPIO_SET)=0;
        (*GPIO_SET)=(mask_l&(~phase_l[i%8]));
        (*GPIO_SET)=(mask_r&(~phase_r[i%8]));
        // (*GPIO_SET)=0;
        float dt = 1000000/v;
        delay_us((int)dt);
        if(v<2000)v+=dt/1000;
    }

    init();
}


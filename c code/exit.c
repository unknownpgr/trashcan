#include <stdio.h>
#include <unistd.h>
#include <sched.h>
#include <sys/shm.h>
#include <math.h>
#include "controlProtocol.h"
#include "log.h"

#include <stdlib.h>  
#include <time.h>

#include <sys/stat.h>    
#include <fcntl.h>
#include <sys/mman.h> 

#define ABS(x) (((x)>0)?(x):(-(x)))
#define BOOL(x) (x)?"True":"False"

int sleep_ms(int ms){
    for(int i =0;i<ms;i++){
        usleep(1000);
    }
}

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

#define MODE_IN(pin)    ((GPIO->SELECT[(pin)/10])&=(~(0x07<<(((pin)%10)*3)))) // Set gpio mode of given pin to input
#define MODE_OUT(pin)   ((GPIO->SELECT[(pin)/10])|=  (0x01<<(((pin)%10)*3)))  // Set gpio mode of given pin to output

#define SET(pin) ((GPIO->SET[0])|=(1<<(pin)))  // Set given gpio pin(If mode is input, do nothing.)
#define CLR(pin) ((GPIO->CLR[0])|=(1<<(pin)))  // Clear given gpio pin(If mode is input, do nothing.)

// GPIO memory map base address
#define   GPIO_BASE      0x3F200000

// Initialize the GPIO memory map 
void init_gpio_mmap(){
    // Open file
    int fd = open("/dev/mem", O_RDWR|O_SYNC);    
    if ( fd < 0 ){
        GPIO==NULL;
        ERR("Cannot open /dev/mem");
        return;
    }
    // Memory mapping
    GPIO = (GPIO_t*)mmap(0, 4096, PROT_READ|PROT_WRITE, MAP_SHARED, fd, GPIO_BASE);        
    if(GPIO == MAP_FAILED){
        GPIO == NULL;
        ERR("Cannot map memory");
    }
}

void initGPIO(){
    for(int i=0;i<28;i++){
        MODE_OUT(i);
        CLR(i);
        MODE_IN(i);
    }
}

int main(){
    // Get control object from shared memory.
    MOTOR_CONTROL* control = getControlStruct();
    if((int)control<0){
        ERR("Cannot get shared memory. err code : %d",control);
        return -1;
    }


    init_gpio_mmap();
    if(GPIO==NULL){
        printf("GPIO memory map error.");
        return -1;
    }
    initGPIO();
    LOG("GPIO initialized.");

    // Exit existing child processes
    for(int i =0;i<5;i++){
        control->exit = 1;
        sleep_ms(100);
        // Check motor and server
        LOG("Motor alive : %s",BOOL(control->motorAlive));
        LOG("Server alive : %s",BOOL(control->serverAlive));
    }

    initGPIO();

    if(removeControlStruct(control)==-1){ERR("Cannot remove shared memory.");}
    else LOG("Shared memory removed.");
}
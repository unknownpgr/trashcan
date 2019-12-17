#include <stdio.h>
#include <unistd.h>
#include "controlProtocol.h"
#include "log.h"
#include "gpio.h"

#define ABS(x) (((x)>0)?(x):(-(x)))
#define BOOL(x) (x)?"True":"False"

int sleep_ms(int ms){
    for(int i =0;i<ms;i++){
        usleep(1000);
    }
}

int main(){
    // Get control object from shared memory.
    MOTOR_CONTROL* control = getControlStruct();
    if((int)control<0){
        ERR("Cannot get shared memory. err code : %d",control);
        return -1;
    }

    initGpioMmap();
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
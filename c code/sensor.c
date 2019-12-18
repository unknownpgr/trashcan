#include "bcm2835.h"
#include <stdio.h>
#include "log.h"
#include "gpio.h"
#include <unistd.h>

int sleep_ms(int ms){
    for(int i =0;i<ms;i++){
        usleep(1000);
    }
}

int getSensorData(int channel){
    uint8_t sendData[3] = {0x01, (0x08 + channel) << 4, 0x00};
    bcm2835_spi_transfern(sendData,sizeof(sendData));
    return ((sendData[1] & 3) << 8) + sendData[2];
}

// Error : 5,6,13 pin is never changed.
int pins[6] = {5,6,12,13,19,16};
int mask = 0x00;
int time_us = 1000;

void initPins(){
    for(int i =0 ;i<6;i++){
        MODE_OUT(pins[i]);
        CLR(pins[i]);
    }
}

int getIRData(int channel){
    initPins();
    GPIO->CLR[0]|=mask;
    GPIO->SET[0]|=(0x01<<(pins[channel]));
    sleep_ms(200);
    int data = getSensorData(channel);
    GPIO->CLR[0]|=mask;
    return data;
}

//Print bit of int.
void printBit(int x){
    printf("0b");
    for(int i = 31; i>=0;i--) printf("%d",1&&(x&1<<i));
    printf("\n");
}

int main(){
    initGpioMmap();
    initPins();
    // return 0;
    mask = getMask(pins,6);
    printBit(mask);

    if(!bcm2835_init()){
        ERR("bcm283 initialization failed.");
        return -1;
    }       
    if(!bcm2835_spi_begin()){
        ERR("SPI initialization failed.");
        ERR("Are you running as root?.");
        return -1;
    }
    LOG("SPI successfully opened.");

    initPins();

    for(int i = 0;i<1000;i++){
        for(int j = 0;j<6;j++){
            getIRData(j);
        }
    }

    bcm2835_spi_end();
    bcm2835_close();
    return 0;
}
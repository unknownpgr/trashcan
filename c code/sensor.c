#include "bcm2835.h"
#include <stdio.h>
#include "log.h"
#include "gpio.h"
#include <unistd.h>
#include "Control"

int sleep_ms(int ms){
    for(int i =0;i<ms;i++){
        usleep(1000);
    }
}

// Get sensor data from adc module.
int getSensorData(int channel){
    uint8_t sendData[3] = {0x01, (0x08 + channel) << 4, 0x00};
    bcm2835_spi_transfern(sendData,sizeof(sendData));
    return ((sendData[1] & 3) << 8) + sendData[2];
}

// Error : 5,6,13 pin is never changed.
int pins[6] = {5,6,12,13,19,16};
float sensorWeights[6] = {-5.f, -3.f, -1.f, 1.f, 3.f, 5.f};
int mask = 0x00;
int time_us = 1000;

// Set pin mode and clear it.
void initPins(){
    for(int i =0 ;i<6;i++){
        bcm2835_gpio_fsel(pins[i],BCM2835_GPIO_FSEL_OUTP);
        bcm2835_gpio_clr(pins[i]);
    }
}

// Get IR sensor data
int getIRData(int channel){
    bcm2835_gpio_set(pins[channel]);
    usleep(100);
    int data = getSensorData(channel);
    bcm2835_gpio_clr(pins[channel]);
    return data;
}

//Print bit of int.
void printBit(int x){
    printf("0b");
    for(int i = 31; i>=0;i--) printf("%d",1&&(x&1<<i));
    printf("\n");
}

int main(){
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
    bcm2835_spi_set_speed_hz(15600000);
    LOG("SPI successfully opened.");

    initPins();

    for(int i = 0;i<100000;i++){
        float valueSum = 0;
        float weightedSum = 0;
        for(int j = 0;j<6;j++){
            float weight = getIRData(j);
            valueSum += weight;
            weightedSum += weight*sensorWeights[j];
        }
        float position = weightedSum/valueSum;
        printf("%f\n",position);
    }

    bcm2835_spi_end();
    bcm2835_close();
    return 0;
}
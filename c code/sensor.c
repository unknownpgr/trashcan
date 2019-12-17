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

int main(){

    initGpioMmap();

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

    for(int i = 0;i<1000;i++){
        sleep_ms(10);
        printf("%d\n",getSensorData(0));
    }

    bcm2835_spi_end();
    bcm2835_close();
    return 0;
}
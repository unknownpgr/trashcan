#include "bcm2835.h"
#include <stdio.h>
#include "log.h"
#include "gpio.h"
#include <unistd.h>
#include "controlProtocol.h"

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

#define SENSOR_NUM          6                   // Number of sensors
#define SENSOR_CALIB_NUM    5000                // Sensor calibration iteration number
#define CALIB_FILE          "./calibration.cal" // Sensor setting file name
#define SENSING_TIME        100                 // Sensor sening time in us 

// Error : 5,6,13 pin is never changed.
int pins[SENSOR_NUM]            = {5,6,12,13,19,16};
float sensorWeights[SENSOR_NUM] = {-5.f, -3.f, -1.f, 1.f, 3.f, 5.f};

// Set pin mode and clear it.
void initPins(){
    for(int i =0 ;i<SENSOR_NUM;i++){
        bcm2835_gpio_fsel(pins[i],BCM2835_GPIO_FSEL_OUTP);
        bcm2835_gpio_clr(pins[i]);
    }
}

// Get raw IR sensor data
int getRawIRData(int channel){
    bcm2835_gpio_set(pins[channel]);
    usleep(SENSING_TIME);
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

typedef struct{
    int black[SENSOR_NUM];      // Dark min or max value for calibration
    int white[SENSOR_NUM];      // Light min or max value for calibration
    int threshold[SENSOR_NUM];  // Reserved

    // Calibrated sensor value = (raw value+calibBias)*calibSlope
    float calibSlope[8];
    float calibBias[8];
}SENSOR_SETTING;

int loadSetting(SENSOR_SETTING* sensorSetting){
    // Load senser setting from the setting file
    FILE *settingFile; 
    settingFile = fopen(CALIB_FILE, "r");
    if (settingFile == NULL) return -1;

    // If the setting file exists, read data from the file.
    fread(sensorSetting, sizeof(SENSOR_SETTING), 1, settingFile);        
    fclose(settingFile);
    return 0;
}

int saveSetting(SENSOR_SETTING* sensorSetting){
    // Write the given setting to file.
    FILE *settingFile;
    settingFile = fopen (CALIB_FILE, "w");
    if(settingFile == NULL)return -1;

    fwrite (sensorSetting, sizeof(SENSOR_SETTING), 1, settingFile);  
    fclose (settingFile); 
    return 0;
}

int calibration(SENSOR_SETTING* sensorSetting){

    // Read adc SENSOR_CALIB_NUM times per SENSOR_NUM ir sensors and get  black max and white max.
    LOG("Press enter to read blackMax.");
    getchar();
    LOG("Calibrating...");
    for(int i = 0;i<SENSOR_NUM*SENSOR_CALIB_NUM;i++){
        int irData = getRawIRData(i%SENSOR_NUM);
        if(irData>sensorSetting->black[i%SENSOR_NUM])sensorSetting->black[i%SENSOR_NUM]=irData;
    }

    LOG("Press enter to read whiteMax.");
    getchar();
    LOG("Calibrating...");
    for(int i = 0;i<SENSOR_NUM*SENSOR_CALIB_NUM;i++){
        int irData = getRawIRData(i%SENSOR_NUM);
        if(irData>sensorSetting->white[i%SENSOR_NUM])sensorSetting->white[i%SENSOR_NUM]=irData;
    }

    // Update threshold, slope and bias.
    for(int i = 0;i<SENSOR_NUM;i++){
        sensorSetting->calibSlope[i]    = 1.f/(sensorSetting->white[i]-sensorSetting->black[i]);
        sensorSetting->calibBias[i]     = -sensorSetting->black[i];
    }
    LOG("Calibration finished.");
}

// Get IR sensor data
float getCalibratedIRData(int channel,SENSOR_SETTING* setting){
    bcm2835_gpio_set(pins[channel]);
    usleep(SENSING_TIME);
    int data = getSensorData(channel);
    bcm2835_gpio_clr(pins[channel]);
    data-=setting->calibBias[channel];
    data*=setting->calibSlope[channel];
    if(data>1)return 1.f;
    if(data<0)return 0.f;
    return data;
}

int main(){

    // Initialize bcm library itself and spi
    if(!bcm2835_init()){
        ERR("bcm283 initialization failed.");
        return -1;
    }       
    if(!bcm2835_spi_begin()){
        ERR("SPI initialization failed.");
        ERR("Are you running as root?.");
        return -1;
    }

    // Set speed if spi communication
    bcm2835_spi_set_speed_hz(15600000);
    LOG("SPI successfully opened.");

    // Initialize pins for sensor
    initPins();

    // Do calibration
    SENSOR_SETTING sensorSetting;
    calibration(&sensorSetting);

    // Load shared memory control structure
    SHM_CONTROL* control = getControlStruct();

    // Start sensing
    for(int i = 0;!(control->exit);i++){
        float valueSum = 0;
        float weightedSum = 0;
        for(int j = 0;j<SENSOR_NUM;j++){
            float weight = getCalibratedIRData(j,control);
            valueSum += weight;
            weightedSum += weight*sensorWeights[j];
        }
        float position = weightedSum/valueSum;
        printf("%d\n",position);
    }

    bcm2835_spi_end();
    bcm2835_close();
    return 0;
}
#include <stdio.h>
#include <unistd.h>
#include "bcm2835.h"
#include "log.h"
#include "gpio.h"
#include "controlProtocol.h"

int sleep_ms(int ms){
    for(int i =0;i<ms;i++){
        usleep(1000);
    }
}

// Get sensor data from adc module.
int getSensorData(int channel){
    /*
    Python code.
    r = self._spi.xfer2([0x01, (0x08 + channel) << 4, 0x00])
    adc_out = ((r[1] & 3) << 8) + r[2]
    */
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
    int threshold[SENSOR_NUM];  // Reserved for state

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

    // Initialize sensor setting
    // Warning : Setting for max-brightness. you should initizize it to infinity if you want to use min-brightness based calibration.
    for(int i = 0;i<SENSOR_NUM;i++){
        sensorSetting->black[i]=sensorSetting->white[i]=0;
    }

    // Read adc SENSOR_CALIB_NUM times per SENSOR_NUM ir sensors and get  black max and white max.
    LOG("Press enter to read blackMax.");
    getchar();
    LOG("Calibrating...");
    for(int i = 0;i<SENSOR_NUM*SENSOR_CALIB_NUM;i++){
        if(!((i+1)%SENSOR_CALIB_NUM)){
            LOG("\tCalibrating...(%d/%d)",(i+1)/SENSOR_CALIB_NUM,SENSOR_NUM);
            for(int j = 0;j<SENSOR_NUM;j++) printf("%d ",sensorSetting->black[j]);
            printf("\n");
        }
        int irData = getRawIRData(i%SENSOR_NUM);
        if(irData>sensorSetting->black[i%SENSOR_NUM])sensorSetting->black[i%SENSOR_NUM]=irData;
    }

    LOG("Press enter to read whiteMax.");
    getchar();
    LOG("Calibrating...");
    for(int i = 0;i<SENSOR_NUM*SENSOR_CALIB_NUM;i++){
        if(!((i+1)%SENSOR_CALIB_NUM)){
            LOG("\tCalibrating...(%d/%d)",(i+1)/SENSOR_CALIB_NUM,SENSOR_NUM);
            for(int j = 0;j<SENSOR_NUM;j++) printf("%d ",sensorSetting->white[j]);
            printf("\n");
        }
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
    float data = getSensorData(channel);
    bcm2835_gpio_clr(pins[channel]);
    data+=setting->calibBias[channel];
    data*=setting->calibSlope[channel];
    if(data>1)return 1.f;
    if(data<0)return 0.f;
    return data;
}

#define repeat(var,iter) for(int var = 0;var<iter;var++)

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
    bcm2835_spi_set_speed_hz(1560000);
    LOG("SPI successfully opened.");

    // Initialize pins for sensor
    initPins();

    // Do calibration
    SENSOR_SETTING sensorSetting;
    if(loadSetting(&sensorSetting)==-1){
        calibration(&sensorSetting);
        saveSetting(&sensorSetting);
    }
    LOG("Calibration data loaded.");

    // Load shared memory control structure
    SHM_CONTROL* control = getControlStruct();
    if((int)control<0){
        ERR("Cannot get shared memory. err code : %d",control);
        return -1;
    }

    // Start sensing
    float position;
    for(int i = 0;!(control->exit);i++){
        float valueSum = 0;
        float weightedSum = 0;
        for(int j = 0;j<SENSOR_NUM;j++){
            float value = getCalibratedIRData(j, &sensorSetting);
            valueSum += value;
            weightedSum += value*sensorWeights[j];
            control->sensorValue[j] = value;
        }
   
        if(valueSum>0.1f){
            position = weightedSum/valueSum;
            control->position = position;
            control->lineout  = 0;
        }else{
            control->position = 0;
            control->lineout  = 1;
        }
        if(!(i%1000)){
            printf("%f %f %f %d\n",valueSum,weightedSum,control->position,control->lineout);
        }
    }

    bcm2835_spi_end();
    bcm2835_close();
    return 0;
}
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
#define STATE_THRESHOLD     0.25f

int   pins[SENSOR_NUM]           = {5,     6,     12,   13,  19,    16};
float sensorPosition[SENSOR_NUM] = {-5.f,  -3.f,  -1.f, 1.f, 3.f,   5.f};
float sensorWeight[SENSOR_NUM]   = {1.f,   1.f,  1.f,  1.f,  1.f,  1.f};
// float sensorWeight[SENSOR_NUM]   = {0.125f, 0.125f, 1.f, 1.f, 0.125f, 0.125f};

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
    int data  = 0;
    int data1 = getSensorData(channel);
    int data2 = getSensorData(channel);
    int data3 = getSensorData(channel);

    if(data1>data2){
             if(data2>data3) data = data2; // 3 2 1
        else if(data3>data1) data = data1; // 2 1 3
        else                 data = data3; // 2 3 1
    }else{
             if(data2<data3) data = data2; // 1 2 3
        else if(data3<data1) data = data1; // 3 1 2
        else                 data = data3; // 2 1 3
    }

    bcm2835_gpio_clr(pins[channel]);
    return data;
}

//Print bit of int.
void printBit(int x){
    printf("0b");
    for(int i = 31; i>=0;i--) printf("%d",1&&(x&1<<i));
    printf("\n");
}

//Print bit of int.
void printBit8(int x){
    printf("0b");
    for(int i = 7; i>=0;i--) printf("%d",1&&(x&1<<i));
    printf("\n");
}

SHM_CONTROL* control;

// Comment here if background is black
#define BACK_WHITE

typedef struct{
    int black[SENSOR_NUM];      // Dark min or max value for calibration
    int white[SENSOR_NUM];      // Light min or max value for calibration

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

    #ifndef BACK_WHILE
        #define COMP        <
        #define INIT_VALUE  2048
    #else
        #define COMP        >
        #define INIT_VALUE  0
    #endif

    // Initialize sensor setting
    // Warning : Setting for max-brightness. you should initizize it to infinity if you want to use min-brightness based calibration.
    for(int i = 0;i<SENSOR_NUM;i++){
        sensorSetting->black[i]=sensorSetting->white[i]=INIT_VALUE;
    }

    // Read adc SENSOR_CALIB_NUM times per SENSOR_NUM ir sensors and get  black max and white max.
    LOG("Press enter to read black.");
    getchar();
    LOG("Calibrating...");
    for(int i = 0;i<SENSOR_NUM*SENSOR_CALIB_NUM;i++){

        // Log
        if(!((i+1)%SENSOR_CALIB_NUM)){
            LOG("\tCalibrating...(%d/%d)",(i+1)/SENSOR_CALIB_NUM,SENSOR_NUM);
            for(int j = 0;j<SENSOR_NUM;j++) printf("%d ",sensorSetting->black[j]);
            printf("\n");
        }

        // 
        int irData = getRawIRData(i%SENSOR_NUM);
        if(irData COMP sensorSetting->black[i%SENSOR_NUM])sensorSetting->black[i%SENSOR_NUM]=irData;
    }

    LOG("Press enter to read white.");
    getchar();
    LOG("Calibrating...");
    for(int i = 0;i<SENSOR_NUM*SENSOR_CALIB_NUM;i++){

        if(!((i+1)%SENSOR_CALIB_NUM)){
            LOG("\tCalibrating...(%d/%d)",(i+1)/SENSOR_CALIB_NUM,SENSOR_NUM);
            for(int j = 0;j<SENSOR_NUM;j++) printf("%d ",sensorSetting->white[j]);
            printf("\n");
        }

        int irData = getRawIRData(i%SENSOR_NUM);
        if(irData COMP sensorSetting->white[i%SENSOR_NUM])sensorSetting->white[i%SENSOR_NUM]=irData;
    }

    // Update threshold, slope and bias.
    for(int i = 0;i<SENSOR_NUM;i++){
        #ifndef BACK_WHITE
            sensorSetting->calibSlope[i]    = 1.f/(sensorSetting->white[i]-sensorSetting->black[i]);
            sensorSetting->calibBias[i]     = -sensorSetting->black[i];
        #else
            sensorSetting->calibSlope[i]    = -1.f/(sensorSetting->white[i]-sensorSetting->black[i]);
            sensorSetting->calibBias[i]     = -sensorSetting->white[i];
        #endif
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

#define NODE_NONE       0x00 //00000000
#define NODE_LEFT 		0x01 //00000001
#define NODE_RIGHT		0x02 //00000010
#define NODE_BOTH 		0x04 //00000100
#define CROSS_SECTION 	0x08 //00001000

#define PI          3.141592653589793238462643383279f
#define WHEEL_RAD   2.5f    // Wheel radius in centimeter

// Finite state machine for route detecting
int8_t nodeCheck(int8_t currentState){

    // 0x1E : 00 011110
    // 0x20 : 00 100000
    // 0x01 : 00 000001
    // 0x3F : 00 111111

    int8_t sensorCount	= 0; 
    for(char i = 1;i<7;i++) sensorCount+=(currentState&(0x01<<i))>0;		

    static int8_t isNode = 0;
    static int64_t nearDist = 0;

    if((sensorCount==0)||(sensorCount>3)||(currentState&0x20)||(currentState&0x01)){
        if(isNode==1){
            if(control->tickC-nearDist>15){
                return 1;
            }
        }else{
            isNode = 1;
            nearDist = control->tickC;
        }
    }else{
        isNode = 0;
        nearDist = 0;
    }

    return 0x00;
}

// #define repeat(var,iter) for(int var = 0;var<iter;var++)

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
    control = getControlStruct();
    if((int)control<0){
        ERR("Cannot get shared memory. err code : %d",control);
        return -1;
    }

    // Start sensing
    float position, valueSum, weightedSum;
    control->position    = 0;
    control->lineout     = 0;
    control->sensorState = 0;

    // Set flag
    control->sensorAlive = 1;

    for(int i = 0;!(control->exit);i++){
        valueSum = 0;
        weightedSum = 0;

        for(int j = 0;j<SENSOR_NUM;j++){
            float value = getCalibratedIRData(j, &sensorSetting);
            valueSum += value;
            weightedSum += value*sensorPosition[j];
            control->sensorValue[j] = value;
            if(value>STATE_THRESHOLD){
                control->sensorState |= (0x01<<j);
            }else{
                control->sensorState &= ~(0x01<<j);                
            }
        }

        uint8_t node = nodeCheck(control->sensorState);
        control->node = node;
   
        // Check lineout and update control value
        if(valueSum>0.1f){
            position = weightedSum/valueSum;
            control->position = position;
            control->lineout  = 0;
        }else{
            control->position = 0;
            control->lineout  = 1;
        }

        // Print some values for debugging
        // if(!(i%100)){
        //     printBit8(control->sensorState);
        //     printf("%f %f %f %d\n",valueSum,weightedSum,control->position,control->lineout);
        // }
    }

    bcm2835_spi_end();
    bcm2835_close();

    LOG("Sensor processor successfully terminated.");

    control->sensorAlive = 0;
    return 0;
}
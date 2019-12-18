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
#define STATE_THRESHOLD     0.3f

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

//Print bit of int.
void printBit8(int x){
    printf("0b");
    for(int i = 7; i>=0;i--) printf("%d",1&&(x&1<<i));
    printf("\n");
}

SHM_CONTROL* control;

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

#define MARK_NONE       0x00 //00000000
#define MARK_LEFT 		0x01 //00000001
#define MARK_RIGHT		0x02 //00000010
#define MARK_BOTH 		0x04 //00000100
#define CROSS_SECTION 	0x08 //00001000

#define PI          3.141592653589793238462643383279f
#define WHEEL_RAD   2.5f    // Wheel radius in centimeter

// Finite state machine for route detecting
uint8_t routeFSM(uint8_t currentState){
	const static float decideDist   = 1.f; //1cm
	const static int   decideTics	= (decideDist) / (2*PI*WHEEL_RAD) * 400 * 2;
		
	//========[ Check condition ]========
	static uint8_t accumState		= 0x00;
	static uint8_t markerCondition 	= 0;

	{//Check accumulation condition
		static uint8_t sensorCount 	= 0; 
		sensorCount = 0;
		for(char i = 1;i<7;i++) sensorCount+=(currentState&(0x01<<i))>0;		
		/*
		This condition meanse that more than one of these three condition are true.
		1. More than 4 line sensors detected the line.
		2. Left marker sensor detected line.
		3. Right marker sensor detected line.
		*/
		static uint8_t tempCondition;
        // 0x20 = 00100000
        // 0x01 = 00000001
		tempCondition = (sensorCount>3)||((currentState&0x20)>0)||((currentState&0x01)>0);
		
		//Marker decision distance check
		static uint8_t destCondition	= 0;
        static uint32_t destDistance	= 0;
		if(tempCondition!=destCondition){
			destCondition = tempCondition;
			destDistance  = control->tickC+decideTics;
		}		
		if(control->tickC>=destDistance)markerCondition = destCondition;
	}
	
	#define ST_READY  0x00
	#define ST_ACCUM  0x01
	#define ST_DECIDE 0x02
	
	static uint8_t machineState = ST_READY;
	static uint8_t marker;
	
	marker = MARK_NONE;
	switch(machineState){
		case ST_READY:{
			/*
			 At the ready state, check the accumulation condition.
			When the accumulation condition is true,
			initialize the accumState and go to the accumulation state.
			*/
			if(markerCondition) {
				accumState		= 0x00;
				machineState  = ST_ACCUM;
			}
		}
		break;
		case ST_ACCUM :{
			/*
			 At the accumulation state, accumulate the ` and check the condition.
			Whent the accumulate condition is false, It means the end of the mark.
			Therefore, go to decide state.
			*/
			accumState |= currentState;
			if(!markerCondition) machineState = ST_DECIDE;
		}
		break;
		case ST_DECIDE :{
			/*
			 At the decide state, determinate the marker.
			If the marker is the cross section but not all the state is 1,
			It means not all the sensors have passed cross.
			Therefore, go back to accumState again.
			*/

            // 0x1E : 00 011110
            // 0x20 : 00 100000
            // 0x01 : 00 000001
            // 0x3F : 00 111111

			if((accumState&0x1E)==0x1E)	marker = CROSS_SECTION;
			else if(accumState&0x20){
				if(accumState&0x01)  	marker = MARK_BOTH;	
				else 					marker = MARK_RIGHT;
			}else if(accumState&0x01) 	marker = MARK_LEFT;
			else						marker = MARK_NONE;
			
			if((marker==CROSS_SECTION)&&accumState!=0x3F){
				marker=MARK_NONE;
				machineState = ST_ACCUM;
			}
			else machineState = ST_READY;
		}
		break;
	}
	
	return marker;
	
	#undef ST_READY
	#undef ST_ACCUM
	#undef ST_DECIDE
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
    control->sensorAlive = 1;

    // Start sensing
    float position, valueSum, weightedSum;
    control->position    = 0;
    control->lineout     = 0;
    control->sensorState = 0;
    for(int i = 0;!(control->exit);i++){
        
        valueSum = 0;
        weightedSum = 0;

        for(int j = 0;j<SENSOR_NUM;j++){
            float value = getCalibratedIRData(j, &sensorSetting);
            valueSum += value;
            weightedSum += value*sensorWeights[j];
            control->sensorValue[j] = value;
            if(value>STATE_THRESHOLD){
                control->sensorState |= (0x01<<j);
            }else{
                control->sensorState &= ~(0x01<<j);                
            }
        }

        uint8_t mark = routeFSM(control->sensorState);
        switch(mark){
            case MARK_NONE      : break;
            case MARK_LEFT 	    : LOG("Mark detected : left");
            break;
            case MARK_RIGHT	    : LOG("Mark detected : right");
            break;
            case MARK_BOTH 	    : LOG("Mark detected : both");
            break;
            case CROSS_SECTION  : LOG("Mark detected : cross");
            break;
            default             : ERR("Unknown mark");
        }
   
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
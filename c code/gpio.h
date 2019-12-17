#include <sys/mman.h>

// GPIO memory map base address
#define   GPIO_BASE      0x3F200000

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

typedef struct{
    int* pins;          // Used pins
    int  pinMask;       // Bit mask that indicate
    int* phaseMasks;    // Bit masks of each phase
    int  phase;         // Current phase = index of phaseMasks
}MOTOR;

#define MODE_IN(pin)    ((GPIO->SELECT[(pin)/10])&=(~(0x07<<(((pin)%10)*3)))) // Set gpio mode of given pin to input
#define MODE_OUT(pin)   ((GPIO->SELECT[(pin)/10])|=  (0x01<<(((pin)%10)*3)))  // Set gpio mode of given pin to output

#define SET(pin) ((GPIO->SET[0])|=(1<<(pin)))  // Set given gpio pin(If mode is input, do nothing.)
#define CLR(pin) ((GPIO->CLR[0])|=(1<<(pin)))  // Clear given gpio pin(If mode is input, do nothing.)

// Initialize = Clear all gpio pin.
// Turn of all gpio and set mode to in.
void initGPIO(){
    for(int i=0;i<28;i++){
        MODE_OUT(i);
        CLR(i);
        MODE_IN(i);
    }
}

// Get 32bit length bitmask that bits[0<=i<len]th bits are 1 and the others are 0.
int getMask(int*bits, int len){
    int mask = 0;
    for(int i = 0; i<len; i++) mask|=0x01<<bits[i];
    return mask;
}

// pins     : array of pins to use.
// pinLen   : length of pins
// phase    : array of phases to use.
// phaseLen : length of phase
void phaseToMask(int* pins, int pinLen, int* phase, int phaseLen){
    for(int i=0;i<phaseLen;i++){
        int mask = 0;
        // If jth bit of phase[i] is 1, set pins[j]th bit of mask to 1.
        // For example, if pins = {1,2,3,5} and phase[i] is 0b1001,
        // It means that 1st and 5th bit of mask should be 1.
        for(int j = 0; j<pinLen; j++) if(phase[i]&(0x01<<j))mask|=0x01<<pins[j];
        phase[i] = mask;
    }
}
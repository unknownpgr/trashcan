#ifndef _CONTROL_PROTOCOL
#define _CONTROL_PROTOCOL

#include <sys/types.h>    
#include <sys/shm.h>

#define bool unsigned char

typedef struct{
    /*============================================================*/
    // Control process sector (set from control processor)
    /*============================================================*/
    bool run;   // Continue control loop if 0
    bool exit;  // Exit control loop if 1, therefore exit process soon.

    // dt(=1/velocity) of each velocity. if positive, clockwise. if negative, anticlockwise.
    // If zero, stop wheel.
    int64_t dtL;   // Left wheel velocity
    int64_t dtR;   // Right wheel velocity

    /*============================================================*/
    // Motor process sector (set from motor process)
    /*============================================================*/
    bool    motorAlive;
    int     status;     // Reserved
    int64_t tickL;
    int64_t tickR;
    int64_t tickC;

    /*============================================================*/
    // Server process sector
    /*============================================================*/
    bool  serverAlive;
    float userVL;
    float userVR;

    /*============================================================*/
    // Sensor process sector
    /*============================================================*/
    bool  sensorAlive;
    float sensorValue[8];
    int8_t sensorState;
    float position;
    bool  lineout;
    int8_t mark;

    /*============================================================*/
    // Shared memory information sector
    /*============================================================*/
    key_t   shmKey;     // Key to identify shared memory
    int     shmSize;    // Size of shared memory
    int     shmID;      // ID of shared memory given by kernel. used to control or remove shared memory
    void*   shm;        // Pointer of shared memory. therefore it indicates motor control object itself.
}SHM_CONTROL;

#undef bool

#define SHM_ERR_GET     -1
#define SHM_ERR_ATTACH  -2
#define SHM_KEY         8080
#define SHM_SIZE        1024

SHM_CONTROL* getControlStruct(){
    int shmID = shmget(SHM_KEY,SHM_SIZE,IPC_CREAT|0666); //0666 = Read and write
    if(shmID==-1) return (SHM_CONTROL*)SHM_ERR_GET;

    // Paramter : (Key, Address(0 = assigned by kernel),  Permission(0 = READ/WRITE))
    // Return   : Pointer of start address of shared memory
    void* sharedMemory = shmat(shmID,(void*)0,0);   // Attach shared memory to variable
    if((int)sharedMemory==-1)return (SHM_CONTROL*)SHM_ERR_ATTACH;

    SHM_CONTROL* motorControl = (SHM_CONTROL*)sharedMemory;
    motorControl->shmKey    = SHM_KEY;
    motorControl->shmSize   = SHM_SIZE;
    motorControl->shmID     = shmID;
    motorControl->shm       = sharedMemory;

    return motorControl;
}

int removeControlStruct(SHM_CONTROL* control){
    return shmctl(control->shmID, IPC_RMID, 0);
}

#endif
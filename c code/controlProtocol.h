#ifndef _CONTROL_PROTOCOL
#define _CONTROL_PROTOCOL

#include <sys/types.h>    
#include <sys/shm.h>

#define bool unsigned char
typedef struct{
    //Main process sector
    bool run;
    bool exit;
    float vL;   // Left wheel velocity
    float vR;   // Right wheel velocity
    int command;

    //Motor process sector
    bool motorAlive;
    int status;

    //Shared memory information sector
    key_t shmKey;   // Key to identify shared memory
    int shmSize;    // Size of shared memory
    int shmID;      // ID of shared memory given by kernel. used to control shared memory
    void* shm;      // Pointer of shared memory
}MOTOR_CONTROL;
#undef bool

#define SHM_ERR_GET     -1
#define SHM_ERR_ATTACH  -2
#define SHM_KEY 8080
#define SHM_SIZE 1024

MOTOR_CONTROL* getControlStruct(){
    int shmID = shmget(SHM_KEY,SHM_SIZE,IPC_CREAT|0666); //0666 = Read and write
    if(shmID==-1) return (MOTOR_CONTROL*)SHM_ERR_GET;

    // Paramter : (Key, Address(0 = assigned by kernel),  Permission(0 = READ/WRITE))
    // Return   : Pointer of start address of shared memory
    void* sharedMemory = shmat(shmID,(void*)0,0);   // Attach shared memory to variable
    if((int)sharedMemory==-1)return (MOTOR_CONTROL*)SHM_ERR_ATTACH;

    MOTOR_CONTROL* motorControl = (MOTOR_CONTROL*)sharedMemory;
    motorControl->shmKey    = SHM_KEY;
    motorControl->shmSize   = SHM_SIZE;
    motorControl->shmID     = shmID;
    motorControl->shm       = sharedMemory;

    return motorControl;
}

#endif
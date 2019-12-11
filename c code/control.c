#include <stdio.h>
#include <unistd.h>
#include <sched.h>
#include <sys/shm.h>
#include "controlProtocol.h"

int main(){
    printf("Start controller.\n");

    key_t shmKey = 8080;
    int shmSize = 1024;
    int shmid = shmget(shmKey,shmSize,IPC_CREAT|0666);
    char* sharedMemory;

    if(shmid==-1){
        printf("Cannot get/create shared memory.\n");
        return -1;
    }
    printf("Shared memory successfully assigned.\n");

    // Paramter : (Key, Address(0 = assigned by kernel),  Permission(0 = READ/WRITE))
    sharedMemory = shmat(shmid,(void*)0,0);
    if((int)sharedMemory==-1){
        printf("Cannot attach shared memory. : \n");
        return -1;
    }

    // Print some bytes of memory to check if memory is initialized.
    printf("Memory check : \n");
    for(int i = 0;i<8;i++) printf("|%c",*(sharedMemory+i));

    printf("|\n");
    MOTOR_CONTROL* control = (MOTOR_CONTROL*)sharedMemory;
    sleep(1);
    printf("MotorAlive : %d\n",control->motorAlive);

    control->run=1;
    sleep(1);
    printf("MotorAlive : %d\n",control->motorAlive);

    control->run=0;
    sleep(1);
    printf("MotorAlive : %d\n",control->motorAlive);

    control->exit=1;
    sleep(1);
    printf("MotorAlive : %d\n",control->motorAlive);

    // if(-1 == (shmctl(shmid, IPC_STAT, 0)))
    // {   
    //     printf("Error");
    //     perror("shmctl");
    // }   

    if(-1 == (shmctl(shmid, IPC_RMID, 0)))
    {   
        printf("Failed to remove shared memory\n");
    }   

    printf("End control.\n");
}
#include <stdio.h>
#include <unistd.h>
#include <sched.h>
#include <sys/shm.h>
#include "controlProtocol.h"

#define ABS(x) (((x)>0)?(x):(-(x)))

int main(){
    printf("Start controller.\n");

    key_t shmKey  = 8080;
    int   shmSize = 1024;
    int   shmid   = shmget(shmKey,shmSize,IPC_CREAT|0666);
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

    printf("Start motor : ");
    // Get control object from shared memory.
    MOTOR_CONTROL* control = (MOTOR_CONTROL*)sharedMemory;
    control->run = 0;
    sleep(0.1);
    printf("%d\n",control->motorAlive);
    
    float velocity = 0;

    for(;;){
        // Get velocity from user
        printf("Enter velocity. enter -128 for exit. >> ");
        scanf("%f",&velocity);
        printf("\n");

        // Clear buffer
        int c;
        while ((c = getchar()) != '\n' && c != EOF);

        if(velocity==-128.f)break;
        if(ABS(velocity)<1){
            control->run =0;
            printf("Stop motor\n");
        }
        if(ABS(velocity)<2000){
            printf("Velocity is too slow.\n");
            if(velocity<0){
                velocity = -2000;
                printf("Set velocity to -2000\n");
            }else{
                velocity = 2000;
                printf("Set velocity to 2000\n");
            }
        }
        control->run =1;
        control->velocity = velocity;
        printf("Set velocity to %f\n",velocity);
    }

    control->run=0;
    sleep(0.1);
    printf("MotorAlive : %d\n",control->motorAlive);

    control->exit=1;
    sleep(0.1);
    printf("MotorAlive : %d\n",control->motorAlive);

    if(shmctl(shmid, IPC_RMID, 0)==-1){   
        printf("Failed to remove shared memory\n");
    }
    printf("Shared memory removed.\n");

    printf("End control.\n");
}
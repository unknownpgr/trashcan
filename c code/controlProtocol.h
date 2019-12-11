#ifndef _CONTROL_PROTOCOL
#define _CONTROL_PROTOCOL
#define bool unsigned char
typedef struct{
    bool run;
    bool exit;
    bool motorAlive;
    float velocity;
    int command;
    int status;
}MOTOR_CONTROL;
#undef bool
#endif
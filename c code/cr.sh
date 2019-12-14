#! /bin/bash
echo compile control;
gcc -lm -o cont control.c;
echo compile motor;
gcc -o motor motor.c;
echo compile finished.;
echo run;
sudo ./motor& sudo ./cont
echo everything finished.;

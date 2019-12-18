#! /bin/bash
# bash script to compile the codes and run them.
echo compile...;
gcc -lm -o cont.o control.c;
gcc -o motor.o motor.c;
gcc -o comm.o communication.c;
gcc -o sensor.o sensor.c -lbcm2835;
echo compile finished.;
echo run program;
sudo sudo ./cont.o
echo everything finished.;

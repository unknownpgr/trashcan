#! /bin/bash
# bash script to compile the codes and run them.
echo compile control...;
gcc -lm -o cont control.c;
echo compile motor...;
gcc -o motor motor.c;
echo compile finished.;
echo run program;
sudo ./motor& sudo ./cont
echo everything finished.;

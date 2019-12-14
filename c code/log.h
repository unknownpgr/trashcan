#ifndef LOG_H
#define LOG_H

// Control variable
#define VERBOSE

#include <sys/time.h>

#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#ifdef VERBOSE
    #define KNRM  "\x1B[0m"
    #define KRED  "\x1B[31m"
    #define KGRN  "\x1B[32m"
    #define KYEL  "\x1B[33m"
    #define KBLU  "\x1B[34m"
    #define KMAG  "\x1B[35m"
    #define KCYN  "\x1B[36m"
    #define KWHT  "\x1B[37m"

    void timestamp(){
        struct timeval  now;
        struct tm*      local;
        gettimeofday(&now, NULL);
        local = localtime(&now.tv_sec);
        printf(KBLU "[%02d:%02d:%02d.%03d]" KNRM, local->tm_hour, local->tm_min, local->tm_sec, now.tv_usec / 1000);
    }

    extern char *__progname;

    #define LOG(...) {timestamp();printf("[%s]", __progname);printf(KGRN __VA_ARGS__);printf(KNRM "\n");}
    #define ERR(...) {timestamp();printf("[%s]", __progname);printf(KRED __VA_ARGS__);printf(KNRM "\n");}
#else
    #define LOG(...)
    #define ERR(...)
#endif 

#endif
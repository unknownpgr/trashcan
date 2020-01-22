#ifndef _INTERVAL_H
#define _INTERVAL_H
#include <sys/types.h>

#define NANOSEC 1000000000

// INTERVAL_LOOP interval structure. must be initialized before being used.
// You muse initialize INTERVAL.recent to 0.
typedef struct{
    int64_t interval;
    int64_t recent;
}INTERVAL;

// Make a for statement that have __time variable which is updated in each loop.
#define INTERVAL_LOOP for(struct timespec __time;;clock_gettime(CLOCK_REALTIME, &__time))

/*
RUN_TASK : run given task in INTERVAL_LOOP
RUN_TASK(thread,task){
    static long __td;
    __td=__time.tv_nsec-(thread).recent;
    __td = (__td<0)?(__td+1000000000):(__td);
    if(__td>(thread).interval){
        (thread).recent+=(thread).interval;
        if((thread).recent>NANOSEC)(thread).recent-=NANOSEC;
        {task;}
    }
}
*/
#define RUN_TASK(thread,task) {static long __td;__td=__time.tv_nsec-(thread).recent;__td = (__td<0)?(__td+NANOSEC):(__td);if(__td>(thread).interval){(thread).recent+=(thread).interval;if((thread).recent>NANOSEC)(thread).recent-=NANOSEC;{task;}}}

// Get interval struct and interval and return interval in second.
double initInterval(INTERVAL* interval, int64_t nanoSec){
    interval->recent = 0;
    interval->interval = nanoSec;
    return nanoSec*1.0/NANOSEC;
}

#endif
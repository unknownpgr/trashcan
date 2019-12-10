# How to isolate specific core from kernel and user proceses
1. append isolcpus=N to boot/cmdline.txt. N is cpu number. To isolate cpu 3(which means 4th core), append isolcpus=3 to boot/cmdline.txt
1. CPU useage can be checked with 'top' command. To check cpu usage of each core, press [1] key.
1. sched_setaffinity 함수는 3개의 매개변수를 받는데 첫번째는 프로세스 ID(pid)입니다. pid 대신 0을 넘기면 자동으로 현재 동작중인 프로세스로 설정됩니다. 두번째는 cpusetsize 입니다. 보통은 sizeof(cpu_set_t)로 설정하면 됩니다. 세번째는 mask 포인터입니다. mask 포인터는 아래의 매크
로 함수들을 사용해서 편리하게 설정 할 수 있습니다.
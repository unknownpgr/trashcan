#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include "log.h"
#include "controlProtocol.h"

#define BUFF_SIZE   1024
#define SERVER_PORT 8080

void delay_us(long int delay){
	static long int start_time;
	static long int time_difference;
	static struct timespec gettime_now;

	clock_gettime(CLOCK_REALTIME, &gettime_now);
	start_time = gettime_now.tv_nsec;

	for(;;){
		clock_gettime(CLOCK_REALTIME, &gettime_now);
		time_difference = gettime_now.tv_nsec - start_time;

        /*
        The maximum value of tv_nsec maximum is 1000000000 = 1sec.
        Therefore if time_differece is negative, add 1000000000 to time differece.
        */
    	if (time_difference < 0) time_difference += 1000000000;
		if (time_difference > (delay * 1000)) break;
	}
}

int getSocketServer(int port){
    int    server_socket;
    struct sockaddr_in  server_addr;

    server_socket  = socket(PF_INET, SOCK_STREAM, 0);
    if(-1 == server_socket)
    {
        ERR("Cannot create socket.");
        return -1;
    }

    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family      = AF_INET;
    server_addr.sin_port        = htons(port);
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if(-1 == bind(server_socket, (struct sockaddr*)&server_addr, sizeof(server_addr) ) )
    {
        ERR("Cannot bind socket to port %d.",port);
        return -1;
    }

    if(-1 == listen(server_socket, 5))
    {
        ERR("Cannot listen.");
        return -1;
    }

    return server_socket;
}

int main(){
    int   client_socket;
    int   client_addr_size;

    struct sockaddr_in   client_addr;

    char   buff_rcv[BUFF_SIZE+5];
    char   buff_snd[BUFF_SIZE+5];

    // Get control object from shared memory.
    SHM_CONTROL* control = getControlStruct();
    if((int)control<0){
        ERR("Cannot get shared memory. err code : %d",control);
        return -1;
    }

    // Initialize check
    if(control->serverAlive){
        ERR("Another server process still alive.");
        return -1;
    }
    // Set flag
    control->serverAlive = 1;

    int server_socket  = getSocketServer(SERVER_PORT);
    while(1)
    {
        if(control->exit)break;

        LOG("Waiting clinet...");
        client_addr_size  = sizeof(client_addr);
        client_socket     = accept(server_socket, (struct sockaddr*)&client_addr, &client_addr_size);

        LOG("Client connected!");
        if (-1 == client_socket){
            ERR("Client connection failed.\n");
            break;
        }
        char flag = 0;
        
        for(;;){
            if(control->exit)break;
            flag=!flag;
            int len = read (client_socket, buff_rcv, BUFF_SIZE);
            if(len<1)break;
            float v;
            v = *(float*)buff_rcv;
            if(flag){
                LOG("L: %f",len, v);
                control->userVL = v;
            }else{
                LOG("R: %f",len, v);
                control->userVR = v;
            }            
            write(client_socket, buff_rcv,4);
            buff_rcv[0]=0;
        }
        LOG("End connection.");
    }
    control->serverAlive = 0;
    LOG("Server process successfully terminated.");
}
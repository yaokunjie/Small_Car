
#include<stdlib.h>
#include<stdio.h>
#include<string.h>
#include<netdb.h>
#include<sys/types.h>
#include<netinet/in.h>
#include<sys/socket.h>
#include<unistd.h>
#include<arpa/inet.h>
#include<errno.h>
#include<iostream>
class Transform
{
    public:
    Transform();
    ~Transform();
    void correspondence(char*sendbuffer);
    float car_x, car_y;
	float goal_x, goal_y;
    private:
       int sockfd;
       struct sockaddr_in server_addr;
       struct hostent *host;
       char *recv_str;
       char *cut_str;
       bool flag;
       void init_socket();
       void send_command(char *sendbuffer);
       void recv_data(char *recvbuffer);
       void extract();
};


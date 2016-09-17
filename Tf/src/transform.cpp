#include "transform.h"
#include <vector>
using namespace std;
Transform::Transform()
{
}

Transform::~Transform()
{
    close(sockfd);

}

void Transform::init_socket()
{
       // struct sockaddr_in server_addr;
      //  struct hostent *host;
        int portnumber,nbytes;
        host=gethostbyname("192.168.1.106");
        portnumber=6666;
        sockfd=socket(AF_INET,SOCK_STREAM,0);
        bzero(&server_addr,sizeof(server_addr));
        server_addr.sin_family=AF_INET;
        server_addr.sin_port=htons(portnumber);
        server_addr.sin_addr=*((struct in_addr *)host->h_addr);
}
void Transform::recv_data(char *recvbuffer)
{
//    if(connect(sockfd,(struct sockaddr *)(&server_addr),sizeof(struct sockaddr))==-1)
//    {
//    fprintf(stderr,"Connect error:%s\n",strerror(errno));
//    exit(1);
//    }
    recv(sockfd,recvbuffer,50,0);
    recv_str=recvbuffer;

}
void Transform::send_command(char*sendbuffer)
{

    if(connect(sockfd,(struct sockaddr *)(&server_addr),sizeof(struct sockaddr))==-1)
    {
    fprintf(stderr,"Connect error:%s\n",strerror(errno));
    exit(1);
    }
    send(sockfd, sendbuffer, 50, 0);

}
void Transform::extract()
{


    int end=strlen(recv_str);
    string str=recv_str;
    string goal_xx,goal_yy,car_xx,car_yy;
    vector<int> tmp;

    for(int i=0;i<end;i++)
    {
        if(str.at(i)==',')
        {
            tmp.push_back(i);
        }
    }
    if(str.at(0)=='('&&str.at(end-1)==')')
    {
        goal_xx=str.substr(1,tmp.at(0)-1);
        goal_yy=str.substr(tmp.at(0)+1,tmp.at(1)-tmp.at(0)-1);


        car_xx=str.substr(tmp.at(1)+1,tmp.at(2)-tmp.at(1)-1);
        car_yy=str.substr(tmp.at(2)+1,(end-1)-tmp.at(2)-1);


        goal_x=atof(goal_xx.c_str());
        goal_y=atof(goal_yy.c_str());
        car_x=atof(car_xx.c_str());
        car_y=atof(car_yy.c_str());


    }
    else
    {
        flag=false;

    }
}
void Transform::correspondence(char*sendbuffer)
{
    char recv[50];
    printf("working ..");
    init_socket();
    send_command(sendbuffer);
    recv_data(recv);
    extract();
}




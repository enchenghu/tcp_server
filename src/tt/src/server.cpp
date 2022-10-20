#include<stdio.h> 
#include<stdlib.h> 
#include<string.h> 
#include<errno.h> 
#include<sys/types.h> 
#include<sys/socket.h> 
#include<netinet/in.h> 
#include <sys/ioctl.h>
#include <fcntl.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <pthread.h>
#include <arpa/inet.h>
#include <iostream>
using namespace std;
#define MAXLINE 4096 
#define INFO "info"
#define WARNING "warning"
#define ERROR "error"
#define LOG(n) std::cout << n << "|" << __FILE__ << ":" << __LINE__ << "->|"
#define LOG_END << std::endl;
#define TCP_PC_SIZE 32000
#define UDP_IP "127.0.0.1"
typedef enum
{
    POWER_WRITE = 1, 
    CFAR_WRITE,
    DFT3_WRITE,
    DIFF_WRITE,
	REG_WRITE,
    POWER_READ,
    CFAR_READ,
    DFT3_READ,
    DIFF_READ,
	REG_READ,
	PC_READ,
	FFT_ADC_READ_START,
	FFT_ADC_READ_STOP
}commandType;
uint8_t *encode_cali_data = nullptr;
typedef struct API_Header
{
	uint16_t 	usPrefix; // 0xeeff
	uint16_t 	usType; // 0x10 version 1.0
	uint16_t 	usCommand; // command enum
	uint16_t 	usPayloadCrc;
	uint32_t 	unLength;
}API_Header;

typedef struct 
{
	API_Header 	mHead; // 0xeeff
	uint32_t 	mCommandVal[2];
	//float 	mCommandVal_f;
} commandMsg;

typedef struct 
{
	commandMsg 	cmdmsg; 
	uint8_t 	pcTcpData[TCP_PC_SIZE];
} pcData_t;

pcData_t g_msg;
bool check_file_exist(const std::string &path) {
#ifdef _MSC_VER  
	bool ret = 0 == _access(path.c_str(), 0);
#else
	bool ret = 0 == access(path.c_str(), F_OK);
#endif
	if (!ret) {
		LOG(INFO) << path << " not exist";
	}
	return ret;
}

size_t GetFileSize(const string &filepath) {
	FILE *fd = fopen(filepath.c_str(), "rb");
	if (fd == NULL) {
		LOG(ERROR) << "Failed to open file " << filepath;
		return 0;
	}
	// Determine size of the file
	fseek(fd, 0, SEEK_END);
	size_t file_length = static_cast<size_t>(ftell(fd));
	fseek(fd, 0, SEEK_SET);
	fclose(fd);

	return file_length;
}


int  LoadDat(const char *cali_file_path)
{
	if (NULL == cali_file_path) {
		LOG(ERROR) << "Bad cali_file_path " LOG_END;
		return -1;
	}
	if (!check_file_exist(cali_file_path)) {
		LOG(ERROR) << "Cali file " << cali_file_path << " not exist " LOG_END;
		return -2;
	}
	auto file_size = GetFileSize(cali_file_path);
    cout << "file_size is " << file_size << endl;
	if (file_size < 1000) {
		LOG(ERROR) << "Bad cali file size " << file_size LOG_END;
		return -3;
	}

	FILE *file = fopen(cali_file_path, "rb");
	encode_cali_data = (uint8_t *)malloc(file_size);
	fseek(file, 0, SEEK_SET);
	fread(encode_cali_data, file_size, 1, file);
    return file_size;
}



int loopsend(int socket, int size, uint8_t* data){
    int index = 0;
    //pcData_t msg;
    while(index < 200){
        usleep(1000);
        if(!data){
            cout << "data size error" << endl;
            return -1;
        }
        memset(&g_msg, 0, sizeof(g_msg));
        g_msg.cmdmsg.mHead.usCommand = PC_READ;
        g_msg.cmdmsg.mCommandVal[1] = index++;
        //cout << " count is " << index++ << endl;; 
        memcpy(g_msg.pcTcpData, data, TCP_PC_SIZE);
        int ret = send(socket, &g_msg, sizeof(g_msg), 0 );
        cout << "send byte is " << ret << endl;

        if( ret < 0 ){
            cout << "write msg failed" << endl;
            return -2;          
        }else if(ret == 0){
            cout << "write full" << endl;
            return -3;
        }
        data += TCP_PC_SIZE;
    }

    return 0;
}

int udpRecvSocketFd_  = 0;
 void *udp_msg_sender(void *)
 {
     int ret;
    struct sockaddr_in ser_addr; 

    udpRecvSocketFd_ = socket(AF_INET, SOCK_DGRAM, 0); //AF_INET:IPV4;SOCK_DGRAM:UDP
    if(udpRecvSocketFd_ < 0)
    {
        printf("create udpRecvSocketFd fail!");
    }

    memset(&ser_addr, 0, sizeof(ser_addr));
    ser_addr.sin_family = AF_INET;
	ser_addr.sin_addr.s_addr = inet_addr(UDP_IP);
    //ser_addr.sin_addr.s_addr = htonl(INADDR_ANY); //IP地址，需要进行网络序转换，INADDR_ANY：本地地址
    ser_addr.sin_port = htons(8888);  //端口号，需要网络序转换

#if 0

    ret = bind(udpRecvSocketFd_, (struct sockaddr*)&ser_addr, sizeof(ser_addr));
    if(ret < 0)
    {
        printf("socket bind fail!\n");
    }
#endif

    socklen_t len;
    struct sockaddr_in src;
    char buf[1024] = "client send: TEST UDP MSG!\n";
    char buff[1024] = "client send: TEST UDP MSG!\n";
	printf("ready recv udp msg!\n");
    len = sizeof(sockaddr);
    while(1)
    {
        //memset(buf, 0, 1024);
        //recvfrom(udpRecvSocketFd_, buf, 1024, 0, (struct sockaddr*)&src, &len);  //接收来自server的信息
        printf("client send is :%s\n",buff);
#if 1
        int nnn = sendto(udpRecvSocketFd_, buff, 1024, 0, (struct sockaddr*)&ser_addr, len);
#endif
        usleep(500 * 1000);  //一秒发送一次消息
    }
 }

int main(int argc, char** argv) 
{ 
    ros::init(argc, argv, "talker");
    ros::NodeHandle roshandle;
    pthread_t udp_send;
    const char *cali_file_path = "/home/encheng/data/cp_data.dat";
    int  filesize = LoadDat(cali_file_path);


    int listenfd, connfd; 
    commandMsg msg;
    struct sockaddr_in servaddr; 
    char buff[4096]; int n; 
    if( (listenfd = socket(AF_INET, SOCK_STREAM, 0)) == -1 )
    { 
        printf("create socket error: %s(errno: %d)\n",strerror(errno),errno);
        exit(0); 
    } 

    //fcntl(listenfd, F_SETFL, O_NONBLOCK);

    memset(&servaddr, 0, sizeof(servaddr)); 
    servaddr.sin_family = AF_INET; 
    servaddr.sin_addr.s_addr = htonl(INADDR_ANY);             
    servaddr.sin_port = htons(6666); 
    long long index_0 = 0;

    if( bind(listenfd, (struct sockaddr*)&servaddr, sizeof(servaddr)) == -1)
    { 
        printf("bind socket error: %s(errno: %d)\n",strerror(errno),errno);
        exit(0); 
    } 

    if( listen(listenfd, 10) == -1)
    { 
        printf("listen socket error: %s(errno: %d)\n",strerror(errno),errno);
        exit(0); 
    } 
    int para;
    printf("======waiting for client's request======\n"); 
    while(1)
    { 
        if( (connfd = accept(listenfd, (struct sockaddr*)NULL, NULL)) == -1)
        { 
            printf("accept socket error: %s(errno: %d)\n",strerror(errno),errno); 
            usleep(100);
            continue; 
        } else {
            break;
        }
    }
    printf("======receive data======\n"); 
	int nRecvBuf= 320 * 1024;//设置为32K
	setsockopt(connfd, SOL_SOCKET,SO_RCVBUF,(const char*)&nRecvBuf,sizeof(int));

	int nSendBuf= 320 * 1024;//设置为32K
	setsockopt(connfd, SOL_SOCKET,SO_SNDBUF,(const char*)&nSendBuf,sizeof(int));

    while(1)
    {
        memset(&msg, 0,  sizeof(msg)); 
        n = recv(connfd, &msg, sizeof(msg), MSG_WAITALL); 
        printf("buffer len is %d\n", n); 
        if(n == 0){
            close(connfd); 
            connfd = accept(listenfd, (struct sockaddr*)NULL, NULL);
            continue;
        }
        switch (msg.mHead.usCommand)
        {
        case POWER_READ:
        case CFAR_READ:
        case DFT3_READ:
        case DIFF_READ:
        case REG_READ:
            write(connfd, &msg, sizeof(msg));
            break;   
        case PC_READ:
            while(1){
                //cout << "send data times: " << index_0++ << endl;
                if(loopsend(connfd, filesize, encode_cali_data))
                    return -1;
            }
            break;
        case FFT_ADC_READ_START:
        	pthread_create(&udp_send, NULL, udp_msg_sender, NULL);
            break;
        case FFT_ADC_READ_STOP:
            close(udpRecvSocketFd_);
            break;
        default:
            break;
        }

        //buff[n] = '\0'; 
        printf("msg.mCommandVal from client: %d\n", msg.mCommandVal[1]); 
    } 
    close(connfd); 
    close(listenfd); 
    return 0;
}

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
#include <cmath>

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
	FFT_ADC_READ_STOP,
	POINTCLOUD_DISPLAY_START,
	POINTCLOUD_DISPLAY_STOP
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
	uint16_t 	usPrefix; // 0xeeff
	uint16_t 	usType; // 0x10 version 1.0
	uint16_t 	usRollingCounter; 
	uint16_t 	usPayloadCrc;
	uint32_t 	usFrameCounter;
	uint32_t 	unLength;
}UDP_Header;

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

typedef struct 
{
	UDP_Header 	mHead; 
	uint8_t 	pcUdpData[1024];
} udpMsg;

/* pointcloud info*/
#pragma pack(1)    // pack(1): pack之间的数据类型，1字节对齐

typedef struct {
    uint16_t uphPrefix;         // 0xEEFF
    uint16_t uphVersion;        // 0x0102: Version V1.2
    uint32_t uphTimeLsb;        // 时间戳低位（ns为单位，还是UTC时间戳）
    uint32_t uphTimeMsb;        // 时间戳高位
    uint16_t uphPayloadLength;  // 点云数据长度  // 目前固定：1400（包含100个点云元数据）
    uint16_t uphFrameCounter;   // 点云图的帧计数
    uint16_t uphRollingCounter; // rolling counter
    uint16_t uphState;          // 报文标志（比如：是否是最后一帧）
    uint16_t uphHeaderCrc;      // UDP头数据Crc校验值（Header）
    uint16_t uphPayloadCrc;     // UDP载荷数据校验值（Payload）
}UDP_PC_head_st;    // 24Bytes


typedef struct {	
    uint32_t pcmIndensity;  // 功率
    uint32_t pcmDistance;   // 距离（单位：米）   // 实际值：÷65536
    int16_t  pcmSpeed;      // 速度（单位：米/s） // 实际值：÷128(1bit-signed, 8bit-integer, 7bit-decimal-fraction)  // 正数:远离 负数:靠近
    uint16_t pcmVertical;   // 俯仰角（单位：°）  // 实际值：÷256  // 数据0实际上是-2.5°
    uint16_t pcmHorizontal; // 水平角（单位：°）  // 实际值：*360/32000*2  // 0° ~ 359°
}PC_pointMeta_st;   // 14Bytes


typedef struct {
    UDP_PC_head_st  UDP_PC_head;                        // head：24字节
    PC_pointMeta_st UDP_PC_payload[100];   // Payload：1400字节 = 14 * 100
} UDP_PC_package_st;    // 1424字节

#pragma pack()     // pack() 结束
pcData_t g_msg;
UDP_PC_package_st g_msg_pc;

bool ifstop = false;
bool ifPCstop = false;

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
int udpPcRecvSocketFd_  = 0;

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
    ser_addr.sin_port = htons(8889);  //端口号，需要网络序转换

#if 0

    ret = bind(udpRecvSocketFd_, (struct sockaddr*)&ser_addr, sizeof(ser_addr));
    if(ret < 0)
    {
        printf("socket bind fail!\n");
    }
#endif

    socklen_t len;
    struct sockaddr_in src;
    uint8_t buf[1024] = "client send: TEST UDP MSG!\n";
    uint8_t buff[] = "00010203040506070809\n";
    string mbuf = "00010203040506070809";

	printf("ready recv udp msg!\n");
    len = sizeof(sockaddr);
    string header;
    header.resize(sizeof(UDP_Header), '0');
    std::cout << "header is " << header << std::endl;
    string msg = header + mbuf;
    std::cout << "msg size is " << msg.size() << std::endl;
    std::cout << "msg sizeof is " << sizeof(msg) << std::endl;
    std::cout << "header size is " << header.size() << std::endl;
    std::cout << "mbuf size is " << mbuf.size() << std::endl;
    udpMsg sendMsg;
    memset(&sendMsg, 0, sizeof(udpMsg));
    int index = 0;
    while(!ifstop)
    {
        //memset(buf, 0, 1024);
        //recvfrom(udpRecvSocketFd_, buf, 1024, 0, (struct sockaddr*)&src, &len);  //接收来自server的信息
        //printf("client send is :%s\n",msg.c_str());
        //std::cout << "msg is " << msg << std::endl;
        sendMsg.mHead.usFrameCounter = index++;
        memset(&sendMsg, 0, sizeof(sendMsg));
        for(int i = 0; i < 256; i++){
            //sendMsg = i;
            int nnn = sendto(udpRecvSocketFd_, &sendMsg, sizeof(udpMsg), 0, (struct sockaddr*)&ser_addr, len);
            usleep(500);  //一秒发送一次消息
        }
#if 1
#endif
    }
 }



void pcSockerInit()
{
    udpPcRecvSocketFd_ = socket(AF_INET, SOCK_DGRAM, 0); //AF_INET:IPV4;SOCK_DGRAM:UDP
    if(udpPcRecvSocketFd_ < 0)
    {
        printf("create udpRecvSocketFd fail!");
    }
}

void udp_pc_msg_send_once()
 {
     int ret;
    struct sockaddr_in ser_addr; 
    memset(&ser_addr, 0, sizeof(ser_addr));
    ser_addr.sin_family = AF_INET;
	ser_addr.sin_addr.s_addr = inet_addr(UDP_IP);
    //ser_addr.sin_addr.s_addr = htonl(INADDR_ANY); //IP地址，需要进行网络序转换，INADDR_ANY：本地地址
    ser_addr.sin_port = htons(8001);  //端口号，需要网络序转换

    socklen_t len;
    struct sockaddr_in src;
    len = sizeof(sockaddr);

    UDP_PC_package_st sendMsg;
    memset(&sendMsg, 0, sizeof(sendMsg));
    static long long index = 0;
    //while(!ifPCstop)
    //{
        //memset(buf, 0, 1024);
        //recvfrom(udpRecvSocketFd_, buf, 1024, 0, (struct sockaddr*)&src, &len);  //接收来自server的信息
        //printf("client send is :%s\n",msg.c_str());
        //std::cout << "msg is " << msg << std::endl
        sendMsg.UDP_PC_head.uphPrefix  = 0xEEFF;
        sendMsg.UDP_PC_head.uphVersion = 0x0102;
        sendMsg.UDP_PC_head.uphTimeLsb = 0;
        sendMsg.UDP_PC_head.uphTimeMsb = 0;
        sendMsg.UDP_PC_head.uphPayloadLength = 1400; // 每个UDP报文，点云元数据大小（1400=14*100）
        sendMsg.UDP_PC_head.uphFrameCounter = 0;
        sendMsg.UDP_PC_head.uphRollingCounter = 0;
        sendMsg.UDP_PC_head.uphState = 0;
        sendMsg.UDP_PC_head.uphHeaderCrc = 0;
        sendMsg.UDP_PC_head.uphPayloadCrc = 0;
        //memset(&sendMsg, 0, sizeof(sendMsg));
        for(int i = 0; i < 200; i++){
            //memset(&sendMsg, 0, sizeof(sendMsg));
#if 0
            for(int j = 0; j < 100; j++){
                sendMsg.UDP_PC_payload[j].pcmIndensity = 123456;
                sendMsg.UDP_PC_payload[j].pcmDistance = 65536;
                sendMsg.UDP_PC_payload[j].pcmSpeed = -128;
                sendMsg.UDP_PC_payload[j].pcmVertical = 256;
                sendMsg.UDP_PC_payload[j].pcmHorizontal = 10000;

            }
#endif
            //int nnn = sendto(udpPcRecvSocketFd_, &sendMsg, sizeof(sendMsg), 0, (struct sockaddr*)&ser_addr, len);
            int nnn = sendto(udpPcRecvSocketFd_, encode_cali_data + i * 1424, 1424, 0, (struct sockaddr*)&ser_addr, len);
            usleep(500);  //一秒发送一次消息
        }
    //}
 }


void *udppc_msg_sender(void *)
 {
     int ret;
    struct sockaddr_in ser_addr; 

    udpPcRecvSocketFd_ = socket(AF_INET, SOCK_DGRAM, 0); //AF_INET:IPV4;SOCK_DGRAM:UDP
    if(udpPcRecvSocketFd_ < 0)
    {
        printf("create udpRecvSocketFd fail!");
    }

    memset(&ser_addr, 0, sizeof(ser_addr));
    ser_addr.sin_family = AF_INET;
	ser_addr.sin_addr.s_addr = inet_addr(UDP_IP);
    //ser_addr.sin_addr.s_addr = htonl(INADDR_ANY); //IP地址，需要进行网络序转换，INADDR_ANY：本地地址
    ser_addr.sin_port = htons(8001);  //端口号，需要网络序转换

#if 0

    ret = bind(udpRecvSocketFd_, (struct sockaddr*)&ser_addr, sizeof(ser_addr));
    if(ret < 0)
    {
        printf("socket bind fail!\n");
    }
#endif

    socklen_t len;
    struct sockaddr_in src;
    len = sizeof(sockaddr);

    UDP_PC_package_st sendMsg;
    memset(&sendMsg, 0, sizeof(sendMsg));
    long long index = 0;
    while(!ifPCstop)
    {
        //memset(buf, 0, 1024);
        //recvfrom(udpRecvSocketFd_, buf, 1024, 0, (struct sockaddr*)&src, &len);  //接收来自server的信息
        //printf("client send is :%s\n",msg.c_str());
        //std::cout << "msg is " << msg << std::endl;
        sendMsg.UDP_PC_head.uphFrameCounter = index++;
        memset(&sendMsg, 0, sizeof(sendMsg));
        for(int i = 0; i < 200; i++){
            for(int j = 0; j < 100; j++){
                sendMsg.UDP_PC_payload[j].pcmDistance = (j + i + index % 100) * 65536;
                sendMsg.UDP_PC_payload[j].pcmIndensity = 32768 + j + i;
                if(j % 2){
                    sendMsg.UDP_PC_payload[j].pcmHorizontal = 30 * 32000 / 720;
                    sendMsg.UDP_PC_payload[j].pcmVertical = 0;
                    sendMsg.UDP_PC_payload[j].pcmSpeed = (10.2) * 128;
                }
                else{
                    sendMsg.UDP_PC_payload[j].pcmHorizontal = 330 * 32000 / 720;
                    sendMsg.UDP_PC_payload[j].pcmVertical = 5 * 256;
                    sendMsg.UDP_PC_payload[j].pcmSpeed = (-10.2) * 128;

                }

            }
            int nnn = sendto(udpPcRecvSocketFd_, &sendMsg, sizeof(sendMsg), 0, (struct sockaddr*)&ser_addr, len);
            usleep(500);  //一秒发送一次消息
        }
#if 1
#endif
    }
 }

double fft2dBm(double x){
	double inputV = (x / pow(2, 4.5) + 22) / 7.048;
	//double res = 10 * log10(20 * pow((inputV / 4000 / sqrt(2)), 2));
	double res = 10 * log10(20 * (inputV / 4000 / sqrt(2)) * (inputV / 4000 / sqrt(2)));
	return res; 
}
using namespace std;

void FloatToChar(float fNum, unsigned char *strBuf, int nLen) 
{
  if (nLen < 4)
    return;
  int i = 0;
  unsigned char nTmp;
  char *p = (char *)&fNum;
  for (i = 0; i < 4; i++) {
    strBuf[i] = *p;
    p++;
  }
}

float CharToFloat(unsigned char *strBuf, int nLen) 
{
  if (nLen < 4)
    return 0;
  int i = 0;
  float fNum;
  unsigned char nTmp;
  char *p = (char *)&fNum;
  for (i = 0; i < 4; i++) {
    *p = strBuf[i];
    p++;
  }
  return fNum;
}

int main(int argc, char** argv) 
{ 
    ros::init(argc, argv, "talker");
    ros::NodeHandle roshandle;
    pthread_t udp_send;
    pthread_t udp_PC_send;

    const char *cali_file_path = "/home/encheng/data/data_pc_raw_index1.bin";
    int  filesize = LoadDat(cali_file_path);
    std::chrono::duration<double> elapsed;
    auto start = std::chrono::steady_clock::now();
    float data_float = -8.888;
    float data_float_out = 6.00;

    uint8_t data_u8[4];
    FloatToChar(data_float, data_u8, 4);
    std::cout << "data_float_out is " <<  CharToFloat(data_u8, 4) << std::endl;

    auto end = std::chrono::steady_clock::now();
    elapsed = end - start;
    std::cout << "time for fft2dBm: " <<  elapsed.count() * 1000 << " ms" << std::endl;    
    string test_str = "123456\t\n";
    cout << "test_str size is " << test_str.size() << endl;
    cout << test_str;
    cout << "test_str" << endl;
    int listenfd, connfd; 
    commandMsg msg;
    struct sockaddr_in servaddr; 
    char buff[4096]; int n; 
    pcSockerInit();
    if( (listenfd = socket(AF_INET, SOCK_STREAM, 0)) == -1 )
    { 
        printf("create socket error: %s(errno: %d)\n",strerror(errno),errno);
        exit(0); 
    } 
    int one = 1;
	setsockopt(listenfd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

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
            ifstop = true;
            break;
        case POINTCLOUD_DISPLAY_START:
        	//pthread_create(&udp_PC_send, NULL, udppc_msg_sender, NULL);
            udp_pc_msg_send_once();
            break;       
        case POINTCLOUD_DISPLAY_STOP:
            close(udpPcRecvSocketFd_);
            ifPCstop = true;
            break;
        default:
            break;
        }

        if(ifPCstop) break;
        //buff[n] = '\0'; 
        printf("msg.mCommandVal from client: %d\n", msg.mCommandVal[1]); 
    } 
    close(connfd); 
    close(listenfd); 
    return 0;
}
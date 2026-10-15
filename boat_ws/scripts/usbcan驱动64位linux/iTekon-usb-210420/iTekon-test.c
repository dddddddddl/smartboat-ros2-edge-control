
#include <stdio.h>
#include <pthread.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>

#include "lib/iTekon-usb.h"

int breakflag = 0;
void exit_can(int flag);
void *handle_send(void *arg)
{
    int ret = 0;
    int i = 0;
    VCI_CAN_OBJ send_msg[3];
    unsigned char test[8] = {1,2,3,4,5,6,'a','1'};
    memset (send_msg, 0 ,sizeof(VCI_CAN_OBJ) * 3);
    while (1) 
    {
	    i++;
	    if (breakflag) {
		    breakflag = 2;
		    return NULL;
	    }
	    usleep (1000*1000);
	    if (i>0 && i<50)
	    {
		    test[7]++;
		    test[6] = 'a';
		    memcpy(send_msg[0].Data, test, sizeof(test));
		    send_msg[0].ID = 1;
		    send_msg[0].DataLen = sizeof(test);
		    send_msg[0].ExternFlag = 0;
		    send_msg[0].RemoteFlag = 0;
		    send_msg[0].SendType = 0;
		    send_msg[0].TimeFlag = 1;
		    send_msg[0].TimeStamp = 0x12345678;

		    test[7]++;
		    send_msg[1].ID = 0x12;
		    memcpy(send_msg[1].Data, test, sizeof(test));
		    send_msg[1].DataLen = sizeof(test);
		    send_msg[1].ExternFlag = 0;
		    send_msg[1].RemoteFlag = 0;
		    send_msg[1].SendType = 0;
		    send_msg[1].TimeFlag = 1;
		    send_msg[1].TimeStamp = 0x12345678;

		    test[7]++;
		    send_msg[2].ID = 0x12;
		    memcpy(send_msg[2].Data, test, sizeof(test));
		    send_msg[2].DataLen = sizeof(test);
		    send_msg[2].ExternFlag = 1;
		    send_msg[2].RemoteFlag = 0;
		    send_msg[2].SendType = 0;
		    send_msg[2].TimeFlag = 1;
		    send_msg[2].TimeStamp = 0x12345678;
		    
            ret = VCI_Transmit (4, 0, 1, send_msg, 3);
            printf ("-----send :%d\n", ret);
			if (ret == 0)
			{
				break;
			}
	    }
	    else
		    i=0;
    }
	exit_can(1);
    return NULL;
}

void exit_can(int flag)
{
	int time = 0;
    if (1 != flag)
    {
		printf("---------------------------------------ctrl+c\n");
        //关闭发送线程。
        breakflag = 1;
        while(1)
        {
            if((breakflag == 2) || (breakflag == 0)) {
                break;
            }
			sleep(1);
			time++;
			if (time>5)
				break;
        }
        breakflag = 0;
    }
	
	//复位CAN0通道
    VCI_ResetCAN(4, 0, 0);	
	//复位CAN1通道
    VCI_ResetCAN(4, 0, 1);
	//关闭设备
    VCI_CloseDevice (4, 0);
	//释放USB连接
    VCI_UsbExit();
    exit(0);
}

void signal_handle()
{
	printf("---%s\n", __func__);
    struct sigaction sig_handle;
    memset(&sig_handle, 0, sizeof(sig_handle));
    sigemptyset(&sig_handle.sa_mask);
    sig_handle.sa_flags = SA_SIGINFO;
    sig_handle.sa_sigaction = exit_can;
    sigaction(SIGINT, &sig_handle, NULL);
}

#if 1
int main(void)
{
    pthread_t recv_handle, send_handle;
	VCI_BOARD_INFO board_info;
	VCI_INIT_CONFIG init_config0, init_config1;
    int ret = 0;
	
	//初始化USB
    ret = VCI_UsbInit ();
    if (ret == 0) {
        exit_can(1);
        return 0;
    }
	
	//打开设备
    ret = VCI_OpenDevice(4, 0, 0);
    if (ret == 0) {
        exit_can(1);
        return 0;
    }

	bzero(&init_config0, sizeof(VCI_INIT_CONFIG));
	bzero(&init_config1, sizeof(VCI_INIT_CONFIG));
	
	//CAN0初始化参数
	init_config0.filter_num = 1;								//CAN0通道使用过滤器组数量
	init_config0.filter_info[0].FilterEN = 1;					//默认第一组过滤器使能（至少应使能一组过滤器）
	init_config0.filter_info[0].FilterMode = 0x01;				//32位掩码模式
	init_config0.filter_info[0].FilterId = 0x00000000;			
	init_config0.filter_info[0].FilterMask = 0x00000000;		//所有Id全部接收
	init_config0.Mode = MODE_NORMAL;							//正常工作模式
	init_config0.BotRate =BOT_1000K;							//1M波特率

	//CAN1初始化参数
	init_config1.filter_num = 1;								//CAN1通道使用过滤器组数量
	init_config1.filter_info[0].FilterEN = 1;					//默认第一组过滤器使能（至少应使能一组过滤器）
	init_config1.filter_info[0].FilterMode = 0x01;				//32位掩码模式	
	init_config1.filter_info[0].FilterId = 0x00000000;
	init_config1.filter_info[0].FilterMask = 0x00000000;		//所有Id全部接收
	init_config1.Mode = MODE_NORMAL;							//正常工作模式
	init_config1.BotRate =BOT_1000K;							//1M波特率
	
	//初始化CAN0通道
    ret = VCI_InitCan(4, 0, 0, &init_config0);					
    if (ret == 0) {
        exit_can(1);
        return 0;
    }

	//初始化CAN1通道
    ret = VCI_InitCan(4, 0, 1, &init_config1);
    if (ret == 0) {
        exit_can(1);
        return 0;
    }
	
	//读设备信息
	bzero(&board_info, sizeof(board_info));
    ret = VCI_ReadBoardInfo(4, 0, &board_info);
    if (ret == 0) {
        exit_can(1);
        return 0;
    }

	//启动CAN0通道
    ret = VCI_StartCAN (4, 0, 0);
    if (ret == 0) {
        exit_can(1);
        return 0;
    }

	//启动CAN1通道
    ret = VCI_StartCAN (4, 0, 1);
    if (ret == 0) {
        exit_can(1);
    }
    signal_handle ();

//    ret = pthread_create(&send_handle, NULL, handle_send, NULL);

    VCI_CAN_OBJ recv_msg[10];
    memset(recv_msg, 0, sizeof(VCI_CAN_OBJ)*10);
	while (1) {
        int recvlen = VCI_Receive(4, 0, 0, recv_msg, 10, 0);
        printf("---recvnum:%d\n", recvlen);
		int i,j;
		for (i=0; i<recvlen; i++) {
			printf("ID: %08x DataLen:%02x ExternFlag:%02x RemoteFlag: %02x data:", recv_msg[i].ID, recv_msg[i].DataLen, recv_msg[i].ExternFlag, recv_msg[i].RemoteFlag);
			for (j=0; j< recv_msg[i].DataLen; j++) {
				printf("%02x ", recv_msg[i].Data[j]);
			}
			printf("\n");
		}
		sleep(1);
	}
	goto selfstop;

    VCI_ResetCAN (4, 0, 0);
selfstop:
    VCI_CloseDevice (4, 0);
	return 0;
}

#endif

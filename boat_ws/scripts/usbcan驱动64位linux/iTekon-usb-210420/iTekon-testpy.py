#coding:utf-8
from ctypes import *
import thread
import time
import signal
import sys

#so = ctypes.CDLL("./libtest.so")

#print so.sum(2, 3)

#typedef struct _VCI_BOARD_INFO {
#        USHORT  hw_Version;
#        USHORT  fw_Version;
#        USHORT  dr_Version;
#        USHORT  in_Version;
#        USHORT  irq_Num;
#        BYTE    can_Num;
#        CHAR    str_Serial_Num[20];
#        CHAR    str_hw_Type[40];
#        USHORT  Reserved[4];
#        } VCI_BOARD_INFO, *PVCI_BOARD_INFO;
#
#typedef struct _INIT_CONFIG {
#        DWORD   AccCode;
#        DWORD   AccMask;
#        DWORD   Reserved;
#        UCHAR   Filter;
#        UCHAR   Timing0;
#        UCHAR   Timing1;
#        UCHAR   Mode;
#        UINT   BotRate;
#        } VCI_INIT_CONFIG, *PVCI_INIT_CONFIG;

#typedef struct _VCI_CAN_OBJ {
#    UINT    ID;
#    UINT    TimeStamp;
#    BYTE    TimeFlag;
#    BYTE    SendType;
#    BYTE    RemoteFlag;
#    BYTE    ExternFlag;
#    BYTE    DataLen;
#    BYTE    Data[8];
#    BYTE    Reserved[3];
#} VCI_CAN_OBJ, *PVCI_CAN_OBJ;

class VCI_CAN_OBJ(Structure):
    _fields_ = [
            ('ID', c_uint),
            ('TimeStamp', c_uint),
            ('TimeFlag', c_byte),
            ('SendType', c_byte),
            ('RemoteFlag', c_byte),
            ('ExternFlag', c_byte),
            ('DataLen', c_byte),
            ('Data', c_byte * 8),
            ('Reserved', c_byte * 3)
            ]

class VCI_INIT_CONFIG(Structure):
    _fields_ = [
            ('AccCode', c_ulong),
            ('AccMask', c_ulong),
            ('Reserved', c_ulong),
            ('Filter', c_ubyte),
            ('Timing0', c_ubyte),
            ('Timing1', c_ubyte),
            ('Mode', c_ubyte),
            ("BotRate", c_uint)
            ]

class VCI_BOARD_INFO(Structure):
    _fields_ = [
            ('hw_Version', c_ushort),
            ('fw_Version', c_ushort),
            ('dr_Version', c_ushort),
            ('in_Version', c_ushort),
            ('irq_Num', c_ushort),
            ('can_Num', c_ubyte),
            ('str_Serial_Num', c_byte * 20),
            ('str_hw_Type', c_byte * 40),
            ('Reserved', c_ushort * 4)
            ]

breakflag = 0

def exit_can(flag):
    timet=0
    if flag != 1:
        global breakflag
        breakflag = 1
        while True:
            if breakflag == 2 or breakflag == 0:
                break
            time.sleep(1)
            timet = timet + 1
            if timet > 5:
                break

        breakflag = 0

    so.VCI_ResetCAN(254, 0, 0); 
    so.VCI_ResetCAN(254, 0, 1); 
    so.VCI_CloseDevice (254, 0); 
    so.VCI_UsbExit();
    sys.exit()


def handler(signum, frame):
    exit_can (0)

def send_data_th(i):
    i=0
    send_msg = VCI_CAN_OBJ(ID = 1, DataLen = 8, ExternFlag = 0, RemoteFlag = 0, SendType = 0, TimeFlag = 1, TimeStamp = 0x12345678)
    byte8 = c_byte * 8
    tmp1=ord('1')

    while True:
        i=i+1
        global breakflag
        if breakflag > 0:
            breakflag = 2 
            return 

        time.sleep(0.1)
        if i > 0 and i < 50:
            tmp1 = tmp1 +1
            send_msg.Data[0] = 1
            send_msg.Data[1] = 2
            send_msg.Data[2] = 3
            send_msg.Data[3] = 4
            send_msg.Data[4] = 5
            send_msg.Data[5] = 6
            send_msg.Data[6] = 7
            send_msg.Data[7] = tmp1
            ret = so.VCI_Transmit (254, 0, 0, pointer(send_msg), 1);
            if ret == 0:
                break
        else:
            i=0

    exit_can(1)
    return 




if __name__ == "__main__":
    so = CDLL("./libiTekon-usb.so")
    print ('usbinit result:', so.VCI_UsbInit())
    print ("opendevice result:", so.VCI_OpenDevice(4, 0, 0))

    init_config = VCI_INIT_CONFIG(AccCode = 0, AccMask = 0, Filter = 0, Mode=0, BotRate=0x00120005)

    print ("initcan result:", so.VCI_InitCan(254, 0, 0, pointer(init_config)))
    print ("initcan result:", so.VCI_InitCan(254, 0, 1, pointer(init_config)))

    print ("startcan result:", so.VCI_StartCAN (254, 0, 0))
    print ("startcan result:", so.VCI_StartCAN (254, 0, 1))

    signal.signal(signal.SIGINT, handler)

    # 发送线程
    thread.start_new_thread(send_data_th, (1,))
    recv_msg = VCI_CAN_OBJ()
    while True:
        recvnum = so.VCI_GetReceiveNum(254, 0, 0)
        print ("recv", recvnum)
        so.VCI_Receive(254, 0, 1, pointer(recv_msg), 1, 0)
        print (recv_msg.ID, recv_msg.Data[7])

        time.sleep(1)

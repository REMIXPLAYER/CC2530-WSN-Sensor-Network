#include <iocc2530.h>
#include "sys_init.h"
#include "uart.h"
#include "info.h"
#include "hal_mcu.h"
#include "hal_assert.h"
#include "hal_board.h"
#include "hal_rf.h"
#include "basic_rf.h"
#include <stdio.h>
#include "string.h"

#define RF_CHANNEL            24                                // 2.4 GHz RF channel 
#define PAN_ID                0x1410                            // 班级PAN ID
#define MY_ADDR               0x4102                            // 本机短地址
#define DHT11_ADDR            0x4103                            // DHT11节点短地址
#define PHOTO_ADDR            0x4101                            // 光敏节点短地址

static basicRfCfg_t basicRfConfig;

/* 获取随机退避时间 */
unsigned int getRandomDelay(void) {
    static unsigned int seed = 0x4102;
    seed = (seed * 1103515245 + 12345) & 0x7FFF;
    return seed;
}

/* 读取CC2530的64位IEEE MAC地址，地址 0x780C~0x7813 */
void readMacAddress(uint8* macAddr) {
    uint8 i;
    uint8 * flashPtr = (uint8 *)(P_INFOPAGE + 0x0C);
    for (i = 0; i < 8; i++) {
        macAddr[i] = flashPtr[i];
    }
}

/* CSMA-CA：指数退避（≤150ms后封顶12次重试，最大间隔≤151ms＜200ms RX窗） */
uint8 csmaCaSendPacket(uint16 destAddr, uint8* pPayload, uint8 length) {
    uint8 retries = 0;
    while (retries < 12) {
        uint32 backoff;
        if (retries < 5) {
            backoff = (getRandomDelay() % (1 << retries)) * 10;  // 0~10~20~40~80ms
        } else {
            backoff = (getRandomDelay() % 17) * 10;               // 0~160ms 封顶
        }
        halMcuWaitMs(backoff);
        if (basicRfSendPacket(destAddr, pPayload, length) == SUCCESS) {
            return SUCCESS;
        }
        retries++;
    }
    return FAILED;
}

/* 发送无线控制指令到指定节点（通用） */
void sendWirelessCmd(uint16 targetAddr, char* cmd) {
    uint8 result;
    basicRfReceiveOff();
    result = csmaCaSendPacket(targetAddr, (uint8*)cmd, strlen(cmd) + 1);
    basicRfReceiveOn();
    if (result == SUCCESS) {
        printf("{cmd=%s->0x%04X}\r\n", cmd, targetAddr);
    } else {
        printf("{cmd=FAIL->0x%04X}\r\n", targetAddr);
    }
}

/* 发送LED控制指令 */
void sendLedCmd(uint16 targetAddr, uint8 on) {
    char cmd[10];
    sprintf(cmd, "{LED=%d}", on);
    sendWirelessCmd(targetAddr, cmd);
}

/* 发送数据采集控制指令 */
void sendDataCmd(uint16 targetAddr, uint8 on) {
    char cmd[10];
    sprintf(cmd, "{SEND=%d}", on);
    sendWirelessCmd(targetAddr, cmd);
}

// 无线控制命令标志（ISR只设标志，主循环处理）
volatile uint8  cmdPending = 0;
volatile uint16 cmdTarget  = 0;
volatile uint8  cmdOn      = 0;

// 串口接收中断函数（只设标志位，不阻塞）
#pragma vector = URX0_VECTOR
__interrupt void UART0_ISR(void)
{
  uchar rx_data;

  URX0IF = 0;
  rx_data = U0DBUF;

  switch (rx_data) {
    case '@':  D7 = 0;  break;                     // 本地D7亮
    case '!':  D7 = 1;  break;                     // 本地D7灭
    case '1':  cmdTarget = DHT11_ADDR; cmdOn = 1; cmdPending = 1;  break;
    case '2':  cmdTarget = DHT11_ADDR; cmdOn = 0; cmdPending = 1;  break;
    case '3':  cmdTarget = PHOTO_ADDR; cmdOn = 1; cmdPending = 1;  break;
    case '4':  cmdTarget = PHOTO_ADDR; cmdOn = 0; cmdPending = 1;  break;
    case '5':  cmdTarget = DHT11_ADDR; cmdOn = 1; cmdPending = 2;  break;  // 2=数据采集控制
    case '6':  cmdTarget = DHT11_ADDR; cmdOn = 0; cmdPending = 2;  break;
    case '7':  cmdTarget = PHOTO_ADDR; cmdOn = 1; cmdPending = 2;  break;
    case '8':  cmdTarget = PHOTO_ADDR; cmdOn = 0; cmdPending = 2;  break;
    default:  break;
  }
}

/* 射频模块接收数据函数 — 双标志位触发本机状态打印 */
void rfRecvData(void)
{
    char pRxData[128];
    uint8 macAddr[8];
    int rlen;
    uint8 dht11Flag = 0;    // DHT11节点数据接收标志
    uint8 photoFlag = 0;    // 光敏节点数据接收标志
    
    // 读取本机MAC地址
    readMacAddress(macAddr);
    
    basicRfReceiveOn();
    printf("{data=recv node start up...}");
    
    while (TRUE) {
        // 处理无线控制命令（cmdPending: 1=LED控制, 2=数据采集控制）
        if (cmdPending) {
            uint8 type = cmdPending;
            uint16 target = cmdTarget;
            uint8 on = cmdOn;
            cmdPending = 0;
            if (type == 2) {
                sendDataCmd(target, on);
            } else {
                sendLedCmd(target, on);
            }
        }
        
        if (basicRfPacketIsReady()) {
            rlen = basicRfReceive((uint8*)pRxData, sizeof pRxData, NULL);
            if (rlen > 0) {
                pRxData[rlen] = 0;
                
                // 识别数据来源，置位对应标志
                if (strstr(pRxData, "Humidity") != NULL) {
                    dht11Flag = 1;
                } else if (strstr(pRxData, "Light") != NULL) {
                    photoFlag = 1;
                }
                
                // 直接打印收到的传感器数据
                printf("%s\r\n", pRxData);
                
                // 两个节点数据都收到后，立即打印本机状态
                if (dht11Flag && photoFlag) {
                    printf("{D=%d, MAC=%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X}\r\n",
                           (D7 == 0) ? 1 : 0,
                           macAddr[7], macAddr[6], macAddr[5], macAddr[4],
                           macAddr[3], macAddr[2], macAddr[1], macAddr[0]);
                    // 重置标志，进入下一轮
                    dht11Flag = 0;
                    photoFlag = 0;
                }
            }
        }
        
        halMcuWaitMs(10);
    }
}

void main(void)
{
    xtal_init();                                                //初始化系统时钟
    led_init();                                                 //初始化LED
    uart0_init(0x00, 0x00);                                     //初始化串口波特率为38400
    lcd_dis();                                                  //在LCD上显示相关信息
    
    if (FAILED == halRfInit()) {                                // halRfInit()为射频初始化函数
        HAL_ASSERT(FALSE);
    }
    // Config basicRF
    basicRfConfig.panId = PAN_ID;                               //panId
    basicRfConfig.channel = RF_CHANNEL;                         //通信信道
    basicRfConfig.ackRequest = TRUE;                            //应答请求
#ifdef SECURITY_CCM
    basicRfConfig.securityKey = key;                            //安全密钥
#endif

    
    // Initialize BasicRF
    basicRfConfig.myAddr = MY_ADDR;                             //设置本机地址
    
    if(basicRfInit(&basicRfConfig)==FAILED) {
      HAL_ASSERT(FALSE);
    }
    
    // 调用rfRecvData函数处理无线接收（内部完成接收器初始化和MAC读取）
    rfRecvData();
}

/*系统时钟初始化
-------------------------------------------------------*/
void xtal_init(void)
{
  SLEEPCMD &= ~0x04;              //上电
  while(!(CLKCONSTA & 0x40));     //等待晶振振荡器稳定
  CLKCONCMD &= ~0x47;             //选择32MHz晶振
  SLEEPCMD |= 0x04;
}

/*led初始化
-------------------------------------------------------*/
void led_init(void)
{
  P1SEL &= ~0x03;          //P1.0 P1.1为普通 I/O 口
  P1DIR |= 0x03;           //输出
  
  D7 = 1;                //关LED
  D6 = 1;
}

/*uart0初始化
-------------------------------------------------------*/
void uart0_init(unsigned char StopBits,unsigned char Parity)
{
  P0SEL |=  0x0C;                 //初始化UART0端口
  PERCFG&= ~0x01;                 //选择UART0为备用位置一
  P2DIR &= ~0xC0;                 //P0优先权为UART0
  U0CSR = 0xC0;                   //设置为UART模式,接收器使能开启
   
  U0GCR = 0x0A;
  U0BAUD = 0x3B;                  //设置串口波特率为38400
  
  U0UCR |= StopBits|Parity;       //设置停止位和奇偶校验
  
  URX0IF = 0;                     //清UART0接收中断标志位
  URX0IE = 1;                     //开UART0接收中断
  EA = 1;                         //开全局中断
}

/*串口发送字节函数
-------------------------------------------------------*/
void Uart_Send_char(char ch)
{
  U0DBUF = ch;
  while(UTX0IF == 0);
  UTX0IF = 0;
}

/*串口发送字符串函数
-------------------------------------------------------*/
void Uart_Send_String(char *Data)
{
  while (*Data != '\0')
  {
    Uart_Send_char(*Data++);
  }
}

/*串口接收字节函数
-------------------------------------------------------*/
int Uart_Recv_char(void)
{
  int ch;
    
  while (URX0IF == 0);
  ch = U0DBUF;
  URX0IF = 0;
  return ch;
}
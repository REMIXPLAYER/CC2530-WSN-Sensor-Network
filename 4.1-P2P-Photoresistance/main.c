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
#define PAN_ID                0x1410                            // 班级
#define SEND_ADDR             0x4101                            // 学号
#define RECV_ADDR             0x4102                         // 学号

#define NODE_TYPE             1                                //0:接收节点，1：发送节点

static basicRfCfg_t basicRfConfig;

int getADC(void);

/* 获取ADC值 */
int getADC(void) 
{
  unsigned int  value;
  
  P0SEL |= 0x02;
  ADCCON3  = (0xB1);                    //选择AVDD5为参考电压，12位分辨率，P0_1  ADC
  
  ADCCON1 |= 0x30;                      //选择ADC启动模式为手动
  ADCCON1 |= 0x40;                      //启动AD转换              
  
  while(!(ADCCON1 & 0x80));             //等待ADC转换结束
  
  value =  ADCL >> 2;
  value |= (ADCH << 6);                 //取得最终转换结果赋给value
  
  return ((value) >> 2);        
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

/*延时函数
-------------------------------------------------------*/
void halWait(unsigned char wait)
{
  unsigned long largeWait;

  if(wait == 0)
  {return;}
  largeWait = ((unsigned short) (wait << 7));
  largeWait += 114*wait;

  largeWait = (largeWait >> CLKSPD);
  while(largeWait--);

  return;
}

/* 光敏电阻测试函数（LCD显示屏用，保持 {A0=} 格式兼容） */
void Photoresistance_Test(void)
{
  int AdcValue;  
  AdcValue = getADC();
 
  //在串口/LCD上发送光照值（{A0=} 格式兼容LCD显示屏解析）
  char  dbuf[20] = {0};
  sprintf(dbuf,"{A0=%d}",AdcValue);
  Uart_Send_String(dbuf);
  
}

/* 获取随机退避时间 */
unsigned int getRandomDelay(void) {
    // 简单的伪随机数生成，基于系统时钟
    // 使用不同的初始种子，确保每个节点的退避序列不同
    static unsigned int seed = 0x4101; // 使用发送地址作为初始种子，确保唯一性
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

/* CSMA-CA 精简版：指数退避 + 硬件CCA（basicRfSendPacket内置） */
uint8 csmaCaSendPacket(uint16 destAddr, uint8* pPayload, uint8 length) {
    uint8 retries = 0;
    while (retries < 5) {
        uint32 backoff = (getRandomDelay() % (1 << retries)) * 10;
        halMcuWaitMs(backoff);
        if (basicRfSendPacket(destAddr, pPayload, length) == SUCCESS) {
            return SUCCESS;
        }
        retries++;
    }
    return FAILED;
}

/* 射频模块发送数据函数（含MAC地址 + RX监听窗口 + 数据采集开关） */
void rfSendData(void){
    char pTxData[70];
    uint8 pRxData[16];
    uint8 macAddr[8];
    uint8 ret;
    int rlen;
    int AdcValue;
    uint8 sendEnable = 1;    // 数据采集开关: 1=发送, 0=暂停
    
    // 启动时读取本机MAC地址（只读一次）
    readMacAddress(macAddr);
    
    basicRfReceiveOff();
    
    while (TRUE) {
        // === 阶段1：采集并发送光照数据（sendEnable=0时跳过） ===
        if (sendEnable) {
            AdcValue = getADC();
            Photoresistance_Test();  // LCD/串口打印 {A0=%d}
            sprintf(pTxData, "{Light=%d, D=%d, MAC=%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X}",
                    AdcValue,
                    (D7 == 0) ? 1 : 0,
                    macAddr[7], macAddr[6], macAddr[5], macAddr[4],
                    macAddr[3], macAddr[2], macAddr[1], macAddr[0]);
            
            ret = csmaCaSendPacket(RECV_ADDR, (uint8*)pTxData, strlen(pTxData) + 1);
            if (ret == SUCCESS) {
                D6 = 0;       // D6 闪烁表示发送成功
                halMcuWaitMs(100);
                D6 = 1;
            }
        }
        
        // === 阶段2：RX监听窗口（200ms，接收控制指令） ===
        basicRfReceiveOn();
        halMcuWaitMs(200);
        
        if (basicRfPacketIsReady()) {
            rlen = basicRfReceive(pRxData, sizeof pRxData, NULL);
            if (rlen > 0) {
                pRxData[rlen] = 0;
                if (strstr((char*)pRxData, "LED=1")) {
                    D7 = 0;
                    printf("{cmd=LED ON}\r\n");
                } else if (strstr((char*)pRxData, "LED=0")) {
                    D7 = 1;
                    printf("{cmd=LED OFF}\r\n");
                } else if (strstr((char*)pRxData, "SEND=1")) {
                    sendEnable = 1;
                    printf("{cmd=SEND ON}\r\n");
                } else if (strstr((char*)pRxData, "SEND=0")) {
                    sendEnable = 0;
                    printf("{cmd=SEND OFF}\r\n");
                }
            }
        }
        basicRfReceiveOff();
        
        // === 阶段3：等待下个周期（总周期约1秒） ===
        halMcuWaitMs(700);
    }
}

/* 射频模块接收数据函数 */
void rfRecvData(void)
{
    uint8 pRxData[128];
    int rlen;
    static unsigned int rec_counter = 0;                        //接收次数计数器
    printf("{data=recv node start up...}");
    basicRfReceiveOn();                                         //打开射频接收器
    // Main loop
    while (TRUE) {
        while(!basicRfPacketIsReady());
        rlen = basicRfReceive(pRxData, sizeof pRxData, NULL);
        if(rlen > 0) {
          pRxData[rlen] = 0;
          printf("{data=");                                     //接收到
          printf((char *)pRxData);                              //数据后
          printf(" %d",++rec_counter);                          //接收数据的次数
          printf("}");                                          //在LCD上显示
            
        }
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
#if NODE_TYPE
    basicRfConfig.myAddr = SEND_ADDR;                          //发送地址
#else
    basicRfConfig.myAddr = RECV_ADDR;                          //接收地址
#endif
    
    if(basicRfInit(&basicRfConfig)==FAILED) {
      HAL_ASSERT(FALSE);
    }
#if NODE_TYPE
  rfSendData();                                                //发送函数
#else
  rfRecvData();                                                //接收函数
#endif
}
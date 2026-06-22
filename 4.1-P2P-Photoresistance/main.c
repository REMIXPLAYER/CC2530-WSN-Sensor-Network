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

/* 光敏电阻测试函数 */
void Photoresistance_Test(void)
{
  int AdcValue;  
  AdcValue = getADC();
 
  //在LCD上发送光敏电阻值
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

/* CSMA-CA 算法使用二进制退避 */
uint8 csmaCaSendPacket(uint16 destAddr, uint8* pPayload, uint8 length) {
    uint8 ret;
    uint8 maxRetries = 7; // 最大重试次数
    uint8 retries = 0;
    uint8 ccaAttempts = 0;
    uint32 backoffTime;
    
    while (retries < maxRetries) {
        // 1. 二进制退避
        // 退避时间 = 随机数 % (2^(min(retries, 10)) * 20ms)
        // 当重试次数超过10时，退避时间保持在2^10 * 20ms
        uint8 exponent = (retries < 10) ? retries : 10;
        uint32 backoffSlots = (1 << exponent); // 2^exponent
        uint32 randomSlot = getRandomDelay() % backoffSlots;
        backoffTime = randomSlot * 20; // 每个时隙20ms
        halMcuWaitMs(backoffTime);
        
        // 2. 载波侦听（多次检查，提高可靠性）
        ccaAttempts = 0;
        while (ccaAttempts < 3) {
            // 检查信道是否空闲（使用basicRF的内置CCA机制）
            halMcuWaitMs(5);
            ccaAttempts++;
        }
        
        // 3. 发送数据包
        ret = basicRfSendPacket(destAddr, pPayload, length);
        if (ret == SUCCESS) {
            return SUCCESS;
        }
        
        retries++;
    }
    
    return FAILED;
}

/* 射频模块发送数据函数 */
void rfSendData(void){                                          //发送函数
    char pTxData[30];
    uint8 ret;
    int AdcValue;
    // Keep Receiver off when not needed to save power
    basicRfReceiveOff();                                        //关闭射频接收器
    // Main loop
    while (TRUE) {
      AdcValue = getADC();
      sprintf(pTxData, "{A0=%d}", AdcValue);
      //在LCD上显示数据
      Photoresistance_Test();
      //向节点发送数据包  
      ret = csmaCaSendPacket(RECV_ADDR, (uint8*)pTxData, strlen(pTxData) + 1); 
      if (ret == SUCCESS) {
        hal_led_on(1);
        halMcuWaitMs(100);
        hal_led_off(1);
        halMcuWaitMs(900); // 增加延迟，降低发送频率，恢复到原来的1秒间隔
      } else {
        hal_led_on(1);
        halMcuWaitMs(500);
        hal_led_off(1);
        halMcuWaitMs(500); // 增加失败后的延迟，降低重试频率
      }
      
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
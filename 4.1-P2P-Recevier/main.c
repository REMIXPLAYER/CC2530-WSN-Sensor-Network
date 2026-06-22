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
#define PAN_ID                0x1410                            // 与4.5-WirelessControl保持一致
#define RECV_ADDR             0x4102                            // 与4.5-WirelessControl保持一致  

static basicRfCfg_t basicRfConfig;

// 数据融合全局变量
static int g_temp = 0;
static int g_humidity = 0;
static int g_light = 0;

/* 从字符串中提取 key= 后面的数值 */
static int get_val(const char* str, const char* key) {
    const char* p;
    int val = 0;
    p = strstr(str, key);
    if (p == NULL) return -1;
    p += strlen(key);
    while (*p >= '0' && *p <= '9') {
        val = val * 10 + (*p - '0');
        p++;
    }
    return val;
}

// 串口接收中断函数

#pragma vector = URX0_VECTOR
__interrupt void UART0_ISR(void)
{
  uchar rx_data;
  
  URX0IF = 0;  // 清除接收中断标志
  rx_data = U0DBUF;  // 读取接收到的数据

  // 指令接收逻辑
  switch (rx_data) {
    case '@':  // 单字符命令: @ 控制LED点亮
      D7 = 0;  // 低电平点亮D7
      
      break;
    case '!':  // 单字符命令: # 控制LED熄灭
      D7 = 1;  // 高电平熄灭D7
      
      break;
    default:  // 无效命令
     
      break;
  }
}

/* 射频模块接收数据函数 — 数据融合 */
void rfRecvData(void)
{
    char pRxData[128];
    int rlen;

    // Main loop
    while (TRUE) {
        // 非阻塞式检查是否有无线数据包
        if (basicRfPacketIsReady()) {
            rlen = basicRfReceive(pRxData, sizeof pRxData, NULL);
            if(rlen > 0) {
              pRxData[rlen] = 0;
              
              // 判断数据类型并更新对应的全局变量
              if (strstr(pRxData, "Humidity") != NULL) {
                  // 温湿度数据: {Humidity=xx, Temp=xx}
                  g_humidity = get_val(pRxData, "Humidity=");
                  g_temp = get_val(pRxData, "Temp=");
              } else if (strstr(pRxData, "Light") != NULL) {
                  // 光照数据: {Light=xx}
                  g_light = get_val(pRxData, "Light=");
              }
              
              // 一次打印融合后的所有数据 + LED状态
              printf("{Temp=%d, Humidity=%d, Light=%d, D7=%d}\r\n",
                     g_temp, g_humidity, g_light, (D7 == 0) ? 1 : 0);
            }
        }
    }
}

void main(void)
{
    xtal_init();                                                //初始化系统时钟
    led_init();                                                 //初始化LED
    uart0_init(0x00, 0x00);                                     //初始化串口波特率为38400
    lcd_dis();                                                  //在LCD上显示相关信息
// 确保中断已开启
    EA = 1;  // 全局中断使能
    URX0IE = 1;  // UART0接收中断使能
    
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
    basicRfConfig.myAddr = RECV_ADDR;                          //接收地址
    
    if(basicRfInit(&basicRfConfig)==FAILED) {
      HAL_ASSERT(FALSE);
    }
    
    basicRfReceiveOn();                                         //打开射频接收器
    printf("{data=recv node start up...}");
    
    // 调用rfRecvData函数处理无线接收
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
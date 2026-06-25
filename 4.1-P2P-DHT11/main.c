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

// DHT11 总线引脚定义
#define       PIN_OUT       (P0DIR |= 0x20)
#define       PIN_IN        (P0DIR &= ~0x20)
#define       PIN_CLR       (P0_5 = 0)
#define       PIN_SET       (P0_5 = 1)
#define       PIN_R         (P0_5)

#define       COM_IN          PIN_IN  
#define       COM_OUT         PIN_OUT
#define       COM_CLR         PIN_CLR
#define       COM_SET         PIN_SET
#define       COM_R           PIN_R

// DHT11 温湿度全局变量
unsigned char sTemp;
unsigned char sHumidity;

// DHT11 驱动函数声明
static char dht11_read_bit(void);
static unsigned char dht11_read_byte(void);
void dht11_io_init(void);
unsigned char dht11_temp(void);
unsigned char dht11_humidity(void);
void dht11_update(void);

#define RF_CHANNEL            24                                // 2.4 GHz RF channel
#define PAN_ID                0x1410                            // 班级
#define SEND_ADDR             0x4103                            // 学号
#define RECV_ADDR             0x4102                         // 学号

#define NODE_TYPE             1                                //0:接收节点，1：发送节点

static basicRfCfg_t basicRfConfig;

/* 获取随机退避时间 */
unsigned int getRandomDelay(void) {
    // 简单的伪随机数生成，基于系统时钟
    // 使用不同的初始种子，确保每个节点的退避序列不同
    static unsigned int seed = 0x4103; // 使用发送地址作为初始种子，确保唯一性
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
        // 指数退避：10ms × 2^retries，退避窗口逐步扩大
        uint32 backoff = (getRandomDelay() % (1 << retries)) * 10;
        halMcuWaitMs(backoff);
        // basicRfSendPacket 内部已含硬件CCA + ACK确认
        if (basicRfSendPacket(destAddr, pPayload, length) == SUCCESS) {
            return SUCCESS;
        }
        retries++;
    }
    return FAILED;
}

/* 射频模块发送数据函数（含MAC地址 + RX监听窗口 + 数据采集开关） */
void rfSendData(void){
    char pTxData[80];
    uint8 pRxData[16];
    uint8 macAddr[8];
    uint8 ret;
    int rlen;
    unsigned char sTemp, sHumidity;
    uint8 sendEnable = 1;    // 数据采集开关: 1=发送, 0=暂停
    
    // 启动时读取本机MAC地址（只读一次）
    readMacAddress(macAddr);
    
    basicRfReceiveOff();
    
    while (TRUE) {
        // === 阶段1：采集并发送传感器数据（sendEnable=0时跳过） ===
        if (sendEnable) {
            dht11_update();  // LCD协议打印 {A0=湿度,A1=温度}
            sTemp = dht11_temp();
            sHumidity = dht11_humidity();
            sprintf(pTxData, "{Humidity=%u, Temp=%u, D=%d, MAC=%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X}",
                    sHumidity, sTemp,
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
    dht11_io_init();                                            //初始化DHT11
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

// DHT11 温湿度函数实现
static char dht11_read_bit(void)
{
  int i = 0;
  
  while (!COM_R);
  for (i=0; i<200; i++) {
    if (COM_R == 0) break;
  }
  if (i<30)return 0;  //30us
  return 1;
}

static unsigned char dht11_read_byte(void)
{
  unsigned char v = 0, b;
  int i;
  for (i=7; i>=0; i--) {
    b = dht11_read_bit();
    v |= b<<i;
  }
  return v; 
}

void dht11_io_init(void)
{
  P0SEL  &= ~0x20;          //P1为普通 I/O 口
  COM_OUT;
  COM_SET;  
}

unsigned char dht11_temp(void)
{
  return sTemp;
}

unsigned char dht11_humidity(void)
{
  return sHumidity;
}

void dht11_update(void)
{
  int flag = 1;
  unsigned char dat1, dat2, dat3, dat4, dat5, ck;
  
  //拉低延时18ms 
  COM_CLR;
  halMcuWaitMs(18);
  COM_SET;
  
  flag = 0;
  while (COM_R && ++flag);
  if (flag == 0) return;
  
  //上拉延时20us 
  //等待从机应答信号  
  //判断从机是否有低电平应答信号 如不应答则跳出检测超时错误	    
  flag = 0;
  while (!COM_R && ++flag);
  if (flag == 0) return;
  flag = 0;
  while (COM_R && ++flag);
  if (flag == 0) return;
  
  
  dat1 = dht11_read_byte();
  
  dat2 = dht11_read_byte();
  
  dat3 = dht11_read_byte();
   
  dat4 = dht11_read_byte();  
  
  dat5 = dht11_read_byte();            
  
  ck = dat1 + dat2 + dat3 + dat4;
  
  if (ck == dat5) {
    sTemp = dat3;
    sHumidity = dat1;        
  }
 
  //在LCD上发送温湿度（A0=湿度, A1=温度，SPI-LCD协议格式）
  char  dbuf[20] = {0};
  sprintf(dbuf,"{A0=%u,A1=%u}",dat1,dat3); //A0湿度，A1温度
  Uart_Send_String(dbuf);
  
}
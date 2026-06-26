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

#define NODE_TYPE             1                                //0:接收节点，1：发送节点

static basicRfCfg_t basicRfConfig;

// === v3.0 动态地址 ===
uint16 myRecvAddr = 0;       // 从信标获取的接收节点地址
uint8  registered = 0;       // 注册状态
char   myMacStr[24];         // 本机 MAC 字符串
uint8  lastBeaconCh = RF_CHANNEL;  // P1-8：缓存上次信标信道，加速重试

// RF 接收缓冲（全局，避免 64 字节数组占用 xdata 栈导致栈溢出）
uint8 gRxData[64];

/* 随机种子：从本机 MAC 派生，确保每个节点退避序列不同（P0-1 修复） */
static uint16 randSeed = 0;

/* 用 MAC 地址初始化随机种子（在 readMacAddress 后调用一次） */
void rand_init(uint8* macAddr) {
    randSeed = ((uint16)macAddr[0] << 8) | macAddr[1];
    if (randSeed == 0) randSeed = 0x4103;   // 避免 0 种子导致退避序列退化
}

/* 获取随机退避时间 */
unsigned int getRandomDelay(void) {
    randSeed = (randSeed * 1103515245 + 12345) & 0x7FFF;
    return randSeed;
}

/* 读取CC2530的64位IEEE MAC地址，地址 0x780C~0x7813 */
void readMacAddress(uint8* macAddr) {
    uint8 i;
    uint8 * flashPtr = (uint8 *)(P_INFOPAGE + 0x0C);
    for (i = 0; i < 8; i++) {
        macAddr[i] = flashPtr[i];
    }
}

/* 前向声明 */
uint8 csmaCaSendPacket(uint16 destAddr, uint8* pPayload, uint8 length);

/* 信道扫描 + 信标发现 + 注册（分阶段扫描：默认信道长等待 + 其他信道快速容错） */
void doRegister(void) {
    uint8 ch;
    uint16 tried;
    char rxBuf[32];
    int rlen;
    // 阶段1：优先在默认信道（RF_CHANNEL=24）分片轮询 3.5s，覆盖近 2 个信标周期（~2s）
    // 修复1：原 50ms 窗口 vs 2s 信标周期，命中率仅 2.5%，注册需 ~30s
    // 修复2：收到信标后延时 10ms 再发 REG，避开接收节点发信标后的 RF 状态切换过渡期
    // 修复3：REG 失败不 return，继续循环等下一个信标重试，避免每次重新等 3.5s
    halRfSetChannel(RF_CHANNEL);
    basicRfReceiveOn();    // 用 basicRfReceiveOn 设 txState.receiveOn=TRUE，避免 basicRfSendPacket 发完关 RX 导致 ISFLUSHRX 丢包
    for (tried = 0; tried < 350; tried++) {     // 350 × 10ms = 3.5s
        halMcuWaitMs(10);
        if (basicRfPacketIsReady()) {
            rlen = basicRfReceive((uint8*)rxBuf, sizeof(rxBuf), NULL);
            if (rlen > 0 && rlen < sizeof(rxBuf)) {
                rxBuf[rlen] = 0;
                if (strstr(rxBuf, "TYPE=RECV")) {
                    myRecvAddr = basicRfReceiveAddress();
                    lastBeaconCh = RF_CHANNEL;
                    halRfSetChannel(RF_CHANNEL);
                    halMcuWaitMs(10);          // 避开接收节点 RF 状态切换过渡期
                    char reg[50];
                    uint8 regResult;
                    sprintf(reg, "{REG=MAC:%s}", myMacStr);
                    regResult = csmaCaSendPacket(myRecvAddr, (uint8*)reg, strlen(reg) + 1);
                    if (regResult == SUCCESS) {
                        registered = 1;
                        printf("{reg=OK, MAC=%s}\r\n", myMacStr);
                        basicRfReceiveOff();
                        return;
                    }
                    printf("{reg=RETRY}\r\n");
                    // RX 保持开（txState.receiveOn=TRUE，basicRfSendPacket 发完不关 RX），继续等下一个信标
                }
            }
        }
    }
    basicRfReceiveOff();

    // 阶段2：默认信道未命中，快速扫描其他 15 个信道（容错：接收节点可能切信道）
    for (tried = 1; tried < 16; tried++) {
        ch = 11 + ((RF_CHANNEL - 11 + tried) % 16);
        halRfSetChannel(ch);
        halRfReceiveOn();
        halMcuWaitMs(50);
        if (basicRfPacketIsReady()) {
            rlen = basicRfReceive((uint8*)rxBuf, sizeof(rxBuf), NULL);
            if (rlen > 0 && rlen < sizeof(rxBuf)) {
                rxBuf[rlen] = 0;
                if (strstr(rxBuf, "TYPE=RECV")) {
                    myRecvAddr = basicRfReceiveAddress();
                    lastBeaconCh = ch;
                    halRfSetChannel(ch);
                    halMcuWaitMs(10);          // 避开接收节点 RF 状态切换过渡期
                    char reg[50];
                    uint8 regResult;
                    sprintf(reg, "{REG=MAC:%s}", myMacStr);
                    regResult = csmaCaSendPacket(myRecvAddr, (uint8*)reg, strlen(reg) + 1);
                    if (regResult == SUCCESS) {
                        registered = 1;
                        printf("{reg=OK, MAC=%s}\r\n", myMacStr);
                    } else {
                        printf("{reg=RETRY}\r\n");
                    }
                    return;
                }
            }
        }
        halRfReceiveOff();
    }
    printf("{reg=FAIL}\r\n");
    halRfSetChannel(RF_CHANNEL);   // P1-13：切回默认信道，避免 RX 窗口在 ch=26 空转
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

/* 射频模块发送数据函数（含MAC地址 + 信标注册 + RX监听窗口 + STATUS采集开关） */
void rfSendData(void){
    char pTxData[80];
    uint8 macAddr[8];
    uint8 ret;
    int rlen;
    // P1-6：删除局部 sTemp/sHumidity 声明，使用全局变量（dht11_update 写，rfSendData 读）
    uint8 sendEnable = 1;    // 数据采集开关: 1=发送, 0=暂停
    uint8 failCount = 0;    // 连续发送失败计数，超阈值触发重注册（业务缺陷2修复）
    
    // 启动时读取本机MAC地址（只读一次，后续直接用缓存）
    readMacAddress(macAddr);
    rand_init(macAddr);              // P0-1：从 MAC 派生随机种子

    // 生成 MAC 字符串 + 注册（短地址保持 0x0001 统一占位，节点识别依赖 payload MAC）
    sprintf(myMacStr, "%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
            macAddr[7], macAddr[6], macAddr[5], macAddr[4],
            macAddr[3], macAddr[2], macAddr[1], macAddr[0]);
    doRegister();
    
    basicRfReceiveOff();
    
    while (TRUE) {
        // === 阶段1：采集并发送传感器数据（sendEnable=0 或 未注册时跳过） ===
        if (sendEnable && registered) {
            dht11_update();  // LCD协议打印 {A0=湿度,A1=温度}
            sTemp = dht11_temp();
            sHumidity = dht11_humidity();
            sprintf(pTxData, "{Humidity=%u, Temp=%u, D=%d, MAC=%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X}",
                    sHumidity, sTemp,
                    (D7 == 0) ? 1 : 0,
                    macAddr[7], macAddr[6], macAddr[5], macAddr[4],
                    macAddr[3], macAddr[2], macAddr[1], macAddr[0]);
            
            ret = csmaCaSendPacket(myRecvAddr, (uint8*)pTxData, strlen(pTxData) + 1);
            if (ret == SUCCESS) {
                failCount = 0;       // 成功则清零
                D6 = 0;       // P1-14：D6 短闪 50ms 表示正在发送数据
                halMcuWaitMs(50);
                D6 = 1;
            } else {
                if (++failCount >= 3) {   // 业务缺陷2：连续失败 3 次判定接收节点离线
                    registered = 0;
                    failCount = 0;
                }
            }
        }
        
        // === 阶段2：RX监听窗口（900ms 分片轮询，及时取包避免 rxCallback 覆盖未读包） ===
        basicRfReceiveOn();
        {
            uint8 waitMs;
            for (waitMs = 0; waitMs < 90; waitMs++) {
                halMcuWaitMs(10);
                // P1-7 + 业务缺陷1修复：分片轮询及时取包，避免阻塞期间 rxCallback 覆盖未读的 rxi
                while (basicRfPacketIsReady()) {
                    rlen = basicRfReceive(gRxData, sizeof(gRxData), NULL);
                    if (rlen > 0 && rlen < sizeof(gRxData)) {
                        gRxData[rlen] = 0;
                        // MAC 比对：只响应发给本节点的指令
                        // P0-6：未注册时不响应指令（myRecvAddr=0，ACK 会发到地址 0 导致丢包）
                        if (registered && strstr((char*)gRxData, myMacStr)) {
                            char ack[60] = {0};
                            if (strstr((char*)gRxData, "LED=1")) {
                                D7 = 0;
                                printf("{cmd=LED ON}\r\n");
                                sprintf(ack, "{CMD=SUCCESS, LED=1, MAC=%s}", myMacStr);
                            } else if (strstr((char*)gRxData, "LED=0")) {
                                D7 = 1;
                                printf("{cmd=LED OFF}\r\n");
                                sprintf(ack, "{CMD=SUCCESS, LED=0, MAC=%s}", myMacStr);
                            } else if (strstr((char*)gRxData, "STATUS=1")) {
                                sendEnable = 1;
                                printf("{cmd=STATUS ON}\r\n");
                                sprintf(ack, "{CMD=SUCCESS, STATUS=1, MAC=%s}", myMacStr);
                            } else if (strstr((char*)gRxData, "STATUS=0")) {
                                sendEnable = 0;
                                printf("{cmd=STATUS OFF}\r\n");
                                sprintf(ack, "{CMD=SUCCESS, STATUS=0, MAC=%s}", myMacStr);
                            }
                            if (ack[0]) {
                                basicRfReceiveOff();
                                basicRfConfig.ackRequest = FALSE;    // P1-11：ACK 包无需接收节点再回硬件 ACK
                                csmaCaSendPacket(myRecvAddr, (uint8*)ack, strlen(ack) + 1);
                                basicRfConfig.ackRequest = TRUE;     // 恢复
                                basicRfReceiveOn();
                            }
                        }
                    }
                }
            }
        }
        basicRfReceiveOff();
        
        // 未注册则重试，成功后恢复数据发送
        if (!registered) { 
            doRegister();
            if (registered) { sendEnable = 1; }
        }
    }
}

/* 射频模块接收数据函数 */
void rfRecvData(void)
{
    int rlen;
    static unsigned int rec_counter = 0;                        //接收次数计数器
    printf("{data=recv node start up...}");
    basicRfReceiveOn();                                         //打开射频接收器
    // Main loop
    while (TRUE) {
        while(!basicRfPacketIsReady());
        rlen = basicRfReceive(gRxData, sizeof gRxData, NULL);
        if(rlen > 0) {
          gRxData[rlen] = 0;
          printf("{data=");                                     //接收到
          printf((char *)gRxData);                              //数据后
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

    
    // Initialize BasicRF（短地址统一 0x0001 占位，节点识别依赖 payload MAC 字符串）
    basicRfConfig.myAddr = 0x0001;
    
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
static uint8 dht11TimedOut = 0;   // 超时标志：DHT11 通信异常时置位，避免死循环

static char dht11_read_bit(void)
{
  int i = 0;
  int to = 0;

  while (!COM_R && ++to < 500);   // 超时保护：数据线卡低时退出，避免死机
  if (to >= 500) { dht11TimedOut = 1; return 0; }
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
    if (dht11TimedOut) return 0;   // 超时则中断读取
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

  dht11TimedOut = 0;           // 复位超时标志
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

  if (dht11TimedOut) return;   // 超时则保留上次有效数据，不更新

  ck = dat1 + dat2 + dat3 + dat4;
  
  if (ck == dat5) {
    sTemp = dat3;
    sHumidity = dat1;        
  }

  //在LCD上发送温湿度（A0=湿度, A1=温度，SPI-LCD协议格式）
  //P0-4：校验失败时显示 ERR，避免 LCD 显示未校验数据与 RF 发送的上次有效数据不一致
  char  dbuf[20] = {0};
  if (ck == dat5) {
    sprintf(dbuf,"{A0=%u,A1=%u}",sHumidity,sTemp); //校验通过，显示有效数据
  } else {
    sprintf(dbuf,"{A0=ERR,A1=ERR}");               //校验失败，明确提示
  }
  Uart_Send_String(dbuf);
  
}
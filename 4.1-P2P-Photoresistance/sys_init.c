#include <ioCC2530.h>             //包含CC2530对应头文件（包含对SFR的定义）
#include "sys_init.h"

/*系统时钟初始化
-------------------------------------------------------*/
void xtal_init(void)
{
  SLEEPCMD &= ~0x04;              //上电
  while(!(CLKCONSTA & 0x40));     //晶振请求标志直到稳定
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

/*外部中断初始化
-------------------------------------------------------*/
void ext_init(void)
{
    P0SEL &= ~0x10;        //通用IO    
    P0DIR &= ~0x10;        //输入方向  
    P0INP &= ~0x10;        //0:三态1:上下拉
    P0IEN |= 0x10;         //开P0口中断
    PICTL &=~ 0x01;        //下降沿触发    
    P0IFG &= ~0x10;        //P0.4中断标志清0    
    P0IE = 1;              //P0中断使能    
    EA = 1;                //总中断使能    
}

/*timer1初始化
-------------------------------------------------------*/
void time1_init(void)
{
  T1CTL = 0x05;                   //8分频，模模式
  T1STAT= 0x21;                   //通道0,中断有效;自动重装模式(0x0000->0xffff)
  
  IEN1|=0X02;                     //定时器1中断使能
  EA=1;                           //开总中断 
}

/*uart0初始化
-------------------------------------------------------*/
void uart0_init(unsigned char StopBits,unsigned char Parity)
{
  P0SEL |=  0x0C;                 //初始化UART0端口
  PERCFG&= ~0x01;                 //选择UART0为可选位置一
  P2DIR &= ~0xC0;                 //P0优先作为串口0
  U0CSR = 0xC0;                   //设置为UART模式,并且使能接收器
   
  U0GCR = 0x0A;                  
  U0BAUD = 0x3B;                  //设置波特率为38400
  
  U0UCR |= StopBits|Parity;       //设置停止位和奇偶校验
}

/*看门狗初始化
-------------------------------------------------------*/
void watchdog_init(void)   
{
  WDCTL = 0x00;                   //看门狗模式，定时一秒
  WDCTL |= 0x08;                  //启动看门狗
}

/*睡眠定时器中断初始化
-------------------------------------------------------*/
void sleepTimer_init(void)
{
  STIF=0;                         //睡眠定时器中断标志清0   
  STIE=1;                         //开睡眠定时器中断   
  EA=1;                           //开总中断
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
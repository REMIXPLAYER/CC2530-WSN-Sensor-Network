/*********************************************************************************************
* �ļ���info.c
* ���ߣ�liutong 2016.7.20
* ˵����ͨ�����ڿ���LCD��ʾ����س���
* �޸ģ�
* ע�ͣ�
*********************************************************************************************/
#include <ioCC2530.h>             
//#include "sys_init.h"
//#include "uart.h"
#include <stdio.h>
#include "info.h"
#include "string.h"
#include "hal_mcu.h"
#include "stdio.h"

#define HAL_INFOP_IEEE_OSET        0xC                          //mac��ַƫ����
#define halWait(x)  halMcuWaitMs(x)
/*********************************************************************************************
* ���ƣ�sensor_init()
* ���ܣ�������Ӳ����ʼ��
* ��������
* ���أ���
* �޸ģ�
* ע�ͣ�
*********************************************************************************************/
void lcd_dis(void){
  for(unsigned char i = 0;i<2;i++){                              //����TYPE,��2��
    printf("{TYPE=02101}");                                      //��Ե�ʵ��
    halWait(250);
    halWait(250);
  }
  
  halWait(250);
  halWait(250);
  char CC2530_MAC[30] = {0};                                     //���MAC
  char devmacaddr[8];
  unsigned char *macaddrptr = (unsigned char *)(P_INFOPAGE+HAL_INFOP_IEEE_OSET);
  for(int i=0;i<8;i++) {
    devmacaddr[i] = macaddrptr[i];                              //��ȡmac��ַ
  }
  strcat(CC2530_MAC,"{MAC=");
  sprintf(&CC2530_MAC[5],"%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
                          devmacaddr[7],devmacaddr[6],devmacaddr[5],
                          devmacaddr[4],devmacaddr[3],devmacaddr[2],
                          devmacaddr[1],devmacaddr[0]);
  CC2530_MAC[28]='}';
   for(unsigned char i = 0;i<2;i++){                            //����MAC����2��
   printf(CC2530_MAC);  
   halWait(250);
   halWait(250);
  }
}
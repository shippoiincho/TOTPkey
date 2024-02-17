/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2020/04/30
 * Description        : Main program body.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

/*
 * @Note
 * In addition, when the system frequency is selected as the USB clock source, only 144MHz/96MHz/48MHz
 * are supported.
 */

/*
 * Connections:
 *   OLED   -> PB6/PB7 SSD1306 OLED 128x32
 *   Keypad -> PB0/PB1/PB3
 *   USBD   -> PA11/PA12
 *   USART1 -> PA9/PA10
 */

/*******************************************************************************/
/* Header Files */
#include <time.h>
#include <string.h>
#include <stdlib.h>
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_pwr.h"
#include "usb_prop.h"
#include "usbd_composite_km.h"
#include "glcdfont.h"
#include "TOTP.h"
#include "totpkey.h"

#define ssd1306Address   0x78
#define TIMEOFFSET     9*3600        // JST+9

#define TIMEOUT  180                 // Timeout to Lock (sec)
#define KEYINTERVAL 50               // Key press/depress interval (ms)

#define RX_BUFFER_LEN 64

const uint8_t unlock[6] = { 1, 1, 1, 2, 2, 2 };  // Unlock code


// initialize command for 128x32 oled

uint8_t ssd1306InitCommand[] = { 0x00, 0xA8, 0x1F, 0xDA, 0x02, 0x20, 0x02, 0x21, 0x00, 0x7F, 0x22, 0x00,
        0x07, 0x8D, 0x14, 0xAF };

const uint8_t mon_table[12] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
extern uint8_t  KB_Data_Pack[ DEF_ENDP_SIZE_KB ];

// USB Keycodes
uint8_t  USB_Keycodes[] = { 0x27, 0x1e, 0x1f, 0x20, 0x21, 0x22, 0x23, 0x24 ,0x25, 0x26};  // FullKey
//uint8_t  USB_Keycodes[] = { 0x62, 0x59, 0x5a, 0x5b, 0x5c, 0x5d, 0x5e, 0x5f ,0x60, 0x61};  // NumKey

volatile uint8_t rxbuff[RX_BUFFER_LEN];
uint8_t rxptr = 0;
uint32_t lastptr = RX_BUFFER_LEN;

/*********************************************************************
 * @fn      RTC_Init
 *
 * @brief   Initializes RTC collection.
 *
 * @return  1 - Init Fail
 *          0 - Init Success
 */
u8 RTC_Init(void) {
    u8 temp = 0;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
    PWR_BackupAccessCmd(ENABLE);

    /* Is it the first configuration */

    BKP_DeInit();
    RCC_LSEConfig(RCC_LSE_ON);
    while(RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET && temp < 250)
    {
        temp++;
        Delay_Ms(20);
    }
    if (temp >= 250)
        return 1;
    RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
    RCC_RTCCLKCmd(ENABLE);
    RTC_WaitForLastTask();
    RTC_WaitForSynchro();

    RTC_WaitForLastTask();
    RTC_EnterConfigMode();
    RTC_SetPrescaler(32767);
    RTC_WaitForLastTask();

    RTC_ExitConfigMode();

    return 0;
}

/*********************************************************************
 * @fn      IIC_Init
 *
 * @brief   Initializes the IIC peripheral.
 *
 * @return  none
 */
void IIC_Init(u32 bound, u16 address) {
    GPIO_InitTypeDef GPIO_InitStructure = { 0 };
    I2C_InitTypeDef I2C_InitTSturcture = { 0 };

    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd( RCC_APB1Periph_I2C1, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOB, &GPIO_InitStructure);

    I2C_InitTSturcture.I2C_ClockSpeed = bound;
    I2C_InitTSturcture.I2C_Mode = I2C_Mode_I2C;
    I2C_InitTSturcture.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitTSturcture.I2C_OwnAddress1 = address;
    I2C_InitTSturcture.I2C_Ack = I2C_Ack_Enable;
    I2C_InitTSturcture.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init( I2C1, &I2C_InitTSturcture);

    I2C_Cmd( I2C1, ENABLE);

    I2C_AcknowledgeConfig( I2C1, ENABLE);

}

void ssd1306Init(void) {

    while( I2C_GetFlagStatus( I2C1, I2C_FLAG_BUSY ) != RESET );
    I2C_GenerateSTART( I2C1, ENABLE);

    while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_MODE_SELECT ) );
    I2C_Send7bitAddress( I2C1, ssd1306Address, I2C_Direction_Transmitter);

    while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED ) );

    for (int i = 0; i < 16; i++) {
        I2C_SendData( I2C1, ssd1306InitCommand[i]);
        while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED ) );
    }

    I2C_GenerateSTOP( I2C1, ENABLE);

}

void ssd1306Fill(uint8_t pattern) {

    for (int page = 0; page < 8; page++) {

        // Set draw position

        while( I2C_GetFlagStatus( I2C1, I2C_FLAG_BUSY ) != RESET );

        I2C_GenerateSTART( I2C1, ENABLE);

        while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_MODE_SELECT ) );
        I2C_Send7bitAddress( I2C1, ssd1306Address, I2C_Direction_Transmitter);

        while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED ) );

        I2C_SendData( I2C1, 0x00);
        while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED ) );
        I2C_SendData( I2C1, 0x20);
        while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED ) );
        I2C_SendData( I2C1, 0x02);
        while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED ) );

        I2C_SendData( I2C1, 0xb0 + page);
        while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED ) );
        I2C_SendData( I2C1, 0x10);
        while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED ) );
        I2C_SendData( I2C1, 0x00);
        while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED ) );

        I2C_GenerateSTOP( I2C1, ENABLE);

        // send bitmap

        while( I2C_GetFlagStatus( I2C1, I2C_FLAG_BUSY ) != RESET );

        I2C_GenerateSTART( I2C1, ENABLE);

        while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_MODE_SELECT ) );
        I2C_Send7bitAddress( I2C1, ssd1306Address, I2C_Direction_Transmitter);

        while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED ) );

        I2C_SendData( I2C1, 0x40);
        while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED ) );
        for (int i = 0; i < 128; i++) {
            I2C_SendData( I2C1, pattern);
            while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED ) );
        }

        I2C_GenerateSTOP( I2C1, ENABLE);

    }

}

void ssd1306PrintChar(uint8_t x, uint8_t y, uint8_t chr) {

    unsigned int xpos;

    // 128x32 = 21 x 4 characters

    if ((x < 21) && (y < 4)) {

        xpos = x * 6;

        // Set draw position

        while( I2C_GetFlagStatus( I2C1, I2C_FLAG_BUSY ) != RESET );

        I2C_GenerateSTART( I2C1, ENABLE);

        while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_MODE_SELECT ) );
        I2C_Send7bitAddress( I2C1, ssd1306Address, I2C_Direction_Transmitter);

        while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED ) );

        I2C_SendData( I2C1, 0x00);
        while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED ) );
        I2C_SendData( I2C1, 0x20);
        while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED ) );
        I2C_SendData( I2C1, 0x02);
        while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED ) );
        I2C_SendData( I2C1, 0xb0 + (y & 0x7));
        while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED ) );
        I2C_SendData( I2C1, 0x10 + ((xpos & 0xf0) >> 4));
        while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED ) );
        I2C_SendData( I2C1, 0x00 + xpos & 0x0f);
        while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED ) );

        I2C_GenerateSTOP( I2C1, ENABLE);

        // send bitmap

        while( I2C_GetFlagStatus( I2C1, I2C_FLAG_BUSY ) != RESET );

        I2C_GenerateSTART( I2C1, ENABLE);

        while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_MODE_SELECT ) );
        I2C_Send7bitAddress( I2C1, ssd1306Address, I2C_Direction_Transmitter);

        while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED ) );

        I2C_SendData( I2C1, 0x40);
        while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED ) );
        for (int i = 0; i < 5; i++) {
            I2C_SendData( I2C1, font[chr * 5 + i]);
            while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED ) );
        }
        I2C_SendData( I2C1, 0x00);
        while( !I2C_CheckEvent( I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED ) );

        I2C_GenerateSTOP( I2C1, ENABLE);

    }

}

void ssd1306Print(uint8_t x, uint8_t y, uint8_t *str) {

    while(*str!=0) {

        ssd1306PrintChar(x, y, *str);
        str++;
        x++;
        if(x>21) break;
    }

}



/*********************************************************************
 * @fn      Is_Leap_Year
 *
 * @brief   Judging whether it is a leap year.
 *
 * @param   year
 *
 * @return  1 - Yes
 *          0 - No
 */
u8 Is_Leap_Year(u16 year) {
    if (year % 4 == 0) {
        if (year % 100 == 0) {
            if (year % 400 == 0)
                return 1;
            else
                return 0;
        } else
            return 1;
    } else
        return 0;
}

//

uint32_t RTC_Set(u16 syear, u8 smon, u8 sday, u8 hour, u8 min, u8 sec) {
    u16 t;
    u32 seccount = 0;
    if (syear < 1970 || syear > 2099)
        return 1;
    for (t = 1970; t < syear; t++) {
        if (Is_Leap_Year(t))
            seccount += 31622400;
        else
            seccount += 31536000;
    }
    smon -= 1;
    for (t = 0; t < smon; t++) {
        seccount += (u32) mon_table[t] * 86400;
        if (Is_Leap_Year(syear) && t == 1)
            seccount += 86400;
    }
    seccount += (u32) (sday - 1) * 86400;
    seccount += (u32) hour * 3600;
    seccount += (u32) min * 60;
    seccount += sec;

    return seccount;

    //   RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
    //   PWR_BackupAccessCmd(ENABLE);
    //   RTC_SetCounter(seccount);
    //   RTC_WaitForLastTask();
    //   return 0;
}

void Keypad_Init(void){
    GPIO_InitTypeDef GPIO_InitStructure = { 0 };

    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB , ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_3 ;
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOB, &GPIO_InitStructure);

}


uint8_t Keypad_Scan(void) {

    uint8_t scancode1,scancode2;
    static uint8_t last_scancode=0;

    scancode1=GPIOB->INDR & 0xb;
    Delay_Ms(1);
    scancode2=GPIOB->INDR & 0xb;

    if(scancode1!=scancode2) {
        return 0;
    }

    if(scancode1!=last_scancode) {
        last_scancode=scancode1;
        return (~scancode1)&0xb;
    } else {
        return 0;
    }

}

void cls(void) {
    ssd1306Print(0, 1, "                     ");
    ssd1306Print(0, 2, "                     ");
    ssd1306Print(0, 3, "                     ");
    return;
}

// init UART1 for key input

void USART_CFG(void) {
    GPIO_InitTypeDef GPIO_InitStructure = { 0 };
    USART_InitTypeDef USART_InitStructure = { 0 };

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

    /* USART2 TX-->A.9   RX-->A.10 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl =
    USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

    USART_Init(USART1, &USART_InitStructure);
    USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
    USART_Cmd(USART1, ENABLE);

}

void DMA_Rx_Init(DMA_Channel_TypeDef *DMA_CHx, u32 ppadr, u32 memadr,
        u16 bufsize) {
    DMA_InitTypeDef DMA_InitStructure = { 0 };

    RCC_AHBPeriphClockCmd( RCC_AHBPeriph_DMA1, ENABLE);

    DMA_DeInit(DMA_CHx);

    DMA_InitStructure.DMA_PeripheralBaseAddr = ppadr;
    DMA_InitStructure.DMA_MemoryBaseAddr = memadr;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = bufsize;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA_CHx, &DMA_InitStructure);

    DMA_Cmd(DMA_CHx, ENABLE);

}

static inline uint8_t usart_getch() {

    uint8_t ch;
    uint32_t currptr;

    currptr = DMA_GetCurrDataCounter(DMA1_Channel5);

    if (currptr == lastptr) {
        return 0;
    }

    ch = rxbuff[rxptr];
    lastptr--;
    if (lastptr == 0) {
        lastptr = RX_BUFFER_LEN;
    }

    rxptr++;
    if (rxptr >= RX_BUFFER_LEN)
        rxptr = 0;

    return ch;

}

uint64_t usart_getint(void) {
    uint64_t value=0;
    uint8_t string[64];
    uint8_t ptr=0;
    uint8_t ch;
    uint8_t done=0;

    memset(string,0,64);

    while(done==0) {
        ch=usart_getch();

        if((ch>='0')&&(ch<='9')) {
            printf("%c",ch);
            fflush(stdout);

            string[ptr++]=ch;
            if(ptr==63) done=1;
        }

        if((ch==0x0a)||(ch==0x0d)) {
            done=1;
        }
    }

    value=atoll(string);

    return value;
}

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program
 *
 * @return  none
 */
int main(void) {

    uint32_t current_time,current_time_local;
    uint32_t time_last_adjust,time_diff,new_time;
    int16_t new_caliblation;

    uint32_t totpkey,totpdiv;
    uint8_t status;
    uint8_t sendkey;
    static uint8_t last_sec=0;
    int8_t mode=0;     // 0:Locked 1:Unlocking 2:TOTP -1: Calibration -2: Calibration exec
    uint8_t subcode;
    uint8_t unlock_fail;
    uint8_t numkey;
    uint8_t keycode;
    uint8_t str[64];
    time_t unixtime;
    uint32_t last_press_time=0;
    struct tm* calendar_time;
    uint64_t input_time;
    uint8_t done=0;
    uint32_t my_year,my_mon,my_mday,my_hour,my_min;

    /* Initialize system configuration */
    NVIC_PriorityGroupConfig( NVIC_PriorityGroup_2);
    Delay_Init();
    USART_CFG();
    DMA_Rx_Init( DMA1_Channel5, (u32) &USART1->DATAR, (u32) &rxbuff,
    RX_BUFFER_LEN);

//    printf("SystemClk:%d\r\n", SystemCoreClock);

    // Initialize display

    IIC_Init(400000, ssd1306Address);
    ssd1306Init();

    // Initialize USB

    Keypad_Init();

    Set_USBConfig();
    USB_Init();
    USB_Interrupts_Config();

    // Check RTC is running

    if (RCC->BDCTLR % 0x80000 == 0) {

        ssd1306Fill(0);

        ssd1306Print(0, 0, "RTC is not running.");
        ssd1306Print(0, 1, "Please set current");
        ssd1306Print(0, 2, "time via USART.");
        ssd1306Print(0, 3, "YYYYMMDDhhmm format");

        Delay_Ms(1000);

        while(done==0) {

            printf("\n\r\n\rInput current time in YYYYMMDDhhmm format.\n\r");

            input_time=usart_getint();


                my_year=input_time/100000000;
                my_mon=(input_time/1000000)%100;
                my_mday=(input_time/10000)%100;
                my_hour=(input_time/100)%100;
                my_min=input_time%100;

//                printf("%d %d %d %d %d\n\r",my_year,my_mon,my_mday,my_hour,my_min);

                if((my_year<2024)||(my_year>2039)) continue;
                if((my_mon>12)||(my_mon==0)) continue;
                if((my_mday>31)||(my_mday==0)) continue;
                if(my_hour>24) continue;
                if(my_min>60) continue;

                current_time = RTC_Set(my_year, my_mon, my_mday, my_hour, my_min, 0); /* Setup Time */
                current_time-=TIMEOFFSET;
                done=1;

        }

        RTC_Init();
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
        PWR_BackupAccessCmd(ENABLE);

        RTC_SetCounter(current_time);
        RTC_WaitForLastTask();

        PWR_BackupAccessCmd(DISABLE);
    } else {
        printf("RTC is already running\n\r");
    }

    ssd1306Fill(0);

    while(1) {

        current_time=RTC_GetCounter();
        current_time_local = current_time + TIMEOFFSET;
        unixtime = (time_t) current_time_local;

        calendar_time = localtime(&unixtime);

        if(last_sec!=calendar_time->tm_sec) {

            sprintf(str,"%04d-%02d-%02d %02d:%02d:%02d",calendar_time->tm_year+1900,
                    calendar_time->tm_mon+1,calendar_time->tm_mday,
                    calendar_time->tm_hour,calendar_time->tm_min,calendar_time->tm_sec);

            ssd1306Print(0, 0, str);

            last_sec=calendar_time->tm_sec;

        }

        if( bDeviceState == CONFIGURED )
        {

            switch(mode) {

            // Locked mode
            // Press any key to Unlock

            case 0:

                ssd1306Print(0, 3, "TOTP Key is Locked.  ");
                keycode=Keypad_Scan();
                if(keycode!=0) {
                    mode=1;
                    subcode=0;
                    unlock_fail=0;
                    cls();
                    last_press_time=current_time;
                }
                break;

                // Unlock screen
                // Press right key sequence to unlock

            case 1:

                ssd1306Print(0, 2, "Press keys to unlock. ");

                keycode=Keypad_Scan();
                if(keycode!=0) {
                    ssd1306Print(5+subcode, 3, "\x04");
                    if(keycode!=unlock[subcode]) {
                        unlock_fail=1;
                    }
                    subcode++;
                    if(subcode==6) {
                        cls();
                        if(unlock_fail==0) {
                            mode=2;
                        } else {
                            mode=0;
                        }
                    }
                    last_press_time=current_time;
                }

                if((current_time-last_press_time)>TIMEOUT) {
                    cls();
                    mode=0;
                }

                break;

            // Show RTC Calibration screen
            // Press OK to enter EXEC screen

            case -1:

                keycode=Keypad_Scan();

                ssd1306Print(0, 2, "Press OK to enter");
                ssd1306Print(0, 3, "RTC Calibration mode.");

                if((keycode&1)!=0) {
                    last_press_time=current_time;
                    mode=TOTPKEYS+1;
                }

                if((keycode&2)!=0) {
                    last_press_time=current_time;
                    cls();
                    mode=2;
                }

                if((keycode&8)!=0) {
                    last_press_time=current_time;
                    cls();
                    mode=-2;
                }

                if((current_time-last_press_time)>TIMEOUT) {
                    cls();
                    mode=0;
                }
                break;

                // Exec RTC Calibration

            case -2:

                keycode=Keypad_Scan();

                new_time=current_time%60;

                if(new_time<30) {
                    new_time = (current_time/60)*60;
                } else {
                    new_time = (current_time/60)*60 + 60;
                }

                sprintf(str,"%d->%d",current_time,new_time);
                ssd1306Print(0, 2, "Press OK to set time.  ");

                current_time_local = new_time + TIMEOFFSET;
                unixtime = (time_t) current_time_local;
                calendar_time = localtime(&unixtime);

                sprintf(str,"%02d:%02d:%02d  CALB=%03d",
                        calendar_time->tm_hour,calendar_time->tm_min,calendar_time->tm_sec,(BKP->OCTLR)&0x7f);
                ssd1306Print(0, 3, str);

                if((keycode&1)!=0) {
                    last_press_time=current_time;
                    mode=-1;
                }

                if((keycode&2)!=0) {
                    last_press_time=current_time;
                    mode=-1;
                }

                if((keycode&8)!=0) {
                    last_press_time=new_time;

                    time_last_adjust = BKP_ReadBackupRegister(BKP_DR1)*65536 + BKP_ReadBackupRegister(BKP_DR2);

                    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
                    PWR_BackupAccessCmd(ENABLE);

                    RTC_EnterConfigMode();
                    RTC_SetCounter(new_time);
                    RTC_WaitForLastTask();
                    RTC_ExitConfigMode();

                    BKP_WriteBackupRegister(BKP_DR1, new_time/65536);
                    BKP_WriteBackupRegister(BKP_DR2, new_time%65536);

                    // calculate new calibation value
                    if((time_last_adjust!=0)&&((new_time-time_last_adjust)>100000)) {

                        time_diff=new_time-time_last_adjust;
                        new_caliblation=(current_time-new_time)*1000000 / time_diff;

                        new_caliblation += (BKP->OCTLR)&0x7f;

                        if(new_caliblation>127) new_caliblation=127;
                        if(new_caliblation<0) new_caliblation=0;

                        BKP_SetRTCCalibrationValue(new_caliblation);

                    }

                    PWR_BackupAccessCmd(DISABLE);

                    current_time=new_time;

                }

                if((current_time-last_press_time)>TIMEOUT) {
                    cls();
                    mode=0;
                }
                break;

            default:

                keycode=Keypad_Scan();
                numkey=mode-2;

                TOTP(hmacKey+10*numkey, 10, 30);
                sprintf(str,"%02d:%s               ",numkey,hmacLabel[numkey]);
                ssd1306Print(0, 2, str);
                sprintf(str,"%06d          ",getCodeFromTimestamp(current_time));
                ssd1306Print(0, 3, str);

                if((keycode&1)!=0) {
                    last_press_time=current_time;
                    cls();
                    mode--;
                }

                if((keycode&2)!=0) {
                    last_press_time=current_time;
                    cls();
                    mode++;
                }

                if(mode==1) mode=-1;
                if(mode==TOTPKEYS+2) mode=-1;

                if((keycode&8)!=0) {
                    last_press_time=current_time;

                    // send TOTP key via USB

                    totpkey=getCodeFromTimestamp(current_time);
                    totpdiv=100000;

                    for(int i=0;i<6;i++) {

                        sendkey=totpkey/totpdiv;
                        totpkey -= sendkey*totpdiv;

                        memset( KB_Data_Pack, 0x00, DEF_ENDP_SIZE_KB );
                        KB_Data_Pack[ 2 ] = USB_Keycodes[sendkey];

                        status = USBD_ENDPx_DataUp(ENDP1, KB_Data_Pack, DEF_ENDP_SIZE_KB);

                        Delay_Ms(KEYINTERVAL);

                        memset( KB_Data_Pack, 0x00, DEF_ENDP_SIZE_KB );
                        status = USBD_ENDPx_DataUp(ENDP1, KB_Data_Pack, DEF_ENDP_SIZE_KB);

                        Delay_Ms(KEYINTERVAL);

                        totpdiv/=10;

                    }
                }
                if((current_time-last_press_time)>TIMEOUT) {
                    cls();
                    mode=0;
                }
            }



        } else {
            ssd1306Print(0, 3, "PC is not connected.");
            mode=0;
        }

        Delay_Ms(10);



    }


}

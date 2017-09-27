/* 
 * File:   GLVS_Main_Driver.h
 * Author: Kristján
 *
 * Created on 5. febrúar 2015, 22:33
 */



#ifndef GLVS_MAIN_DRIVER_H
#define	GLVS_MAIN_DRIVER_H

#include "config.h"
#include <xc.h>
#include <plib/timers.h>
#include <p18f4580.h>
#include <stdlib.h>
#include <plib/usart.h>
#include <plib/spi.h>
#include <adc.h>

#define _XTAL_FREQ      32000000        // Frequency of internal oscillator
#define TRUE            1
#define FALSE           0

// PORTA defines
#define Throttle_1      PORTAbits.RA0   // Throttle 1 signal input
#define Throttle_2      PORTAbits.RA1   // Throttle 2 singal input
#define Brake           PORTAbits.RA2   // Brake signal input
#define Temperature     PORTAbits.RA3   // GLVS Main board tempareture input
#define RTD_Enable      LATAbits.LATA4  // Ready to drive enable
#define Current         PORTAbits.RA5   // Current measurement input
#define RTD_Reset       LATAbits.LATA6  // Ready to drove reset
#define TS_Enable       LATAbits.LATA7  // Tractive system enable

// PORTB defines
#define SD_4            PORTBbits.RB0   // Shutdown status 4 - LSDB
#define SD_3            PORTBbits.RB1   // Shutdown status 3 - RSDB
#define SD_2            PORTBbits.RB4   // Shutdown status 2 - BSPD
#define SD_1            PORTBbits.RB5   // Shutdown status 1 - MCU enable

// PORTC defines
#define LE              LATCbits.LATC0  // Latch enable (16-segment display)
#define OE              LATCbits.LATC1  // Output enable (16-segment display)
#define Fan_Enable      LATCbits.LATC2  // GLVS box fan enable

// PORTD defines
#define SD_10           PORTDbits.RD0   // Shutdown status 10 - IMD
#define SD_9            PORTDbits.RD1   // Shutdown status 9 - BMS
#define SD_8            PORTDbits.RD2   // Shutdown status 8 - HVD
#define SD_7            PORTDbits.RD3   // Shutdown status 7 - Inertia Switch
#define SD_6            PORTDbits.RD4   // Shutdown status 6 - BOTS
#define SD_5            PORTDbits.RD5   // Shutdown status 5 - CSDB
#define Charge          PORTDbits.RD6   // Charge signal
#define TS_Status       PORTDbits.RD7   // Tractive system status

// Software UART defines
#define SWTXD           PORTEbits       // Transmit pin port and pin
#define SWTXDpin        RE0
#define TRIS_SWTXD      TRISEbits       // Transmit pin tris and pin
#define TRISbit_SWTXD   TRISE0
#define SWRXD           PORTEbits       // Receive pin port and pin
#define SWRXDpin        RE1
#define TRIS_SWRXD      TRISEbits       // Receive pin tris and pin
#define TRISbit_SWRXD   TRISE1

// Throttle and brake encoder defines
#define Sensor_Max      1000             // Maximum value of sensor (4.5V)
#define Sensor_Min      90             // Minimum value af sensor (0.5V)
#define Sensor_25       400             // 25% of pedal travel (1V)         OLD: 205
#define Sensor_5        100              // 5% of pedal travel (0.2V)
#define Sensor_Max_Dev  80              // 10% devtation between sensors (0.4V)
#define Brake_On        307             // Defines when brake is on (1.5)

// Timer 0 defines
#define _1s             0x85ED
#define _100ms          0xF3CA
#define _10ms           0xFEC6
#define _1ms            0xFFE0


// 16 segment defines
#define _A              0xB499
#define _B              0xB723
#define _C              0x8389
#define _D              0xA723
#define _0              0xE7C9
#define _1              0x6400
#define _2              0xB391
#define _3              0xB701
#define _4              0x3418
#define _5              0x9719
#define _6              0x9799
#define _7              0xA401
#define _8              0xB799
#define _9              0xB619
#define _O              0xA789
#define _K              0x4898
#define _BLANK          0x0000
#define _DASH           0x1010

// Configuration functions
void    ClockConfig();
void    IOConfig();
void    USARTConfig();
void    SPIConfig();
void    ADCConfig();
void    ConfigMCU();
void    Timer0Config();

// Software UART functions
void    OpenSWUART();
void    BitDelay();
void    putcSWUART(unsigned char);
void    putsSWUART(unsigned char*);

// ADC functions
int     ReadThrottle1();
int     ReadThrottle2();
int     ReadBrake();
int     ReadTemperature();
int     ReadCurrent();

// Torque and brake encoder functions
char    SensorImplausibility();
char    Throttle1RangeTest();
char    Throttle2RangeTest();
char    BrakeRangeCheck();
char    ThrottleBrakePlausibilityCheck();
char    Throttle1ReactivateCheck();
char    PedalSystemCheck();

// 16-segment functions
void    Init16Seg();
void    SendData16Seg(unsigned int, unsigned int, unsigned int);
void    DisplayError();

// Misc functions
void    NewLineUSART();
void    WriteIntUSART(int);
void    HomeUSART();
void    Delay1Second();
void    ReadSDError();
void    Debug();

#endif	/* GLVS_MAIN_DRIVER_H */


/*
 *  File name:
 *      GLVS_Main_Driver.c
 *  Author:
 *      Kristján Bjarki Purkhús
 *  Description:
 *      Driver file containing functions used in GLVS Main project
 *      The PIC is running at 32MHz by using an internal 8MHz crystal and the 4X Phase Locked Loop Function.
 *      Datasheet: http://ww1.microchip.com/downloads/en/devicedoc/39637c.pdfy.
 *  Notes:
 *
 */

#include "GLVS_Main_Driver.h"

char Error_Array[32];
char reactivate_bit = 1;
char error_cnt = 0;

    /*
    *  Error Code for Error_Array
    * --------------------------------------------------------------------------
    *  Bit         Code        Description                         Penalty
    * --------------------------------------------------------------------------
    *   0          A00         MCU Enable                          Tractive System Shutdown
    *   1          A01         Brake System Plausibility Device    Tractive System Shutdown
    *   2          A02         Right Shutdown Button               Tractive System Shutdown
    *   3          A03         Left Shutdown Button                Tractive System Shutdown
    *   4          A04         Cockpit Shutdown Button             Tractive System Shutdown
    *   5          A05         Brake Over-travel Switch            Tractive System Shutdown
    *   6          A06         Inertia Switch                      Tractive System Shutdown
    *   7          A07         High Voltage Disconnect             Tractive System Shutdown
    *   8          A08         Battery Management System           Tractive System Shutdown
    *   9          A09         Insulation Monitoing Device         Tractive System Shutdown
    *   10         B00         Sensor Implausibility Check         Motor Power Shutdown
    *   11         B01         Throttle 1 Range                    Motor Power Shutdown
    *   12         B02         Throttle 2 Range                    Motor Power Shutdown
    *   13         B03         Brake Range                         Motor Power Shutdown
    *   14         B04         Throttle and Brake Plausibility     Motor Power Shutdown
    *   15         B05         Throttle 1 Reactivate               Motor Power Shutdown
    */

// Internal oscillator configuration: Sets the internal oscillator to 32MHz using 4x PLL function
void ClockConfig()
{
    // Internal oscillator configuration
    OSCCONbits.IRCF0 = 1;
    OSCCONbits.IRCF1 = 1;
    OSCCONbits.IRCF2 = 1;

    // Enable 4x Phase Locked Loop funtction
    OSCTUNEbits.PLLEN = 1;
}

void Timer0Config()
{
    OpenTimer0(TIMER_INT_ON & T0_16BIT & T0_SOURCE_INT & T0_PS_1_256);
    WriteTimer0(0xEA60);
    INTCONbits.TMR0IF = 0;
}

// I/O Configuration
void IOConfig()
{
    /*  Pin map
     *  RA0 - Throttle 1
     *  RA1 - Throttle 2
     *  RA2 - Brake
     *  RA3 - Temp
     *  RA4 - RTD Enable
     *  RA5 - Current measurement in
     *  RA6 - RTD Reset
     *  RA7 - TS Enable
     *
     *  RB0 - SD Status 4
     *  RB1 - SD Status 3
     *  RB2 - CANTX
     *  RB3 - CANRX
     *  RB4 - SD Status 2
     *  RB5 - SD Status 1
     *  RB6 - PGC
     *  RB7 - PGD
     *
     *  RC0 - LE
     *  RC1 - OE
     *  RC2 - Fan Enable
     *  RC3 - CLK
     *  RC4 - SDI
     *  RC5 - SDO
     *  RC6 - USART TX
     *  RC7 - USART RX
     *
     *  RD0 - SD Status 10
     *  RD1 - SD Status 9
     *  RD2 - SD Status 8
     *  RD3 - SD Status 7
     *  RD4 - SD Status 6
     *  RD5 - SD Status 5
     *  RD6 - Charge Signal
     *  RD7 - TS Status
     *
     *  RE0 - Software USART TX
     *  RE1 - Software USART RX
     *  RE2 - NC
     *  RE3 - MCLR
     */

    // I/O Config; 0 = Output, 1 = Input
    TRISA = 0x2F;
    TRISB = 0x33;
    TRISC = 0x90;
    TRISD = 0xFF;
    TRISE = 0xFE;

    // Initialize PORTS
    LATA = 0x80;
    LATB = 0x00;
    LATC = 0x00;
    LATD = 0x00;
    LATE = 0x00;

    CCP1CON = 0x00;             // Turn off Capture/Compare/PWM
    CMCON = 0x07;               // Turn off comparators
    ECCP1CONbits.ECCP1M = 0;    // Turn off ECCP module
    TRISEbits.PSPMODE = 0;      // Turn off PSP module
}

// USART Configuration
void USARTConfig()
{
    unsigned char USARTConfig = 0;
    unsigned int baud;

    // USART configuration - See <plib/usart.h>
    USARTConfig = USART_TX_INT_OFF &
                  USART_CONT_RX &
                  USART_RX_INT_OFF &
                  USART_ASYNCH_MODE &
                  USART_EIGHT_BIT &
                  USART_BRGH_HIGH;

    // spbrg bit - sets USART Baud rate; Formula: 16(X+1) = Fosc/Baud Rate
    // baud = 207; // 9600 baud
    
    baud = 34; // 57600 baud

    // Initialize USART
    OpenUSART(USARTConfig,baud);

    // Delay for USART module to initialize
    __delay_ms(20);
    __delay_ms(20);
    __delay_ms(20);
    __delay_ms(20);
    __delay_ms(20);
}

// SPI Configuration
void SPIConfig()
{
    OpenSPI(SPI_FOSC_64, MODE_10, SMPMID); // skoða stillingar
}

// Analog-to-Digital converter configuration
void ADCConfig()
{
    OpenADC(ADC_FOSC_2 &
            ADC_RIGHT_JUST &
            ADC_20_TAD,
            ADC_CH0 &
            ADC_INT_OFF &
            ADC_VREFPLUS_VDD &
            ADC_VREFMINUS_VSS, 
            ADC_4ANA);
}

// Configre MCU
void ConfigMCU()
{
    // Internal Oscillator configuration
    ClockConfig();

    // I/O configuration
    IOConfig();

    // USART configuration
    USARTConfig();

    // SPI configuration
    SPIConfig();

    // ADC configuration
    ADCConfig();

    // Initialize 16 segment display
    Init16Seg();

    // Software UART configration
    OpenSWUART();

    // Timer 0 configuration
    Timer0Config();

    // Enable interrupts
    //ei();
}

// Software UART configuration
void OpenSWUART()
{
    TRIS_SWTXD.TRISbit_SWTXD = 0x00;
    SWTXD.SWTXDpin           = 0x01;
    TRIS_SWRXD.TRISbit_SWRXD = 0x00;
    SWRXD.SWRXDpin           = 0x01;
}

// Delay for 9600 baude rate
void BitDelay()
{
    Delay100TCYx(7);
    Delay10TCYx(6);
}

// Writes one byte on software UART
void putcSWUART(unsigned char uartdata)
{
    unsigned char bitcount = 8;

    // Start bit
    SWTXD.SWTXDpin = 0;
    BitDelay();

    while (bitcount--)
    {
	SWTXD.SWTXDpin = uartdata& 0x01;
	BitDelay();
	uartdata >>= 1;
    }

    // Stop bit
    SWTXD.SWTXDpin = 1;
    BitDelay();
}

// Writes a string on software UART
void putsSWUART(char *data)
{
    do
    {
	putcSWUART(*data);
    } while( *data++ );
}

// Reads from ADC throttle 1 signal value
int ReadThrottle1()
{
    int result;

    SetChanADC(ADC_CH0);    // Select channel
    ConvertADC();           // Start ADC conversion

    while(BusyADC());       // Wait while ADC is busy

    result = ReadADC();     // Read result

    return result;
}

// Reads from ADC throttle 2 signal value
int ReadThrottle2()
{
    int result;

    SetChanADC(ADC_CH1);    // Select channel
    ConvertADC();           // Start ADC conversion

    while(BusyADC());       // Wait while ADC is busy

    result = ReadADC();     // Read result

    return result;
}

// Reads from ADC brake signal value
int ReadBrake()
{
    int result;

    SetChanADC(ADC_CH2);    // Select channel
    ConvertADC();           // Start ADC conversion

    while(BusyADC());       // Wait while ADC is busy

    result = ReadADC();     // Read result

    return result;
}

// Reads from ADC temperature signal value.
int ReadTemperature()
{
    int result;
    float temp;

    SetChanADC(ADC_CH3);    // Select channel
    ConvertADC();           // Start ADC conversion

    while(BusyADC());       // Wait while ADC is busy

    result = ReadADC();     // Read result
    //temp = (1.8663 - ((float)result)*0.004883)/(11.69*0.001)*10;

    //result = (int)temp;

    return result;
}

int ReadCurrent()
{
    int result;

    SetChanADC(ADC_CH5);    // Select channel
    ConvertADC();           // Start ADC conversion

    while(BusyADC());       // Wait while ADC is busy

    result = ReadADC();     // Read result

    return result;
}

// Check sensor implausibility - returns 0 for implausibility
char SensorImplausibilityCheck()
{
    int sensor_1;
    int sensor_2;

    sensor_1 = ReadThrottle1();
    sensor_2 = ReadThrottle2();

    if(abs(sensor_1 - sensor_2) > Sensor_Max_Dev) return 0;
    else return 1;
}

// Check if sensor is within correct range - returns 0 out of range
char Throttle1RangeCheck()
{
    int sensor;

    sensor = ReadThrottle1();

    if(sensor > Sensor_Max || sensor < Sensor_Min) return 0;
    else return 1;
}

// Check if sensor is within correct range - returns 0 out of range
char Throttle2RangeCheck()
{
    int sensor;

    sensor = ReadThrottle2();

    if(sensor > Sensor_Max || sensor < Sensor_Min) return 0;
    else return 1;
}

// Check if sensor is within correct range - returns 0 out of range
char BrakeRangeCheck()
{
    int sensor;

    sensor = ReadBrake();

    if(sensor > Sensor_Max || sensor < Sensor_Min) return 0;
    else return 1;
}

// Torque and brake encoder plausibility check - returns 0 for implausibility
char ThrottleBrakePlausibilityCheck()
{
    int throttle;
    int brake;

    throttle = ReadThrottle1();
    brake = ReadBrake();

    if(brake > Brake_On && throttle > Sensor_25) 
    {
        reactivate_bit = 0;
        return 0;
    }
    else return 1;
}

// Checks if throttle can be reactivated after torque and brake encoder implausibility - returns 0 if error
char Throttle1ReactivateCheck()
{
    int sensor;

    sensor = ReadThrottle1();

    if(sensor < Sensor_5 + Sensor_Min)
    {
        reactivate_bit = TRUE;
        return 1;
    }
    else if(sensor > (Sensor_5 + Sensor_Min) && reactivate_bit == FALSE) return 0;

    return 1;
}

// Check Torque and brake pedal encoder system - returns 1 if no error
char PedalSystemCheck()
{
    int i;

    Error_Array[10] = SensorImplausibilityCheck();
    Error_Array[11] = Throttle1RangeCheck();
    Error_Array[12] = Throttle2RangeCheck();
    Error_Array[13] = BrakeRangeCheck();
    Error_Array[14] = ThrottleBrakePlausibilityCheck();
    Error_Array[15] = Throttle1ReactivateCheck()&Error_Array[14];

    for(i = 10; i < 16; i++)
    {
        if(!Error_Array[i]) return 0;
    }

    return 1;
}

// Initialize 16-segment display
void Init16Seg()
{
    SendData16Seg(_DASH,_DASH,_DASH);
}

// Send data over SPI for 16-segment dislay
void SendData16Seg(unsigned int module, unsigned int num_1, unsigned int num_2)
{

    // Blank display
    OE = 1;
    
    // send high and low byte
    WriteSPI((char)(num_2>>8));
    WriteSPI((char)num_2);

    // send high and low byte
    WriteSPI((char)(num_1>>8));
    WriteSPI((char)num_1);

    // send high and low byte
    WriteSPI((char)(module>>8));
    WriteSPI((char)module);

    LE = 1;
    __delay_us(1);
    LE = 0;

    OE = 0;
}

// Display error on 16-segment display
void DisplayError()
{

    /*
    *  Error Code for Error_Array
    * --------------------------------------------------------------------------
    *  Bit         Code        Description                         Penalty
    * --------------------------------------------------------------------------
    *   0          A00         MCU Enable                          Tractive System Shutdown
    *   1          A01         Brake System Plausibility Device    Tractive System Shutdown
    *   2          A02         Right Shutdown Button               Tractive System Shutdown
    *   3          A03         Left Shutdown Button                Tractive System Shutdown
    *   4          A04         Cockpit Shutdown Button             Tractive System Shutdown
    *   5          A05         Brake Over-travel Switch            Tractive System Shutdown
    *   6          A06         Inertia Switch                      Tractive System Shutdown
    *   7          A07         High Voltage Disconnect             Tractive System Shutdown
    *   8          A08         Battery Management System           Tractive System Shutdown
    *   9          A09         Insulation Monitoing Device         Tractive System Shutdown
    *   10         B00         Sensor Implausibility Check         Motor Power Shutdown
    *   11         B01         Throttle 1 Range                    Motor Power Shutdown
    *   12         B02         Throttle 2 Range                    Motor Power Shutdown
    *   13         B03         Brake Range                         Motor Power Shutdown
    *   14         B04         Throttle and Brake Plausibility     Motor Power Shutdown
    *   15         B05         Throttle 1 Reactivate               Motor Power Shutdown
    */

    for(error_cnt = 0; error_cnt < 32; error_cnt++)
    {
        if(Error_Array[error_cnt] == 0) break;
    }

    switch(error_cnt)
    {
        case 0: SendData16Seg(_A,_0,_0); break;
        case 1: SendData16Seg(_A,_0,_1); break;
        case 2: SendData16Seg(_A,_0,_2); break;
        case 3: SendData16Seg(_A,_0,_3); break;
        case 4: SendData16Seg(_A,_0,_4); break;
        case 5: SendData16Seg(_A,_0,_5); break;
        case 6: SendData16Seg(_A,_0,_6); break;
        case 7: SendData16Seg(_A,_0,_7); break;
        case 8: SendData16Seg(_A,_0,_8); break;
        case 9: SendData16Seg(_A,_0,_9); break;
        case 10: SendData16Seg(_B,_0,_0); break;
        case 11: SendData16Seg(_B,_0,_1); break;
        case 12: SendData16Seg(_B,_0,_2); break;
        case 13: SendData16Seg(_B,_0,_3); break;
        case 14: SendData16Seg(_B,_0,_4); break;
        case 15: SendData16Seg(_B,_0,_5); break;
        default: SendData16Seg(_BLANK,_O,_K); break;
    }
}
// Prints new line with carrage return on USART
void NewLineUSART()
{
    putcUSART(10);
    while(BusyUSART()){}
    putcUSART(13);
    while(BusyUSART()){}
}

// Writes int as string on USART
void WriteIntUSART(int _data)
{
    char buf[8];

    putrsUSART(itoa(buf, _data, 10));
}

// Cursor home
void HomeUSART()
{
    putrsUSART("\033[2J");   // clear screen
    putrsUSART("\033[0;0H"); // set home
}

// Function: 1 second delay
void Delay1Second()
{
    int i;
    for(i=0;i<100;i++)
    {
         __delay_ms(10);
    }
}

// Reads shutdown circuit status
void ReadSDError()
{
    Error_Array[0] = SD_1;
    Error_Array[1] = SD_2;
    Error_Array[2] = SD_3;
    Error_Array[3] = SD_4;
    Error_Array[4] = SD_5;
    Error_Array[5] = SD_6;
    Error_Array[6] = SD_7;
    Error_Array[7] = SD_8;
    Error_Array[8] = SD_9;
    Error_Array[9] = SD_10;
}

// Debug program
void Debug()
{

    // Start debug messages
    putrsUSART("**** START ****");
    NewLineUSART();

    // Read thottle sensor 1
    putrsUSART("Throttle Sensor 1 Value: ");
    WriteIntUSART(ReadThrottle1());
    NewLineUSART();

    // Read throttle sensor 2
    putrsUSART("Throttle Sensor 2 Value: ");
    WriteIntUSART(ReadThrottle2());
    NewLineUSART();

    // Read brake sensor
    putrsUSART("Brake Sensor Value: ");
    WriteIntUSART(ReadBrake());
    NewLineUSART();

    // Read board temperature
    putrsUSART("Temperature Value: ");
    WriteIntUSART(ReadTemperature());
    NewLineUSART();

    // Read current value
    putrsUSART("Current Value: ");
    WriteIntUSART(ReadCurrent());
    NewLineUSART();
    NewLineUSART();

    // ************* Input Status *************
    putrsUSART(" ----- Input Status -----");
    NewLineUSART();

    // Display status - Tractive System
    putrsUSART("Tractive System: ");
    if(TS_Status == 1)  putrsUSART("ON");
    else                putrsUSART("OFF");
    NewLineUSART();

    // Display status - Charge signal
    putrsUSART("Charger: ");
    if(Charge == 1)  putrsUSART("ON");
    else                putrsUSART("OFF");
    NewLineUSART();
    NewLineUSART();

    // ************* Output Status *************
    putrsUSART(" ----- Output Status -----");
    NewLineUSART();

    // Display status - Ready to Drive Enable
    putrsUSART("Ready to Drive Enable: ");
    if(RTD_Enable == 1)  putrsUSART("ON");
    else       putrsUSART("OFF");
    NewLineUSART();

    // Display status - Ready to Drive Reset
    putrsUSART("Ready to Drive Reset: ");
    if(PORTAbits.RA6 == 1)  putrsUSART("ON");
    else       putrsUSART("OFF");
    NewLineUSART();

    // Display status - Tractive System Enable Signal
    putrsUSART("Tractive System Enable: ");
    if(PORTAbits.RA7 == 1)  putrsUSART("ON");
    else                     putrsUSART("OFF");
    NewLineUSART();

    // Display status - Fan Enable
    putrsUSART("GLVS Fan Enable: ");
    if(PORTCbits.RC2 == 1)  putrsUSART("ON");
    else       putrsUSART("OFF");
    NewLineUSART();
    NewLineUSART();

    // ************* Pedal System Test *************
    putrsUSART(" ----- Pedal System Test -----");
    NewLineUSART();

    // Check pedal system - returns 0 for implausibility
    putrsUSART("Pedal System Check: ");
    if(PedalSystemCheck() == 1)         putrsUSART("PASSED");
    else                                putrsUSART("FAILED");
    NewLineUSART();

    // Check sensor implausibility - returns 0 for implausibility
    putrsUSART("Throttle Sensor Implausibilty Check: ");
    if(SensorImplausibilityCheck() == 1)         putrsUSART("PASSED");
    else                                         putrsUSART("FAILED");
    NewLineUSART();

    // Check if sensor is within correct range - returns 0 out of range
    putrsUSART("Throttle Sensor 1 Range Check: ");
    if(Throttle1RangeCheck() == 1)               putrsUSART("PASSED");
    else                                         putrsUSART("FAILED");
    NewLineUSART();

    // Check if sensor is within correct range - returns 0 out of range
    putrsUSART("Throttle Sensor 2 Range Check: ");
    if(Throttle2RangeCheck() == 1)               putrsUSART("PASSED");
    else                                         putrsUSART("FAILED");
    NewLineUSART();

    // Check if sensor is within correct range - returns 0 out of range
    putrsUSART("Brake Sensor Range Check: ");
    if(BrakeRangeCheck() == 1)                   putrsUSART("PASSED");
    else                                         putrsUSART("FAILED");
    NewLineUSART();

    // Torque and brake encoder plausibility check - returns 0 for implausibility
    putrsUSART("Throttle and Brake Plausiblity Check: ");
    if(ThrottleBrakePlausibilityCheck() == 1)    putrsUSART("PASSED");
    else                                         putrsUSART("FAILED");
    NewLineUSART();


    // Checks if throttle can be reactivated after torque and brake encoder implausibility - returns 0 if error
    putrsUSART("Throttle 1 Reactivate Check: ");
    if(Throttle1ReactivateCheck() == 1)          putrsUSART("PASSED");
    else                                         putrsUSART("FAILED");
    NewLineUSART();
    NewLineUSART();
    
    // ************* Shutdown Circuit Status *************
    putrsUSART(" ----- Pedal System Test -----");
    NewLineUSART();
    
    // Shutdown circuit status - MCU enable
    putrsUSART("MCU Enable: ");
    if(SD_1 == 1)  putrsUSART("ON");
    else          putrsUSART("OFF");
    NewLineUSART();

    // Shutdown circuit status - BSPD
    putrsUSART("Brake System Plausibility Device: ");
    if(SD_2 == 1)  putrsUSART("ON");
    else          putrsUSART("OFF");
    NewLineUSART();

    // Shutdown circuit status - RSDB
    putrsUSART("Right Shutdown Button: ");
    if(SD_3 == 1)  putrsUSART("ON");
    else          putrsUSART("OFF");
    NewLineUSART();
    
    // Shutdown circuit status - LSDB
    putrsUSART("Left Shutdown Button: ");
    if(SD_4 == 1)  putrsUSART("ON");
    else          putrsUSART("OFF");
    NewLineUSART();

    // Shutdown circuit status - CSDB
    putrsUSART("Cockpit Shutdown Button: ");
    if(SD_5 == 1)  putrsUSART("ON");
    else          putrsUSART("OFF");
    NewLineUSART();

    // Shutdown circuit status -  Brake over-travel switch
    putrsUSART("Brake Over-Travel Switch: ");
    if(SD_6 == 1)  putrsUSART("ON");
    else          putrsUSART("OFF");
    NewLineUSART();

    // Shutdown circuit status - Inertia switch
    putrsUSART("Inertia Switch: ");
    if(SD_7 == 1)  putrsUSART("ON");
    else          putrsUSART("OFF");
    NewLineUSART();

    // Shutdown circuit status - High voltage disconnect
    putrsUSART("Interlocks: ");
    if(SD_8 == 1)  putrsUSART("ON");
    else          putrsUSART("OFF");
    NewLineUSART();

    // Shutdown circuit status - Battery management system
    putrsUSART("Battery Management System: ");
    if(SD_9 == 1)  putrsUSART("ON");
    else          putrsUSART("OFF");
    NewLineUSART();

    // Shutdown circuit status - Insulation monitoring device
    putrsUSART("Insulation Monitoring Device: ");
    if(SD_10 == 1)  putrsUSART("ON");
    else          putrsUSART("OFF");
    NewLineUSART();

    // Test 16 segment display

}
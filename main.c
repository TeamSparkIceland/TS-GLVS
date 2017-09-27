/*
 *  Project name:
 *      GLVS Main
 *  Author:
 *      Kristján Bjarki Purkhús
 *  Description:
 *      GLVS Main board using PIC18F4580
 *      The PIC is running at 32MHz by using an internal 8MHz crystal and the 4X Phase Locked Loop Function.
 *      Datasheet: http://ww1.microchip.com/downloads/en/devicedoc/39637c.pdfy.
 *  Notes:
 *
 */

#include "GLVS_Main_Driver.h"
#include "ECAN.h"

char display_error = 0;

void main()
{
    // Configures MCU
    ConfigMCU();
    Delay1Second();

    TS_Enable = 1;
    RTD_Reset = 0;
    RTD_Enable = 1;
    Fan_Enable = 1;

    while(1)
    {

        if(PedalSystemCheck() == FALSE) RTD_Enable = 0;
        else RTD_Enable = 1;

        Debug();
        Delay1Second();
        /*
        // Displays error on 16-segment display
        if(display_error)
        {
            // Reads error from shutdown circuit
            ReadSDError();

            // Display error if present
            DisplayError();
            display_error = 0;
            
            HomeUSART();
            Debug();
            
        }
        */


    }

}


void interrupt TimerOverflow()
{
    if(INTCONbits.TMR0IF == 1)
    {
        INTCONbits.TMR0IF = 0;
        display_error = 1;
        WriteTimer0(0x85ED);
    }

}






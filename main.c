//*****************************************************************************
//
// project.c - Simple project to use as a starting point for more complex
//             projects.
//
// Copyright (c) 2013-2017 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
//   Redistribution and use in source and binary forms, with or without
//   modification, are permitted provided that the following conditions
//   are met:
// 
//   Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the  
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// 
// This is part of revision 2.1.4.178 of the Tiva Firmware Development Package.
//
//*****************************************************************************





#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"

#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "driverlib/i2c.h"
#include "driverlib/ssi.h"
#include "driverlib/adc.h"
#include "driverlib/fpu.h"

#include "fatfs/src/ff.h"
#include "fatfs/src/diskio.h"


#include "utils/uartstdio.h"

#include "guru.h"







//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif








//*****************************************************************************
//
// Main Function
//
//*****************************************************************************
int
main(void)
{
 

    //
    // Enable lazy stacking for interrupt handlers.  This allows floating-point
    // instructions to be used within interrupt handlers, but at the expense of
    // extra stack usage.
    //
    FPULazyStackingEnable();

    //
    // Set the clocking to run directly from the crystal.
    //
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
                       SYSCTL_OSC_MAIN);


    //
    // Initialize the UART.
    //
    ConfigureUART();


    //
    // Clear screen and introduce yourself
    //
    UARTprintf("\033[2J");
    UARTprintf("Greenery Guru\n\n");
    
    //
    // Initialize Guru Interfaces
    //
    Guru_Init();
    //SysCtlDelay(SysCtlClockGet()/3);


    //
    // Power On Self Test 
    //
    #ifdef DEBUG
        Guru_POST();
    #endif
    //SysCtlDelay(SysCtlClockGet() );

    UARTIntEnable( UPSTREAM_UART, UART_INT_RX );
    UARTIntEnable( DNSTREAM_UART, UART_INT_RX );

    char loop_var='a';
    
    int16_t Tempature = 0;
    //float Tempature = 0;
    uint16_t Humidity = 0;
    int8_t Tempature2 = 0;
    uint16_t i2c_error_code=0;
    uint32_t brightness = 0;
    uint16_t moisture = 0;
    char up_char;
    char dn_char;
    uint8_t LED = 0b0000;

    I2CMasterErr(I2C_MASTER);

    



    UARTprintf("\033[2J");
    UARTprintf("Greenery Guru\n\n");
    
    /*
    iFResult = f_mount(0, &g_sFatFs);
    if(iFResult != FR_OK)
    {
        UARTprintf("f_mount error: %s\n", StringFromFResult(iFResult));
        //return(1);
    }
    else
    {
        UARTprintf("f_mount success!\n");
    }

    */


    GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3,
        0b0000);
    while(1)
    {
    GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3,
        0b1000);
        
UARTprintf("\033[2J");
        

        if (g_i2c_device_pres & DS1621_PRES)
        {
            i2c_error_code=DS1621Read(&Tempature2);
            g_total_transactions++;
        }

        I2C_Error_Check(i2c_error_code);

        if (g_i2c_device_pres & SI7006_PRES)
        {
            i2c_error_code=SI7006Read(&Tempature, &Humidity);
            g_total_transactions++;
        }

        I2C_Error_Check(i2c_error_code);


        //f_opendir(&g_sDirObject, g_pcCwdBuf);
        
        SSIDataPut(SPI_EEPROM, 0x9000 );
        //SSIDataGet(SPI_EEPROM, &brightness);
        
        
        //UARTCharPut(UPSTREAM_UART, loop_var);
        //if(UARTCharsAvail(UPSTREAM_UART))
        //    up_char=(char)UARTCharGet(UPSTREAM_UART);

        //UARTCharPutNonBlocking(DNSTREAM_UART, loop_var);
        //if(UARTCharsAvail(DNSTREAM_UART))
        //    dn_char=(char)UARTCharGet(DNSTREAM_UART);
        
        switch(loop_var){
            case 'z' :
                loop_var = 'A';
                break;
            case 'Z' :
                loop_var = 'a';
                break;
            default :
            loop_var++;
        }

        
        UARTprintf("Greenery Guru\n\n");
        UARTprintf("Tempature : %d C\n", Tempature);
        UARTprintf("Humidity : %d %%\n", Humidity);

        UARTprintf("Soil Tempature : %d C\n", Tempature2);
        UARTprintf("brightness : %d  \n", brightness);


        UARTprintf("UP_CHAR : %c\n", up_char);
        UARTprintf("DN_CHAR : %c\n", dn_char);

        ADCProcessorTrigger(ADC0_BASE,0);
        while(!ADCIntStatus(ADC0_BASE, 0, false)){}
        ADCSequenceDataGet(ADC0_BASE, 0, &brightness);
        
        brightness=CheckLightSensor();
        UARTprintf("ADC brightness : %4d\n", brightness);

        moisture=CheckMoistureSensor();
        UARTprintf("Soil Moisture : %4d\n", moisture);



        #ifdef DEBUG
            PrintCounters();
        #endif
        SysCtlDelay(SysCtlClockGet()/1);
    GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3,
        0b0000);


    }




}

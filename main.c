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
//! \addtogroup example_list
//! <h1>Simple Project (project)</h1>
//!
//! A very simple example that can be used as a starting point for more complex
//! projects.  Most notably, this project is fully TI BSD licensed, so any and
//! all of the code (including the startup code) can be used as allowed by that
//! license.
//!
//! The provided code simply toggles a GPIO using the Tiva Peripheral Driver
//! Library.
//
//*****************************************************************************

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
    // Enable the GPIO port that is used for the on-board LED.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    //
    // Enable the GPIO pins for the LED (PF2 & PF3).
    //
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);

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
    SysCtlDelay(SysCtlClockGet()/3);

    char loop_var='g';
    
    uint16_t Tempature = 0;
    uint16_t Humidity = 0;
    uint16_t Tempature2 = 0;
    int i2c_error_code=0;
    uint32_t brightness = 0;
    uint16_t moisture = 0;

    I2CMasterErr(I2C_MASTER);

    



    UARTprintf("\033[2J");
    UARTprintf("Greenery Guru\n\n");
    while(1)
    {

        
        //loop_var=UARTCharGet(DEBUG_UART);
        UARTCharPut(UART3_BASE, loop_var);
        loop_var=UARTCharGet(UART3_BASE);




        UARTCharPut(UART4_BASE, loop_var);
        loop_var=UARTCharGet(UART4_BASE);

        i2c_error_code=AM2320Read(&Tempature, &Humidity);

        g_total_transactions++;

        if (i2c_error_code)
        {
            g_error_counter++;

            switch (i2c_error_code)
            {
                case I2C_MASTER_ERR_ADDR_ACK : g_err_addr_ack++;
                case I2C_MASTER_ERR_DATA_ACK : g_err_data_ack++;
                case I2C_MASTER_ERR_ARB_LOST : g_err_arb_lost++;
                case I2C_MASTER_ERR_CLK_TOUT : g_err_clk_tout++;
            }
        }

        i2c_error_code=DS1621Read(&Tempature2);

        g_total_transactions++;

        if (i2c_error_code)
        {
            g_error_counter++;

            switch (i2c_error_code)
            {
                case I2C_MASTER_ERR_ADDR_ACK : g_err_addr_ack++;
                case I2C_MASTER_ERR_DATA_ACK : g_err_data_ack++;
                case I2C_MASTER_ERR_ARB_LOST : g_err_arb_lost++;
                case I2C_MASTER_ERR_CLK_TOUT : g_err_clk_tout++;
            }
        }

        SSIDataPut(SSI0_BASE, 0x9000 );
        SSIDataGet(SSI0_BASE, &brightness);


        UARTprintf("\033[3;0H");
        UARTprintf("Tempature : %d C  \nHumidity : %d %%  \nI2c_Master : %x  \n"
            ,Tempature/10, Humidity/10, i2c_error_code );
        UARTprintf("Tempature2 : %d C\n", Tempature2);
        UARTprintf("brightness : %d  \n", brightness);

        ADCProcessorTrigger(ADC0_BASE,0);
        while(!ADCIntStatus(ADC0_BASE, 0, false)){}
        ADCSequenceDataGet(ADC0_BASE, 0, &brightness);
        
        brightness=CheckLightSensor();
        UARTprintf("ADC brightness : %4d\n", brightness);

        moisture=CheckMoistureSensor();
        UARTprintf("moisture : %4d\n", moisture);

        PrintCounters();
        //SysCtlDelay(SysCtlClockGet() / 600);


    }




}

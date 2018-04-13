// guru.c

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

#include "utils/uartstdio.h"


#include "guru.h"




void Guru_Init(void)
{

    //
    // Set the clocking to run directly from the crystal.
    //
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
                       SYSCTL_OSC_MAIN);

    //
    // Coinfigure LED 
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);

    //
    // Enable processor interrupts.
    //
    IntMasterEnable();

    //
    // Configure UART0 As Debug UART
    // This inteface is used to report actions taken by the device
    //

//    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
//    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
//    
//    GPIOPinConfigure(GPIO_PA0_U0RX);
//    GPIOPinConfigure(GPIO_PA1_U0TX);
//    
//    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
//    UARTConfigSetExpClk(DEBUG_UART, SysCtlClockGet(), BAUD_RATE,
//        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
//		
//    UARTEnable(DEBUG_UART);
//
//    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
//    UARTStdioConfig(DEBUG_UART, BAUD_RATE, SysCtlClockGet());


    //
    // Configure upstream and down stream UART interfaces
    // UART 4 : Upstream 
    // UART 3 : Downstream
    // These interfaces are used to configure the device by a master device
    //

    // UART 4 : Upstream UART
   SysCtlPeripheralEnable(SYSCTL_PERIPH_UART4);
   SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
   GPIOPinConfigure(GPIO_PC4_U4RX);
   GPIOPinConfigure(GPIO_PC5_U4TX);
   GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);
   UARTConfigSetExpClk(UPSTREAM_UART, SysCtlClockGet(), 115200,
       (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
   UARTEnable(UPSTREAM_UART);


   // UART 3 : Downstream UART
   SysCtlPeripheralEnable(SYSCTL_PERIPH_UART3);
   SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
   GPIOPinConfigure(GPIO_PC6_U3RX);
   GPIOPinConfigure(GPIO_PC7_U3TX);
   GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_6 | GPIO_PIN_7);
   UARTConfigSetExpClk(DNSTREAM_UART, SysCtlClockGet(), 115200,
       (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
   UARTEnable(DNSTREAM_UART);

    //
    // Configure I2C 0 as Slave Bus
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);
    I2CSlaveAddressSet(I2C_SLAVE, 1, 0xA5);
    I2CSlaveEnable(I2C_SLAVE);



    //
    // Configure I2C 1 as Master Bus
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinConfigure(GPIO_PA6_I2C1SCL);
    GPIOPinConfigure(GPIO_PA7_I2C1SDA);
    GPIOPinTypeI2CSCL(GPIO_PORTA_BASE, GPIO_PIN_6);
    GPIOPinTypeI2C(GPIO_PORTA_BASE, GPIO_PIN_7);
    I2CMasterInitExpClk(I2C_MASTER, SysCtlClockGet(), false);
    I2CMasterEnable(I2C_MASTER);


    //
    // Configure SPI Bus to EEPROM
    //

    //
    // Configure SPI Bus to Micro SD slot
    //

}


void ConfigureUART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);
}



int AM2320Read(uint16_t *p_tempature, uint16_t *p_humidity)
{
		uint16_t Tempature;
    	uint16_t Humidity;
	    
	    //
        // Wake Sensor
        //
        while(I2CMasterBusy(I2C_MASTER)){}
        I2CMasterSlaveAddrSet(I2C_MASTER, AM2320, false);
        while(I2CMasterBusy(I2C_MASTER)){}
        I2CMasterDataPut(I2C_MASTER, 0);
        while(I2CMasterBusy(I2C_MASTER)){}
        I2CMasterControl(I2C_MASTER, I2C_MASTER_CMD_SINGLE_SEND);        
        
        SysCtlDelay(SysCtlClockGet() / 4000);

        while(I2CMasterBusy(I2C_MASTER)){}
        I2CMasterDataPut(I2C_MASTER, 0x03);
        while(I2CMasterBusy(I2C_MASTER)){}
        I2CMasterControl(I2C_MASTER, I2C_MASTER_CMD_BURST_SEND_START);
        while(I2CMasterBusy(I2C_MASTER)){}
        I2CMasterDataPut(I2C_MASTER, 0x00);
        while(I2CMasterBusy(I2C_MASTER)){}
        I2CMasterControl(I2C_MASTER, I2C_MASTER_CMD_BURST_SEND_CONT);
        while(I2CMasterBusy(I2C_MASTER)){}
        I2CMasterDataPut(I2C_MASTER, 0x04);
        while(I2CMasterBusy(I2C_MASTER)){}
        I2CMasterControl(I2C_MASTER, I2C_MASTER_CMD_BURST_SEND_FINISH);

        SysCtlDelay(SysCtlClockGet() / 4000);

        while(I2CMasterBusy(I2C_MASTER)){}
        I2CMasterSlaveAddrSet(I2C_MASTER, AM2320, true);
        while(I2CMasterBusy(I2C_MASTER)){}
        I2CMasterControl(I2C_MASTER, I2C_MASTER_CMD_BURST_RECEIVE_START);
        while(I2CMasterBusy(I2C_MASTER)){}
        I2CMasterControl(I2C_MASTER, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
        while(I2CMasterBusy(I2C_MASTER)){}
        I2CMasterControl(I2C_MASTER, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
        while(I2CMasterBusy(I2C_MASTER)){}
        Humidity = I2CMasterDataGet(I2C_MASTER);
        while(I2CMasterBusy(I2C_MASTER)){}
        I2CMasterControl(I2C_MASTER, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
        while(I2CMasterBusy(I2C_MASTER)){}
        Humidity = Humidity << 8; 
        Humidity+=I2CMasterDataGet(I2C_MASTER);
        while(I2CMasterBusy(I2C_MASTER)){}
        I2CMasterControl(I2C_MASTER, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
        while(I2CMasterBusy(I2C_MASTER)){}
        Tempature = I2CMasterDataGet(I2C_MASTER);
        while(I2CMasterBusy(I2C_MASTER)){}
        I2CMasterControl(I2C_MASTER, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
        while(I2CMasterBusy(I2C_MASTER)){}
        Tempature = Tempature << 8; 
        Tempature += I2CMasterDataGet(I2C_MASTER);
        while(I2CMasterBusy(I2C_MASTER)){}
        
        *p_humidity=Humidity;
    	*p_tempature=Tempature;

        return I2CMasterErr(I2C_MASTER);

}
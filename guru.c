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
#include "driverlib/adc.h"

#include "utils/uartstdio.h"


#include "guru.h"


void Guru_Init(void)
{

    g_error_counter=0;
    g_err_addr_ack=0;
    g_err_data_ack=0;
    g_err_arb_lost=0;
    g_err_clk_tout=0;
    g_total_transactions=0;

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
    // Configure upstream and down stream UART interfaces
    // UART 4 : Upstream 
    // UART 3 : Downstream
    // These interfaces are used to configure the device by a master device
    //

	UARTprintf("Configuring Upstream UART...");
    // UART 4 : Upstream UART
   SysCtlPeripheralEnable(SYSCTL_PERIPH_UART4);
   SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
   GPIOPinConfigure(GPIO_PC4_U4RX);
   GPIOPinConfigure(GPIO_PC5_U4TX);
   GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);
   UARTConfigSetExpClk(UPSTREAM_UART, SysCtlClockGet(), 115200,
       (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
   UARTEnable(UPSTREAM_UART);
	UARTprintf("DONE!\n");

	UARTprintf("Configuring Downstream UART...");
   // UART 3 : Downstream UART
   SysCtlPeripheralEnable(SYSCTL_PERIPH_UART3);
   SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
   GPIOPinConfigure(GPIO_PC6_U3RX);
   GPIOPinConfigure(GPIO_PC7_U3TX);
   GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_6 | GPIO_PIN_7);
   UARTConfigSetExpClk(DNSTREAM_UART, SysCtlClockGet(), 115200,
       (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
   UARTEnable(DNSTREAM_UART);
	UARTprintf("DONE!\n");


	UARTprintf("Configuring Slave I2C insterface...");
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
	UARTprintf("DONE!\n");

	UARTprintf("Configuring Master I2C insterface...");
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
	UARTprintf("DONE!\n");

	UARTprintf("Configuring EEPROM Interface...");
    //
    // Configure SPI Bus to EEPROM
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinConfigure(GPIO_PA2_SSI0CLK);
    GPIOPinConfigure(GPIO_PA3_SSI0FSS);
    GPIOPinConfigure(GPIO_PA4_SSI0RX);
    GPIOPinConfigure(GPIO_PA5_SSI0TX);
    GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 |
    	GPIO_PIN_5);
    SSIConfigSetExpClk(SPI_EEPROM, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, 
                        SSI_MODE_MASTER, 2000000, 16);
    SSIAdvModeSet(SPI_EEPROM, SSI_ADV_MODE_LEGACY);
    SSIDMAEnable(SPI_EEPROM, SSI_DMA_TX | SSI_DMA_RX);
    SSIEnable(SPI_EEPROM); 
	UARTprintf("DONE!\n");

	UARTprintf("Configuring SD Card Interface...");
    //
    // Configure SPI Bus to Micro SD slot
    //
	UARTprintf("DONE!\n");


	UARTprintf("Configuring Light Sensor...");
    //
    // Configure ADC for light sensor
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3 );
    ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH0 |
    	ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 0);
    ADCIntClear(ADC0_BASE, 0);
	UARTprintf("DONE!\n");


	UARTprintf("Configuring moiture Sensor...");
    //
    // Configure ADC for moisture sensor
    //
    //SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
    //SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    //GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2);
    //ADCSequenceConfigure(ADC1_BASE, 0, ADC_TRIGGER_PROCESSOR, 0);
    //ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH1 |
    //	ADC_CTL_IE | ADC_CTL_END);
    //ADCSequenceEnable(ADC1_BASE, 0);
    //ADCIntClear(ADC1_BASE, 0);
	
    //
    //	Configure Driver Moisture sensor
    //

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_1);
    GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_2);




	UARTprintf("DONE!\n");




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



int DS1621Read(uint16_t *p_tempature)
{

	uint16_t Tempature;


    I2CMasterSlaveAddrSet(I2C_MASTER, DS1621, false);
    while(I2CMasterBusy(I2C_MASTER)){}
    I2CMasterDataPut(I2C_MASTER, DS1621_START_CONVERT);
    while(I2CMasterBusy(I2C_MASTER)){}
    I2CMasterControl(I2C_MASTER, I2C_MASTER_CMD_SINGLE_SEND);

	while(I2CMasterBusy(I2C_MASTER)){}
    I2CMasterDataPut(I2C_MASTER, DS1621_STOP_CONVERT);
    while(I2CMasterBusy(I2C_MASTER)){}
    I2CMasterControl(I2C_MASTER, I2C_MASTER_CMD_SINGLE_SEND);
	
	while(I2CMasterBusy(I2C_MASTER)){}
    I2CMasterDataPut(I2C_MASTER, DS1621_READ_TEMP);
    while(I2CMasterBusy(I2C_MASTER)){}
    I2CMasterControl(I2C_MASTER, I2C_MASTER_CMD_SINGLE_SEND);


    while(I2CMasterBusy(I2C_MASTER)){}
    I2CMasterSlaveAddrSet(I2C_MASTER, DS1621, true);
    while(I2CMasterBusy(I2C_MASTER)){}
    I2CMasterControl(I2C_MASTER, I2C_MASTER_CMD_BURST_RECEIVE_START);
    while(I2CMasterBusy(I2C_MASTER)){}
    Tempature = I2CMasterDataGet(I2C_MASTER);
    while(I2CMasterBusy(I2C_MASTER)){}
    I2CMasterControl(I2C_MASTER, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
    while(I2CMasterBusy(I2C_MASTER)){}
    //Tempature = Tempature << 8; 
    //Tempature += I2CMasterDataGet(I2C_MASTER);
    while(I2CMasterBusy(I2C_MASTER)){}
    
    *p_tempature=Tempature;

    return I2CMasterErr(I2C_MASTER);
}



uint32_t CheckLightSensor()
{
	uint32_t n=10000;
	uint32_t sum=0;

	for (int i = 0; i < n; ++i)
	{
		//
    	// Trigger the ADC conversion.
    	//
    	ADCProcessorTrigger(ADC0_BASE, 0);

    	//
    	// Wait for conversion to be completed.
    	//
    	while(!ADCIntStatus(ADC0_BASE, 0, false)){}

    	//
    	// Read ADC Value.
    	//
    	ADCSequenceDataGet(ADC0_BASE, 0, &pui32ADC0Value);

		sum+=pui32ADC0Value;
	}
	


	return sum/n;
}




uint16_t CheckMoistureSensor()
{

	uint16_t moiture;

	// set drive pin high
	GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, GPIO_PIN_1);

	for (int i = 0; i <= 10000; ++i)
	{
		moiture = i;

		// check status of sense pin
		if (GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_2))
		{
			break;
		}

		//SysCtlDelay(1000);

	}

	// set drive pin low
	GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, false);

	return moiture;

}




int PrintCounters()
{	
	

	UARTprintf("\n\n############### Counters ###################\n");
    UARTprintf("Total Transactions \t: \t%d  \n", g_total_transactions);
    UARTprintf("Total Errors \t\t: \t%d  \n", g_error_counter);
    UARTprintf("Address ACK \t\t: \t%d  \n", g_err_addr_ack);
    UARTprintf("Data ACK \t\t: \t%d  \n", g_err_data_ack);
    UARTprintf("Arb Lost \t\t: \t%d  \n", g_err_arb_lost);
    UARTprintf("CLK Timeout \t\t: \t%d  \n", g_err_clk_tout);
    


    return 0;

}
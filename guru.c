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

#include "fatfs/src/ff.h"
#include "fatfs/src/diskio.h"

#include "guru.h"


void Guru_Init(void)
{


    DEBUG_PRINT("############### DEBUG Build ##############\n");


    g_error_counter=0;
    g_err_addr_ack=0;
    g_err_data_ack=0;
    g_err_arb_lost=0;
    g_err_clk_tout=0;
    g_total_transactions=0;

    //
    // Coinfigure LEDS 
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, 
        GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);

    //
    // Enable processor interrupts.
    //
    IntMasterEnable();


    //
    // Configure upstream and down stream UART interfaces
    // UART 4 : Upstream 
    // UART 6 : Downstream
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
   UARTIntRegister(UPSTREAM_UART,&Up_UART_Hand);
	UARTprintf("DONE!\n");


    UARTprintf("Configuring Downstream UART...");
   // UART 6 : Downstream UART
   SysCtlPeripheralEnable(SYSCTL_PERIPH_UART6);
   SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
   GPIOPinConfigure(GPIO_PD4_U6RX);
   GPIOPinConfigure(GPIO_PD5_U6TX);
   GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_4 | GPIO_PIN_5);
   UARTConfigSetExpClk(DNSTREAM_UART, SysCtlClockGet(), 115200,
       (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
   UARTEnable(DNSTREAM_UART);
   UARTIntRegister(DNSTREAM_UART,&Dn_UART_Hand);
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

	UARTprintf("Configuring SD Card Interface...");
    //
    // Configure SPI Bus to the SDCARD
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinConfigure(GPIO_PA2_SSI0CLK);
    GPIOPinConfigure(GPIO_PA3_SSI0FSS);
    GPIOPinConfigure(GPIO_PA4_SSI0RX);
    GPIOPinConfigure(GPIO_PA5_SSI0TX);
    GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 |
    	GPIO_PIN_5);
    SSIConfigSetExpClk(SPI_SD_CARD, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, 
                        SSI_MODE_MASTER, 2000000, 16);
    SSIAdvModeSet(SPI_SD_CARD, SSI_ADV_MODE_LEGACY);
    SSIDMAEnable(SPI_SD_CARD, SSI_DMA_TX | SSI_DMA_RX);
    SSIEnable(SPI_SD_CARD); 
	UARTprintf("DONE!\n");



	UARTprintf("Configuring EEPROM Interface...");
    
    // Configure SPI Bus to EEPROM
    
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI2);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    GPIOPinConfigure(GPIO_PB4_SSI2CLK);
    GPIOPinConfigure(GPIO_PB5_SSI2FSS);
    GPIOPinConfigure(GPIO_PB6_SSI2RX);
    GPIOPinConfigure(GPIO_PB7_SSI2TX);
    GPIOPinTypeSSI(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 |
                    	GPIO_PIN_7);
    SSIConfigSetExpClk(SPI_EEPROM, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, 
                        SSI_MODE_MASTER, 2000000, 16);
    SSIAdvModeSet(SPI_EEPROM, SSI_ADV_MODE_LEGACY);
    SSIDMAEnable(SPI_EEPROM, SSI_DMA_TX | SSI_DMA_RX);
    SSIEnable(SPI_EEPROM); 
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


int I2C_Scan(void)
{



    int devices_found = 0;

    UARTprintf("Scanning I2C Bus\n\n");

    for (int i = 1; i < 128; ++i)
    {


        while(I2CMasterBusy(I2C_MASTER)){}
        I2CMasterSlaveAddrSet(I2C_MASTER, i, false);
        while(I2CMasterBusy(I2C_MASTER)){}
        I2CMasterDataPut(I2C_MASTER, 0);
        while(I2CMasterBusy(I2C_MASTER)){}
        I2CMasterControl(I2C_MASTER, I2C_MASTER_CMD_SINGLE_SEND);  
        while(I2CMasterBusy(I2C_MASTER)){}

        if (!(I2CMasterErr(I2C_MASTER) & I2C_MASTER_ERR_ADDR_ACK))
        {
            devices_found++;
            SysCtlDelay(SysCtlClockGet()/5);

            switch(i)
            {
                case DS1621:
                    UARTprintf("DS1621 discovered! 0x%x\n",i);
                    g_i2c_device_pres = g_i2c_device_pres | DS1621_PRES;
                    break;
                case SI7006:
                    UARTprintf("SI7006 discovered! 0x%x\n",i);
                    g_i2c_device_pres = g_i2c_device_pres | SI7006_PRES;
                    break;
                case AM2320:
                    UARTprintf("AM2320 discovered! 0x%x\n",i);
                    g_i2c_device_pres = g_i2c_device_pres | AM2320_PRES;
                    break;
                default:
                UARTprintf("unknown device found\t0x%x \n", i);
            }
        

        }


    }

    UARTprintf("%d devices found\n", devices_found);
    UARTprintf("Devices : 0x%x", g_i2c_device_pres);

    return devices_found;
}

void I2C_Error_Check(int error_code)
{

         if (error_code)
        {
            g_error_counter++;

            if(error_code)
            {
                if(error_code & I2C_MASTER_ERR_ADDR_ACK)
                    g_err_addr_ack++;
                if(error_code & I2C_MASTER_ERR_DATA_ACK)
                    g_err_data_ack++;
                if(error_code & I2C_MASTER_ERR_ARB_LOST)
                    g_err_arb_lost++;
                if(error_code & I2C_MASTER_ERR_CLK_TOUT)
                    g_err_clk_tout++;

            }
        }


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



int AM2320Read(int16_t *p_tempature, uint16_t *p_humidity)
{
	int16_t Tempature;
    uint16_t Humidity;

    //
    // Wake Sensor
    //
    while(I2CMasterBusy(I2C_MASTER)){}
    I2CMasterSlaveAddrSet(I2C_MASTER, AM2320, I2C_WRITE);
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
    I2CMasterSlaveAddrSet(I2C_MASTER, AM2320, I2C_READ);
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




int SI7006Read(int16_t *p_tempature, uint16_t *p_humidity)
{

    int16_t Tempature = 0;
    uint16_t Humidity = 0;


    //
    // Reset Sensor before measuriment
    // 
    //while(I2CMasterBusy(I2C_MASTER)){}
    //if (I2CMasterErr(I2C_MASTER)){return I2CMasterErr(I2C_MASTER);}
    //I2CMasterSlaveAddrSet(I2C_MASTER, SI7006, I2C_WRITE);
    //while(I2CMasterBusy(I2C_MASTER)){}
    //if (I2CMasterErr(I2C_MASTER)){return I2CMasterErr(I2C_MASTER);}
    //I2CMasterDataPut(I2C_MASTER, SI7006_RESET);
    //while(I2CMasterBusy(I2C_MASTER)){}
    //if (I2CMasterErr(I2C_MASTER)){return I2CMasterErr(I2C_MASTER);}
    //I2CMasterControl(I2C_MASTER, I2C_MASTER_CMD_SINGLE_SEND); 

    //
    // Make Humididity Measurement 
    //
    while(I2CMasterBusy(I2C_MASTER)){}
    if (I2CMasterErr(I2C_MASTER)){}
    I2CMasterSlaveAddrSet(I2C_MASTER, SI7006, I2C_WRITE);
    while(I2CMasterBusy(I2C_MASTER)){}
    if (I2CMasterErr(I2C_MASTER)){}
    I2CMasterDataPut(I2C_MASTER, SI7006_MEASURE_RH_HOLD);
    while(I2CMasterBusy(I2C_MASTER)){}
    I2CMasterControl(I2C_MASTER, I2C_MASTER_CMD_BURST_RECEIVE_START);        

    //
    // Read Humidity Value
    //
    while(I2CMasterBusy(I2C_MASTER)){}
    I2CMasterSlaveAddrSet(I2C_MASTER, SI7006, I2C_READ);
    while(I2CMasterBusy(I2C_MASTER)){}
    I2CMasterControl(I2C_MASTER, I2C_MASTER_CMD_BURST_RECEIVE_START);
    while(I2CMasterBusy(I2C_MASTER)){}
    Humidity = I2CMasterDataGet(I2C_MASTER);
    while(I2CMasterBusy(I2C_MASTER)){}
    I2CMasterControl(I2C_MASTER, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);

    while(I2CMasterBusy(I2C_MASTER)){}
    Humidity = Humidity << 8; 
    Humidity+=I2CMasterDataGet(I2C_MASTER);
    

    //
    // Make Tempature Measurement 
    //
    while(I2CMasterBusy(I2C_MASTER)){}
    I2CMasterSlaveAddrSet(I2C_MASTER, SI7006, I2C_WRITE);
    while(I2CMasterBusy(I2C_MASTER)){}
    I2CMasterDataPut(I2C_MASTER, SI7006_READ_TEMP);
    while(I2CMasterBusy(I2C_MASTER)){}
    I2CMasterControl(I2C_MASTER, I2C_MASTER_CMD_BURST_RECEIVE_START);        

    //
    // Read Tempature Value
    // 
    while(I2CMasterBusy(I2C_MASTER)){}
    I2CMasterSlaveAddrSet(I2C_MASTER, SI7006, I2C_READ);
    while(I2CMasterBusy(I2C_MASTER)){}
    I2CMasterControl(I2C_MASTER, I2C_MASTER_CMD_BURST_RECEIVE_START);
    while(I2CMasterBusy(I2C_MASTER)){}
    Tempature = I2CMasterDataGet(I2C_MASTER);
    while(I2CMasterBusy(I2C_MASTER)){}
    I2CMasterControl(I2C_MASTER, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
    while(I2CMasterBusy(I2C_MASTER)){}
    Tempature = Tempature << 8; 
    Tempature += I2CMasterDataGet(I2C_MASTER);

    *p_humidity=(( 125 * Humidity ) / 65536 ) - 6 ;
    *p_tempature=(( 175.72 * (float)Tempature ) / 65536.0 ) - 46.85;
    //*p_tempature=(( Tempature ));

    //*p_humidity= 56;

    //*p_tempature= 22.5;
    
    return I2CMasterErr(I2C_MASTER);

}




int DS1621Read(int8_t *p_tempature)
{

	int8_t Tempature;


    I2CMasterSlaveAddrSet(I2C_MASTER, DS1621, I2C_WRITE);
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

    I2CMasterSlaveAddrSet(I2C_MASTER, DS1621, I2C_READ);
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

int Guru_POST()
{

    //
    // Clear screen before running POST
    //
    UARTprintf("\033[2J");

    UARTprintf("\n\n############### Begin POST ###################\n");

    UARTprintf("UP stream UART Loopback test -> ");
    if(UART_Loopback(UPSTREAM_UART))
        UARTprintf("Pass!\n");
    else
        UARTprintf("Fail!\n");

    UARTprintf("Down stream UART Loopback test -> ");
        if(UART_Loopback(DNSTREAM_UART))
        UARTprintf("Pass!\n");
    else
        UARTprintf("Fail!\n");

    I2C_Scan();
}

bool UART_Loopback(uint32_t UARTBus)
{
    char testchar = 'a';
    for (int i = 0; i < 51; ++i)
    {


        UARTCharPutNonBlocking(UARTBus, testchar);
        while(!UARTCharsAvail(UARTBus)){}
            if(testchar!=(char)UARTCharGet(UARTBus))
                return false;

        switch(testchar){
            case 'z' :
                testchar = 'A';
                break;
            case 'Z' :
                testchar = 'a';
                break;
            default :
            testchar++;
        }
    }

return true;
}


void Up_UART_Hand()
{
    GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_0, 0xF);
    char tempchar;
    char command[8];
    int i = 0;
    
    while(UARTCharsAvail(UPSTREAM_UART))
    {
        tempchar = UARTCharGet(UPSTREAM_UART);
        UARTCharPut(DNSTREAM_UART, tempchar);
        command[i]=tempchar;
        i++;
    }
    
    if(strcmp(command,"setA5\r\n"))
    {
        GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_2, 0xF);
        UARTprintf("Lets gooo!!!!");
        SysCtlDelay(SysCtlClockGet() );
    }
    else if (command == "pinall")
    {
        UARTPingChain();
    }
            

    UARTIntClear( UPSTREAM_UART , 0xFFF );
    //GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_0, 0x0);
    return;
}


void Dn_UART_Hand()
{
    GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_1, 0xf);
    char tempchar;

    while(UARTCharsAvail(DNSTREAM_UART))
    {
        tempchar = UARTCharGet(DNSTREAM_UART);
        UARTCharPut(UPSTREAM_UART, tempchar);
    }

    UARTIntClear( DNSTREAM_UART , 0xFFF );
    //GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_1, 0x0);
    return;
}

void UARTPingChain()
{
    UARTprintf("PING ALL");
    SysCtlDelay(SysCtlClockGet() );

    return;
}
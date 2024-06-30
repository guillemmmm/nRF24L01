/*
 * ------------------------------------------------------------
 * Author: Guillem Prenafeta (UB)
 * Modification Date: June 2024
 * License: GNU General Public License (GPL)
 * ------------------------------------------------------------
 * Description:
 * FULL DUPLEX slave code example.
 *
 * The GPL license allows free distribution and modification
 * of the software, as long as the same license is maintained
 * in derivative versions.
 * ------------------------------------------------------------
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "nRF24L01lib.h"

//--- DRIVERLIB libraries only for TIVA-C ---//
#include "inc/hw_memmap.h"
#include <inc/hw_gpio.h>
#include <inc/hw_types.h>
#include "driverlib/sysctl.h" //libreria del sistema
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "inc/hw_ints.h"
#include "driverlib/uart.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "driverlib/ssi.h"
#include "inc/hw_ssi.h"
// ------------------------------------------//

// Functions to be defined::  //TODO depends on your HW and abstraction lvl
/*
 * void SPI_send (uint8_t count, uint8t *TxBuf, uint8_t *RxBuf);  
 * void nRF24L01_CSN(bool state);
 * void nRF24L01_CE(bool state);
 * void delay100us(uint32_t ticks);
 *
 * In this example TIVA-C will be used. Using TIVAWARE library to programm the functions
*/

#define RF_CH 0x07 // RF Chanel, to choose by the user (freq CH 2400 MHz + RF_CH (7) == 2407 MHz)

// HW functions to init clocks, and init 3-wire SPI and GPIOs, 2 output and 1 input
void init_HW(void);
void ISRtimerDelay(void);

// HW functions to controll nRF24
void SPI_send (uint8_t count, uint8t *TxBuf, uint8_t *RxBuf);
void nRF24L01_CSN(bool state);
void nRF24L01_CE(bool state);
void delay100us(uint32_t ticks);

bool RADIOint;
uint32_t payloadRXLEN;
uint8_t payloadRX[32];

void main()
{

	init_HW(); //init_all

	nRF24L01_CE(false); //set CE to disable
	nRF24L01_CSN(true); //SPI disable

	RADIOint = false;

	nRF24L01_init(); //we use pipe0 communicaiton, default adress
    nRF24L01_powerUP(PTRmode); //RX mode

        //escribim resposta inicial
    char array[] = "RESPOSTA HOLA MUNDO"
    nRF24L01_write_tx_payload_PRX((uint8_t*)array, sizeof(array), 0); //pipe 0

	while(1)
	{

		//missatge
		nRF24L01_write_tx_payload_PTX((uint8_t*)array, sizeof(array), true); //enviem
		while(RADIOint==false){ //esperem int
		}
		RADIOint=false;
		uint8_t status = nRF24L01_nop();
		if((status&nrf24l01_STATUS_TX_DS)==nrf24l01_STATUS_TX_DS){ //si es que s'ha enviat correctament
			//llegim missatge
			do{
                nRF24L01_payload_width(&payloadRXLEN); //obtenim longitud del missatge ja que es variable
                nRF24L01_read_rx_payload(payloadRX, payloadRXLEN); //i fem processament
            }while(!nRF24L01_FIFO_RX_empty()); //mentres estigui plena (quan buida passa a true)
            //ja tenim les dades
    		
    		nRF24L01_write_tx_payload_PRX((uint8_t*)array, sizeof(array), 0); //pipe 0 //posem resposta
		}	
		nRF24L01_clear_IRQ();
}



//-----------------------------------------------------------------------------------------------------------//
//  FUNCTIONS
//-----------------------------------------------------------------------------------------------------------//

void nRF24ISR(void){

	GPIOIntClear(GPIO_PORTC_BASE, GPIO_INT_PIN_5);
    nRF24L01_CE(false);

    RADIOint=true; //indiquem que ha saltat int
}

void init_HW(void)
{

	SysCtlClockSet(
            SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ); //50 MHz


	//Timer for nRF24lib (period 100us)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER1))
    {
        __nop();
    }
    TimerClockSourceSet(TIMER1_BASE, TIMER_CLOCK_PIOSC); //16MHz on 100us==16000000*100*10**-6
    TimerConfigure(TIMER1_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PERIODIC);
    TimerLoadSet(TIMER1_BASE, TIMER_A, SysCtlClockGet()/10000); //100us
    IntRegister(INT_TIMER1A, ISRtimerDelay); //enlacem rutina interrrupcio
    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

    //init SPI and GPIO pins
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB))
    {
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    }
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC))
    {
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    }
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA))
    {
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    }
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE))
    {
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    }
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_SSI2))
    {
        SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI2);
    }
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_3); //PA3 SPI_CS
    GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_6);  //PC6 nRF24 CE i PC5 int nRF24
    GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, GPIO_PIN_5); //a canviar (no fem servir)
    GPIOIntRegister(GPIO_PORTC_BASE, nRF24ISR); // TODO // rutina interrupcio modul radio
    GPIOIntTypeSet(GPIO_PORTC_BASE, GPIO_PIN_5, GPIO_FALLING_EDGE); //negedge interrupcio
    GPIOIntEnable(GPIO_PORTC_BASE, GPIO_PIN_5); //habilitem ints de nRF24
    IntEnable(INT_GPIOC);

    nRF24L01_CE(false); //set CE to disable
	nRF24L01_CSN(true); //SPI disable

	GPIOPinConfigure(GPIO_PB4_SSI2CLK); //posar els pins q es fan servir
    GPIOPinConfigure(GPIO_PB6_SSI2RX);
    GPIOPinConfigure(GPIO_PB7_SSI2TX);
    GPIOPinTypeSSI(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_6 | GPIO_PIN_7); //posar pins SPI
    SSIConfigSetExpClk(SSI2_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, 5000000, 8); //5MHz
    SSIEnable(SSI2_BASE);
    // FEM servir SPI sense interrupcions
}

uint32_t TCount;
void delay100us(uint32_t ticks)
{
    TCount=0;
    TimerLoadSet(TIMER1_BASE, TIMER_A, SysCtlClockGet()/10000); //100us
    IntEnable(INT_TIMER1A);
    TimerEnable(TIMER1_BASE, TIMER_A);
    while (TCount < ticks)
    {
    }
    TimerDisable(TIMER1_BASE, TIMER_A);
    IntDisable(INT_TIMER1A);
}

void ISRtimerDelay(void)
{
    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT); //netejem flag
    TCount++;
}

//SSI function without using interrupts
void SPI_send(uint8_t count, uint8_t * dataout, uint8_t * datain)
{
    uint8_t countR = 0;
    uint32_t var;

    while(countR<count){
        SSIDataPutNonBlocking(SSI2_BASE, *(dataout+countR)); //posem la dada
        while((HWREG(SSI2_BASE + SSI_O_SR) & SSI_SR_BSY)); //esperem a que es buidi Tx
        if(datain==NULL){
            SSIDataGetNonBlocking(SSI2_BASE, &var);
        } else{
            SSIDataGetNonBlocking(SSI2_BASE, (uint32_t*)&(*(datain+countR)));
        }
    }
}

void nRF24L01_CE(bool state)
{
    if(state){
        GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, GPIO_PIN_6);
    } else{
        GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0);
    }
}

void nRF24L01_CSN(bool state)
{
    if(state){
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
    } else{
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);
    }
}
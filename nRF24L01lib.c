/* nRF24L01 library by Guillem Prenafeta */

/* SPI library (Library provided by the user micrcontroller). Important functions
*
*	SPI_send (uint8_t count, uint8t *TxBuf, uint8_t *RxBuf);
*/
#include "nRF24L01lib.h"
//#include <xdc/runtime/System.h>
//#include <xdc/cfg/global.h>

#include "mkii.h"

//#include <ti/drivers/GPIO.h>
//#include "ti_drivers_config.h" //myboard

#define RF_CH 0x07

//#define  nRF24L01_CSN(false) GPIO_write(CSN,0)
//#define  nRF24L01_CSN(true) GPIO_write(CSN,1)
//#define  nRF24L01_CE(false) GPIO_write(CE,0)
//#define  nRF24L01_CE(true) GPIO_write(CE,1)

//-----------------------------------------------------------------------------------------------------------//
void nRF24L01_init(void)
{

	uint8_t data;

	data=nrf24l01_EN_AA_ENAA_P0;
	nRF24L01_writeReg(nrf24l01_EN_AA, &data, 1); //configurem pipe 0 de auto AA

	data=0;
	nRF24L01_readReg(nrf24l01_EN_AA, &data, 1); //configurem pipe 0 de auto AA

#if defined(UARTdebug)
	if(data==nrf24l01_EN_AA_ENAA_P0){
	    write("nRF24 configurada correctament", true);
	} else{
	    write("Error configuracio nRF24", true);
	}
#endif

	data=nrf24l01_EN_RXADDR_ERX_P0;
	nRF24L01_writeReg(nrf24l01_EN_RXADDR, &data, 1); //configurem adress recepcio PIPE0

	data=0x01;
	nRF24L01_writeReg(nrf24l01_SETUP_AW, &data, 1); //configurem adress de 3 Bytes

	data=(0x03<<4) | 0x08;
	nRF24L01_writeReg(nrf24l01_SETUP_RETR, &data, 1); //configurem retransmissio, amb 1000us d'espera i 8 intents

	data=RF_CH;
	nRF24L01_writeReg(nrf24l01_RF_CH, &data, 1); //configurem frequency chanel

	data=(0x10<<1);
	nRF24L01_writeReg(nrf24l01_RF_SETUP, &data, 1); //configurem air-dataRate de 1Mbps i output power de -6dBm

	data=nrf24l01_STATUS_RX_DR|nrf24l01_STATUS_TX_DS|nrf24l01_STATUS_MAX_RT;
	nRF24L01_writeReg(nrf24l01_STATUS, &data, 1); //netejem interrupcions per si de cas

	//nRF24L01_writeReg(nrf24l01_RX_ADDR_P0, RX_ADDR_P0, N); //si volem canviar direccio del pipeline //ojo LSB first i s'han d'enviar tants bytes comm configurats al SETUP_AW

	//nRF24L01_writeReg(nrf24l01_TX_ADDR, TX_ADDR, N); //configurem PTX adress que envia, si fem servir ShockBurst dona igual (S'ha de posar RX_ADDR_P0) //ojo LSB first i s'han d'enviar tants bytes comm configurats al SETUP_AW

	//nRF24L01_writeReg(nrf24l01_RX_PW_P0, N, 1);  //quan vulguem posar payload de RX haurem de configurar aquest registre (R/W)

	data=(0x01<<1)|(0x01<<2);
	nRF24L01_writeReg(nrf24l01_FEATURE, &data , 1);  //enable DYNPD i Payload ACK

	//proves mirem que hem escrit algo be
	data=0;
	nRF24L01_readReg(nrf24l01_FEATURE, &data , 1);


	data=0x01;
	nRF24L01_writeReg(nrf24l01_DYNPD, &data, 1);  //activem dynamic payload del pipe 0

	nRF24L01_flush_tx();
	nRF24L01_flush_rx(); //fem un flush
}

void nRF24L01_powerUP(uint8_t mode)
{
	uint8_t data;
	if(mode==PTXmode){
//		data=nrf24l01_CONFIG_MASK_TX_DS|nrf24l01_CONFIG_MASK_MAX_RT|(0x01<<3)|(0x01<<2)|(0x01<<1)|(0x00);  //EN_CRC, CRC 2 Bytes, POWER_UP, TX
	    data=nrf24l01_CONFIG_MASK_RX_DR|(0x01<<3)|(0x01<<2)|(0x01<<1)|(0x00);  //EN_CRC, CRC 2 Bytes, POWER_UP, TX
	} else if(mode==RTXmode){
//		data=nrf24l01_CONFIG_MASK_RX_DR|(0x01<<3)|(0x01<<2)|(0x01<<1)|(0x01);  //EN_CRC, CRC 2 Bytes, POWER_UP, RX
	    data=nrf24l01_CONFIG_MASK_TX_DS|nrf24l01_CONFIG_MASK_MAX_RT|(0x01<<3)|(0x01<<2)|(0x01<<1)|(0x01);  //EN_CRC, CRC 2 Bytes, POWER_UP, RX
	} else{
		while(1); //error
	}
	nRF24L01_writeReg(nrf24l01_CONFIG, &data, 1);
}

void nRF24L01_powerDOWN(void)
{
	uint8_t data;
	nRF24L01_readReg(nrf24l01_CONFIG, &data, 1); //llegim conf actual

	if((data & nrf24l01_CONFIG_PWR_UP) == 0)
		return;

	data &= ~nrf24l01_CONFIG_PWR_UP;

	nRF24L01_writeReg(nrf24l01_CONFIG, &data, 1);
}

//-----------------------------------------------------------------------------------------------------------//
uint8_t nRF24L01_writeReg(uint8_t regnumber, uint8_t * data, uint8_t len)
{
	uint8_t status;
	uint8_t dataR;
	dataR=regnumber | nrf24l01_W_REGISTER; //volem escriure
	//baixem CS
	nRF24L01_CSN(false);
	SPI_send(1, &dataR, &status);
	//ara enviem les dades
	SPI_send(len, data, NULL);
	//pujem CS
	nRF24L01_CSN(true);

	return status;
}

uint8_t nRF24L01_readReg(uint8_t regnumber, uint8_t * data, uint8_t len){
	uint8_t status;
	//baixem CS
	nRF24L01_CSN(false);
	SPI_send(1, &regnumber, &status); //escribim
	//ara enviem les dades
	SPI_send(len, NULL, data); //llegim
	//pujem CS
	nRF24L01_CSN(true);

	return status;
}
uint8_t nRF24L01_write_tx_payload_PTX(uint8_t * data, uint8_t len, bool transmit)
{
	uint8_t status, dataR;
	dataR=nrf24l01_W_TX_PAYLOAD;
	//baixem CS
	nRF24L01_CSN(false);
	SPI_send(1, &dataR, &status); //escribim
	//ara enviem les dades
	SPI_send(len, data, NULL);
	//pujem CS
	nRF24L01_CSN(true);

	if(transmit){
		nRF24L01_transmit();
	}

	return status;
}
uint8_t nRF24L01_read_rx_payload(uint8_t * data, uint8_t len)
{
	uint8_t status, dataR;
	dataR=nrf24l01_R_RX_PAYLOAD;
	//baixem CS
	nRF24L01_CSN(false);
	SPI_send(1, &dataR, &status); //escribim
	//ara enviem les dades
	SPI_send(len, NULL, data);
	//pujem CS
	nRF24L01_CSN(true);

	return status;
}
uint8_t nRF24L01_flush_tx(void)
{
	uint8_t status,data;
	data=nrf24l01_FLUSH_TX;
	//baixem CS
	nRF24L01_CSN(false);
	SPI_send(1, &data, &status); //escribim
	nRF24L01_CSN(true);

	return status;
}
uint8_t nRF24L01_flush_rx(void)
{
	uint8_t status, data;
	data=nrf24l01_FLUSH_RX;
	//baixem CS
	nRF24L01_CSN(false);
	SPI_send(1, &data, &status); //escribim
	nRF24L01_CSN(true);

	return status;
}
uint8_t nRF24L01_reuse_tx_pl(void)
{	
	uint8_t status, data;
	data=nrf24l01_REUSE_TX_PL; //comandes SPI
	//baixem CS
	nRF24L01_CSN(false);
	SPI_send(1, &data, &status); //escribim
	nRF24L01_CSN(true);

	return status;
}
uint8_t nRF24L01_payload_width(uint8_t *data)
{
	uint8_t status, dataR;
	dataR=0x60; //comandes SPI, llegir payload width
	//baixem CS
	nRF24L01_CSN(false);
	SPI_send(1, &dataR, &status); //escribim
	SPI_send(1, NULL, data); //llegim length
	nRF24L01_CSN(true);

	return status;
}
uint8_t nRF24L01_write_tx_payload_PRX(uint8_t * data, uint8_t len, uint8_t pipeline)
{
	uint8_t status, dataR;
	dataR=pipeline | 0xA8;
	//baixem CS
	nRF24L01_CSN(false);
	SPI_send(1, &dataR, &status); //escribim
	//ara enviem les dades
	SPI_send(len, data, NULL);
	//pujem CS
	nRF24L01_CSN(true);

	return status;
}
uint8_t nRF24L01_nop(void)
{
	uint8_t status, data;
	data=nrf24l01_NOP; //comandes SPI
	//baixem CS
	nRF24L01_CSN(false);
	SPI_send(1, &data, &status); //escribim
	nRF24L01_CSN(true);

	return status;
}

void nRF24L01_transmit(void)
{
	nRF24L01_CE(true);
	delay100us(1); //esperem >10us (100us)
	nRF24L01_CE(false);
}

bool nRF24L01_FIFO_RX_empty(void){
    uint8_t data;
    nRF24L01_readReg(nrf24l01_FIFO_STATUS, &data, 1);
    if((data&0x01)==0x01){
        return true;
    } else{
        return false;
    }
}
bool nRF24L01_FIFO_RX_full(void){
    uint8_t data;
    nRF24L01_readReg(nrf24l01_FIFO_STATUS, &data, 1);
    if((data&0x02)==0x02){
        return true;
    } else{
        return false;
    }
}
bool nRF24L01_FIFO_TX_empty(void){
    uint8_t data;
    nRF24L01_readReg(nrf24l01_FIFO_STATUS, &data, 1);
    if((data&0x10)==0x10){
        return true;
    } else{
        return false;
    }
}
bool nRF24L01_FIFO_TX_full(void){
    uint8_t data;
    nRF24L01_readReg(nrf24l01_FIFO_STATUS, &data, 1);
    if((data&0x20)==0x20){
        return true;
    } else{
        return false;
    }
}

//-----------------------------------------------------------------------------------------------------------//

void nRF24L01_clear_IRQ(void)
{
	uint8_t data;
	data=nrf24l01_STATUS_RX_DR|nrf24l01_STATUS_TX_DS|nrf24l01_STATUS_MAX_RT;
	nRF24L01_writeReg(nrf24l01_STATUS, &data, 1);
}

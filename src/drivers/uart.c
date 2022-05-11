#include "drivers/uart.h"
#include <stdint.h>
#include <string.h>

// definitions for use in this header. 
#define UART_OVERSAMPLE (16)
#define BUS_CLOCK 		(24e6)


Q_T uart1_txQ;
Q_T uart1_rxQ;
Q_T uart2_txQ;
Q_T uart2_rxQ;

/* Default config for UART port 1
 * Sets the SCGC clocks, gpio port, and IRQn that will be needed for UART1.
 */ 
uart_cfg_t uart1_cfg = {
	.uartSCGCMask = SIM_SCGC4_UART1_MASK,
	.uartPort = UART1,
	.pinSCGCMask = SIM_SCGC5_PORTE_MASK,
	.gpioPort = PORTE,
	.pcrMux = 3,
	.txPin = 0,
	.rxPin = 1,
	.nvic_irq = UART1_IRQn
};

/* Default config for UART port 2
 * Sets the SCGC clocks, gpio port, and IRQn that will be needed for UART2.
 */
uart_cfg_t uart2_cfg = {
	.uartSCGCMask = SIM_SCGC4_UART2_MASK,
	.uartPort = UART2,
	.pinSCGCMask = SIM_SCGC5_PORTE_MASK,
	.gpioPort = PORTE,
	.pcrMux = 4,
	.txPin = 22,
	.rxPin = 23,
	.nvic_irq = UART2_IRQn
};

/* Generic init function for any UART port
 * 
 * ARGS:
 * - p_cfg     : The config for the UART port to be configured
 * - p_txQ     : The transmit queue for the uart port.
 * - p_rxQ     : The receive queue for the uart port.
 * - baud_rate : The baud rate that the port transmits and receives at.
 * - priority  : The priority for the IRQ
 */
void __uart_init(
	uart_cfg_t* p_cfg, 
	Q_T* p_tx_q,
	Q_T* p_rx_q,
	uint32_t baud_rate,
	uint32_t priority)
{	
	uint32_t divisor;
	
	Q_Init(p_tx_q);
	Q_Init(p_rx_q);

	// enable clock to UART and Port E
	SIM->SCGC4 |= p_cfg->uartSCGCMask;
	SIM->SCGC5 |= p_cfg->pinSCGCMask;


	// select UART pins
	p_cfg->gpioPort->PCR[p_cfg->txPin] = PORT_PCR_MUX(p_cfg->pcrMux);
	p_cfg->gpioPort->PCR[p_cfg->rxPin] = PORT_PCR_MUX(p_cfg->pcrMux);
	
	p_cfg->uartPort->C2 &=  ~(UARTLP_C2_TE_MASK | UARTLP_C2_RE_MASK);
		
	// Set baud rate to baud rate
	divisor = (uint32_t)BUS_CLOCK/(baud_rate*16);
	p_cfg->uartPort->BDH = UART_BDH_SBR(divisor>>8);
	p_cfg->uartPort->BDL = UART_BDL_SBR(divisor);
	
	// No parity, 9 bits, two stop bits, other settings;
	p_cfg->uartPort->C1 = 0; //UART_C1_M_MASK | UART_C1_PE_MASK | UART_C1_PT_MASK; 
	p_cfg->uartPort->S2 = 0;
	p_cfg->uartPort->C3 = 0;
	
    // Enable transmitter and receiver but not interrupts
	p_cfg->uartPort->C2 = UART_C2_TE_MASK | UART_C2_RE_MASK;
	
	NVIC_SetPriority(p_cfg->nvic_irq, priority); // 0, 64, 128 or 192
	NVIC_ClearPendingIRQ(p_cfg->nvic_irq); 
	NVIC_EnableIRQ(p_cfg->nvic_irq);

	p_cfg->uartPort->C2 |= UART_C2_TIE_MASK | UART_C2_RIE_MASK;
}

void __uart_start_tx(uart_cfg_t* p_cfg)
{
	// Enable transmitter and receiver but not interrupts
	p_cfg->uartPort->C2 |= (uint32_t)UART_C2_TE_MASK;
}

void __uart_start_rx(uart_cfg_t* p_cfg)
{
	// Enable transmitter and receiver but not interrupts
	p_cfg->uartPort->C2 |= (uint32_t)UART_C2_RE_MASK;
}

void __uart_stop_tx(uart_cfg_t* p_cfg)
{
	// Enable transmitter and receiver but not interrupts
	p_cfg->uartPort->C2 &= ~(uint32_t)UART_C2_TE_MASK;
	
}

void __uart_stop_rx(uart_cfg_t* p_cfg)
{
	// Enable transmitter and receiver but not interrupts
	p_cfg->uartPort->C2 &= ~(uint32_t)UART_C2_RE_MASK;
	
}

/* Generic send function for any UART port
 * Sends data to the transmit queue.
 * 
 * ARGS:
 * - p_txQ : The transmit queue for the uart port.
 * - msg   : The message to be sent.
 */
int __uart_send(uart_cfg_t *p_cfg, Q_T* p_tx_q, char* send_msg)
{
	for (char* curr_char = send_msg;
		 *curr_char != NULL;
		 curr_char++)
	{
		if (Q_Full(p_tx_q))
		{
			return -1;
		}
		Q_Enqueue(p_tx_q, (uint8_t)*curr_char);
	}
	Q_Enqueue(p_tx_q, (uint8_t)NULL);
	
	
	// Start transmitter if it isn't already running
	if (!(p_cfg->uartPort->C2 & UART_C2_TIE_MASK)) {
		p_cfg->uartPort->C2 |= UART_C2_TIE_MASK;
	}
	return 0;
}

/* Generic read function for any UART port
 * Reads data from the receive queue.
 * 
 * ARGS:
 * - p_rxQ : The recieve queue for the uart port.
 * - msg   : The message that is read.
 */
int __uart_read(Q_T* p_rx_q, char* received_msg)
{
	char buffer[16] = "";
	
	char new_char = (char)Q_Dequeue(p_rx_q);
	while(new_char != NULL)
	{
		if (!Q_Empty(p_rx_q))
		{
			strncat(buffer, &new_char, 1);
		}
		new_char = (char)Q_Dequeue(p_rx_q);
	}

	strcpy(received_msg, buffer);
	return 0;
}

// NOTE Implementing observer pattern

void UART1_IRQHandler(void) {
	uint8_t c;
	NVIC_ClearPendingIRQ(UART1_IRQn);
	if (UART1->S1 & UART_S1_TDRE_MASK) {
		// can send another character
		if (!Q_Empty(&uart1_txQ)) {
			UART1->D = Q_Dequeue(&uart1_txQ);
		} else {
			// queue is empty so disable transmitter
			UART1->C2 &= ~UART_C2_TIE_MASK;
		}
	}
	if (UART1->S1 & UART_S1_RDRF_MASK) {
		// received a character
		if (!Q_Full(&uart1_rxQ)) {
			c = UART1->D;
			Q_Enqueue(&uart1_rxQ, c);
		} else {
			// error - queue full.
			//while (1)
			//	;
			// queue is empty so disable transmitter
			UART1->C2 &= ~UART_C2_RIE_MASK;
		}
	}
	if (UART1->S1 & (UART_S1_OR_MASK |UART_S1_NF_MASK | 
		UART_S1_FE_MASK | UART_S1_PF_MASK)) {
			// handle the error
			// clear the flag
			/*
			UART1->S1 = UART_S1_OR_MASK | UART_S1_NF_MASK | 
				UART_S1_FE_MASK | UART_S1_PF_MASK;
		 */
		}
}

void UART2_IRQHandler(void) {
	uint8_t c;
	NVIC_ClearPendingIRQ(UART2_IRQn);
	if (UART2->S1 & UART_S1_TDRE_MASK) {
		// can send another character
		if (!Q_Empty(&uart2_txQ)) {
			UART2->D = Q_Dequeue(&uart2_txQ);
		} else {
			// queue is empty so disable transmitter
			UART2->C2 &= ~UART_C2_TIE_MASK;
		}
	}
	if (UART2->S1 & UART_S1_RDRF_MASK) {
		// received a character
		if (!Q_Full(&uart2_rxQ)) {
			c = UART2->D;
			Q_Enqueue(&uart2_rxQ, c);
		} else {
			// error - queue full.
			while (1)
				;
		}
	}
	if (UART2->S1 & (UART_S1_OR_MASK |UART_S1_NF_MASK | 
		UART_S1_FE_MASK | UART_S1_PF_MASK)) {
			// handle the error
			// clear the flag
			/*
			UART2->S1 = UART_S1_OR_MASK | UART_S1_NF_MASK | 
				UART_S1_FE_MASK | UART_S1_PF_MASK;
		 */
		}
}

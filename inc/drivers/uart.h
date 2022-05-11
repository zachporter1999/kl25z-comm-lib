#ifndef UART_H
#define UART_H

#include <stdio.h>
#include <stdint.h>
#include <MKL25Z4.h>
#include "queue.h"

// TODO preset baudrates
#define UART_BAUDRATE_300    300
#define UART_BAUDRATE_9600   9600
#define UART_BAUDRATE_115200 115200

/* A data structure to contain the configurations
 * for each UART port.
 */
typedef struct
{
	//SIM_Type* sim;
	uint32_t uartSCGCMask;
	UART_Type* uartPort;
	
	uint32_t pinSCGCMask;
	PORT_Type* gpioPort;
	uint32_t  pcrMux;
	
	uint32_t txPin;
	uint32_t rxPin;
	
	unsigned nvic_irq;
} uart_cfg_t;

/* Predefined transmit and receive queue for
 * the UART ports.
 */
extern Q_T uart1_txQ;
extern Q_T uart1_rxQ;
extern Q_T uart2_txQ;
extern Q_T uart2_rxQ;

// Added counters for the number of messages in the rx queue so reading can only happen when a full word is in the queue
extern volatile uint32_t messages_received_uart1;
extern volatile uint32_t messages_received_uart2;
/* Predefined configurations for
 * the UART ports.
 * 
 * If you need to change the default settings
 * edit the corresponding structure in the
 * source file and recompile.
 */
extern uart_cfg_t uart1_cfg;
extern uart_cfg_t uart2_cfg;

/* Macros for controlling the UART ports.
 * 
 * These are wrappers for the general set of functions.
 * Use these for a simplified interface for interacting 
 * with the UART devices
 */
#define UART1_INIT(baud_rate, nvic_priority) __uart_init (&uart1_cfg, &uart1_txQ, &uart1_rxQ, baud_rate, nvic_priority)
#define UART1_START_TX()                     __uart_start_tx(&uart1_cfg)
#define UART1_START_RX()                     __uart_start_rx(&uart1_cfg)
#define UART1_STOP_TX()                      __uart_stop_tx(&uart1_cfg)
#define UART1_STOP_RX()                      __uart_stop_rx(&uart1_cfg)
#define UART1_SEND(msg)                      __uart_send (&uart1_cfg, &uart1_txQ, msg)
// NOTE the msg in the read macro is a char array and is passed by referance.
#define UART1_READ(msg)                      __uart_read (&uart1_rxQ, msg)

#define UART2_INIT(baud_rate, nvic_priority) __uart_init(&uart2_cfg, &uart2_txQ, &uart2_rxQ, baud_rate, nvic_priority)
#define UART2_START_TX()                     __uart_start_tx(&uart2_cfg)
#define UART2_START_RX()                     __uart_start_rx(&uart2_cfg)
#define UART2_STOP_TX()                      __uart_stop_tx(&uart2_cfg)
#define UART2_STOP_RX()                      __uart_stop_rx(&uart2_cfg)
#define UART2_SEND(msg)                      __uart_send (&uart2_cfg, &uart2_txQ, msg)
// NOTE the msg in the read macro is a char array and is passed by referance.
#define UART2_READ(msg)                      __uart_read (&uart2_rxQ, msg)

/* Gerneral functions for interacting with the UART ports.
 * 
 * Only use these if you wish to have more control over the port.
 */
void __uart_init(uart_cfg_t* p_cfg, Q_T* p_tx_q, Q_T* p_rx_q, uint32_t baud_rate, uint32_t priority);
void __uart_start_tx(uart_cfg_t* p_cfg);
void __uart_start_rx(uart_cfg_t* p_cfg);
void __uart_stop_tx(uart_cfg_t* p_cfg);
void __uart_stop_rx(uart_cfg_t* p_cfg);
int __uart_send(uart_cfg_t *p_cfg, Q_T* p_tx_q, char* send_msg);
int __uart_read(Q_T* p_rx_q, char* received_msg);

#endif

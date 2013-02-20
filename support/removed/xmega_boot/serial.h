/*****************************************************************************
*
* Atmel Corporation
*
* File              : serial.h
* Compiler          : IAR C 3.10C Kickstart, AVR-GCC/avr-libc(>= 1.2.5)
* Revision          : $Revision: 1 $
* Date              : $Date: Wednesday, April 22, 2009 $
* Updated by        : $Author: raapeland $
*
* Support mail      : avr@atmel.com
*
* Target platform   : All AVRs with bootloader support
*
* AppNote           : AVR109 - Self-programming
*
* Description       : Header file for serial.c
****************************************************************************/
/*! \brief Generate UART initialisation section.
 *
 *  \retval None
 */
void initbootuart( void );
/*! \brief UART Transmitting section.
 *
 *  \retval None
 */
void sendchar( unsigned char );
/*! \brief Generate UART initialisation section.
 *
 *  \retval 8-bit(unsigned char) Received Character
 */
unsigned char recchar( void );


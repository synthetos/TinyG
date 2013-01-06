/*
 * xio_spi.h	- General purpose SPI device driver for xmega family
 * 				- works with avr-gcc stdio library
 *
 * Part of TinyG project
 *
 * Copyright (c) 2013 Alden S. Hart Jr.
 *
 * TinyG is free software: you can redistribute it and/or modify it 
 * under the terms of the GNU General Public License as published by 
 * the Free Software Foundation, either version 3 of the License, 
 * or (at your option) any later version.
 *
 * TinyG is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 * See the GNU General Public License for details.
 *
 * You should have received a copy of the GNU General Public License 
 * along with TinyG  If not, see <http://www.gnu.org/licenses/>.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY 
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, 
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef xio_spi_h
#define xio_spi_h

/* 
 * SPI DEVICE CONSTANTS AND PARAMETERS
 */

// Buffer sizing
#define SPIBUF_T uint_fast8_t				// fast, but limits SPI buffers to 255 char max
#define SPI_RX_BUFFER_SIZE (SPIBUF_T)64		// BUFFER_T can be 8 bits
#define SPI_TX_BUFFER_SIZE (SPIBUF_T)64		// BUFFER_T can be 8 bits

// Alternates for larger buffers - mostly for debugging
//#define SPIBUF_T uint16_t					// slower, but supports larger buffers
//#define SPI_RX_BUFFER_SIZE (SPIBUF_T)512	// BUFFER_T must be 16 bits if >255
//#define SPI_TX_BUFFER_SIZE (SPIBUF_T)512	// BUFFER_T must be 16 bits if >255
//#define SPI_RX_BUFFER_SIZE (SPIBUF_T)1024	// 2048 is the practical upper limit
//#define SPI_TX_BUFFER_SIZE (SPIBUF_T)1024	// 2048 is practical upper limit given RAM

/* 
 * SPI extended control structure 
 * Note: As defined this struct won't do buffers larger than 256 chars - 
 *	     or a max of 254 characters usable
 */

struct xioSPI {
	volatile SPIBUF_T rx_buf_tail;			// RX buffer read index
	volatile SPIBUF_T rx_buf_head;			// RX buffer write index (written by ISR)
	volatile SPIBUF_T tx_buf_tail;			// TX buffer read index  (written by ISR)
	volatile SPIBUF_T tx_buf_head;			// TX buffer write index

//	struct USART_struct *usart;				// USART structure
//	struct PORT_struct *port;				// corresponding port

	volatile char rx_buf[SPI_RX_BUFFER_SIZE];	// (written by ISR)
	volatile char tx_buf[SPI_TX_BUFFER_SIZE];
};

// SPI global accessor defines

#define SPI ds[XIO_DEV_SPI1]				// device struct accessor
#define SPIu sp[XIO_DEV_USB_OFFSET]			// usart extended struct accessor

/*
 * SPI FUNCTION PROTOTYPES
 */
/*
void xio_init_spi_chan(const uint8_t dev, 
					const uint8_t index,
					const uint32_t control,
					const struct USART_struct *usart_addr,
					const struct PORT_struct *port_addr,
					const uint8_t dirclr, 
					const uint8_t dirset, 
					const uint8_t outclr, 
					const uint8_t outset);
*/
int xio_cntl_spi(const uint32_t control);
int xio_putc_spi(const char c, FILE *stream);
int xio_getc_spi(FILE *stream);
int xio_gets_spi(char *buf, const int size);

#endif

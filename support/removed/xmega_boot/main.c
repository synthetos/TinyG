/*****************************************************************************
*
* Atmel Corporation
*
* File              : main.c
* Compiler          : IAR C 3.10C Kickstart, AVR-GCC/avr-libc(>= 1.2.5)
* Revision          : $Revision: 1.8 $
* Date              : $Date: Tuesday, Dec 08 $
* Updated by        : $Author: dvtveitaa $
*
* Support mail      : avr@atmel.com
*
* Target platform   : All AVRs with bootloader support
*
* AppNote           : AVR109 - Self-programming
*
* Description   : This Program allows an AVR with bootloader capabilities to 
*                 Read/write its own Flash/EEprom. To enter Programming mode   
*                 an input pin is checked. If this pin is pulled low, programming mode  
*                 is entered. If not, normal execution is done from $0000 
*                 "reset" vector in Application area.
*
* Preparations  : Use the preprocessor.xls file for obtaining a customized
*                 defines.h file and linker-file code-segment definition for
*                 the device you are compiling for.
****************************************************************************/

#include "defines.h"
#include "serial.h"

#include "sp_driver.h"
#include "eeprom_driver.h"

/*If the code space is critical, user can uncomment the following lines*/
//#define REMOVE_AVRPROG_SUPPORT
//#define REMOVE_FUSE_AND_LOCK_BIT_SUPPORT
//#define REMOVE_BLOCK_SUPPORT
//#define REMOVE_EEPROM_BYTE_SUPPORT
//#define REMOVE_FLASH_BYTE_SUPPORT

#ifdef LARGE_MEMORY
#  define ADDR_T unsigned long
#else  /* !LARGE_MEMORY */
#  define ADDR_T unsigned int
#endif /* LARGE_MEMORY */

#ifndef REMOVE_BLOCK_SUPPORT
unsigned char BlockLoad(unsigned int size, unsigned char mem, ADDR_T *address);
void BlockRead(unsigned int size, unsigned char mem, ADDR_T *address);

/* BLOCKSIZE should be chosen so that the following holds: BLOCKSIZE*n = PAGESIZE,  where n=1,2,3... */
#define BLOCKSIZE PAGESIZE

#endif /* REMOVE_BLOCK_SUPPORT */

#ifdef __ICCAVR__
#  define C_TASK __C_task
#else /* ! __ICCAVR__ */
#  define C_TASK /**/
#endif /* __ICCAVR__ */

int main(void)
{
	ADDR_T address = 0;
    unsigned int temp_int = 0;
	unsigned char val = 0;
    /* Initialization */    
    void (*funcptr)( void ) = 0x0000; // Set up function pointer to RESET vector.
  
    EEPROM_FlushBuffer();
	EEPROM_DisableMapping();

    PROGPORT |= (1<<PROG_NO); // Enable pull-up on PROG_NO line on PROGPORT.
    initbootuart(); // Initialize UART.
                
	/* Branch to bootloader or application code? */
    if( !(PROGPIN & (1<<PROG_NO)) ) // If PROGPIN is pulled low, enter programmingmode.
    {
	    /* Main loop */
        for(;;)
        {
            val = recchar(); // Wait for command character.
            // Check autoincrement status.
            if(val=='a')
            {
                sendchar('Y'); // Yes, we do autoincrement.
            }
            // Set address.
            else if(val == 'A') // Set address...
            { // NOTE: Flash addresses are given in words, not bytes.                                            
                address = (recchar() << 8) | recchar(); // Read address high and low byte.
                sendchar('\r'); // Send OK back.
            }
            // Chip erase.
            else if(val=='e')
            {
                for(address = 0; address < APP_END; address += PAGESIZE)
                { // NOTE: Here we use address as a byte-address, not word-address, for convenience.
                    SP_WaitForSPM();
#ifdef __ICCAVR__
#pragma diag_suppress=Pe1053 // Suppress warning for conversion from long-type address to flash ptr.
#endif
		            SP_EraseApplicationPage( address );
#ifdef __ICCAVR__
#pragma diag_default=Pe1053 // Back to default.
#endif
                }
                
                // Writing random values to the page buffer
                EEPROM_LoadPage(&val);
                // Erasing all pages in the EEPROM
                EEPROM_EraseAll();
                
                sendchar('\r'); // Send OK back.
            }
            
#ifndef REMOVE_BLOCK_SUPPORT
            // Check block load support.
            else if(val=='b')
		    {
    			sendchar('Y'); // Report block load supported.
    			sendchar((BLOCKSIZE>>8) & 0xFF); // MSB first.
    			sendchar(BLOCKSIZE&0xFF); // Report BLOCKSIZE (bytes).
    		}

            // Start block load.
    		else if(val=='B')
    		{
	    	    temp_int = (recchar()<<8) | recchar(); // Get block size.
	            val = recchar(); // Get memtype.
  		        sendchar( BlockLoad(temp_int, val, &address) ); // Block load.				
		    }	    
		    // Start block read.
    		else if(val=='g')
    		{
	    	    temp_int = (recchar()<<8) | recchar(); // Get block size.
    		    val = recchar(); // Get memtype
	    		BlockRead(temp_int, val, &address); // Block read
    		}		
#endif /* REMOVE_BLOCK_SUPPORT */

#ifndef REMOVE_FLASH_BYTE_SUPPORT            
            // Read program memory.
		    else if(val=='R')
            {        
                // Send high byte, then low byte of flash word.
		        SP_WaitForSPM();
#ifdef __ICCAVR__
#pragma diag_suppress=Pe1053 // Suppress warning for conversion from long-type address to flash ptr.
#endif
                sendchar( SP_ReadByte( (address << 1)+1) );
                sendchar( SP_ReadByte( (address << 1)+0) );
#ifdef __ICCAVR__
#pragma diag_default=Pe1053 // Back to default.
#endif
                address++; // Auto-advance to next Flash word.
            }
        

            // Write program memory, low byte.        
            else if(val=='c')
            { // NOTE: Always use this command before sending high byte.
                temp_int=recchar(); // Get low byte for later SP_LoadFlashWord
                sendchar('\r'); // Send OK back.
            }

            // Write program memory, high byte.
            else if(val=='C')
            {
                temp_int |= (recchar()<<8); // Get and insert high byte.
		        SP_WaitForSPM();
#ifdef __ICCAVR__
#pragma diag_suppress=Pe1053 // Suppress warning for conversion from long-type address to flash ptr.
#endif
                SP_LoadFlashWord( (address << 1), temp_int );
#ifdef __ICCAVR__
#pragma diag_default=Pe1053 // Back to default.
#endif
                address++; // Auto-advance to next Flash word.
                sendchar('\r'); // Send OK back.
            }
        
        
            // Write page.       
            else if(val== 'm')
            {
                if( address >= (APP_END>>1) ) // Protect bootloader area.		    
                {
                    sendchar('?');
                } 
                else
                {
		            SP_WaitForSPM();
#ifdef __ICCAVR__
#pragma diag_suppress=Pe1053 // Suppress warning for conversion from long-type address to flash ptr.
#endif
                    // Convert word-address to byte-address and write.
                    SP_WriteApplicationPage( address << 1);
#ifdef __ICCAVR__
#pragma diag_default=Pe1053 // Back to default.
#endif
                 
                    sendchar('\r'); // Send OK back.
                }
#endif // REMOVE_FLASH_BYTE_SUPPORT

#ifndef REMOVE_EEPROM_BYTE_SUPPORT
            }
            // Write EEPROM memory.
            else if (val == 'D')
            {
                EEPROM_WriteByte( (unsigned char)(address / EEPROM_BYTES_IN_PAGE) , (unsigned char)(address & EEPROM_BYTE_ADDRESS_MASK), recchar() );
                // Select next EEPROM byte
                address++;
            }

             // Read EEPROM memory.
            else if (val == 'd')
            {
                  
                sendchar( EEPROM_ReadByte( (unsigned char)(address / EEPROM_BYTES_IN_PAGE), (unsigned char)(address & EEPROM_BYTE_ADDRESS_MASK) ) );
                // Select next EEPROM byte
                address++;
             }
			
#endif /* REMOVE_EEPROM_BYTE_SUPPORT */

#ifndef REMOVE_FUSE_AND_LOCK_BIT_SUPPORT
            // Write lockbits.
            else if(val=='l')
            {
                // Wait for NVM to finish.		
                SP_WaitForSPM();
                // Read and set lock bits.
                SP_WriteLockBits( recchar() );
                sendchar('\r'); // Send OK back.
            }
#if defined(_GET_LOCK_BITS)
            // Read lock bits.
            else if(val=='r')
            {                
                SP_WaitForSPM();        
                sendchar( SP_ReadLockBits() );
            }
            // Read low fuse bits.
            else if(val=='F')
            {
                SP_WaitForSPM();
                sendchar(SP_ReadFuseByte(0));
            }
            // Read high fuse bits
            else if(val=='N')
            {
                SP_WaitForSPM();
                sendchar(SP_ReadFuseByte(1));
            }
            // Read extended fuse bits.
            else if(val=='Q')
            {
                SP_WaitForSPM();      
                sendchar(SP_ReadFuseByte(2));
            }
#endif /* defined(_GET_LOCK_BITS) */
#endif /* REMOVE_FUSE_AND_LOCK_BIT_SUPPORT */

#ifndef REMOVE_AVRPROG_SUPPORT        
            // Enter and leave programming mode.
            else if((val=='P')||(val=='L'))
            {
                sendchar('\r'); // Nothing special to do, just answer OK.
            }
            // Exit bootloader.
            else if(val=='E')
            {
                SP_WaitForSPM();
               // SP_LockSPM();
                sendchar('\r');
                EIND = 0x00;
                funcptr(); // Jump to Reset vector 0x0000 in Application Section.
            }
                 // Get programmer type.        
            else if (val=='p')
            {
                sendchar('S'); // Answer 'SERIAL'.
            }
            // Return supported device codes.
            else if(val=='t')
            {
#if PARTCODE+0 > 0
                 sendchar( PARTCODE ); // Supports only this device, of course.
#endif /* PARTCODE */
                 sendchar( 0 ); // Send list terminator.
            }
            // Set LED, clear LED and set device type.
            else if((val=='x')||(val=='y')||(val=='T'))
            {
                recchar(); // Ignore the command and it's parameter.
                sendchar('\r'); // Send OK back.
            }
#endif /* REMOVE_AVRPROG_SUPPORT */
            // Return programmer identifier.
            else if(val=='S')
            {
                sendchar('A'); // Return 'AVRBOOT'.
                sendchar('V'); // Software identifier (aka programmer signature) is always 7 characters.
                sendchar('R');
                sendchar('B');
                sendchar('O');
                sendchar('O');
                sendchar('T');
            }
            // Return software version.
            else if(val=='V')
            {
                sendchar('1');
                sendchar('6');
            }        
            // Return signature bytes.
            else if(val=='s')
            {							
                sendchar( SIGNATURE_BYTE_3 );
                sendchar( SIGNATURE_BYTE_2 );
                sendchar( SIGNATURE_BYTE_1 );
            }       
                 // The last command to accept is ESC (synchronization).
            else if(val!=0x1b) // If not ESC, then it is unrecognized...
            {
               sendchar('?');
            }
        } // end: for(;;)
    }
    else
    {
        SP_WaitForSPM();
        SP_LockSPM();
        EIND = 0x00;
        funcptr(); // Jump to Reset vector 0x0000 in Application Section.
    }
} // end: main


#ifndef REMOVE_BLOCK_SUPPORT

unsigned char BlockLoad(unsigned int size, unsigned char mem, ADDR_T *address)
{   
    unsigned int data;    
    ADDR_T tempaddress;
        	
    // EEPROM memory type.
    if(mem=='E')
    {
        unsigned char pageAddr, byteAddr, value;
        unsigned char buffer[BLOCKSIZE];
    
      	EEPROM_FlushBuffer();
        // disable mapping of EEPROM into data space (enable IO mapped access)
        EEPROM_DisableMapping();

        // Fill buffer first, as EEPROM is too slow to copy with UART speed 
        for(tempaddress=0;tempaddress<size;tempaddress++){
            buffer[tempaddress] = recchar();
        }
        
      // Then program the EEPROM 
    	for( tempaddress=0; tempaddress < size; tempaddress++)
    	{
            // void EEPROM_WriteByte( uint8_t pageAddr, uint8_t byteAddr, uint8_t value )
            pageAddr = (unsigned char)( (*address) / EEPROM_BYTES_IN_PAGE);
            byteAddr = (unsigned char)( (*address) & EEPROM_BYTE_ADDRESS_MASK);
            value = buffer[tempaddress];
            
            EEPROM_WriteByte(pageAddr, byteAddr, value);
            
            (*address)++; // Select next EEPROM byte
        }

        return '\r'; // Report programming OK
    } 
    
    // Flash memory type
	else if(mem=='F')
    { // NOTE: For flash programming, 'address' is given in words.
        (*address) <<= 1; // Convert address to bytes temporarily.
        tempaddress = (*address);  // Store address in page.
	
        do
		{
            data = recchar();
            data |= (recchar() << 8);
#ifdef __ICCAVR__
#pragma diag_suppress=Pe1053 // Suppress warning for conversion from long-type address to flash ptr.
#endif
            SP_LoadFlashWord(*address, data);
#ifdef __ICCAVR__
#pragma diag_default=Pe1053 // Back to default.
#endif
            (*address)+=2; // Select next word in memory.            
            size -= 2; // Reduce number of bytes to write by two.
        } while(size); // Loop until all bytes written.

#ifdef __ICCAVR__
#pragma diag_suppress=Pe1053 // Suppress warning for conversion from long-type address to flash ptr.
#endif
        SP_WriteApplicationPage(tempaddress);
		
#ifdef __ICCAVR__
#pragma diag_default=Pe1053 // Back to default.
#endif        
//	_WAIT_FOR_SPM();
        SP_WaitForSPM();
        (*address) >>= 1; // Convert address back to Flash words again.
        return '\r'; // Report programming OK
    }
    
    // Invalid memory type?
    else
    {
        return '?';
    }
}



void BlockRead(unsigned int size, unsigned char mem, ADDR_T *address)
{
    // EEPROM memory type.
	
    if (mem=='E') // Read EEPROM
    {
        unsigned char byteAddr, pageAddr;
        
        EEPROM_DisableMapping();
        EEPROM_FlushBuffer();

        do
        {
            pageAddr = (unsigned char)(*address / EEPROM_BYTES_IN_PAGE);
            byteAddr = (unsigned char)(*address & EEPROM_BYTE_ADDRESS_MASK);
            
            sendchar( EEPROM_ReadByte( pageAddr, byteAddr ) );
             // Select next EEPROM byte
            (*address)++;            
            size--; // Decrease number of bytes to read
        } while (size); // Repeat until all block has been read
    }
    
    // Flash memory type.
	else if(mem=='F')
	{
        (*address) <<= 1; // Convert address to bytes temporarily.
	
        do
        {
#ifdef __ICCAVR__
#pragma diag_suppress=Pe1053 // Suppress warning for conversion from long-type address to flash ptr.
#endif
          sendchar( SP_ReadByte( *address) );
          sendchar( SP_ReadByte( (*address)+1) );
#ifdef __ICCAVR__
#pragma diag_default=Pe1053     // Back to default.
#endif
            (*address) += 2;    // Select next word in memory.
            size -= 2;          // Subtract two bytes from number of bytes to read
        } while (size);         // Repeat until all block has been read

        (*address) >>= 1;       // Convert address back to Flash words again.
    }
}

#endif /* REMOVE_BLOCK_SUPPORT */
/* end of file */


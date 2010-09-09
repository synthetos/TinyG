

#include <avr/pgmspace.h>

void load_buffer(char c);

char block_P[] PROGMEM = "g0 x10 y20 z30";		// FLASH string
char block[41];									// RAM string

int main () 
{
	int i = 0;

//	strcmp_P("RAM_STRING", PSTR("FLASH_STRING"));
	strcpy_P(block, block_P);

	while (block[i]) {
		load_buffer(block[i++]);
	}
}


void load_buffer(char c)
{
	return;
}


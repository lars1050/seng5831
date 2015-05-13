/* orangutan_app1 - an application for the Pololu Orangutan SVP
 *
 * This application uses the Pololu AVR C/C++ Library.  For help, see:
 * -User's guide: http://www.pololu.com/docs/0J20
 * -Command reference: http://www.pololu.com/docs/0J18
 *
 * Created: 1/28/2013 9:51:18 AM
 *  Author: Administrator
 */

#include <pololu/orangutan.h>
#include "HT1632.h"

// MOSI PB5
// MISO PB6
// SCK PB7
// SS_BAR PB4 -- This is not part of the programming port, but set at output so as not to interfere with SPI

// CONNECTION: Plug the 6-pin connector on the LED array into its keyed port on the orangutan. 
// This connects power, ground, clock ("SCK" on Orangutan, "WR" on LED array), and MOSI ("data" on the LED array)
// Plug the white wire from LED array to general I/O PB4 on Orangutan. This is the SS_Bar ("CS0" on the LED array)

void SPItransmit( unsigned char dataHi, unsigned char dataLo );
void sendcommand( uint8_t cmd );
void senddata( uint8_t address, uint8_t values );
void blink_red();
void printBinary( char, char);

int main()
{
	clear();	// clear the LCD
	print("Initializing System");
	lcd_goto_xy(0, 1);	// go to start of second LCD row

	//delay(300);
	
	clear();
	printBinary( 3, 0);
	printBinary( 6, 1);
	
	// Initialize the SPI
	
	// MOSI, MISO, SCK, and SS_BAR outputs
	DDRB |= ( 1 << PIN_MOSI ) | ( 1 << PIN_SCK ) | ( 0 << PIN_MISO ) | ( 1 << PIN_SSBAR );
	
	// Set CS0 hi to disable communication
	PORTB |= ( 1 << PIN_SSBAR );

	// Enable SPI and set up SPI as master
	SPCR |= ( 1 << SPE ) | ( 1 << MSTR ) ;
	
	// Set Polarity and Phase to set-up on falling edge, and device reads on rising edge
	SPCR |= ( 1 << CPOL ) | ( 1 << CPHA ) ;
	
	// Set frequency of clock signal over SCK. LED board states clock max is 1MHz. Thus setting clock divider to 32.
	// 20million ticks/second / 32 = 625000 ticks/second = 625kHz
	SPCR |= ( 1 << SPR1 );
	SPSR = ( 1 << SPI2X );
	// Alternatively, if this is not working, try to go slowest setting, with div of 128 for 156kHz
	//SPCR |= ( 1 << SPR1 ) | ( 1 << SPR0 );
	
	// Set data direction according to what LED board is expecting, which is Most Significant Bit
	SPCR &= ~( 1 << DORD ); // MSB
	// SPCR |= ( 1 << DORD ); // LSB

	sendcommand(HT1632_SYS_EN);			// enable the system
	sendcommand(HT1632_LED_ON);			// enable the LEDs
	sendcommand(HT1632_BLINK_OFF);		// steady on for LEDs - no Blink
	sendcommand(HT1632_SLAVE_MODE);		// board received from Orangutan
	sendcommand(HT1632_INT_RC);			// use internal clock on LED board to control dimness of LEDs
	sendcommand(HT1632_COMMON_16NMOS);	// using mode 96x4 (indicated by the 16). Not sure of NMOS vs PMOS distinction.
	sendcommand(HT1632_PWM_CONTROL | 0x0F);	// control dimness of LEDs, range is 0x00 to 0xFF
		
	delay(300);		// give the board a second to initialize, just in case
	
	//while (1) {
		senddata( 0, 0x0F );	// set 4 LEDs at address 0 to "on"
		senddata( 1, 0x0F );	// set 4 LEDs at address 1 to "on"
	//}		
}

void sendcommand( uint8_t cmd ) {
	
	// SPI works with 1 byte at a time. Need to convert command into 2 bytes Hi and Lo
	uint8_t dataHi = 0;
	uint8_t dataLo = 0;
	
	// All commands start with 100 (as specified in HT1632 datasheet)
	// commands are in the form of 8 bits, although there is an ignored 9th bit in LSB position, making a complete command 12 bits
	// These 12 bits start at MSB, thus last 4 are always ignore. See table of commands in HT1632 for a complete command list.
	dataHi = 0b10000000 | ( cmd >> 3 ); // start with 100, then add 5 MSB bits of command to Hi
	dataLo = 0x07 & cmd ;    // grab last 3 bits of command
	dataLo = ( cmd << 5 );   // shift those 3 bits to MSB
	
	// some debugging code ...
	//printBinary( dataHi, 0);
	//printBinary( dataLo, 1);
	//delay(500);
	
	SPItransmit(dataHi, dataLo);
}

void senddata( uint8_t address, uint8_t values ) {
	
	// sending data similarly requires 2 bytes Hi and Lo
	// Write commands start with 101, followed by 7 address bits, then 4 on/off bits for the 4 leds at that address
	// An alternate method for sending data is to specify a start address, then send all pixel values.
	// For example, I can start with 1010 0000 | 00xx xxxx | xxxx xxxx | ....
	//    which is write command 101, start at address 0, then set each x to matrix[i][j], which equals either 0 for off, 1 for on 
	uint8_t dataHi = 0;
	uint8_t dataLo = 0;
	
	dataHi = ( address >> 2 );     // make room for write command 101 (address is only 7 bits, not 8)
	dataHi = dataHi | 0b10100000;  // put command and 5 MSB bits of address in HI
	
	dataLo = 0x03 & address;       // grab last 2 bits of address
	dataLo = ( dataLo << 6 ) | ( values << 2 );   // shift those 2 bits to MSB, then add 4-bits of pixel settings to Lo
	
	// some debugging code ...
	//printBinary( dataHi, 0);
	//printBinary( dataLo, 1);
	//delay(500);
	//red_led(1);

	SPItransmit(dataHi, dataLo);
}

void SPItransmit( unsigned char dataHi, unsigned char dataLo ) {

	// pull CS0 low to indicate start of transmission
	DDRB |= (  1 << PIN_SSBAR );
	PORTB &= ~ ( 1 << PIN_SSBAR );

	SPDR = dataHi;
	while ( !( SPSR & ( 1<<SPIF )) );
	SPDR = dataLo;
	while ( !( SPSR & ( 1<<SPIF )) );

	// pull CS0 hi to signal complete
	PORTB |= ( 1 << PIN_SSBAR );
}

void blink_red() {
	red_led(1);     // Turn on the red LED.
	delay_ms(300);  // Wait for 200 ms.
	red_led(0);     // Turn off the red LED.
	delay_ms(300);  // Wait for 200 ms.
}

void printBinary( char byte, char row ) {
	//clear();
	lcd_goto_xy( 0, row );
	print("0b");
	int i;
	for (i=7; i>-1; i--) {
		if ( byte & (1 << i) )
			print("1");
		else
			print("0");
	}
}

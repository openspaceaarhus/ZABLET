/*
TV-B-Gone Firmware version 1.2
for use with ATtiny85v and v1.2 hardware
(c) Mitch Altman + Limor Fried 2009
Last edits, August 16 2009

With some code from:
Kevin Timmerman & Damien Good 7-Dec-07

Distributed under Creative Commons 2.5 -- Attib & Share Alike

This is the 'universal' code designed for v1.2 - it will select EU or NA
depending on a pulldown resistor on pin B1 !
*/

#include <limits.h>
#include <avr/io.h>             // this contains all the IO port definitions
#include <avr/eeprom.h>
#include <avr/sleep.h>          // definitions for power-down modes
#include <avr/pgmspace.h>       // definitions or keeping constants in program memory
#include <avr/wdt.h>
#include <avr/interrupt.h>

#include "main.h"
#include "util.h"

#define USE_EEPROM	1

#define irled_on()	do { PORTB &= ~_BV(IRLED); } while(0)
#define irled_off()	do { PORTB |= _BV(IRLED); } while(0)
#define flash_on()	do { DDRA |= _BV(FLASHLED); } while(0)
#define flash_off()	do { DDRA &= ~_BV(FLASHLED); } while(0)
#define brownout_on()	do { DDRA |= _BV(BOLED); } while(0)
#define brownout_off()	do { DDRA &= ~_BV(BOLED); } while(0)

#define mode_tvbg()	do { irled_off(); DDRB |= _BV(IRLED); } while(0)
#define mode_pov()	do { irled_on(); DDRB &= ~_BV(IRLED); } while(0)

#define OCR_CTC		(F_CPU/64/100-1)		/* CTC mode ~100Hz */
#define start_timer1()	do { \
				OCR1AH = ((OCR_CTC >> 8) & 0xff); \
				OCR1AL = ((OCR_CTC) & 0xff); \
				TCCR1A = 0; \
				TCCR1B = _BV(WGM12) | _BV(CS11) | _BV(CS10);	/* Prescale /64 */ \
				TIMSK1 = _BV(OCIE1A);				/* Interrupt on OCR1A match */ \
			} while(0)
#define stop_timer1()	do { \
				TIMSK1 = 0; \
				TCCR1A = 0; \
				TCCR1B = 0; \
				OCR1AH = 0; \
				OCR1AL = 0; \
			} while(0)

#define F_BUTTON	0x01
#define F_POV		0x02

#define BUTTON_ONTIME		(INT8_MAX-1)	// If we hold the button 1.26 seconds, drop out of TVBG
#define BUTTON_OFFTIME		(-3)		// And come out when released
#define BUTTON_BLINKTIME	50


#ifndef ALL_CODES
uint8_t region = EU;		// by default our code is EU
#endif

volatile uint8_t wuflags;	// Wakeup flags to indicate POV switch or button
volatile int8_t button;		// Button indication


/*
 * This project transmits a bunch of TV POWER codes, one right after the other,
 * with a pause in between each.  (To have a visible indication that it is
 * transmitting, it also pulses a visible LED once each time a POWER code is
 * transmitted.)  That is all TV-B-Gone does.  The tricky part of TV-B-Gone was
 * collecting all of the POWER codes, and getting rid of the duplicates and
 * near-duplicates (because if there is a duplicate, then one POWER code will
 * turn a TV off, and the duplicate will turn it on again (which we certainly
 * do not want).  I have compiled the most popular codes with the duplicates
 * eliminated, both for North America (which is the same as Asia, as far as
 * POWER codes are concerned -- even though much of Asia USES PAL video) and
 * for Europe (which works for Australia, New Zealand, the Middle East, and
 * other parts of the world that use PAL video).
 *
 * Before creating a TV-B-Gone Kit, I originally started this project by
 * hacking the MiniPOV kit.  This presents a limitation, based on the size of
 * the Atmel ATtiny2313 internal flash memory, which is 2KB.  With 2KB we can
 * only fit about 7 POWER codes into the firmware's database of POWER codes.
 * However, the more codes the better! Which is why we chose the ATtiny85 for
 * the TV-B-Gone Kit.
 *
 * This version of the firmware has the most popular 100+ POWER codes for North
 * America and 100+ POWER codes for Europe. You can select which region to use
 * by soldering a 10K pulldown resistor.
 */


/*
This project is a good example of how to use the AVR chip timers.
*/


/*
The hardware for this project is very simple:
     ATtiny85 has 8 pins:
       pin 1   RST + Button
       pin 2   one pin of ceramic resonator MUST be 8.0 mhz
       pin 3   other pin of ceramic resonator
       pin 4   ground
       pin 5   OC1A - IR emitters, through a '2907 PNP driver that connects
               to 4 (or more!) PN2222A drivers, with 1000 ohm base resistor
               and also connects to programming circuitry
       pin 6   Region selector. Float for US, 10K pulldown for EU,
               also connects to programming circuitry
       pin 7   PB0 - visible LED, and also connects to programming circuitry
       pin 8   +3-5v DC (such as 2-4 AA batteries!)
    See the schematic for more details.

    This firmware requires using an 8.0MHz ceramic resonator
       (since the internal oscillator may not be accurate enough).

    IMPORTANT:  to use the ceramic resonator, you must perform the following:
                    make burn-fuse_cr
*/



// This function delays the specified number of 10 microseconds it is
// 'hardcoded' and is calibrated by adjusting DELAY_CNT in main.h Unless you
// are changing the crystal from 8mhz, dont mess with this.
void delay_ten_us(uint16_t us)
{
	uint8_t timer;
	while (us != 0) {
		// for 8MHz we want to delay 80 cycles per 10 microseconds
		// this code is tweaked to give about that amount.
		for (timer=0; timer <= DELAY_CNT; timer++) {
			NOP;
			NOP;
		}
		NOP;
		us--;
	}
}


void powerDown(void)
{
	// Shut down everything and put the CPU to sleep
	TCCR0A = 0;			// turn off frequency generator (should be off already)
	TCCR0B = 0;			// turn off frequency generator (should be off already)
	mode_pov();
	flash_off();
	stop_timer1();
	PORTA = 0xff;			// Turn on all pull-ups to prevent indeterminate input states

	wdt_disable();			// turn off the watchdog (since we want to sleep
	delay_ten_us(1000);		// wait 10 millisec

	MCUCR = _BV(SM1) | _BV(SE);	// power down mode,  SE enables Sleep Modes
	sleep_cpu();			// put CPU into Power Down Sleep Mode
	delay_ten_us(10);		// Let any interrupts be processed
	PORTA = _BV(BUTTONPIN);		// Turn off all pull-ups except input button
	start_timer1();
}


/*
 * This function is the 'workhorse' of transmitting IR codes. Given the on and
 * off times, it turns on the PWM output on and off to generate one 'pair' from
 * a long code. Each code has ~50 pairs!
 */
void xmitCodeElement(uint16_t ontime, uint16_t offtime, uint8_t PWM_code )
{
	// start Timer0 outputting the carrier frequency to IR emitters on and OC0A
	// (PB2, pin 5)
	TCNT0 = 0; // reset the timers so they are aligned
	TIFR0 = 0;  // clean out the timer flags

	if(PWM_code) {
		// 99% of codes are PWM codes, they are pulses of a carrier
		// frequecy Usually the carrier is around 38KHz, and we
		// generate that with PWM timer 0
		TCCR0A =_BV(COM0A0) | _BV(WGM01);          // set up timer 0
		TCCR0B = _BV(CS00);
	} else {
		// However some codes dont use PWM in which case we just turn
		// the IR LED on for the period of time.
		irled_on();
	}

	// Now we wait, allowing the PWM hardware to pulse out the carrier
	// frequency for the specified 'on' time
	delay_ten_us(ontime);

	// Now we have to turn it off so disable the PWM output
	TCCR0A = 0;
	TCCR0B = 0;
	// And make sure that the IR LED is off too (since the PWM may have
	// been stopped while the LED is on!)
	irled_off();

	// Now we wait for the specified 'off' time
	delay_ten_us(offtime);
}

/*
 * This is kind of a strange but very useful helper function Because we are
 * using compression, we index to the timer table not with a full 8-bit byte
 * (which is wasteful) but 2 or 3 bits. Once code_ptr is set up to point to
 * the right part of memory, this function will let us read 'count' bits at a
 * time which it does by reading a byte into 'bits_r' and then buffering it.
 */

uint8_t bitsleft_r;	/* = 0;*/ // By def. it is init to zero
uint8_t bits_r;		/* = 0;*/
PGM_P code_ptr;

// we cant read more than 8 bits at a time so dont try!
uint8_t read_bits(uint8_t count)
{
	uint8_t i;
	uint8_t tmp = 0;

	// we need to read back count bytes
	for(i = 0; i < count; i++) {
		// check if the 8-bit buffer we have has run out
		if(bitsleft_r == 0) {
			// in which case we read a new byte in
			bits_r = pgm_read_byte(code_ptr++);
			// and reset the buffer size (8 bites in a byte)
			bitsleft_r = 8;
		}
		// remove one bit
		bitsleft_r--;
		// and shift it off of the end of 'bits_r'
		tmp |= (((bits_r >> (bitsleft_r)) & 1) << (count-1-i));
	}
	// return the selected bits in the LSB part of tmp
	return tmp;
}


/*
 * The C compiler creates code that will transfer all constants into RAM when
 * the microcontroller resets. Since this firmware has a table (powerCodes)
 * that is too large to transfer into RAM, the C compiler needs to be told to
 * keep it in program memory space. This is accomplished by the macro PROGMEM
 * (this is used in the definition for powerCodes). Since the C compiler
 * assumes that constants are in RAM, rather than in program memory, when
 * accessing powerCodes, we need to use the pgm_read_word() and pgm_read_byte
 * macros, and we need to use powerCodes as an address. This is done with
 * PGM_P, defined below.
 * For example, when we start a new powerCode, we first point to it with the
 * following statement:
 *	PGM_P thecode_p = pgm_read_word(powerCodes+i);
 * The next read from the powerCode is a byte that indicates the carrier
 * frequency, read as follows:
 *	const uint8_t freq = pgm_read_byte(code_ptr++);
 * After that is a byte that tells us how many 'onTime/offTime' pairs we have:
 *	const uint8_t numpairs = pgm_read_byte(code_ptr++);
 * The next byte tells us the compression method. Since we are going to use a
 * timing table to keep track of how to pulse the LED, and the tables are
 * pretty short (usually only 4-8 entries), we can index into the table with
 * only 2 to 4 bits. Once we know the bit-packing-size we can decode the pairs
 *	const uint8_t bitcompression = pgm_read_byte(code_ptr++);
 * Subsequent reads from the powerCode are n bits (same as the packing size)
 * that index into another table in ROM that actually stores the on/off times
 *	const PGM_P time_ptr = (PGM_P)pgm_read_word(code_ptr);
 */

ISR(PCINT0_vect)
{
	wuflags |= F_BUTTON;
}

ISR(PCINT1_vect)
{
	wuflags |= F_POV;
}

ISR(TIM1_COMPA_vect)
{
	uint8_t p, d, b;
	p = PORTA;
	d = DDRA;
	PORTA |= _BV(BUTTONPIN);
	DDRA &= ~_BV(BUTTONPIN);
	b = PINA & _BV(BUTTONPIN);
	PORTA = p;
	DDRA = d;

	/*
	 * Count button up when pressed
	 * Count button down when released
	 * Set to 0 on change
	 */
	if(button >= 0 && !b) {
		if(button < INT8_MAX)
			button++;
	} else if(button <= 0 && b) {
		if(button > INT8_MIN)
			button--;
	} else
		button = 0;
}

#define PROGRESS_BITS 8


void turnOffTVs(void)
{
#ifndef ALL_CODES
	uint8_t i;
	uint8_t numCodes = region == US ? pgm_read_byte(&num_NAcodes) : pgm_read_byte(&num_EUcodes);
	// Indicate how big our database is
	DEBUGP(putstring("\n\rNA Codesize: "); putnum_ud(pgm_read_byte(&num_NAcodes)););
	DEBUGP(putstring("\n\rEU Codesize: "); putnum_ud(pgm_read_byte(&num_EUcodes)););
#else
	uint16_t i;
	uint16_t numCodes = pgm_read_word(&num_Allcodes);
#endif
	mode_tvbg();	// Set the IRLED pin to output
	PORTA = 0x00;	// And disable pull-up the switch input

	// Starting execution loop
	delay_ten_us(25000);

	// We may have different number of codes in either database

	// for every POWER code in our collection
	for(i = 0 ; i < numCodes; i++) {
		// print out the code # we are about to transmit
		DEBUGP(putstring("\n\r\n\rCode #: "); putnum_ud(i));

		// If we hold the button x seconds, drop out
		if(button >= BUTTON_ONTIME) {
			button = 0;
			DDRA = 0xaa;
			while(button >= BUTTON_OFFTIME) {
				if(button >= BUTTON_BLINKTIME) {
					button = 0;
					DDRA ^= 0xff;
				}
			}
			break;
		}

		/*
		 * Output some indication of progress on the POV leds while
		 * sending zap codes.
		 */
#ifndef ALL_CODES
		uint8_t pbit = i / (numCodes/PROGRESS_BITS);
#else
		uint8_t pbit = i * PROGRESS_BITS / numCodes;
#endif
		uint8_t leds = ((1 << pbit)-1);
		if(button <= -(BUTTON_BLINKTIME)) {
			button = 0;
			leds |= 1 << pbit;
		}
		DDRA = leds;

		// Flush previous bits, so that next code starts with a fresh
		// set of 8 bits.
		bitsleft_r = 0;

		// To keep Watchdog from resetting in middle of code.
		wdt_reset();

		// point to next POWER code, from the right database
#ifdef ALL_CODES
		code_ptr = (PGM_P)pgm_read_word(AllpowerCodes+i);
#else
		if (region == US) {
			code_ptr = (PGM_P)pgm_read_word(NApowerCodes+i);
		} else {
			code_ptr = (PGM_P)pgm_read_word(EUpowerCodes+i);
		}
#endif

		// Read the carrier frequency from the first byte of code structure
		// set OCR for Timer1 to output this POWER code's carrier frequency
		const uint8_t freq = pgm_read_byte(code_ptr++);
		OCR0A = freq;

		// Print out the frequency of the carrier and the PWM settings
		DEBUGP(putstring("\n\rOCR1: "); putnum_ud(freq););
		DEBUGP(uint16_t x = (freq+1) * 2; putstring("\n\rFreq: "); putnum_ud(F_CPU/x););

		// Get the number of pairs, the second byte from the code struct
		const uint8_t numpairs = pgm_read_byte(code_ptr++);
		DEBUGP(putstring("\n\rOn/off pairs: "); putnum_ud(numpairs));

		// Get the number of bits we use to index into the timer table
		// This is the third byte of the structure
		const uint8_t bitcompression = pgm_read_byte(code_ptr++);
		DEBUGP(putstring("\n\rCompression: "); putnum_ud(bitcompression));

		// Get pointer (address in memory) to pulse-times table
		// The address is 16-bits (2 byte, 1 word)
		const PGM_P time_ptr = (PGM_P)pgm_read_word(code_ptr);
		code_ptr += 2;

		// Transmit all codeElements for this POWER code
		// (a codeElement is an onTime and an offTime)
		// transmitting onTime means pulsing the IR emitters at the carrier
		// frequency for the length of time specified in onTime
		// transmitting offTime means no output from the IR emitters for the
		// length of time specified in offTime

		/*
		// print out all of the pulse pairs
		for (uint8_t k=0; k<numpairs; k++) {
			uint8_t ti;
			ti = (read_bits(bitcompression)) * 4;
			// read the onTime and offTime from the program memory
			ontime = pgm_read_word(time_ptr+ti);
			offtime = pgm_read_word(time_ptr+ti+2);
			DEBUGP(putstring("\n\rti = "); putnum_ud(ti>>2); putstring("\tPair = "); putnum_ud(ontime));
			DEBUGP(putstring("\t"); putnum_ud(offtime));
		}
		*/

		// For EACH pair in this code....
		for(uint8_t k = 0; k < numpairs; k++) {
			// Read the next 'n' bits as indicated by the compression variable
			// The multiply by 4 because there are 2 timing numbers per pair
			// and each timing number is one word long, so 4 bytes total!
			uint8_t ti = (read_bits(bitcompression)) * 4;

			// read the onTime and offTime from the program memory
			uint16_t ontime = pgm_read_word(time_ptr+ti);		// read word 1 - ontime
			uint16_t offtime = pgm_read_word(time_ptr+ti+2);	// read word 2 - offtime

			// transmit this codeElement (ontime and offtime)
			xmitCodeElement(ontime, offtime, (freq!=0));
		}

		// delay before transmitting next POWER code
		delay_ten_us(5000);
	}

	mode_pov();

	DDRA = 0x00;		// Turn off POV leds
	PORTA = _BV(BUTTONPIN);	// And pull-up the switch input

	// flash the visible LED on PB0  4 times to indicate that we're done
	delay_ten_us(-1);	// wait maxtime
	for(i = 0; i < 4; i++) {
		wdt_reset();
		flash_on();
		delay_ten_us(3000);	// 30 millisec delay
		flash_off();
		if(i == 3)
			break;
		delay_ten_us(15000);	// 150 millisec delay
	}
}

struct eeprom_data_t {
	uint16_t	pre_delay;	// Delay from switch detect before starting show
	uint16_t	int_delay;	// Inter-pixel delay
	uint16_t	nelem;		// Number of elements of data
	uint8_t		data[];
};

#ifdef USE_EEPROM
const struct eeprom_data_t pov_data EEMEM = {
#else
const struct eeprom_data_t pov_data PROGMEM = {
#endif
#if 0
	5000,
	200,
	82
	{
		0xff,0xff,0xff,0xff,0xc3,0xbd,0x7e,0x7e,0x7e,0xbd,0xc3,0xff,0xff,0x71,0x76,0x76,
		0x6e,0x8c,0xff,0xff,0x03,0xdc,0xde,0xdc,0x03,0xff,0xff,0x03,0xdc,0xde,0xdc,0x03,
		0xff,0xff,0xff,0xff,0xff,0x3e,0x1e,0x6e,0x76,0x78,0x7c,0xff,0xff,0x03,0xdc,0xde,
		0xdc,0x03,0xff,0xff,0x00,0x76,0x76,0x76,0x89,0xff,0xff,0x00,0x7f,0x7f,0x7f,0x7f,
		0xff,0x00,0x76,0x76,0x76,0x7e,0xff,0xfe,0xfe,0xfe,0x00,0xfe,0xfe,0xfe,0xff,0xff,
		0xff,0xff,
	}
#endif
#if 0
	5000,
	200,
	45,
	{
		0xff,0xff,0xff,0x00,0x76,0x76,0x76,0x89,0xff,0xff,0xff,0x87,0x6b,0x6b,0x6b,0x67,
		0xff,0xff,0xff,0x03,0xfb,0xfb,0xff,0xfb,0x00,0x7b,0x7b,0xff,0xff,0x00,0xf7,0xfb,
		0xfb,0x07,0xff,0xff,0xff,0x87,0x7b,0x7b,0x7b,0x87,0xff,0xff,0xff,
	}
#endif
#if 1
	5000,
	200,
	41,
	{
		0x3e,0x1e,0x6e,0x76,0x78,0x7c,0xff,0xff,0x03,0xdc,0xde,0xdc,0x03,0xff,0xff,0x00,
		0x76,0x76,0x76,0x89,0xff,0xff,0x00,0x7f,0x7f,0x7f,0x7f,0xff,0x00,0x76,0x76,0x76,
		0x7e,0xff,0xfe,0xfe,0xfe,0x00,0xfe,0xfe,0xfe,
	}
#endif
};


void povShow(uint8_t dir)
{
	uint16_t pre_delay;
	uint16_t int_delay;
	uint16_t nelem;

#ifdef USE_EEPROM
	pre_delay = eeprom_read_word(&pov_data.pre_delay);
	int_delay = eeprom_read_word(&pov_data.int_delay);
	nelem = eeprom_read_word(&pov_data.nelem);
#else
	pre_delay = pgm_read_word(&pov_data.pre_delay);
	int_delay = pgm_read_word(&pov_data.int_delay);
	nelem = pgm_read_word(&pov_data.nelem);
#endif
	PORTA = 0;
	DDRA = 0;

	wdt_reset();			// All should be done in 0.5s, so the dog should keep quiet
	delay_ten_us(pre_delay);	// Wait for the movement to pick up
	wuflags = 0;			// Next detection may be earlier than expected; setup detection
	wdt_reset();
	if(dir) {
		for(int16_t i = 0; i < nelem; i++) {
#ifdef USE_EEPROM
			DDRA = ~eeprom_read_byte(&pov_data.data[i]);	// Set the port data from eeprom
#else
			DDRA = ~pgm_read_byte(&pov_data.data[i]);	// Set the port data from eeprom
#endif
			delay_ten_us(int_delay);
		}
	} else {
		for(int16_t i = nelem-1; i >= 0; i--) {
#ifdef USE_EEPROM
			DDRA = ~eeprom_read_byte(&pov_data.data[i]);	// Set the port data from eeprom
#else
			DDRA = ~pgm_read_byte(&pov_data.data[i]);	// Set the port data from eeprom
#endif
			delay_ten_us(int_delay);
		}
	}
	DDRA = 0;
	PORTA = _BV(BUTTONPIN);
}

int main(void)
{
	DDRA  = 0x00;		// Port A to input (POV LEDs off)
	PORTA = _BV(BUTTONPIN);	// Leds off with pull-up on PA7 (switch)

	DDRB  = 0x00;		// Input on the combined POV input/TVBG IR LED output
	PORTB = 0x00;		// Disable pull-up. The pin is externally pulled up for TVBG

	TCCR0A = 0;		// Turn off PWM/freq gen, should be off already
	TCCR0B = 0;		// Turn off PWM/freq gen, should be off already

	start_timer1();

	uint8_t i = MCUSR;		// Save reset reason
	MCUSR = 0;			// clear watchdog flag
	WDTCSR = _BV(WDCE) | _BV(WDE);	// enable WDT disable
	WDTCSR = 0;			// disable WDT while we setup

	DEBUGP(putstring_nl("Hello!"));

	PCMSK0 = _BV(PCINT7);		// Enable pin change int on PA7
	PCMSK1 = _BV(PCINT10);		// Enable pinc change int on PB2
	GIMSK |= _BV(PCIE0) | _BV(PCIE1);
	sei();

	// check the reset flags
	if(i & _BV(BORF)) {
		// Brownout detected
		DEBUGP(putstring_nl("Battery low!"));

		// Flash out an error and go to sleep
		wdt_reset();
		brownout_on();
		delay_ten_us(50);
		brownout_off();
		delay_ten_us(50000);
		brownout_on();
		delay_ten_us(50);
		brownout_off();
		powerDown();
	}

	delay_ten_us(5000);		// Let everything settle for a bit

	while (1) {
		powerDown();		// Wait for the user to hit the button.
		wdt_enable(WDTO_8S);
		button = 0;

		if(wuflags & F_POV) {
			// wakeup flags are cleared delayed in the show
			povShow(PINB & _BV(IRLED));
		} else if(wuflags & F_BUTTON) {
			turnOffTVs();	// The button woke us, turn off those damn TVs
			wuflags = 0;
		}
	}
}


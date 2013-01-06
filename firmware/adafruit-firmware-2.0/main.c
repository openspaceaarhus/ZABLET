/*
TV-B-Gone Firmware version 2.0
for use with ATtiny85v and v2.0 hardware (see below)

**********  Version History  **********

v1.2	(c) Mitch Altman + Limor Fried 2009
	Last edits, August 16 2009
	With some code from Kevin Timmerman & Damien Good 7-Dec-07

v2.0	(c) PorkRhombus December 2012			**** This is a universal version with most of the US and EU codes ****

	Added modes and made a minor hardware change to support this. The switch to GND now connects to pin 6 (PB1) so there
	is no region select resistor. For EU operation, power up the device with the button pressed and immediately release.
	The debugging outputs have been removed from this version. Note that in v2.0 the pushbutton does not reset the
	device, it simply wakes it from sleep by means of a pin change interrupt. The mode and region settings are maintained
	until power is disconnected or low battery voltage causes a brownout reset. If using a rechargeable battery and
	charging through the ICP header, settings can be maintained indefinitely.

	Modes are as follows:

	Mode 1:	Default mode. Pressing the button causes all codes to be transmitted just like before.
		However the LED is now solid ON because the blinking draws too much unwanted attention.
		Also, the pause between codes has been reduced to 75ms. There are no region blinks or end-of-run blinks
		as I didn't find these remotely useful, nor indeed could I tell the difference between 3 and 4 rapid blinks.

	Mode 2: Codes are transmitted as long as the button is held down.
		Each time it is pressed, transmission restarts from code #1.

	Mode 3: Codes are transmitted one at a time. Each code is accompanied by a flash (double flash every multiple of 10).
		In this way one can determine which code is switching off a particular TV and the WORLDcodes.c file can then
		be edited, and the program recompiled, to change the transmission sequence.

	After power-up, the LED flashes for 2 seconds during which time any further press will cause the device to enter mode
	selection. Subsequent presses at this time cause the modes to be cycled through. To enter your selection, wait a few
	seconds after the last button press.

	Technical notes can be found a little lower down.


This is free software and no warranty is provided or implied. The authors are not responsible for use or abuse of the
software, nor do they accept any liability for any harm to man or beast arising therefrom, whether directly or indirectly.

		<<<<<<<<<<<<<<  Distributed under Creative Commons 2.5 -- Attribution & Share Alike >>>>>>>>>>>>>>>

That means that if you re-distribute all or part of this code, you should (a) credit the original authors and (b) likewise share
any projects that you base on it (include a notice like the one above in your distribution). For more info go to: 

						www.creativecommons.org. Thank you.


FINALLY ......... please use the TV-b-Gone as a force for GOOD in the universe.
There's no glory in switching off TVs in places where people have gathered specifically to watch them.

******************************************
*CA9620455B9CDC1BBB04094FA206C462FCFD1C3F*
******************************************
*/

#include <avr/io.h>             // this contains all the IO port definitions
#include <avr/eeprom.h>
#include <avr/sleep.h>          // definitions for power-down modes
#include <avr/pgmspace.h>       // definitions for keeping constants in program memory
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include "main.h"
#include "util.h"


/*
This project transmits a bunch of TV POWER codes, one right after the other, 
with a pause in between each.  (To have a visible indication that it is 
transmitting, it also pulses a visible LED once each time a POWER code is 
transmitted.)  That is all TV-B-Gone does.  The tricky part of TV-B-Gone 
was collecting all of the POWER codes, and getting rid of the duplicates and 
near-duplicates (because if there is a duplicate, then one POWER code will 
turn a TV off, and the duplicate will turn it on again (which we certainly 
do not want).  I have compiled the most popular codes with the 
duplicates eliminated, both for North America (which is the same as Asia, as 
far as POWER codes are concerned -- even though much of Asia USES PAL video) 
and for Europe (which works for Australia, New Zealand, the Middle East, and 
other parts of the world that use PAL video).

Before creating a TV-B-Gone Kit, I originally started this project by hacking 
the MiniPOV kit.  This presents a limitation, based on the size of
the Atmel ATtiny2313 internal flash memory, which is 2KB.  With 2KB we can only 
fit about 7 POWER codes into the firmware's database of POWER codes.  However,
the more codes the better! Which is why we chose the ATtiny85 for the 
TV-B-Gone Kit.

This version of the firmware has the most popular 100+ POWER codes for 
North America and 100+ POWER codes for Europe.
*/


/*
This project is a good example of how to use the AVR chip timers.
On the other hand, it sets a bad example by making extensive use of delays !!
*/


/*	T E C H N I C A L   N O T E S

The hardware for this project is very simple:

     ATtiny85 has 8 pins:

       pin 1   RST
       pin 2   one pin of 8MHz ceramic resonator
       pin 3   other pin of 8MHz ceramic resonator
       pin 4   ground
       pin 5   (OC1A) IR emitters, through a 1kOhm res to a PN2907 PNP driver that connects 
               to 4 (or more) PN2222A drivers
       pin 6   (PB1) pushbutton switch to GND (replaces region select resistor in v1.2)
       pin 7   (PB0) visible LED
       pin 8   3-5v DC (such as 2-3 AA alkaline batteries or a single Li-ion rechargeable)

    See the schematic for more details (but remember the switch S1 is now located where R3 is shown on the official schematic)

    Notes on Region Selection:

	With the standard 'everywhere' software build you cannot access ALL the codes for US (or EU) since it includes a subset (most,
	in fact) of each region. You can do a region-specific build by changing line 10 of the makefile to make it say NA_CODE = 1
	or EU_CODE = 1. Then you will get all the NA (or all the EU) codes and none of the others. You might think that this is
	pointless but in fact some useful codes are found further down. DYNEX, for example, was at position 134 in the NA list, so
	it was impossible to access even by bumping it up the transmit order. In fact the program would not compile because code 134,
	being in the 'extra' block, was not defined. You can only find out if the device has the code for your TV by building it
	region-specific so it transmits all the codes for your region, not just most of them.

    NOTE:

	In this case, programming the microcontroller in-circuit is not recommended because :

	(a) it generally results in failed writes and
	(b) it can result in damage to the IR leds, since they have no current limiting resistors.
  
    IMPORTANT: Tips for building and burning the software

	This project will not work with incorrect fuse settings. The fuse settings are:

	Lfuse=FD  Hfuse=DE  Efuse=FF  (these settings specify an external crystal or reso and a brownout level of 1.8V)

	Whereas the standard fuse settings on a fresh chip are 62/DF/FF

	Although you can use the 'make' utility to set the fuses, the most direct way is to open a command prompt and enter:

	avrdude -c usbtiny -p T85 -U lfuse:w:0xfd:m -U hfuse:w:0xde:m

	Obviously you need to have installed avrdude, and if you are using a programmer other than usbtiny/sparkfun pocket
	programmer, change that parameter accordingly. avrdude is the AVR Downloader/UploaDEr, used to set fuses and
	transfer compiled firmware ('hex files') to (and from) AVR devices.

	To compile and build the software (eg. if you have changed the transmission sequence in WORLDcodes.c), you need 
	to have installed WinAVR (which also includes avrdude). Just navigate to the firmware folder and key in:

	make all

	This will also burn the software to the chip if your programmer is correctly specified in the makefile.
	If that doesn't happen, just key in :

	avrdude -c usbtiny -p T85 -U flash:w:tvbgone.hex

	if you just want to see the fuse settings, use:

	avrdude -c usbtiny -p T85 -v  (the v stands for verbose output)

	Note that if (re)programming the chip out of the device (as recommended), you will need to have it connected to
	an 8MHz resonator or crystal once the new fuse settings are in.

	Also note that if you are using a li-ion rechargeable, you can set the brownout detect level to 2.7V rather than 1.8V
	by using DD rather than DE for the hfuse (because a lithium rechargeable should never be allowed to discharge that low).
	You can also get away with using a regular ATTINY85-20PU rather than the low-voltage version ATTINY85V-10PU. It's about
	half the price.

	Thanks to engbedded.com for their great AVR fuse configurator ... check it out.

	LBNL, massive props and shout-outs to Lady Ada. Woot woot!
*/



extern const PGM_P * const NApowerCodes[] PROGMEM;
extern const PGM_P * const EUpowerCodes[] PROGMEM;
extern const uint8_t num_NAcodes, num_EUcodes;


/* This function is the 'workhorse' of transmitting IR codes.
   Given the on and off times, it turns on the PWM output on and off
   to generate one 'pair' from a long code. Each code has ~50 pairs */

void xmitCodeElement(uint16_t ontime, uint16_t offtime, uint8_t PWM_code )
{
  // start Timer0 outputting the carrier frequency to IR emitters on and OC0A 
  // (PB0, pin 5)
  TCNT0 = 0; // reset the timers so they are aligned
  TIFR = 0;  // clean out the timer flags

  if(PWM_code) {
    // 99% of codes are PWM codes, they are pulses of a carrier frequecy
    // Usually the carrier is around 38KHz, and we generate that with PWM
    // timer 0
    TCCR0A =_BV(COM0A0) | _BV(WGM01);          // set up timer 0
    TCCR0B = _BV(CS00);
  } else {
    // However some codes dont use PWM in which case we just turn the IR
    // LED on for the period of time.
    SWITCH_ON(IRLED);
  }

  // Now we wait, allowing the PWM hardware to pulse out the carrier 
  // frequency for the specified 'on' time
  delay_ten_us(ontime);
  
  // Now we have to turn it off so disable the PWM output
  TCCR0A = 0;
  TCCR0B = 0;
  // And make sure that the IR LED is off too (since the PWM may have 
  // been stopped while the LED is on!)
  SWITCH_OFF(IRLED);

  // Now we wait for the specified 'off' time
  delay_ten_us(offtime);
}

/* This is kind of a strange but very useful helper function
   Because we are using compression, we index to the timer table
   not with a full 8-bit byte (which is wasteful) but 2 or 3 bits.
   Once code_ptr is set up to point to the right part of memory,
   this function will let us read 'count' bits at a time which
   it does by reading a byte into 'bits_r' and then buffering it. */

uint8_t bitsleft_r = 0;
uint8_t bits_r=0;
PGM_P code_ptr;

// we cant read more than 8 bits at a time so dont try.
uint8_t read_bits(uint8_t count)
{
  uint8_t i;
  uint8_t tmp=0;
  
  // we need to read back count bytes
  for (i=0; i<count; i++) {
    // check if the 8-bit buffer we have has run out
    if (bitsleft_r == 0) {
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


/*Normally all variables and constants are loaded into ram when the 
microcontroller boots up.  Since this firmware has a table (powerCodes) 
that is too large to transfer into RAM, the C compiler needs to be told to 
keep it in program memory space.  This is accomplished by the macro PROGMEM 
(this is used in the definition for powerCodes).  Since the C compiler assumes 
that constants are in RAM, rather than in program memory, when accessing
powerCodes, we need to use the pgm_read_word() and pgm_read_byte macros, and 
we need to use powerCodes as an address.  This is done with PGM_P, defined 
below.  
For example, when we start a new powerCode, we first point to it with the 
following statement:
    PGM_P thecode_p = pgm_read_word(powerCodes+i);
The next read from the powerCode is a byte that indicates the carrier 
frequency, read as follows:
      const uint8_t freq = pgm_read_byte(code_ptr++);
After that is a byte that tells us how many 'onTime/offTime' pairs we have:
      const uint8_t numpairs = pgm_read_byte(code_ptr++);
The next byte tells us the compression method. Since we are going to use a
timing table to keep track of how to pulse the LED, and the tables are 
pretty short (usually only 4-8 entries), we can index into the table with only
2 to 4 bits. Once we know the bit-packing-size we can decode the pairs
      const uint8_t bitcompression = pgm_read_byte(code_ptr++);
Subsequent reads from the powerCode are n bits (same as the packing size) 
that index into another table in ROM that actually stores the on/off times
      const PGM_P time_ptr = (PGM_P)pgm_read_word(code_ptr);
*/


int main(void) {
  uint16_t ontime, offtime, wait_time;

  byte region = US;     // by default our code is US
  byte initialized=NO;
  byte escape;
  byte input_status;		// for debouncing the pushbutton

  byte mode=1;			// mode 1: default. Push to run through complete library.
				// mode 2: codes are only transmitted as long as the button is depressed
				// mode 3: codes are transmitted one by one (one per click). every 10th code
				// is indicated by a double flash so the user can keep track of which code
				// turned off the target. Then that code can be moved up the firing order if desired
				// by editing WORLDcodes.c. If you wait more than 5sec between clicks the sequence 
				// starts again from the top.
				// Mode can be set upon connecting power to the device. Thereafter it will be
				// retained until power is disconnected. The pushbutton no longer resets the device
				// but wakes it by means of a pin change interrupt.

  byte i,j;

  TCCR1 = 0;		   // Turn off PWM/freq gen, should be off already
  TCCR0A = 0;
  TCCR0B = 0;

  i = MCUSR;                     // Save reset reason
  MCUSR = 0;                     // clear watchdog flag
  WDTCR = _BV(WDCE) | _BV(WDE);  // enable WDT disable  ??? Hunh

  WDTCR = 0;                     // disable WDT while we setup

  DDRB = _BV(LED) | _BV(IRLED);   // set the visible and IR LED pins to outputs
  PORTB = _BV(LED) |              //  visible LED is off when pin is high
          _BV(IRLED) |            // IR LED is off when pin is high
          _BV(PUSHBUTTON);        // Turn on pullup on pushbutton input
 
  // check the reset flags

  while ((i & _BV(BORF)) && !(i & _BV(PORF))) {		// Brownout (the device reset itself because Vcc dropped below
							// 1.8V, probably when it tried to start transmitting)
      flashslowLEDx(2);	
      tvbgone_sleep();  				// Unlike before, it's now locked in to brownout/low-voltage
  }							// warning mode and will not try to transmit anything again
							// until a different type of reset occurs (external or power-on)
  delay_ten_us(2500);


  while(BUTTON_IS_PRESSED) {				// The only way we could be here is if we just powered up, in
	region=EU;					// which case the button should not be pressed unless the user
	delay_ten_us(2500);				// means to select EU region.
  }
 
  	// Flash LED and check for button press

	for(byte p=0; p<200; p++) {	// 2 second chance to change mode on power-up

		if(BUTTON_IS_RELEASED) input_status=1;
		else if(input_status==1) {
			input_status=2;
			break;
		}

		delay_ten_us(1000);
		if(!(p%10)) {		// every 10th time through the loop
			TOGGLE(LED);
			wdt_reset();
		}
	}

	SWITCH_OFF(LED);

	if(input_status==2) {		// Set the mode

		input_status=0;

		for(byte q=0; q<5; q++) {	// max 5 flashes before the device continues in the selected mode


			for(byte r=0; r<100; r++) {

				if(!(r&15)) wdt_reset();	// i.e. if r is a multiple of 16 / every 16th time through

				delay_ten_us(1000);

				if(BUTTON_IS_RELEASED) input_status=1;
				else if(input_status==1) {
					input_status=0;
					mode++;
					if (mode==4) mode=1;
					q=0;	// reset the time-out
					break;
				}

			}

			quickflashLEDx(mode);
		}

	}

  
	// turn on watchdog timer immediately, this protects against
	// a 'stuck' system by resetting it
	wdt_enable(WDTO_8S); // 1 second long timeout

	while(1) {

		// We may have different number of codes in either database
		if (region == US) j = num_NAcodes;
		else j = num_EUcodes;

		input_status=0;
		escape=NO;
		
		if(initialized) {			// don't transmit codes if the device has just powered up

			for(i=0; i<j; i++) {		// transmit the codes in the order they are listed at the
							// end of WORLDcodes.c

				wdt_reset();
  							    // point to next POWER code, from the right database

   				if (region == US) code_ptr = (PGM_P)pgm_read_word(NApowerCodes+i);  
				else code_ptr = (PGM_P)pgm_read_word(EUpowerCodes+i);  

      
				// Read the carrier frequency from the first byte of code structure
				const uint8_t freq = pgm_read_byte(code_ptr++);
				// set OCR for Timer1 to output this POWER code's carrier frequency

				OCR0A = freq; 
      
     
				// Get the number of pairs, the second byte from the code struct
				const uint8_t numpairs = pgm_read_byte(code_ptr++);

				// Get the number of bits we use to index into the timer table
				// This is the third byte of the structure
				const uint8_t bitcompression = pgm_read_byte(code_ptr++);

				// Get pointer (address in memory) to pulse-times table
				// The address is 16-bits (2 byte, 1 word)
				const PGM_P time_ptr = (PGM_P)pgm_read_word(code_ptr);
				code_ptr+=2;

				// Transmit all codeElements for this POWER code 
				// (a codeElement is an onTime and an offTime)
				// transmitting onTime means pulsing the IR emitters at the carrier 
				// frequency for the length of time specified in onTime
				// transmitting offTime means no output from the IR emitters for the 
				// length of time specified in offTime

 
				// For EACH pair in this code....
				for (uint8_t k=0; k<numpairs; k++) {
					uint8_t ti;

					// Read the next 'n' bits as indicated by the compression variable
					// Then multiply by 4 because there are 2 timing numbers per pair
					// and each timing number is one word long, so 4 bytes total

					ti = (read_bits(bitcompression)) * 4;

					// read the onTime and offTime from the program memory

					ontime = pgm_read_word(time_ptr+ti);  // read word 1 - ontime
					offtime = pgm_read_word(time_ptr+ti+2);  // read word 2 - offtime

					// transmit this codeElement (ontime and offtime)

					xmitCodeElement(ontime, offtime, (freq!=0));  
      				} 
      
				//Flush remaining bits, so that next code starts with a fresh set of 8 bits.

				bitsleft_r=0;

				SWITCH_ON(LED);					// indicates device is transmitting

				switch(mode) {

					case 1:
						delay_ten_us(7500);		// wait 75ms before transmitting next code
					break;

					case 2:
						if(BUTTON_IS_RELEASED) escape=YES;
						else delay_ten_us(7500);	// wait 75ms before transmitting next code
					break;

					case 3:
						if(!((i+1)%10)) quickflashLEDx(2);  // dbl-flash to indicate every 10th code
						else quickflashLEDx(1);

						input_status=0;

						for(wait_time=0; wait_time<500; wait_time++) {
							if(!(wait_time&15)) wdt_reset();	// every 16th time through loop
							delay_ten_us(1000);
							if(BUTTON_IS_RELEASED) input_status=1;
							else if(input_status==1) break;

						}

						if(wait_time>=500) escape=YES;	// more than 5 seconds since last press
						
					break;
				}

				if(escape==YES) break;	// A time-out has occurred in mode 3 or the button has been
							// released in mode 2. Either way, Kansas is going bye-byes.
			}

		}


		tvbgone_sleep();	// inside this function is where we go to sleep and wake up again

		initialized=YES;	// if we have got to this point we must have completed the mode set-up,
					// so it's OK to go into the transmission loop now.
	}

}


/****************************** SLEEP FUNCTIONS ********/

void tvbgone_sleep(void)
{
  // Shut down everything and put the CPU to bed. Then when it wakes give it a good breakfast.

  TCCR0A = 0;           // turn off frequency generator (should be off already)
  TCCR0B = 0;           // turn off frequency generator (should be off already)
  SWITCH_OFF(LED);       // turn off visible LED
  SWITCH_OFF(IRLED);     // turn off IR LED

  while(BUTTON_IS_PRESSED) {  // get your sweaty thumb off that button, please ...
    delay_ten_us(5000);
    wdt_reset();	// .. this could take a while
  }


  delay_ten_us(5000);

  PCMSK|=_BV(PCINT1);	// enable pin change interrupt on PB1. This will allow the pushbutton to wake the MCU.
  GIMSK|=_BV(PCIE);

  sei();				// global enable interrupts

  wdt_disable();           // turn off the watchdog since we want to sleep.

  delay_ten_us(1000);      // wait 10 millisec

  MCUCR = _BV(SM1) |  _BV(SE);    // power down mode,  SE enables Sleep Modes
  sleep_cpu();                    // put CPU into Power Down Sleep Mode

  delay_ten_us(1000);

  // GIMSK&=~_BV(PCIE);
  cli();					// we might as well disable interrupts globally since this is the only one we are using.

  wdt_enable(WDTO_8S);		// re-enable 1 second watchdog timer
}


/****************************** LED AND DELAY FUNCTIONS ********/


// This function delays the specified number of 10 microseconds
// it is 'hardcoded' and is calibrated by adjusting DELAY_CNT 
// in main.h Unless you are changing the crystal from 8mhz, dont
// mess with this.
void delay_ten_us(uint16_t us) {
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


// This function quickly pulses the visible LED (connected to PB0, pin 5)
// This will indicate to the user that a code is being transmitted

void quickflashLED(void) {
  SWITCH_ON(LED);   // turn on visible LED at PB0 by pulling pin to ground
  delay_ten_us(3000);   // 30 millisec delay
  SWITCH_OFF(LED);    // turn off visible LED at PB0 by pulling pin to +3V
}



// This function just flashes the visible LED a couple times, used to
// tell the user what region is selected

void quickflashLEDx(uint8_t x) {
  quickflashLED();
  while(--x) {
  	wdt_reset();
	delay_ten_us(15000);     // 150 millisec delay between flahes
	quickflashLED();
  }
  wdt_reset();                // kick the dog
}



// This is like the above but way slower, used for when the tvbgone
// crashes and wants to warn the user
void flashslowLEDx( uint8_t num_blinks )
{
  uint8_t i;
  for(i=0;i<num_blinks;i++)
    {
      SWITCH_ON(LED);    
      delay_ten_us(50000);         // 500 millisec delay
      wdt_reset();                 // kick the dog
      SWITCH_OFF(LED);          
      delay_ten_us(50000);	   // 500 millisec delay
      wdt_reset();                 // kick the dog
    }
}

EMPTY_INTERRUPT(PCINT0_vect);		// when using an interrupt to wake up, it is necessary to define the ISR even if it is null


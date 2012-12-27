#ifndef __TVBG_MAIN_H
#define __TVBG_MAIN_H

// What pins do what
#define FLASHLED	PA0
#define BOLED		PA1
#define IRLED		PB2

// Two regions!
#define US 1
#define EU 0

// Lets us calculate the size of the NA/EU databases
#define NUM_ELEM(x) (sizeof (x) / sizeof (*(x)));

// set define to 0 to turn off debug output
#define DEBUG 0
#define DEBUGP(x) if (DEBUG == 1) { x ; }


// Shortcut to insert single, non-optimized-out nop
#define NOP __asm__ __volatile__ ("nop")

// Tweak this if neccessary to change timing
#define DELAY_CNT 11

// Makes the codes more readable. the OCRA is actually programmed in terms of
// 'periods' not 'freqs' - that is, the inverse!
#define freq_to_timerval(x) ((F_CPU / x - 1)/ 2)

#define PULSE_CODE 0

// The structure of compressed code entries
struct IrCode {
  uint8_t timer_val;
  uint8_t numpairs;
  uint8_t bitcompression;
  uint16_t const *times;
  uint8_t codes[];
};

extern const struct IrCode * const NApowerCodes[];
extern const struct IrCode * const EUpowerCodes[];
extern const uint8_t num_NAcodes;
extern const uint8_t num_EUcodes;

#endif

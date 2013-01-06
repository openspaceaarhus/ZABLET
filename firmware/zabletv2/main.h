#ifndef __TVBG_MAIN_H
#define __TVBG_MAIN_H

// What pins do what
#define FLASHLED	PA0
#define BOLED		PA1
#define IRLED		PB2
#define BUTTONPIN	PA7

// Two regions!
#define US 1
#define EU 0

// Lets us calculate the size of the NA/EU databases
#define NUM_ELEM(x) (sizeof (x) / sizeof (*(x)))

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

#ifdef ALL_CODES
extern const struct IrCode * const AllpowerCodes[];
extern const uint16_t num_Allcodes;
#endif
#ifdef NA_CODES
extern const struct IrCode * const NApowerCodes[];
extern const uint8_t num_NAcodes;
#endif
#ifdef EU_CODES
extern const struct IrCode * const EUpowerCodes[];
extern const uint8_t num_EUcodes;
#endif

struct eeprom_data_t {
	uint16_t	pre_delay;	// Delay from switch detect before starting show
	uint16_t	int_delay;	// Inter-pixel delay
	uint16_t	nelem;		// Number of elements of data
	uint8_t		data[];
};

#ifdef USE_EEPROM
extern const struct eeprom_data_t pov_data;
#else
extern const struct eeprom_data_t pov_data;
#endif

#endif

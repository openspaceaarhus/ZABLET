// What PORTB pins do what

#define IRLED 0
#define PUSHBUTTON 1
#define LED 2

#define US 0
#define EU 1

#define NO 0
#define YES 255

#define byte uint8_t
#define BUTTON_IS_RELEASED PINB&_BV(PUSHBUTTON)
#define BUTTON_IS_PRESSED !(PINB&_BV(PUSHBUTTON))

#define SWITCH_ON(x) PORTB&=~_BV(x)
#define SWITCH_OFF(x) PORTB|=_BV(x)
#define TOGGLE(x) PINB=_BV(x)

// Lets us calculate the size of the NA/EU databases
#define NUM_ELEM(x) (sizeof (x) / sizeof (*(x)));

// set define to 0 to turn off debug output
#define DEBUG 0
#define DEBUGP(x) if (DEBUG == 1) { x ; }


// Shortcut to insert single, non-optimized-out nop
#define NOP __asm__ __volatile__ ("nop")

// Tweak this if neccessary to change timing
#define DELAY_CNT 11

// Makes the codes more readable. the OCRA is actually
// programmed in terms of 'periods' not 'freqs' - that
// is, the reciprocal
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

void xmitCodeElement(uint16_t ontime, uint16_t offtime, uint8_t PWM_code );
void flashslowLEDx( uint8_t num_blinks );
void quickflashLEDx( uint8_t x );
void tvbgone_sleep( void );
void delay_ten_us(uint16_t us);
void quickflashLED( void );

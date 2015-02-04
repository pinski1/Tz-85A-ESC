/**
 *
 *
 * Fuses: HIGH = 0xCF, LOW = 0x2E
 */
#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
//#include <math.h>
#include <avr/eeprom.h>

/** Mode selection */
#define MODE_ESC
//#define MODE_SERVO
//#define CALIBRATE

/** Defaults */
const int fetDelay = 200; // 200 microseconds
const int defaultMinPulse = 2000; // 1 millisecond, in 0.5us ticks
const int defaultMaxPulse = 4000; // 2 millisecond, in 0.5us ticks
const int defaultMinPosition = 0; // pot fully down
const int defaultMaxPosition = 255; // pot fully up
const int kProportional = 1;
const int kDifferential = 0;
const int maxSlew = 30;

/** Globals */
enum _state {forwards, brake, backwards} requestedState, currentState;
enum _temp {cool, warm, hot} tempState;
volatile int requestedVelocity = 0; // global speed & direction
volatile int requestedPosition = 0; // global requested position
volatile int currentPosition = 0;   // global current position
volatile char timeout = 0;
volatile char failsafe = 0;
int minPulse = 0;
int maxPulse = 0;
int minPosition = 0;
int maxPosition = 0;

/** Pin Map */
#define enbRCpin (DDRD &= !_BV(2)) // Port D, Pin 2 - RC signal input
#define readRCpin (PIND &= _BV(2))
#define selTempPin 0b00000001 // Port C, Pin 1 - temperature sensor input
#define selPosPin 0b00000010 // Port C, Pin 2 - position sensor input
#define enbForwardHigh (DDRC |= _BV(5)) // Port C, Pin 5 - high side gate, forward
#define setForwardHigh (PORTC |= _BV(5))
#define clrForwardHigh (PORTC &= !_BV(5))
#define enbForwardLow (DDRD |= _BV(5)) // Port D, Pin 5 - low side gate, forwards
#define setForwardLow (PORTD |= _BV(5))
#define clrForwardLow (PORTD &= !_BV(5))
#define enbBackwardHigh (DDRD |= _BV(4)) // Port D, Pin 4 - high side gate, backwards
#define clrBackwardHigh (PORTD |= _BV(4))
#define setBackwardHigh (PORTD &= !_BV(4))
#define enbBackwardLow (DDRC |= _BV(4))// Port C, Pin 4 - low side gate, backwards
#define clrBackwardLow (PORTC |= _BV(4))
#define setBackwardLow (PORTC &= !_BV(4))

/** Prototypes */
long map(long x, long in_min, long in_max, long out_min, long out_max);
void goForwards(void);
void goBackwards(void);
void braking(void);
void motorBeep(unsigned char length);


int main(void) {
	cli(); // disable global interrupts

	// H-Bridge pin set up
	enbForwardHigh;
	enbForwardLow;
	enbBackwardHigh;
	enbBackwardLow;

	// RC pin set up
	enbRCpin;
	MCUCR |= _BV(ISC00); // RC pin change triggers interrupt
	GICR  |= _BV(INT0); // enable RC pin interrupt

	// Timer 0 set up - control loop interrupt
	TCCR0 |= _BV(CS01) | _BV(CS00); // set to 250KHz, overflows @ 1KHz

	// Timer 1 set up - RC pulse timing
	TCCR1A = 0x00;
	TCCR1B = _BV(CS11); // set to 2MHz, overflows @ 32.8ms, normal mode

	// Timer 2 set up - H-Bridge PWM
	OCR2  = 0x00;
	TCCR2 = _BV(CS21); // set to 2MHz, overflows @ 7.8KHz, normal mode, OC2 disconnected

	// Timer interrupts
	TIFR  = 0x00; // clear flags
	TIMSK = _BV(OCIE2) | _BV(TOIE2) | _BV(TOIE1) | _BV(TOIE0); // enable all timers for overflow and timer2 match

	/** Read EEPROM for saved values, if empty (0xFF) use defaults. */
	minPulse = eeprom_read_word((unsigned int*)0x0000);
	if((unsigned int)minPulse == 0xFFFF) minPulse = defaultMinPulse;
	maxPulse = eeprom_read_word((unsigned int*)0x0002);
	if((unsigned int)maxPulse == 0xFFFF) maxPulse = defaultMaxPulse;
	minPosition = eeprom_read_word((unsigned int*)0x0004);
	if((unsigned int)minPosition == 0xFFFF) minPosition = defaultMinPosition;
	maxPosition = eeprom_read_word((unsigned int*)0x0006);
	if((unsigned int)maxPosition == 0xFFFF) maxPosition = defaultMaxPosition;

	// initialise states
	requestedState = brake;
	currentState = brake;
	tempState = cool;
	braking();

	_delay_ms(100);

	// ADC set up
	ADCSRA |= _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0); // set to 125KHz sample rate
	ADMUX  |= _BV(REFS0) | _BV(ADLAR); // Set ADC reference to AVCC, left adjust ADC result
	// need to enable, enable interrupts and free running mode

	sei(); // enable global interrupts

	#ifdef CALIBRATE

	// temperature limits
	// exponential?
	// slew rate

	eeprom_write_word((unsigned int*)0x0000, 0xFFFF); // new minPulse
	eeprom_write_word((unsigned int*)0x0002, 0xFFFF); // new maxPulse
	eeprom_write_word((unsigned int*)0x0004, 0xFFFF); // new minPosition
	eeprom_write_word((unsigned int*)0x0006, 0xFFFF); // new maxPosition
	#endif

	motorBeep(3);

	while(1);
	// all following actions are carried out as the result of interrupts

	return 0;
}

/** Motor value calculations
 *
 */
ISR(TIMER0_OVF_vect) {

	#ifdef MODE_SERVO
	static int oldError = 0, currentError = 0;
	oldError = currentError;

	// sum
	currentError = requestedPosition - currentPosition;

	// PD calculation
	requestedVelocity   = kProportional * currentError;
	requestedVelocity  += kDifferential * (currentError - oldError);
	#endif

	// clamp
	if(requestedVelocity > 255) requestedVelocity = 255;
	if(requestedVelocity < -255) requestedVelocity = -255;

	// temperature management
	if(requestedVelocity < -128 && tempState == warm) requestedVelocity = -128;
	if(requestedVelocity > 128 && tempState == warm) requestedVelocity = 128;

	// fault management
	if(timeout >= 15 || failsafe >= 15) requestedVelocity = 0;

	// convert requestedVelocity to requestedState
	if(requestedVelocity < 0)
	{
		requestedState = backwards;
		requestedVelocity = -requestedVelocity; // make positive value 0 - 255
	}
	else if(requestedVelocity > 0) requestedState = forwards;
	else requestedState = brake;

	// modify outputs
	if(requestedState != currentState)
	{
		if(currentState == brake) // brake to fwd or brake to bwd
		{
			// set direction
			currentState = requestedState;

			// set pins
			if(currentState == forwards) goForwards();
			else goBackwards();

			// requestedState = currentState, speed increases will be done elsewhere.
		}
		else // fwd to bwk or bwk to fwd
		{
			// decrease speed
			if(OCR2 > maxSlew) OCR2 -= maxSlew;
			else
			{
				// close enough to jump to brake state
				OCR2 = 0;
				currentState = brake;
				braking();
			}
		}
	}
	else // fwd to fwd, brake to brake, bwd to bwd
	{
		if(requestedVelocity > OCR2)
		{
			if(requestedVelocity > (OCR2 + maxSlew)) OCR2 += maxSlew;
			else OCR2 = requestedVelocity;
		}
		else if(requestedVelocity < OCR2)
		{
			if(requestedVelocity < (OCR2 - maxSlew)) OCR2 -= maxSlew;
			else OCR2 = requestedVelocity;
		}
		// else do nothing
	}
}

/**
 *
 */
ISR(TIMER1_OVF_vect) {
	// no pulse received in 32.767 milliseconds
	if(timeout < 30) timeout ++;
}

/** RC pulse capture
 *
 */
ISR(INT0_vect) {

	unsigned int pulseLength;

	if(readRCpin) TCNT1 = 0x0000; // clear timer1
	else
	{
		pulseLength = TCNT1; // copy timer1

		if(pulseLength >= (unsigned int)minPulse && pulseLength <= (unsigned int)maxPulse) // valid pulse?
		{
			// valid pulse, therefore map to requested variable
			#ifdef MODE_SERVO
				requestedPosition = map(pulseLength, minPulse, maxPulse, 0, 512); // map pulse to position
				currentPosition = map(ADC_pin, minPosition, maxPosition, 0, 512); // get & map error signal
			#else // default to ESC mode
				requestedVelocity = map(pulseLength, minPulse, maxPulse, -255, 255); // output value in -255 to +255
			#endif

			if(timeout < 3) timeout = 0; // clear timeout 3 times faster than it accumulates
			else timeout -= 3;

			if(failsafe < 3) failsafe = 0; // clear failsafe 3 times faster than it accumulates
			else failsafe -= 3;
		}
		else
		{
			// invalid pulse length
			if(failsafe < 30) failsafe ++;
		}
	}
}

/** When comparator is triggered clear low ports (off part of PWM cycle)
 *
 */
ISR(TIMER2_COMP_vect) {

	if (OCR2 < 255)
	{
        if (currentState != brake)
		{
			clrForwardHigh; // clear high side gates
            clrBackwardHigh;
        }
    }
}

/** When timer overflows enable low port for current state
 *
 */
ISR(TIMER2_OVF_vect) {

	static unsigned char counterCharge = 0;
    if (OCR2 > 0)
	{
        if (currentState == forwards) setForwardHigh;
        else if (currentState == backwards) setBackwardHigh;
    }

	if (OCR2 > 250)
	{
        // The IR2101 gate drivers don't have internal charge pump, so
		// need to drop high side every ~6.5 milliseconds to recharge.
		if (counterCharge > 50)
        {
			clrForwardHigh;
            clrBackwardHigh;
            counterCharge = 0;
        }
		else counterCharge ++;
    }
}

/** Re-maps a number from one range to another.
 * Copied from http://arduino.cc/en/reference/map
 * @param x The number to map.
 * @param in_min The lower bound of the number's current range.
 * @param in_max The upper bound of the number's current range.
 * @param out_min The lower bound of the target range.
 * @param out_max The upper bound of the target range.
 * @returns The mapped number.
 */
long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
 *
 */
void goForwards(void) {

	cli();
	clrBackwardHigh;
	clrBackwardLow;
	_delay_us(fetDelay);
	setForwardLow;
	_delay_us(fetDelay);
	setForwardHigh;
	sei();
}

/**
 *
 */
void goBackwards(void) {

	cli();
	clrForwardHigh;
	clrForwardLow;
	_delay_us(fetDelay);
	setBackwardLow;
	_delay_us(fetDelay);
	setBackwardHigh;
	sei();
}

/**
 *
 */
void braking(void) {

	cli();
	clrForwardHigh;
	clrBackwardHigh;
	_delay_us(fetDelay);
	setForwardLow;
	setBackwardLow;
	_delay_ms(10);
	sei();
}

/**
 *
 */
void motorBeep(unsigned char length) {
	OCR2 = 10;
	unsigned char countBeep = 0;
	while (countBeep < length)
	{
		if (countBeep % 2)
		{
			goForwards();
		}
		else
		{
			goBackwards();
		}
		_delay_ms(200);
		countBeep++;
		braking();
		_delay_ms(300);
	}
	braking();
	OCR2 = 0;
}


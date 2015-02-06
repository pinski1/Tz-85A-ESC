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
const unsigned int fetDelay = 200; // 200 microseconds
const int defaultMinPulse = 2000; // 1 millisecond, in 0.4us ticks
const int defaultMaxPulse = 4000; // 2 millisecond, in 0.4us ticks
const int defaultMinPosition = 0; // pot fully down
const int defaultMaxPosition = 255; // pot fully up
const unsigned char defaultTempHot = 170;
const unsigned char defaultTempWarm = 180;
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
unsigned char temperatureHot = 0;
unsigned char temperatureWarm = 0;

/** Pin Map */
#define readRCpin (PIND &= _BV(2)) // Port D, Pin 2 - RC signal input
#define selTempPin 0b00000001 // Port C, Pin 1 - temperature sensor input
#define selPosPin 0b00000010 // Port C, Pin 2 - position sensor input
#define enbForwardHigh (DDRC |= _BV(5)) // Port C, Pin 5 - high side gate, forward
#define setForwardHigh (PORTC |= _BV(5))
#define clrForwardHigh (PORTC &= ~_BV(5))
#define enbForwardLow (DDRD |= _BV(5)) // Port D, Pin 5 - low side gate, forwards
#define setForwardLow (PORTD |= _BV(5))
#define clrForwardLow (PORTD &= ~_BV(5))
#define enbBackwardHigh (DDRD |= _BV(4)) // Port D, Pin 4 - high side gate, backwards
#define setBackwardHigh (PORTD |= _BV(4))
#define clrBackwardHigh (PORTD &= ~_BV(4))
#define enbBackwardLow (DDRC |= _BV(4)) // Port C, Pin 4 - low side gate, backwards
#define setBackwardLow (PORTC |= _BV(4))
#define clrBackwardLow (PORTC &= ~_BV(4))

/** Prototypes */
long map(long x, long in_min, long in_max, long out_min, long out_max);
void goForwards(void);
void goBackwards(void);
void braking(void);
void motorBeep(unsigned char length);
unsigned char readAnalogue(void);


int main(void) {
	cli(); // disable global interrupts

	_delay_ms(10); // let the power settle

	// H-Bridge pin set up
	enbForwardHigh;
	enbForwardLow;
	enbBackwardHigh;
	enbBackwardLow;

	// RC pin set up
	MCUCR |= _BV(ISC00); // RC pin change triggers interrupt
	GICR  |= _BV(INT0); // enable RC pin interrupt

	// Timer 0 set up - control loop interrupt
	TCNT0 = 0x00;
	TCCR0 |= _BV(CS01) | _BV(CS00); // set to 250KHz, overflows @ 1KHz

	// Timer 1 set up - RC pulse timing
	TCNT1 = 0x0000;
	TCCR1A = 0x00;
	TCCR1B = _BV(CS11); // set to 2MHz, overflows @ 32.8ms, normal mode

	// Timer 2 set up - H-Bridge PWM
	OCR2  = 0x00;
	TCNT2 = 0x00;
	TCCR2 = _BV(CS21); // set to 2MHz, overflows @ 7.8KHz, normal mode, OC2 disconnected

	// Timer interrupts
	TIFR  = 0x00; // clear flags
	TIMSK = _BV(OCIE2) | _BV(TOIE2) | _BV(TOIE1) | _BV(TOIE0); // enable all timers for overflow and timer2 match

	/** Read EEPROM for saved values, if empty (0xFF) use defaults. */
	minPulse = eeprom_read_word((unsigned int*)0x0001);
	if((unsigned int)minPulse == 0xFFFF) minPulse = defaultMinPulse;
	maxPulse = eeprom_read_word((unsigned int*)0x0003);
	if((unsigned int)maxPulse == 0xFFFF) maxPulse = defaultMaxPulse;
	minPosition = eeprom_read_word((unsigned int*)0x0005);
	if((unsigned int)minPosition == 0xFFFF) minPosition = defaultMinPosition;
	maxPosition = eeprom_read_word((unsigned int*)0x0007);
	if((unsigned int)maxPosition == 0xFFFF) maxPosition = defaultMaxPosition;
	temperatureHot = eeprom_read_byte((unsigned char*)0x08);
	if(temperatureHot == 0xFF) temperatureHot = defaultTempHot;
	temperatureWarm = eeprom_read_byte((unsigned char*)0x09);
	if(temperatureWarm == 0xFF) temperatureWarm = defaultTempWarm;

	// initialise states
	tempState = cool;
	requestedState = brake;
	currentState = brake;
	braking();

	_delay_ms(100);

	// ADC set up
	ADCSRA |= _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0); // set to 125KHz sample rate
	ADMUX  |= _BV(REFS0) | _BV(ADLAR); // Set ADC reference to AVCC, left adjust ADC result
	ADCSRA |= _BV(ADEN)  | _BV(ADIE) | _BV(ADIF); // enable ADC & interrupts, clear flag
	ADCSRA |= _BV(ADFR)  | _BV(ADSC); // enable free running & start

	sei(); // enable global interrupts

	// Finished initializing the Motor Controller

	#ifdef CALIBRATE

	unsigned char counterCal = 0;

	if(failsafe < 10)
	{
		// receiving valid pulses
		if(requestedVelocity > 100 && requestedState == forwards)
		{
			// stick at max
			while(counterCal < 199 && requestedVelocity > 100 && requestedState == forwards)
			{
				// check it stays at max for 2 seconds
				_delay_ms(10);
				counterCal++;
			}

			if(counterCal > 190)
			{

				// calibrate RC

				// calibrate Slew Rate

				// calibrate Expo

				// calibrate Temp

				#ifdef MODE_SERVO

				// calibrate pot

				#endif // MODE_SERVO

				eeprom_write_word((unsigned int*)0x0001, 0xFFFF); // new minPulse
				eeprom_write_word((unsigned int*)0x0003, 0xFFFF); // new maxPulse
				eeprom_write_word((unsigned int*)0x0005, 0xFFFF); // new minPosition
				eeprom_write_word((unsigned int*)0x0007, 0xFFFF); // new maxPosition
				eeprom_write_byte((unsigned char*)0x008, 0xFF);   // new temperatureHot
				eeprom_write_byte((unsigned char*)0x009, 0xFF);   // new temperatureWarm

				// signal completed calibration
				motorBeep(2);

			}
		}
		motorBeep(2);
	}
	else motorBeep(1);
	// add a retry functionality?

	_delay_ms(500);

	#endif // CALIBRATE

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
	#endif // MODE_SERVO

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
			// set pins & direction
			if(requestedState == forwards) goForwards();
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
				unsigned char readPosition = readAnalogue();
				currentPosition = map(readPosition, minPosition, maxPosition, 0, 512); // get & map error signal
			#else // default to ESC mode
				requestedVelocity = map(pulseLength, minPulse, maxPulse, -255, 255); // output value in -255 to +255
			#endif // MODE_SERVO

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
        // The IR2101 gate drivers don't have an internal charge pump,
		// so need to drop high side every ~6.5 milliseconds to recharge.
		if (counterCharge > 50)
        {
			clrForwardHigh;
            clrBackwardHigh;
            counterCharge = 0;
        }
		else counterCharge ++;
    }
}

/**
 *
 */
ISR(ADC_vect) {

	if (ADCH < temperatureHot)
	{
		tempState = hot;
		requestedState = brake;
	}
	else if (ADCH < temperatureWarm) tempState = warm;
	else tempState = cool;
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
	currentState = forwards;
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
	currentState = backwards;
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
	currentState = brake;
	_delay_ms(10);
	sei();
}

/**
 *
 */
void motorBeep(unsigned char length) {

	unsigned char countBeep = 0;
	unsigned char savedTimer0Ctrl = TCCR0;
	TCCR0 = 0x00; // clear & stop timer0
	OCR2 = 10;

	while (countBeep < length)
	{
		if (countBeep % 2) goForwards();
		else goBackwards();

		_delay_ms(200);
		countBeep++;
		braking();
		_delay_ms(300);
	}

	OCR2 = 0;

	braking();

	TCCR0 = savedTimer0Ctrl; // restart timer0

}

/**
 *
 */
unsigned char readAnalogue(void) {

	cli();

	ADCSRA &= ~_BV(ADFR); // stop free running ADC
	ADMUX = (ADMUX & 0xF0) | selPosPin; // select pot, keep rate the same

	while(ADCSRA & ADSC); // wait for ADC to finish
	ADCSRA |= _BV(ADSC); // start conversion
	while(ADCSRA & ADSC); // wait for ADC to finish

	unsigned char adcValue = ADCH;

	ADMUX = (ADMUX & 0xF0) | selTempPin; // select temp, keep rate the same
	ADCSRA |= _BV(ADFR); // start free running ADC
	ADCSRA |= _BV(ADSC) | _BV(ADIF); // start first conversion & clear interrupt flag

	sei();
	return adcValue;
}


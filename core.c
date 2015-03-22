/**
 *
 *
 * Fuses: HIGH = 0xCF, LOW = 0x2E
 */
#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>

/** Mode selection */
//#define MODE_ESC
#define MODE_ENDSTOP
//#define CALIBRATE

/** Saved values in EEPROM */
unsigned  int EEMEM savedMinPulse;
unsigned  int EEMEM savedMaxPulse;
unsigned  int EEMEM savedMinPosition;
unsigned  int EEMEM savedMaxPosition;
unsigned char EEMEM savedTempHot;
unsigned char EEMEM savedTempWarm;

/** Default values */
const unsigned int fetDelay = 200; // in microseconds
const int defaultMinPulse = 2000; // 1 millisecond, in 0.5us ticks
const int defaultMaxPulse = 4000; // 2 millisecond, in 0.5us ticks
const int defaultMinPosition = 0; // pot fully down
const int defaultMaxPosition = 255; // pot fully up
const unsigned char defaultTempHot = 170;
const unsigned char defaultTempWarm = 180;
const int maxSlew = 30;
const int deadBand = 5;

/** Global values */
volatile enum _state {forwards, brake, backwards} requestedState, currentState;
volatile enum _temp {cool, warm, hot} temperatureState;
volatile enum _position {top, middle, bottom} positionState;
volatile int requestedVelocity = 0; // global speed & direction
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
inline long map(long x, long in_min, long in_max, long out_min, long out_max);
void goForwards(void);
void goBackwards(void);
void braking(void);
void motorBeep(unsigned char length);


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
	//TCCR0 |= _BV(CS01) | _BV(CS00); // set to 250KHz, overflows @ 1KHz
	TCCR0 |= _BV(CS02); // set to 62.5 KHz, overflows @ 245Hz

	// Timer 1 set up - RC pulse timing
	TCNT1 = 0x0000;
	TCCR1A = 0x00;
	TCCR1B = _BV(CS11); // set to 2MHz, overflows @ 32.8ms, normal mode

	// Timer 2 set up - H-Bridge PWM
	OCR2  = 0x00;
	TCNT2 = 0x00;
	TCCR2 = _BV(CS21); // set to 2MHz, overflows @ 7.8KHz, normal mode, OC2 disconnected

	// Timer interrupts
	TIFR  = 0xFF; // clear timer flags
	TIMSK = _BV(OCIE2) | _BV(TOIE2) | _BV(TOIE1) | _BV(TOIE0); // enable all timers for overflow and timer2 match

	// ADC set up
	ADMUX = _BV(REFS0) | _BV(ADLAR) | selTempPin; // ref to AVcc, left adjust, temperature pin
	ADCSRA = _BV(ADEN) | _BV(ADIF) | _BV(ADIE) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0); // enable ADC, clear and enable interrupt, 125kHz clock

	/** Read EEPROM for saved values, if empty (0xFF) use defaults. */
	minPulse = (signed)eeprom_read_word(&savedMinPulse);
	if((unsigned int)minPulse == 0xFFFF) minPulse = defaultMinPulse;
	maxPulse = (signed)eeprom_read_word(&savedMaxPulse);
	if((unsigned int)maxPulse == 0xFFFF) maxPulse = defaultMaxPulse;
	minPosition = (signed)eeprom_read_word(&savedMinPosition);
	if((unsigned int)minPosition == 0xFFFF) minPosition = defaultMinPosition;
	maxPosition = (signed)eeprom_read_word(&savedMaxPosition);
	if((unsigned int)maxPosition == 0xFFFF) maxPosition = defaultMaxPosition;
	temperatureHot = eeprom_read_byte(&savedTempHot);
	if((unsigned char)temperatureHot == 0xFF) temperatureHot = defaultTempHot;
	temperatureWarm = eeprom_read_byte(&savedTempWarm);
	if((unsigned char)temperatureWarm == 0xFF) temperatureWarm = defaultTempWarm;

	/** initialise states */
	temperatureState = cool;
	positionState = middle;
	requestedState = brake;
	currentState = brake;
	braking();

	_delay_ms(100);

	sei(); // enable global interrupts

	// Finished initialising the Motor Controller

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

				#ifdef MODE_ENDSTOP

				// calibrate pot

				#endif // MODE_ENDSTOP

				eeprom_update_word(&savedMinPulse, 0xFFFF); // new minPulse
				eeprom_update_word(&savedMaxPulse, 0xFFFF); // new maxPulse
				eeprom_update_word(&savedMinPosition, 0xFFFF); // new minPosition
				eeprom_update_word(&savedMaxPosition, 0xFFFF); // new maxPosition
				eeprom_update_byte(&savedTempHot, 0xFF);   // new temperatureHot
				eeprom_update_byte(&savedTempWarm, 0xFF);   // new temperatureWarm

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

	ADCSRA |= _BV(ADSC); // start ADC conversions

	while(1);
	// all following actions are carried out as the result of interrupts

	return 0;
}

/** Motor value calculations */
ISR(TIMER0_OVF_vect) {

	int requestedVelocityLocal = requestedVelocity;
	unsigned char requestedSpeed = 0; // local speed variable

	// clamp
	if(requestedVelocityLocal >  255) requestedVelocityLocal =  255;
	if(requestedVelocityLocal < -255) requestedVelocityLocal = -255;

	// temperature management
	if(requestedVelocityLocal >  128 && temperatureState == warm) requestedVelocityLocal =  128;
	if(requestedVelocityLocal < -128 && temperatureState == warm) requestedVelocityLocal = -128;

	// position management
	if(requestedVelocityLocal > 0 && positionState == top)    requestedVelocityLocal = 0;
	if(requestedVelocityLocal < 0 && positionState == bottom) requestedVelocityLocal = 0;

	// fault management
	if(timeout >= 15 || failsafe >= 15) requestedVelocityLocal = 0;

	// convert requestedVelTemp to requestedState
	if(requestedVelocityLocal < 0)
	{
		requestedState = backwards;
		requestedSpeed = (unsigned char)-requestedVelocityLocal; // make positive value 0 - 255
	}
	else if(requestedVelocityLocal > 0)
	{
		requestedState = forwards;
		requestedSpeed = (unsigned char)requestedVelocityLocal;
	}
	else
	{
		requestedState = brake;
		requestedSpeed = 0;
	}

	// modify outputs
	if(requestedState != currentState)
	{
		if(currentState == brake) // brake to fwd or brake to bwd
		{
			// set pins & direction
			if(requestedState == forwards) goForwards();
			else goBackwards();
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
		if((char)requestedSpeed > OCR2)
		{
			if((char)requestedSpeed > (OCR2 + maxSlew)) OCR2 += maxSlew;
			else OCR2 = (char)requestedSpeed;
		}
		else if((char)requestedSpeed < OCR2)
		{
			if((char)requestedSpeed < (OCR2 - maxSlew)) OCR2 -= maxSlew;
			else OCR2 = (char)requestedSpeed;
		}
		// else do nothing
	}
}

/** RC pulse timeout handling */
ISR(TIMER1_OVF_vect) {
	// no pulse received in 32.767 milliseconds
	if(timeout < 30) timeout ++;
}

/** RC pulse capture */
ISR(INT0_vect) {

	unsigned int pulseLength;
	int requestedVelocityLocal = 0; // local storage of velocity

	if(readRCpin) TCNT1 = 0x0000; // clear timer1 if rising edge
	else // falling edge
	{
		pulseLength = TCNT1; // save a local copy of timer1

		if(pulseLength >= (unsigned int)minPulse && pulseLength <= (unsigned int)maxPulse) // valid pulse?
		{
			requestedVelocityLocal = map(pulseLength, minPulse, maxPulse, (-255 - deadBand), (255 + deadBand)); // map pulse to speed
			if(requestedVelocityLocal >= -deadBand && requestedVelocityLocal <= deadBand) requestedVelocity = 0; // dead band adjustment
			else if(requestedVelocityLocal > 0) requestedVelocity = (int)requestedVelocityLocal - deadBand;
			else if(requestedVelocityLocal < 0) requestedVelocity = (int)requestedVelocityLocal + deadBand;

			if(timeout < 3) timeout = 0; // clear timeout 3 times faster than it accumulates
			else timeout -= 3;

			if(failsafe < 3) failsafe = 0; // clear failsafe 3 times faster than it accumulates
			else failsafe -= 3;
		}
		else
		{
			if(failsafe < 30) failsafe ++; // invalid pulse length
		}
	}
}

/** When comparator is triggered clear low ports (off part of PWM cycle) */
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

/** When timer overflows enable low port for current state */
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

/** Updates the temperature & position state */
ISR(ADC_vect) {

	static enum _adcChannel {temperature, position} adcChannel = temperature;

	if(adcChannel == temperature)
	{
		if (ADCH < temperatureHot)
		{
			temperatureState = hot;
			requestedState = brake;
		}
		else if(ADCH < temperatureWarm) temperatureState = warm;

		#ifdef MODE_ENDSTOP
		adcChannel = position;
		ADMUX = (ADMUX & 0xF0) | (selPosPin & 0x0F);
	}
	else
	{
		int positionPercent = (ADCH - minPosition) * (100 - 0) / (maxPosition - minPosition) + 0;

		if(positionPercent > 90) positionState = top;
		else if(positionPercent < 10) positionState = bottom;
		else positionState = middle;
		#endif // MODE_ENDSTOP

		adcChannel = temperature;
		ADMUX = (ADMUX & 0xF0) | (selTempPin & 0x0F);
	}

	ADCSRA |= _BV(ADSC); // start conversion
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
inline long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/** Configures the H-Bridge to move the motor forwards */
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

/** Configures the H-Bridge to move the motor backwards */
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

/** Configures the H-Bridge to short both side of the motor */
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

/** Pulses the motor quickly to generate beeps */
void motorBeep(unsigned char length) {

	unsigned char countBeep = 0;

	TIMSK &= ~_BV(TOIE0); // disable timer0 interrupts for a bit

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

	TIFR  |= _BV(TOV0); // clear flag just in case
	TIMSK |= _BV(TOIE0); // re-enable timer0 interrupts

}


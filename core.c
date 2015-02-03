/**
 *
 *
 */
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <stdint.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <math.h>
#include <avr/eeprom.h>

/** Mode selection */
#define MODE_ESC
//#define MODE_SERVO
//#define CALIBRATE



/** Defaults */
const int fetDelay = 200; // 200 microseconds
const int defaultMinPulse = 1000; // 1 millisecond
const int defaultMaxPulse = 2000; // 2 millisecond
const int defaultMinPosition = 0; // pot fully down
const int defaultMaxPosition = 1023; // pot fully up
const int kProportional = 1;
const int kDifferential = 0;
const int maxSlew = 30;

volatile int requestedVelocity = 0; // global speed & direction
volatile int requestedPosition = 0; // global requested position
volatile int currentPosition = 0;   // global current position

enum _state {forwards, brake, backwards} requestedState, currentState;

/** Pin Map (TZ-85A) */

// Port D, Pin 2 - RC signal input
#define enbRCpin DDRD &= !_BV(2)
#define readRCpin PIND &= _BV(2)

// Port , Pin - temperature sensor input
#define selTempPin 0b00000001

// Port, Pin - position sensor input
#define selPosPin 0b00000010

// Port C, Pin 5 - high side gate, forward
#define enbForwardHigh DDRC |= _BV(5)
#define setForwardHigh PORTC |= _BV(5)
#define clrForwardHigh PORTC &= !_BV(5)
// Port D, Pin 5 - low side gate, forwards
#define enbForwardLow DDRD |= _BV(5)
#define setForwardLow PORTD |= _BV(5)
#define clrForwardLow PORTD &= !_BV(5)
// Port D, Pin 4 - high side gate, backwards
#define enbBackwardHigh DDRD |= _BV(4)
#define clrBackwardHigh PORTD |= _BV(4)
#define setBackwardHigh PORTD &= !_BV(4)
// Port C, Pin 4 - low side gate, backwards
#define enbBackwardLow DDRC |= _BV(4)
#define clrBackwardLow PORTC |= _BV(4)
#define setBackwardLow PORTC &= !_BV(4)



void main(void) {
	
	// set things up
	
	// Pin set up
	enbRCpin;
	enbForwardHigh;
	enbForwardLow;
	enbBackwardHigh;
	enbBackwardLow;
	
	// Timers
	//Timer0 for timeout & motor calcs
	//Timer1 for RC pulse timing
	//Timer2 for PWM generation
	
	// ADC setup
	
	#ifdef CALIBRATE
	
	// temperature limits
	// exponential?
	// slew rate
	
	eeprom_write_word(0x0000, 0xFFFF); // new minPulse
	eeprom_write_word(0x0000, 0xFFFF); // new maxPulse
	eeprom_write_word(0x0000, 0xFFFF); // new minPosition
	eeprom_write_word(0x0000, 0xFFFF); // new maxPosition	
	#endif

	/** Read EEPROM for saved values, if empty (0xFF) use defaults. */
	minPulse = eeprom_read_word(0x0000);
	if(minPulse == 0xFFFF) defaultMinPulse;
	maxPulse = eeprom_read_word(0x0000);
	if(maxPulse == 0xFFFF) defaultMaxPulse;
	minPosition = eeprom_read_word(0x0000);
	if(minPosition == 0xFFFF) defaultMinPosition;
	maxPosition = eeprom_read_word(0x0000);
	if(maxPosition == 0xFFFF) defaultMaxPosition;
	
	requestedState = brake;
	currentState = brake;
	
	timeout = 0;
	
	motorBeep(3);
	
	while(1)
	{
		// meat of it
		
		if(requestedVelocity > 0)
		{
			// FORWARD!
		}
		else if(requestedVelocity < 0)
		{
			// BACKWARD!
		}
		else
		{
			// BRAKE!
		}	
		
		
	}
	
	return;
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
	
	// convert requestedVelocity to requestedState
	if(requestedVelocity < 0) 
	{
		requestedState = backwards;
		requestedVelocity *= -1; // make positive value 0 - 255
	}
	else if(requestedVelocity > 0) requestedState = forward;
	else requestedState = brake;	
	
	// modify outputs
	if(requestedState != currentState)
	{
		if(currentState == brake) // brake to fwd or brake to bwd
		{
			// set direction
			currentState = requestedState;
			
			// set pins
			//if(currentState == forwards) goForwards(); else goBackwards();
			
			// increase speed
			if(requestedVelocity > (OCR2 + maxSlew)) OCR2 += maxSlew;
			else OCR2 = requestedVelocity;
		}
		else // fwd to bwk or bwk to fwd
		{
			// decrease speed
			if(requestedVelocity < (OCR2 - maxSlew)) OCR2 -= maxSlew;
			else OCR2 = requestedVelocity;
			
			// set pins
			// braking();
			
			// set direction
			if(OCR2 == 0) currentState = brake;
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

/** RC pulse capture
 *
 */
ISR(INT0_vect) {
	
	unsigned int pulseLength;
	
	if(rising_edge(RC_Pin)) TCNT1 = 0x0000; // clear timer1
	else pulseLength = TCNT1; // copy timer1 
	
	if(pulseLength >= minPulse && pulseLength <= maxPulse) // valid pulse?
	{
		// valid pulse, therefore map to requested variable
		#ifdef MODE_SERVO
			requestedPosition = map(pulseLength, minPulse, maxPulse, 0, 512); // map pulse to position		
			currentPosition = map(ADC_pin, minPosition, maxPosition, 0, 512); // get & map error signal		
		#else // default to ESC mode
			requestedVelocity = map(pulseLength, minPulse, maxPulse, -255, 255); // output value in -255 to +255
		#endif

	}
	else
	{
		// invalid pulse
		failsafe --;
		if(failsafe <= 0) requestedVelocity = 0;
	}
}

/** When comparator is triggered clear low ports (off part of PWM cycle)
 *
 */
ISR(TIMER2_COMP_vect) { //
    if (OCR2 < 255) {
        if (currentState != brake) {
            //Clear high
            clrForwardHigh();
            clrBackwardHigh();
        }
    }
}

/** When timer overflows enable low port for current state
 *
 */
ISR(TIMER2_OVF_vect) { 
    if (OCR2 > 0) {
        if (currentState == forwards) {
            setForwardHigh();
        } else if (currentState == backwards) {
            setBackwardHigh();
        }
    }
    if (OCR2 > 250 && currentState != brake) {
        lets_get_high++; //Gotta keep the charge pump circuit charged
        if (lets_get_high > 50) { //If it hasn't had a chance to charge in a while
            clrForwardHigh(); //Clear it then reset counter   
            clrBackwardHigh(); //Now pumped up and remaining high so we don't nasty shoot through which ends in magic smoke :)
            lets_get_high = 0;
        }
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


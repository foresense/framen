/*	

FRAMEN v2.0
by Robert Beenen

	MOD1:	1-15 -> amen slice
			16 -> random sample
	MOD2:	0-50% -> play from start to 0-100%
			51-100% -> loop starts at 0-100% to end
	KNOB3:	0-100% -> pitch -1 oct to +1 oct
	INPUT3:	trigger input

	Just play around with it, it's pretty straight forward.

	*/

#include "amen.h"

#define MOD1_PIN	A2		// KNOB1 / INPUT1
#define MOD2_PIN	A1		// KNOB2 / INPUT2
#define KNOB3_PIN	A0		// KNOB3
#define INPUT3_PIN	A3		// INPUT3
#define OUT_PIN		11

#define SAMPLERATE	8000
#define UPDATERATE	(F_CPU / SAMPLERATE)
#define SILENCE		0x80

// inputs
uint8_t mod1;
uint16_t mod2;
uint16_t knob3;
uint8_t input3;

// internal
uint16_t offset;
uint16_t length;
uint16_t loop_start;
uint16_t index;

bool playing;
bool triggered;
bool looping;

uint8_t seed = 1;
uint8_t xorshift(void) {
	if(!seed) seed++;
	seed ^= (seed << 7);
    seed ^= (seed >> 5);
	seed ^= (seed << 3);
	return seed;
}

void setup() {
	pinMode(OUT_PIN, OUTPUT);

	// Setup Timer 2 to do pulse width modulation on the speaker pin
    ASSR &= ~(_BV(EXCLK) | _BV(AS2));	// use internal clock (datasheet p.160)
    TCCR2A |= _BV(WGM21) | _BV(WGM20);	// set fast PWM mode (p.155)
    TCCR2B &= ~_BV(WGM22);    
    TCCR2A = (TCCR2A | _BV(COM2A1)) & ~_BV(COM2A0);	// non-inverting PWM on OC2A (p.155)
    TCCR2A &= ~(_BV(COM2B1) | _BV(COM2B0));			// OC2A = OUT_PIN
    TCCR2B = (TCCR2B & ~(_BV(CS12) | _BV(CS11))) | _BV(CS10);	// no prescaler (p.158)

    OCR2A = SILENCE;	// set initial output to silent
    
	// Setup Timer 1 to send a sample every interrupt.
    cli();
    TCCR1B = (TCCR1B & ~_BV(WGM13)) | _BV(WGM12);	// CTC mode (p.133)
    TCCR1A = TCCR1A & ~(_BV(WGM11) | _BV(WGM10));
    TCCR1B = (TCCR1B & ~(_BV(CS12) | _BV(CS11))) | _BV(CS10);	// no prescaler
    
    OCR1A = UPDATERATE;
    
    TIMSK1 |= _BV(OCIE1A);		// enable interrupt when TCNT1 == OCR1A (p.136)
    sei();
}

void loop() {
	mod1 = analogRead(MOD1_PIN) >> 6;		// reduce to 4 bits
	mod2 = analogRead(MOD2_PIN);
	knob3 = analogRead(KNOB3_PIN);
	input3 = digitalRead(INPUT3_PIN);		// using digital read on an analog input works

	if(mod1 == 0x0F) {
		mod1 = map(xorshift(), 0, 255, 0, 14);	// use a randomizer when mod1 is at max value
	}
	offset = slice_start[mod1];

	if(mod2 & 0x200) {
		looping = true;
		length = slice_length[mod1];
		mod2 ^= 0x3FF;
		loop_start = length - map(mod2, 0, 511, 4 * GRAINSIZE, length);
	}
	else {
		looping = false;
		length = map(mod2, 0, 511, 4 * GRAINSIZE, slice_length[mod1]);
	}

	OCR1A = map(analogRead(KNOB3_PIN), 0, 1023, 4000, 1000);
}

ISR(TIMER1_COMPA_vect) {
	if(input3 && !triggered) {
		index = 0;
		playing = true;
		triggered = true;
	}
	else if(!input3 && triggered) {
		triggered = false;
	}

	if(index >= length) {
		if(looping) {
			index = loop_start;
		}
		else {
			playing = false;
			OCR2A = SILENCE;
		}
	}
	else {
		index++;
	}
	
	if(playing) {
		OCR2A = pgm_read_byte(&sample_data[(offset + index) % SAMPLESIZE]);
	}
}

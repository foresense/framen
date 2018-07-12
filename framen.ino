/*	

FRAMEN v2.0
by Robert Beenen

	MOD1:	Sample Start (0-31)
	MOD2:	Loop Size	(0-1023) -- closer to 1023 more bits get used
	KNOB3:	Pitch
	INPUT3:	Sample Trigger

	loop:

		mod2 -> 0-1023
		0-511: No looping, cut off sample (center = whole sample)
			   Play from start to 511 = end
		512-1023: Looping playback from end (center = loop whole sample)
				  Loop start moves to the back (right most = tiny loop)

	               non looping portion
	0b0000000001 - (1) * 2 * 27
	0b0000100000 - 27 << 6 == 1728 (average loop length)
	0b0001000000 - 27 << 7 == 3456 (2x)
	0b0010000000 - 27 << 8 == 6912 (4x)
	0b0100000000 - 27 << 9 == 13824 (8x)
	               looping portion
	0b1000000000 - 512 (8x average loop length)
	0b1000000001 -  4x loop

	*/


// mod1 == 16 -> pick a random offset between 0-14
//
// do a fade out on the last few samples if (mod2 < 512)

#include "amen.h"

#define MOD1_PIN	A2		// KNOB1 / INPUT1
#define MOD2_PIN	A1		// KNOB2 / INPUT2
#define KNOB3_PIN	A0		// KNOB3
#define INPUT3_PIN	A3		// INPUT3
#define OUT_PIN	11

#define SAMPLERATE	8000
#define UPDATERATE	2000	// (F_CPU / SAMPLERATE)
#define SILENCE		0x80

// inputs
uint8_t mod1;
uint16_t mod2;
uint16_t knob3;
uint8_t input3;

// internal
uint16_t offset;
uint16_t offset_p;
uint16_t length;
uint16_t length_p;
uint16_t loop_start;
uint16_t index;

bool playing;
bool triggered;
bool looping;

uint8_t seed = 1;
void xorshift(void) {
	if(!seed) seed ++;
	seed ^= (seed << 7);
    seed ^= (seed >> 5);
	seed ^= (seed << 3);
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
    OCR1A = F_CPU / SAMPLERATE;
    TIMSK1 |= _BV(OCIE1A);		// enable interrupt when TCNT1 == OCR1A (p.136)
    sei();
}

void loop() {
	mod1 = analogRead(MOD1_PIN) >> 6;
	mod2 = analogRead(MOD2_PIN);
	knob3 = analogRead(KNOB3_PIN);
	input3 = digitalRead(INPUT3_PIN);

	if(mod1 == 0x0F) mod1--;
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
		index ++;
	}
	
	if(playing) {
		OCR2A = pgm_read_byte(&sample_data[(offset + index) % SAMPLESIZE]);
	}
}

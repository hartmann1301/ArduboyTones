/**
 * @file ArduboyTones.cpp
 * \brief An Arduino library for playing tones and tone sequences, 
 * intended for the Arduboy game system.
 */

/*****************************************************************************
  ArduboyTones

An Arduino library to play tones and tone sequences.

Specifically written for use by the Arduboy miniature game system
https://www.arduboy.com/
but could work with other Arduino AVR boards that have 16 bit timer 3
available, by changing the port and bit definintions for the pin(s)
if necessary.

Copyright (c) 2017 Scott Allen

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*****************************************************************************/

#ifdef ESP8266
// this library is needed to get easy access to the esp82666 timer functions 
#include <Ticker.h>

Ticker tonesTicker;
#endif

#include "ArduboyTones.h"

// pointer to a function that indicates if sound is enabled
static bool (*outputEnabled)();

static volatile long durationToggleCount = 0;
static volatile bool tonesPlaying = false;
static volatile bool toneSilent;

#ifdef ESP8266
uint16_t *tonesStart;	
uint16_t *tonesIndex;	
uint16_t toneSequence[MAX_TONES * 2 + 1];
#else 
static volatile uint16_t *tonesStart;	
static volatile uint16_t *tonesIndex;	
static volatile uint16_t toneSequence[MAX_TONES * 2 + 1];
#endif 

static volatile bool inProgmem;

void updateTones();

ArduboyTones::ArduboyTones(bool (*outEn)())
{
  outputEnabled = outEn;

  toneSequence[MAX_TONES * 2] = TONES_END;

#ifdef ESP8266
	// sets the update call interval
  //tonesTicker.attach_ms(1000, updateTones);

#else
  bitClear(TONE_PIN_PORT, TONE_PIN); // set the pin low
  bitSet(TONE_PIN_DDR, TONE_PIN); // set the pin to output mode
#endif
}

void ArduboyTones::tone(uint16_t freq, uint16_t dur)
{
#ifdef ESP8266

#else
  bitWrite(TIMSK3, OCIE3A, 0); // disable the output compare match interrupt	
#endif

  inProgmem = false;
  tonesStart = tonesIndex = toneSequence; // set to start of sequence array
  toneSequence[0] = freq;
  toneSequence[1] = dur;
  toneSequence[2] = TONES_END; // set end marker
  nextTone(); // start playing
}

void ArduboyTones::tone(uint16_t freq1, uint16_t dur1,
                        uint16_t freq2, uint16_t dur2)
{
#ifdef ESP8266

#else
  bitWrite(TIMSK3, OCIE3A, 0); // disable the output compare match interrupt	
#endif

  inProgmem = false;
  tonesStart = tonesIndex = toneSequence; // set to start of sequence array
  toneSequence[0] = freq1;
  toneSequence[1] = dur1;
  toneSequence[2] = freq2;
  toneSequence[3] = dur2;
  toneSequence[4] = TONES_END; // set end marker
  nextTone(); // start playing
}

void ArduboyTones::tone(uint16_t freq1, uint16_t dur1,
                        uint16_t freq2, uint16_t dur2,
                        uint16_t freq3, uint16_t dur3)
{
#ifdef ESP8266

#else
  bitWrite(TIMSK3, OCIE3A, 0); // disable the output compare match interrupt	
#endif

  inProgmem = false;
  tonesStart = tonesIndex = toneSequence; // set to start of sequence array
  toneSequence[0] = freq1;
  toneSequence[1] = dur1;
  toneSequence[2] = freq2;
  toneSequence[3] = dur2;
  toneSequence[4] = freq3;
  toneSequence[5] = dur3;
  // end marker was set in the constructor and will never change
  nextTone(); // start playing
}

void ArduboyTones::tones(const uint16_t *tones)
{
#ifdef ESP8266

#else	
  bitWrite(TIMSK3, OCIE3A, 0); // disable the output compare match interrupt	
#endif

  inProgmem = true;
  tonesStart = tonesIndex = (uint16_t *)tones; // set to start of sequence array
  nextTone(); // start playing
}

void ArduboyTones::tonesInRAM(uint16_t *tones)
{
#ifdef ESP8266

#else	
  bitWrite(TIMSK3, OCIE3A, 0); // disable the output compare match interrupt	
#endif

  inProgmem = false;
  tonesStart = tonesIndex = tones; // set to start of sequence array
  nextTone(); // start playing
}

void ArduboyTones::noTone()
{
#ifdef ESP8266
	::noTone(TONES_PIN);
#else	
  bitWrite(TIMSK3, OCIE3A, 0); // disable the output compare match interrupt
  TCCR3B = 0; // stop the counter
  bitClear(TONE_PIN_PORT, TONE_PIN); // set the pin low	
#endif		

  tonesPlaying = false;
}

void ArduboyTones::volumeMode(uint8_t mode)
{
}

bool ArduboyTones::playing()
{
  return tonesPlaying;
}

void ArduboyTones::nextTone()
{
  uint16_t freq;
  uint16_t dur;
  long toggleCount;
#ifndef ESP8266
  uint32_t ocrValue;
#endif

  freq = getNext(); // get tone frequency

  if (freq == TONES_END) { // if freq is actually an "end of sequence" marker
    noTone(); // stop playing
    return;
  }

  tonesPlaying = true;

  if (freq == TONES_REPEAT) { // if frequency is actually a "repeat" marker
    tonesIndex = tonesStart; // reset to start of sequence
    freq = getNext();
  }

  freq &= ~TONE_HIGH_VOLUME; // strip volume indicator from frequency

	if (freq == 0) { // if tone is silent
#ifdef ESP8266
		noTone();
#else		
		ocrValue = F_CPU / 8 / SILENT_FREQ / 2 - 1; // dummy tone for silence
		freq = SILENT_FREQ;
		bitClear(TONE_PIN_PORT, TONE_PIN); // set the pin low		
#endif		
		toneSilent = true;
	}
	else {
		
#ifndef ESP8266	
		ocrValue = F_CPU / 8 / freq / 2 - 1;
#endif
		toneSilent = false;
	}

  if (!outputEnabled()) { // if sound has been muted
    toneSilent = true;
  }

#ifdef ESP8266

#ifdef TONES_SERIAL_DEBUG
	Serial.print(millis(), DEC);
	Serial.print(" freq:");
	Serial.println(freq, DEC);
#endif 
	
	// play the actual tone with the std tone library if not muted
	if (toneSilent) {
		noTone();
	} else {
		::tone(TONES_PIN, freq);
	}	
#endif

  dur = getNext(); // get tone duration
	
#ifdef ESP8266	

#ifdef TONES_SERIAL_DEBUG
	Serial.print(" next Tone in:");
	Serial.println(dur, DEC);
#endif 

	// set a timer to update the tone after its duration
	tonesTicker.once_ms(dur, updateTones);

#else	
  if (dur != 0) {
    // A right shift is used to divide by 512 for efficency.
    // For durations in milliseconds it should actually be a divide by 500,
    // so durations will by shorter by 2.34% of what is specified.
    toggleCount = ((long)dur * freq) >> 9;
  }
  else {
    toggleCount = -1; // indicate infinite duration
  }

  TCCR3A = 0;

  TCCR3B = _BV(WGM32) | _BV(CS31); // CTC mode, prescaler /8

  OCR3A = ocrValue;
  durationToggleCount = toggleCount;
  bitWrite(TIMSK3, OCIE3A, 1); // enable the output compare match interrupt
#endif
}

uint16_t ArduboyTones::getNext()
{
  if (inProgmem) {
    return pgm_read_word(tonesIndex++);
  }
  return *tonesIndex++;
}

#ifdef ESP8266
void updateTones(){
	//Serial.println("tik tok");
	
	// there is no need for toggling, this is the problem of tone()

#ifdef TONES_SERIAL_DEBUG
	Serial.print(millis(), DEC);
	Serial.println(" nextTone()");
#endif 

	ArduboyTones::nextTone();
}
#else
ISR(TIMER3_COMPA_vect)
{
  if (durationToggleCount != 0) {
    if (!toneSilent) {
      *(&TONE_PIN_PORT) ^= TONE_PIN_MASK; // toggle the pin
    }
    if (durationToggleCount > 0) {
      durationToggleCount--;
    }
  }
  else {
    ArduboyTones::nextTone();
  }
}
#endif

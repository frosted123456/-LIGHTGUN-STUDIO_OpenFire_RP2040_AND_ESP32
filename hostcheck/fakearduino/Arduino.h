// Just enough Arduino for a SYNTAX check of the RP2040-only sources.
//
// recoil_fx_glue.cpp is the file that actually drives the solenoid and rumble
// pins, and it is compiled only by the Arduino toolchain -- which nobody here
// has. So its real branch (ARDUINO_ARCH_RP2040 && LIGHTGUN_RECOIL_FX) went
// through no compiler at all between edits, while the host suite happily
// checked the inert stub next to it and reported OK.
//
// These declarations exist to be compiled against, not linked: they say what
// the functions are CALLED, and nothing about what they do. A signature here
// that disagrees with the real core would make this check meaningless, so keep
// them matching arduino-pico's.
#pragma once
#include <stdint.h>
#include <stddef.h>

#define HIGH 1
#define LOW  0
#define INPUT        0
#define OUTPUT       1
#define INPUT_PULLUP 2
#define INPUT_PULLDOWN 3

void digitalWrite(unsigned char pin, unsigned char val);
int  digitalRead(unsigned char pin);
void pinMode(unsigned char pin, unsigned char mode);
void analogWrite(unsigned char pin, int val);
void analogWriteFreq(uint32_t freq);
unsigned long millis(void);
unsigned long micros(void);
void delay(unsigned long ms);
void delayMicroseconds(unsigned int us);
uint64_t time_us_64(void);

/**
\mainpage Main Page
Rotary Encoder Library for Arduino
----------------------------------

This is an adaptation of Ben Buxton's excellent 
[rotary library](http://www.buxtronix.net/2011/10/rotary-encoders-done-properly.html)
and implements additional features for encoder rotation speed.

Original library copyright 2011 Ben Buxton. Licensed under the GNU GPL Version 3.
Contact: bb@cactii.net

FELA fork (MD_REncoder_fela) feeds A/B from a mux snapshot via read(valueA, valueB)
instead of digitalRead(), and uses a shorter speed window for synth-panel acceleration.

Features
--------
- Debounce handling with support for high rotation speeds
- Correctly handles direction changes mid-step
- Checks for valid state changes for more robust counting and noise immunity
- Interrupt based or polling in loop()
- Counts full-steps (default) or half-steps
- Calculates speed of rotation
- Mux-fed A/B bits (no GPIO read inside the decoder)

If you like and use this library please consider making a small donation using [PayPal](https://paypal.me/MajicDesigns/4USD)

Topics
------
- \subpage pageBackground
- \subpage pageLibrary

Revision History
----------------
Aug 2026 - version 1.1.1 (FELA)
- MD_RENCODER_FELA_SRAM_HOT pins read() in SRAM (__not_in_flash_func)

Aug 2026 - version 1.1.0 (FELA)
- read(valueA, valueB) takes mux (or other) A/B bits instead of digitalRead()
- begin() does not call pinMode (caller / mux owns GPIO)
- DEFAULT_PERIOD 60 ms; speed = ClickCount * (400 / period)

Jan 2020 - version 1.0.1
- Adjusted order of class initializers to fix errors in some compilers

April 2014 - version 1.0
- Initial implementation from Ben's code
- Cleaned up some compile issues and added begin() method
- Updated and documented
- Added speed functionality

Copyright
---------
This adaptation copyright (C) 2014 Marco Colli. All rights reserved.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 3 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA

\page pageBackground Detecting an Encoder's Rotation
_This great explanation is reproduced from Ben Buxton's example code._

A typical mechanical rotary encoder emits a two bit gray code on 3 output pins.
Every step in the output (often accompanied by a physical 'click') generates a 
specific sequence of output codes on the pins.

There are 3 pins used for the rotary encoding - one common and two 'bit' pins
(called A and B).

The following is the typical sequence of codes output when moving from one 
step to the next:

Position | Bit1| Bit2|
--------:|:---:|:---:|
Step1    |  0  |  0  |
  1/4    |  1  |  0  |
  1/2    |  1  |  1  |
  3/4    |  0  |  1  |
Step2    |  0  |  0  |

From this table, we can see that when moving from one 'click' to the next, 
there are 4 changes in the output code. 

- From an initial 0 - 0, Bit1 goes high, Bit0 stays low.
- Then both bits are high, halfway through the step.
- Then Bit1 goes low, but Bit2 stays high.
- Finally at the end of the step, both bits return to 0.

Detecting the direction is easy - the table simply goes in the other direction
(read up instead of down).

To decode this, we use a simple state machine. Every time the output code changes,
it follows state, until finally a full steps worth of code is received (in the 
correct order). At the final 0-0, it returns a value indicating a step in one 
direction or the other.

It's also possible to use 'half-step' mode. This just emits an event at both the 
0-0 and 1-1 positions. This might be useful for some encoders where you want to 
detect all positions. In MD_REncoder_fela.h set ENABLE_HALF_STEP to 1 to enable 
half-step mode.

If an invalid state happens (for example we go from '0-1' straight to '1-0'), the 
state machine resets to the start until 0-0 and the next valid codes occur.

The biggest advantage of using a state machine over other algorithms is that this 
has inherent debounce built in. Other algorithms emit spurious output with switch 
bounce, but this one will simply flip between sub-states until the bounce settles, 
then continue along the state machine. A side effect of debounce is that fast 
rotations can cause steps to be skipped. By not requiring debounce, fast rotations 
can be accurately measured. Another advantage is the ability to properly handle bad 
state, such as due to EMI, etc. It is also a lot simpler than others - a static 
state table and less than 10 lines of logic.

\page pageLibrary The REncoder Library

Compile Time Switches
---------------------

ENABLE_HALF_STEP is 0 by default. Set this to 1 to emit codes when the rotary encoder
is at 11 as well as 00. The default is to emit codes only at 00.

ENABLE_PULLUPS is set to 1 by default. Set this 0 if internal pullup resistors on the
input pins are not required. FELA begin() does not apply pinMode; pullups are only
relevant if the caller configures GPIO itself.

ENABLE_SPEED is set to 1 by default. Set this to 0 to disable the code and storage used to 
calculate the speed of the encoder rotation.

Speed Calculation
-----------------
The number of clicks is accumulated during the period defined by setPeriod(). Once the time 
has expired the velocity is calculated as a panel acceleration gain (not SI clicks/s):

speed = ClickCount * (400 / period)

With DEFAULT_PERIOD 60 this is ClickCount * 6 (integer). Shorter periods update
acceleration more often; 400 (instead of 1000) keeps the gain usable on a synth panel.

*/
#ifndef _MD_RENCODER_FELA_H
#define _MD_RENCODER_FELA_H

#include <Arduino.h>

// 1 = RP2040 __not_in_flash_func on read() (define before include).
// 0 = portable / flash (library default). No-op if the attribute is missing.
#ifndef MD_RENCODER_FELA_SRAM_HOT
#define MD_RENCODER_FELA_SRAM_HOT 0
#endif
#if MD_RENCODER_FELA_SRAM_HOT
#ifndef __not_in_flash_func
#define __not_in_flash_func(fn) fn
#endif
#define MD_RENCODER_FELA_HOT(fn) __not_in_flash_func(fn)
#else
#define MD_RENCODER_FELA_HOT(fn) fn
#endif

// Library options
#define ENABLE_HALF_STEP  0
#define ENABLE_PULLUPS    1
#define ENABLE_SPEED      1
#define DEFAULT_PERIOD    40

// Direction values returned by read() method 
#define DIR_NONE  0x00
#define DIR_CW    0x10
#define DIR_CCW   0x20  

class MD_REncoder
{
  public:
    MD_REncoder(uint8_t pinA, uint8_t pinB);
    void begin(void);

    // Standard read (emits on every single click)
    uint8_t read(uint8_t valueA, uint8_t valueB);

    // Rate-limited read: accumulates steps and emits at most once every intervalMs (default 20ms)
    // Returns 0 if throttled/idle, or the total signed movement (+1, -1, +3, -5, etc.)
    int16_t readDelta(uint8_t valueA, uint8_t valueB, uint32_t intervalMs = 20);

#if ENABLE_SPEED
    inline void setPeriod(uint16_t t) { if ((t != 0) && (t <= 1000)) _period = t; };
    inline uint16_t speed(void) { return(_spd); };
#endif

  private:
    uint8_t _pinA;
    uint8_t _pinB;
    uint8_t _state;

    // Rate-limiting accumulator
    int16_t  _delta;
    uint32_t _lastEmit;

#if ENABLE_SPEED    
    uint16_t  _period;
    uint16_t  _count;
    uint16_t  _spd;
    uint32_t  _timeLast;
#endif
};

#endif
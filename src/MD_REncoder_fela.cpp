/*
MD_REncoder_fela - Library for Rotary Encoders

See header file for comments

This version copyright (C) 2014 Marco Colli. All rights reserved.

Original library copyright 2011 Ben Buxton. Licenced under the GNU GPL Version 3.
Contact: bb@cactii.net

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
*/

/**
 * \file
 * \brief Implements core MD_REncoder class methods
 */
 #include "MD_REncoder_fela.h"

 #define R_START 0x0
 
 #if ENABLE_HALF_STEP
 // Use the half-step state table (emits a code at 00 and 11)
 #define R_CCW_BEGIN   0x1
 #define R_CW_BEGIN    0x2
 #define R_START_M     0x3
 #define R_CW_BEGIN_M  0x4
 #define R_CCW_BEGIN_M 0x5
 
 const unsigned char ttable[][4] = 
 {
   // 00                  01              10            11
   {R_START_M,           R_CW_BEGIN,     R_CCW_BEGIN,  R_START},           // R_START (00)
   {R_START_M | DIR_CCW, R_START,        R_CCW_BEGIN,  R_START},           // R_CCW_BEGIN
   {R_START_M | DIR_CW,  R_CW_BEGIN,     R_START,      R_START},           // R_CW_BEGIN
   {R_START_M,           R_CCW_BEGIN_M,  R_CW_BEGIN_M, R_START},           // R_START_M (11)
   {R_START_M,           R_START_M,      R_CW_BEGIN_M, R_START | DIR_CW},  // R_CW_BEGIN_M 
   {R_START_M,           R_CCW_BEGIN_M,  R_START_M,    R_START | DIR_CCW}  // R_CCW_BEGIN_M
 };
 #else
 // Use the full-step state table (emits a code at 00 only)
 #define R_CW_FINAL   0x1
 #define R_CW_BEGIN   0x2
 #define R_CW_NEXT    0x3
 #define R_CCW_BEGIN  0x4
 #define R_CCW_FINAL  0x5
 #define R_CCW_NEXT   0x6
 
 const unsigned char ttable[][4] = 
 {
   // 00         01           10           11
   {R_START,    R_CW_BEGIN,  R_CCW_BEGIN, R_START},           // R_START
   {R_CW_NEXT,  R_START,     R_CW_FINAL,  R_START | DIR_CW},  // R_CW_FINAL
   {R_CW_NEXT,  R_CW_BEGIN,  R_START,     R_START},           // R_CW_BEGIN
   {R_CW_NEXT,  R_CW_BEGIN,  R_CW_FINAL,  R_START},           // R_CW_NEXT
   {R_CCW_NEXT, R_START,     R_CCW_BEGIN, R_START},           // R_CCW_BEGIN
   {R_CCW_NEXT, R_CCW_FINAL, R_START,     R_START | DIR_CCW}, // R_CCW_FINAL
   {R_CCW_NEXT, R_CCW_FINAL, R_CCW_BEGIN, R_START}            // R_CCW_NEXT
 };
 #endif
 
 MD_REncoder::MD_REncoder(uint8_t pinA, uint8_t pinB):
 _pinA (pinA), _pinB (pinB), _state(R_START), _delta(0), _lastEmit(0)
 #if ENABLE_SPEED
 , _period(DEFAULT_PERIOD), _count(0), _spd(0), _timeLast(0)
 #endif
 {
 }
 
 void MD_REncoder::begin(void)
 {
 }
 
 uint8_t MD_RENCODER_FELA_HOT(MD_REncoder::read)(uint8_t valueA, uint8_t valueB) 
 {
   uint8_t pinstate = (valueA << 1) | (valueB);
   
   _state = ttable[_state & 0xf][pinstate]; 
   
 #if ENABLE_SPEED
   // handle the encoder velocity calc
   if (_state & 0x30) _count++;
   if (millis() - _timeLast >= _period)
   {
     _spd = _count * (400/_period);
     _timeLast = millis();
     _count = 0;
   }
 #endif
 
   return (_state & 0x30);
 }
 
 int16_t MD_RENCODER_FELA_HOT(MD_REncoder::readDelta)(uint8_t valueA, uint8_t valueB, uint32_t intervalMs)
 {
   // 1. Process physical pins through state machine
   uint8_t event = read(valueA, valueB);
   if (event == DIR_CW) {
     _delta++;
   } else if (event == DIR_CCW) {
     _delta--;
   }
 
   // 2. Rate limit: only emit the accumulated sum once per intervalMs
   uint32_t now = millis();
   if (_delta != 0 && (now - _lastEmit >= intervalMs)) {
     int16_t result = _delta;
     _delta = 0;
     _lastEmit = now;
     return result;
   }
 
   return 0;
 }
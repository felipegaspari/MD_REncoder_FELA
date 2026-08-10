/*
Rotary Encoder - Polling Example (MD_REncoder_fela)

The circuit:
* encoder pin A to Arduino pin 2
* encoder pin B to Arduino pin 3
* encoder ground pin to ground (GND)

FELA read() takes A/B bits (mux or digitalRead). begin() does not pinMode.
*/

#include <MD_REncoder_fela.h>

const uint8_t PIN_A = 2;
const uint8_t PIN_B = 3;

// set up encoder object
MD_REncoder R = MD_REncoder(PIN_A, PIN_B);

void setup() 
{
  Serial.begin(57600);
  pinMode(PIN_A, INPUT_PULLUP);
  pinMode(PIN_B, INPUT_PULLUP);
  R.begin();
}

void loop() 
{
  uint8_t x = R.read(digitalRead(PIN_A), digitalRead(PIN_B));
  
  if (x) 
  {
    Serial.print(x == DIR_CW ? "\n+1" : "\n-1");
#if ENABLE_SPEED
    Serial.print("  ");
    Serial.print(R.speed());
#endif
  }
}
